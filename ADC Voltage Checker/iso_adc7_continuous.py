#!/usr/bin/env python3
"""
Read ISO ADC 7 / AMC131M03 voltage data on Raspberry Pi SPI0.

A background thread polls SPI, tracks positive-going zero crossings, captures
100-sample waveform blocks, calibrates AC RMS baselines, and detects RMS
deviations. The main thread displays captured blocks and cycle-aligned RMS.
"""

from collections import deque
from datetime import datetime
import json
import logging
import math
from pathlib import Path
import signal
import sys
import threading
from threading import Event, Lock
import time

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from gpiozero import DigitalOutputDevice
import spidev

# ==============================================================================
# HARDWARE AND SPI CONFIGURATION PARAMETERS
# ==============================================================================

SPI_BUS = 0
SPI_DEVICE = 0
RST_BCM = 24
XEN_BCM = 25

SPI_MODE = 0b01
SPI_HZ = 500_000
FRAME_WORDS = 5
FRAME_BYTES = FRAME_WORDS * 3

REG_CLOCK = 0x03
CLOCK_1000_SPS = 0x0716
REG_DCDC_CTRL = 0x31
DCDC_FREQ_4P096_MHZ_AND_ENABLE = 0x0001

STATUS_SEC_FAIL = 1 << 6
STATUS_READY_MASK = 0b111

VOLTS_PER_CODE = 1.2 / (1 << 23)
SAMPLING_RATE_HZ = 1000.0  # Nominal; verify against the ADC configuration.
TRIGGER_BUFFER_SIZE = 100
MAX_OVERLAY_BLOCKS = 10
STARTUP_DISCARD_SAMPLES = 10
PRINT_INTERVAL = 0.5
ANOMALY_THRESHOLD_V = 0.10  # Difference in AC RMS from calibrated baseline.
BASELINE_SAMPLES = 100
RMS_CYCLES = 5
PLOT_REFRESH_SECONDS = 0.04

LOG_FILE = Path(__file__).resolve().with_name("log.txt")

logging.basicConfig(
    filename=LOG_FILE,
    filemode="a",
    level=logging.WARNING,
    format="%(asctime)s %(levelname)s %(message)s",
)

# ==============================================================================
# MATHEMATICAL CALCULATIONS AND UTILITY FUNCTIONS
# ==============================================================================

def ac_rms_from_volts(samples):
    """Return mean-removed AC RMS for a sequence of voltage samples."""
    if len(samples) < 2:
        return None

    mean = sum(samples) / len(samples)
    return math.sqrt(
        sum((value - mean) ** 2 for value in samples) / len(samples)
    )

def synchronized_rms_from_volts(samples, crossing_times, target_cycles=RMS_CYCLES):
    """
    Calculate AC and total RMS over an interval bounded by positive-going
    zero crossings. samples is an iterable of (timestamp, voltage) pairs.
    """
    if len(crossing_times) < target_cycles + 1:
        return None, None

    start_time = crossing_times[-target_cycles - 1]
    end_time = crossing_times[-1]

    active = [
        voltage
        for timestamp, voltage in samples
        if start_time <= timestamp <= end_time
    ]

    if len(active) < 2:
        return None, None

    mean = sum(active) / len(active)
    ac_rms = math.sqrt(
        sum((value - mean) ** 2 for value in active) / len(active)
    )
    total_rms = math.sqrt(sum(value ** 2 for value in active) / len(active))
    return ac_rms, total_rms

def report_anomaly(reason, **details):
    """Print and log a structured anomaly record."""
    record = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "anomaly": reason,
        **details,
    }
    message = json.dumps(record, default=str)
    print(f"ANOMALY: {message}", flush=True)
    logging.warning(message)

def word24(word16: int) -> list[int]:
    """Pack a 16-bit word into a 24-bit SPI word, MSB first."""
    return [(word16 >> 8) & 0xFF, word16 & 0xFF, 0]

def command_frame(command: int) -> list[int]:
    """Build a five-word SPI frame containing a command and zero padding."""
    return word24(command) + [0] * (FRAME_BYTES - 3)

def wreg_frame(address: int, value: int) -> list[int]:
    """Build a one-register write frame."""
    command = 0x6000 | ((address & 0x3F) << 7)
    return word24(command) + word24(value) + [0] * (FRAME_BYTES - 6)

def signed24(data: list[int]) -> int:
    """Convert a 24-bit two's-complement value to a signed integer."""
    value = (data[0] << 16) | (data[1] << 8) | data[2]
    return value - (1 << 24) if value & 0x800000 else value

def status_from_frame(rx: list[int]) -> int:
    """Extract the 16-bit status value from a received frame."""
    return (rx[0] << 8) | rx[1]

def channels_from_frame(rx: list[int]) -> list[int]:
    """Extract and sign-extend the three channel readings."""
    return [
        signed24(rx[3:6]),
        signed24(rx[6:9]),
        signed24(rx[9:12]),
    ]

# ==============================================================================
# BACKGROUND ACQUISITION WORKER THREAD
# ==============================================================================

def adc_acquisition_worker(spi, stop_event, data_lock, shared_state):
    """Poll the ADC and update capture, calibration, and monitoring state."""
    try:
        def null_frame() -> list[int]:
            return spi.xfer2(command_frame(0x0000))

        # Flush stale pipeline data after configuration.
        null_frame()
        null_frame()

        # Wait for the isolated-side supply fault flag to clear.
        deadline = time.monotonic() + 5.0
        status = 0
        device_ready = False

        while not stop_event.is_set() and time.monotonic() < deadline:
            rx = null_frame()
            status = status_from_frame(rx)

            if ((status >> 8) & 0b11) != 0b01:
                raise RuntimeError(
                    f"Unexpected STATUS 0x{status:04X}; "
                    "check SPI wiring and communication mode."
                )

            if not (status & STATUS_SEC_FAIL):
                device_ready = True
                break

            time.sleep(0.001)

        # A requested shutdown is not an ADC startup timeout.
        if stop_event.is_set():
            return

        if not device_ready:
            raise TimeoutError(
                f"SEC_FAIL remained active (STATUS=0x{status:04X}). "
                "Verify power, ground, RST, XEN, and SPI wiring."
            )

        print("ADC device ready; beginning background SPI polling.")
        print("Default hardware gain is 1 (nominal differential range ±1.2 V).")

        discard = STARTUP_DISCARD_SAMPLES
        calibration_windows = [
            deque(maxlen=BASELINE_SAMPLES),
            deque(maxlen=BASELINE_SAMPLES),
        ]
        baseline_rms = None
        anomaly_active = [False, False]
        last_print = 0.0

        prev_voltage = 0.0
        prev_crossing_time = None
        current_frequency = 60.0

        capturing_triggered = False
        capture_start_time = None
        temp_triggered_times = []
        temp_triggered_vals = [[], [], []]

        start_time = time.monotonic()

        while not stop_event.is_set():
            rx = null_frame()
            status = status_from_frame(rx)

            if status & STATUS_SEC_FAIL:
                raise RuntimeError(
                    f"ADC reported critical SEC_FAIL error: STATUS=0x{status:04X}"
                )

            if (status & STATUS_READY_MASK) != STATUS_READY_MASK:
                continue

            raw = channels_from_frame(rx)
            volts = [value * VOLTS_PER_CODE for value in raw]

            sample_time = time.monotonic() - start_time
            ch0_volts = volts[0]
            is_zero_crossing = prev_voltage < 0.0 and ch0_volts >= 0.0

            # Update frequency at every positive-going crossing.
            if is_zero_crossing:
                if prev_crossing_time is not None:
                    period = sample_time - prev_crossing_time
                    if 0.01 < period < 0.04:
                        measured_freq = 1.0 / period
                        current_frequency = (
                            0.8 * current_frequency + 0.2 * measured_freq
                        )

                prev_crossing_time = sample_time

            # Start a block only if one is not already being captured.
            # Crossings during an active capture are appended as ordinary samples.
            if is_zero_crossing and not capturing_triggered:
                capturing_triggered = True
                capture_start_time = sample_time
                temp_triggered_times = [0.0]
                temp_triggered_vals = [[volts[0]], [volts[1]], [volts[2]]]

            elif capturing_triggered:
                temp_triggered_times.append(sample_time - capture_start_time)
                for channel, value in enumerate(volts):
                    temp_triggered_vals[channel].append(value)

                if len(temp_triggered_times) >= TRIGGER_BUFFER_SIZE:
                    with data_lock:
                        shared_state["triggered_blocks"].append({
                            "times": list(temp_triggered_times),
                            "values": [
                                list(channel_values)
                                for channel_values in temp_triggered_vals
                            ],
                        })

                    capturing_triggered = False
                    capture_start_time = None
                    temp_triggered_times = []
                    temp_triggered_vals = [[], [], []]

            prev_voltage = ch0_volts

            # Store rolling history and crossings under the same lock.
            with data_lock:
                shared_state["times"].append(sample_time)
                shared_state["frequency"] = current_frequency

                if is_zero_crossing:
                    shared_state["crossings"].append(sample_time)

                for channel, value in enumerate(volts):
                    shared_state["values"][channel].append(value)

                while (
                    shared_state["times"]
                    and sample_time - shared_state["times"][0] > 1.0
                ):
                    shared_state["times"].popleft()
                    for channel_values in shared_state["values"]:
                        channel_values.popleft()

                while (
                    shared_state["crossings"]
                    and sample_time - shared_state["crossings"][0] > 1.0
                ):
                    shared_state["crossings"].popleft()

            # Discard startup samples, then calibrate the AC RMS baseline.
            if discard:
                discard -= 1

            elif baseline_rms is None:
                for channel in range(2):
                    calibration_windows[channel].append(volts[channel])

                if all(
                    len(window) == BASELINE_SAMPLES
                    for window in calibration_windows
                ):
                    baseline_rms = [
                        ac_rms_from_volts(list(window))
                        for window in calibration_windows
                    ]
                    print(
                        "AC RMS baseline calibrated: "
                        f"CH0={baseline_rms[0]:.4f}V, "
                        f"CH1={baseline_rms[1]:.4f}V"
                    )

            else:
                for channel in range(2):
                    window = calibration_windows[channel]
                    window.append(volts[channel])
                    measured_rms = ac_rms_from_volts(list(window))

                    if measured_rms is None or baseline_rms[channel] is None:
                        continue

                    deviation = measured_rms - baseline_rms[channel]
                    is_anomaly = abs(deviation) > ANOMALY_THRESHOLD_V

                    if is_anomaly and not anomaly_active[channel]:
                        report_anomaly(
                            f"CH{channel} AC RMS deviation exceeded threshold",
                            status=f"0x{status:04X}",
                            raw_frame=rx,
                            raw_codes=raw,
                            voltages=volts,
                            channel=channel,
                            measured_ac_rms_volts=measured_rms,
                            baseline_ac_rms_volts=baseline_rms[channel],
                            deviation_volts=deviation,
                            threshold_volts=ANOMALY_THRESHOLD_V,
                        )

                    anomaly_active[channel] = is_anomaly

                now = time.monotonic()
                if now - last_print >= PRINT_INTERVAL:
                    print(
                        f"Status=0x{status:04X}  Freq={current_frequency:.2f}Hz  "
                        f"CH0={volts[0]:+.4f}V  "
                        f"CH1={volts[1]:+.4f}V  "
                        f"CH2={volts[2]:+.4f}V"
                    )
                    last_print = now

    except Exception as exc:
        with data_lock:
            shared_state["worker_error"] = str(exc)

        print(
            f"Error occurred inside acquisition worker thread: {exc}",
            file=sys.stderr,
        )
        logging.exception("Acquisition worker encountered an unexpected exception")
        stop_event.set()

# ==============================================================================
# MAIN APPLICATION AND VISUALIZATION CONTROLLER
# ==============================================================================

def main() -> int:
    spi = spidev.SpiDev()
    reset = None
    xen = None
    plot_fig = None
    acquisition_thread = None

    stop_event = Event()
    data_lock = Lock()

    shared_state = {
        "times": deque(),
        "values": [deque(), deque(), deque()],
        "crossings": deque(),
        "triggered_blocks": deque(maxlen=MAX_OVERLAY_BLOCKS),
        "frequency": 60.0,
        "worker_error": None,
    }

    def request_shutdown(_signum, _frame):
        stop_event.set()

    previous_sigint_handler = signal.getsignal(signal.SIGINT)

    try:
        signal.signal(signal.SIGINT, request_shutdown)

        reset = DigitalOutputDevice(RST_BCM, initial_value=True)
        xen = DigitalOutputDevice(XEN_BCM, initial_value=False)

        spi.open(SPI_BUS, SPI_DEVICE)
        spi.mode = SPI_MODE
        spi.max_speed_hz = SPI_HZ
        spi.bits_per_word = 8

        # Power-on sequencing and hardware reset.
        time.sleep(0.05)
        xen.on()
        time.sleep(0.01)
        reset.off()
        time.sleep(0.001)
        reset.on()
        time.sleep(0.01)
        xen.off()
        time.sleep(0.005)

        # Configure ADC registers.
        spi.xfer2(wreg_frame(REG_CLOCK, CLOCK_1000_SPS))
        spi.xfer2(wreg_frame(REG_DCDC_CTRL, 0x0000))
        spi.xfer2(
            wreg_frame(REG_DCDC_CTRL, DCDC_FREQ_4P096_MHZ_AND_ENABLE)
        )

        xen.on()
        time.sleep(0.01)

        acquisition_thread = threading.Thread(
            target=adc_acquisition_worker,
            args=(spi, stop_event, data_lock, shared_state),
            daemon=False,
        )
        acquisition_thread.start()

        plt.ion()
        plot_fig, plot_ax = plt.subplots()
        plot_ax.set_xlabel("Time from Trigger (s)")
        plot_ax.set_ylabel("Voltage (V)")
        plot_ax.set_title("AMC131M03 Zero-Crossing Overlay Persistence View")
        plot_ax.grid(True)

        overlay_lines = [[], [], []]
        colors = ["tab:blue", "tab:orange", "tab:green"]

        for channel in range(3):
            for index in range(MAX_OVERLAY_BLOCKS):
                alpha = 0.15 + 0.85 * (
                    index / max(1, MAX_OVERLAY_BLOCKS - 1)
                )
                label = (
                    f"CH{channel}"
                    if index == MAX_OVERLAY_BLOCKS - 1
                    else None
                )
                line, = plot_ax.plot(
                    [],
                    [],
                    color=colors[channel],
                    alpha=alpha,
                    linewidth=1.2,
                    label=label,
                )
                overlay_lines[channel].append(line)

        plot_ax.legend(loc="upper right")
        plot_ax.set_xlim(0.0, 0.10)
        plot_ax.set_ylim(-1.3, 1.3)

        rms_text = plot_ax.text(
            0.98,
            0.68,
            "RMS Values: Collecting Sample Data...",
            transform=plot_ax.transAxes,
            ha="right",
            va="top",
            fontsize=8,
            bbox={
                "facecolor": "white",
                "alpha": 0.8,
                "edgecolor": "none",
            },
        )

        plot_fig.tight_layout()
        plot_fig.canvas.mpl_connect(
            "close_event",
            lambda _event: stop_event.set(),
        )

        def stop_from_plot_key(event):
            if event.key and event.key.lower() == "ctrl+c":
                print("Stopping execution.")
                stop_event.set()

        plot_fig.canvas.mpl_connect("key_press_event", stop_from_plot_key)
        plt.show(block=False)

        last_plot_update = time.monotonic()

        while not stop_event.is_set():
            now = time.monotonic()

            if now - last_plot_update >= PLOT_REFRESH_SECONDS:
                with data_lock:
                    local_blocks = list(shared_state["triggered_blocks"])
                    local_times = list(shared_state["times"])
                    local_values = [
                        list(channel_values)
                        for channel_values in shared_state["values"]
                    ]
                    local_crossings = list(shared_state["crossings"])
                    local_freq = shared_state["frequency"]

                if local_blocks:
                    for index in range(MAX_OVERLAY_BLOCKS):
                        for channel in range(3):
                            line = overlay_lines[channel][index]
                            if index < len(local_blocks):
                                block = local_blocks[index]
                                line.set_data(
                                    block["times"],
                                    block["values"][channel],
                                )
                            else:
                                line.set_data([], [])

                    newest_times = local_blocks[-1]["times"]
                    if newest_times:
                        plot_ax.set_xlim(0.0, newest_times[-1])

                rms_rows = [
                    f"Grid Frequency: {local_freq:.2f} Hz "
                    f"(Persistence Mode)"
                ]

                for channel, values in enumerate(local_values):
                    paired_samples = list(zip(local_times, values))
                    ac_rms, total_rms = synchronized_rms_from_volts(
                        paired_samples,
                        local_crossings,
                    )

                    if ac_rms is not None:
                        rms_rows.append(
                            f"CH{channel}: AC {ac_rms:.4f}V | "
                            f"Total {total_rms:.4f}V"
                        )
                    else:
                        rms_rows.append(f"CH{channel}: Collecting...")

                rms_text.set_text("\n".join(rms_rows))
                plot_fig.canvas.draw_idle()
                plot_fig.canvas.flush_events()
                last_plot_update = now

            time.sleep(0.01)

        # Wait for the worker before reading its result or closing SPI.
        if acquisition_thread is not None:
            acquisition_thread.join()

        with data_lock:
            worker_error = shared_state["worker_error"]

        if worker_error:
            print(f"Acquisition failed: {worker_error}", file=sys.stderr)
            return 1

        print("Execution stopped cleanly.")
        return 0

    except KeyboardInterrupt:
        print("Execution interrupted by user.")
        return 130

    except Exception as exc:
        print(f"Uncaught error encountered: {exc}", file=sys.stderr)
        return 1

    finally:
        # Stop and join the worker before closing SPI or GPIO resources.
        stop_event.set()
        if acquisition_thread is not None and acquisition_thread.is_alive():
            acquisition_thread.join()

        if reset is not None:
            try:
                reset.off()
                time.sleep(0.001)
                reset.on()
                time.sleep(0.01)
            except Exception:
                logging.exception(
                    "Failed to toggle ADC reset line during shutdown"
                )

        if xen is not None:
            try:
                xen.off()
            except Exception:
                logging.exception(
                    "Failed to disable external ADC clock during shutdown"
                )

        try:
            spi.close()
        except Exception:
            logging.exception("Failed to close SPI bus interface during shutdown")

        if xen is not None:
            try:
                xen.close()
            except Exception:
                logging.exception("Failed to close clock GPIO line during shutdown")

        if reset is not None:
            try:
                reset.close()
            except Exception:
                logging.exception("Failed to close reset GPIO line during shutdown")

        if plot_fig is not None:
            try:
                plt.close(plot_fig)
            except Exception:
                logging.exception("Failed to close Matplotlib figure")

        signal.signal(signal.SIGINT, previous_sigint_handler)

if __name__ == "__main__":
    raise SystemExit(main())
