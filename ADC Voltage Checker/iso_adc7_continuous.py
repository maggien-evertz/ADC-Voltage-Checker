#!/usr/bin/env python3
"""
Read ISO ADC 7 / AMC131M03 voltage data on Raspberry Pi SPI0.
Multi-threaded architecture: background thread handles high-speed SPI polling,
calibration, and anomaly detection; main thread manages live Matplotlib visualization.
Optimized with a short time window to display clean AC sine waves.
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
# set up bcm pin numbers for raspberry pi gpio and spi bus addressing:
# rst and xen use physical pins 18 and 22 respectively.
SPI_BUS = 0
SPI_DEVICE = 0  # gpio8, chip select physical pin 24
RST_BCM = 24    # gpio 24, physical pin 18 (active-low reset line)
XEN_BCM = 25    # gpio 25, physical pin 22 (external oscillator clock enable)

# configure spi communication settings: defaults to 24-bit words, spi mode 1, and input crc disabled.
SPI_MODE = 0b01
SPI_HZ = 500_000
FRAME_WORDS = 5         # each adc frame contains 5 words; each word is 24 bits (3 bytes)
FRAME_BYTES = FRAME_WORDS * 3  # total frame size is 15 bytes

# define register addresses and configuration bitmasks for 1000 sps sampling rate
REG_CLOCK = 0x03                         # register selecting clock frequency and data rate settings
CLOCK_1000_SPS = 0x0716                  # enables 3 channels, high-res mode, div-by-2 clock, and osr=4096 for 1000 sps
REG_DCDC_CTRL = 0x31                     # register controlling dcdc converter frequency and enable state
DCDC_FREQ_4P096_MHZ_AND_ENABLE = 0x0001  # sets dcdc frequency range and enables internal power conversion

# define status bitmasks from the amc131m03 datasheet for error and data readiness checking
STATUS_SEC_FAIL = 1 << 6         # bit 6 indicates if the isolated-side supply voltage is invalid
STATUS_READY_MASK = 0b111        # bits 0-2 indicate data conversion readiness for ch0, ch1, and ch2

# voltage conversion scale factor: gain=1 gives a differential full-scale range of +/-1.2 v across 2^23 resolution steps.
VOLTS_PER_CODE = 1.2 / (1 << 23) 
STARTUP_DISCARD_SAMPLES = 10
PRINT_INTERVAL = 0.5
ANOMALY_THRESHOLD_V = 0.10
BASELINE_SAMPLES = 100

# configuration settings for matplotlib live waveform animation
# set to 0.2 seconds to focus the view on roughly 10 cycles of a 50/60 hz ac waveform.
PLOT_WINDOW_SECONDS = 0.2
PLOT_REFRESH_SECONDS = 0.03  # fast refresh interval for smooth graphical updates

# configure persistent logging to write json-formatted anomaly events to a local text file
LOG_FILE = Path(__file__).resolve().with_name("log.txt")

logging.basicConfig(
    filename=LOG_FILE,
    filemode="a",
    level=logging.WARNING,
    format="%(asctime)s %(levelname)s %(message)s",
)


# ==============================================================================
# MATHEMATICAL CALCULATIONS AND UTILITY HELPER FUNCTIONS
# ==============================================================================

def rms_from_volts(samples):
    """
    calculates ac rms and total rms voltage from a list of collected samples.
    ac rms removes the dc offset (mean), whereas total rms includes the dc component.
    """
    n = len(samples)
    if n < 2:
        return None, None

    mean = sum(samples) / n
    ac_rms = math.sqrt(sum((v - mean) ** 2 for v in samples) / n)
    total_rms = math.sqrt(sum(v ** 2 for v in samples) / n)
    return ac_rms, total_rms


def report_anomaly(reason, **details):
    """
    constructs a structured dictionary record when a voltage deviation or hardware error occurs,
    prints it to standard output, and appends it to the warning log file as a json string.
    """
    record = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "anomaly": reason,
        **details,
    }
    message = json.dumps(record, default=str)
    print(f"ANOMALY: {message}", flush=True)
    logging.warning(message)
    

def word24(word16: int) -> list[int]:
    """
    packs a 16-bit command or register configuration word into a 24-bit spi data word format 
    (most significant byte first), appending a trailing zero padding byte required by the adc.
    """
    return [(word16 >> 8) & 0xFF, word16 & 0xFF, 0]     


def command_frame(command: int) -> list[int]:
    """builds a standard 5-word spi transmission frame containing the command in word one and zero padding."""
    return word24(command) + [0] * (FRAME_BYTES - 3)


def wreg_frame(address: int, value: int) -> list[int]:
    """
    encodes a single-register write (wreg) command frame:
    wreg opcode uses bits 011 followed by a 6-bit register address and 7-bit count-minus-one,
    followed by the target value word and zero padding to complete the 5-word frame structure.
    """
    command = 0x6000 | ((address & 0x3F) << 7)
    return word24(command) + word24(value) + [0] * (FRAME_BYTES - 6)


def signed24(data: list[int]) -> int:
    """
    combines three raw incoming bytes into a signed 24-bit integer value (msb first).
    evaluates two's complement format: if the sign bit is active, subtracts 2^24 to yield a negative number.
    """
    value = (data[0] << 16) | (data[1] << 8) | data[2]
    return value - (1 << 24) if value & 0x800000 else value


def status_from_frame(rx: list[int]) -> int:
    """extracts and combines the first two bytes of a received spi frame into a 16-bit status word."""
    return (rx[0] << 8) | rx[1]


def channels_from_frame(rx: list[int]) -> list[int]:
    """extracts three separate 24-bit channel readings from the receive buffer and converts them to signed integers."""
    return [signed24(rx[3:6]), signed24(rx[6:9]), signed24(rx[9:12])]


# ==============================================================================
# BACKGROUND ACQUISITION WORKER THREAD
# ==============================================================================

def adc_acquisition_worker(spi, stop_event, data_lock, shared_state):
    """
    runs independently in a background thread to ensure continuous, high-speed spi data collection
    without getting blocked by ui rendering delays. handles register flushing, power checks, baseline calibration, and monitoring.
    """
    try:
        # local helper function to send a null command frame and fetch the raw response bytes from the spi bus
        def null_frame() -> list[int]:
            return spi.xfer2(command_frame(0x0000))

        # flush any stale data pipelines remaining from previous register configuration commands
        null_frame()
        null_frame()

        # poll the status register until the isolated power supply flag (sec_fail) clears or times out after 5 seconds
        deadline = time.monotonic() + 5.0
        status = 0
        while time.monotonic() < deadline and not stop_event.is_set():
            rx = null_frame()
            status = status_from_frame(rx)
            if ((status >> 8) & 0b11) != 0b01:
                raise RuntimeError(
                    f"Unexpected STATUS 0x{status:04X}; Check SPI wiring and communication mode."
                )
            if not (status & STATUS_SEC_FAIL):
                break
            time.sleep(0.001)
        else:
            raise TimeoutError(
                f"SEC_FAIL error flag remained active (STATUS=0x{status:04X}). "
                "Verify 3.3V power, ground, RST, XEN, and SPI wiring connections."
            )

        print("ADC device ready; Beginning background SPI status and data polling loop.")
        print("Default hardware gain is set to 1 (Nominal differential voltage range of +/-1.2V).")

        discard = STARTUP_DISCARD_SAMPLES
        calibration_sums = [0.0, 0.0]
        calibration_count = 0
        baseline = None
        anomaly_active = [False, False]
        last_print = 0.0
        
        start_time = time.monotonic()

        # continuous polling loop executing in the background thread
        while not stop_event.is_set():
            rx = null_frame()
            status = status_from_frame(rx)
            
            # verify that the isolated power failure bit has not triggered during runtime operation
            if status & STATUS_SEC_FAIL:
                raise RuntimeError(f"ADC reported critical SEC_FAIL error: STATUS=0x{status:04X}")

            # parse voltage samples only when all channels indicate conversion readiness via the status mask
            if (status & STATUS_READY_MASK) == STATUS_READY_MASK:
                raw = channels_from_frame(rx)
                volts = [value * VOLTS_PER_CODE for value in raw]
                
                sample_time = time.monotonic() - start_time

                # safely acquire thread lock to append new samples into the shared data structures for the ui thread
                with data_lock:
                    shared_state["times"].append(sample_time)
                    for channel, val in enumerate(volts):
                        shared_state["values"][channel].append(val)

                    # trim rolling queues to keep only data points within the active plot window length
                    while shared_state["times"] and sample_time - shared_state["times"][0] > PLOT_WINDOW_SECONDS:
                        shared_state["times"].popleft()
                        for channel_values in shared_state["values"]:
                            channel_values.popleft()

                # discard initial samples to let the analog input circuitry and filters stabilize after startup
                if discard:
                    discard -= 1

                # accumulate samples over the calibration period to establish a steady-state reference baseline
                elif baseline is None:
                    for channel in range(2):
                        calibration_sums[channel] += volts[channel]
                    calibration_count += 1

                    if calibration_count == BASELINE_SAMPLES:
                        baseline = [
                            calibration_sums[0] / calibration_count,
                            calibration_sums[1] / calibration_count,
                        ]
                        print(
                            f"Baseline calibration completed: CH0={baseline[0]:+.4f}V, "
                            f"CH1={baseline[1]:+.4f}V"
                        )

                # evaluate incoming voltage readings against the established baseline to detect anomalies
                else:
                    for channel in range(2):
                        deviation = volts[channel] - baseline[channel]
                        is_anomaly = abs(deviation) > ANOMALY_THRESHOLD_V

                        if is_anomaly and not anomaly_active[channel]:
                            report_anomaly(
                                f"CH{channel} voltage deviation exceeded limit threshold",
                                status=f"0x{status:04X}",
                                raw_frame=rx,
                                raw_codes=raw,
                                voltages=volts,
                                channel=channel,
                                reading_volts=volts[channel],
                                baseline_volts=baseline[channel],
                                deviation_volts=deviation,
                                threshold_volts=ANOMALY_THRESHOLD_V,
                            )

                        anomaly_active[channel] = is_anomaly

                    # throttle console text updates to maintain readable output frequencies
                    now = time.monotonic()
                    if now - last_print >= PRINT_INTERVAL:
                        print(
                            f"Status=0x{status:04X}  "
                            f"CH0={volts[0]:+.4f}V  "
                            f"CH1={volts[1]:+.4f}V  "
                            f"CH2={volts[2]:+.4f}V"
                        )
                        last_print = now

    except Exception as exc:
        print(f"Error occurred inside acquisition worker thread: {exc}", file=sys.stderr)
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
    stop_event = Event()
    data_lock = Lock()

    # define shared state dictionary containing rolling deques accessed safely across threads
    shared_state = {
        "times": deque(),
        "values": [deque(), deque(), deque()]
    }

    def request_shutdown(_signum, _frame):
        """signal handler intercepting interrupt signals (ctrl+c) to trigger clean application exit."""
        stop_event.set()

    previous_sigint_handler = signal.getsignal(signal.SIGINT)

    try:
        signal.signal(signal.SIGINT, request_shutdown)

        # initialize physical control pins for reset and clock enable via gpiozero library
        reset = DigitalOutputDevice(RST_BCM, initial_value=True)
        xen = DigitalOutputDevice(XEN_BCM, initial_value=False)

        # open spi bus device interface and assign operating parameters
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.mode = SPI_MODE
        spi.max_speed_hz = SPI_HZ
        spi.bits_per_word = 8

        # execute power-on sequencing and hardware reset pulse routine for the adc chip
        time.sleep(0.05)  # allow board power rails to stabilize completely
        xen.on()
        time.sleep(0.01)  # allow the external 8.192 mhz oscillator module to start up
        reset.off()       # trigger active-low hardware reset signal pulse
        time.sleep(0.001) # maintain reset state for required clock cycles (>2048 clkin cycles)
        reset.on()
        time.sleep(0.01)  # allow internal registers to reload configuration defaults
        xen.off()
        time.sleep(0.005)

        # write configuration settings to adc registers for 1000 sps and internal dcdc clock operation
        spi.xfer2(wreg_frame(REG_CLOCK, CLOCK_1000_SPS))
        spi.xfer2(wreg_frame(REG_DCDC_CTRL, 0x0000))
        spi.xfer2(wreg_frame(REG_DCDC_CTRL, DCDC_FREQ_4P096_MHZ_AND_ENABLE))

        xen.on()  # activate the click board's 8.192 mhz clock source output
        time.sleep(0.01)

        # initialize and launch the background spi data acquisition thread as a daemon process
        acquisition_thread = threading.Thread(
            target=adc_acquisition_worker,
            args=(spi, stop_event, data_lock, shared_state),
            daemon=True,
        )
        acquisition_thread.start()

        # set up matplotlib interactive user interface window for real-time waveform plotting in the main thread
        plt.ion()
        plot_fig, plot_ax = plt.subplots()
        plot_lines = [
            plot_ax.plot([], [], label=f"CH{i}")[0]
            for i in range(3)
        ]
        plot_ax.set_xlabel("Time (s)")
        plot_ax.set_ylabel("Voltage (V)")
        plot_ax.set_title("AMC131M03 Live AC Sine Wave Monitor")
        plot_ax.grid(True)
        plot_ax.legend(
            loc="upper right",
            bbox_to_anchor=(0.98, 0.50),
        )
        rms_text = plot_ax.text(
            0.98, 0.68, "RMS Values: Collecting Sample Data...",
            transform=plot_ax.transAxes,
            ha="right", va="top",
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )

        plot_fig.tight_layout()
        plot_fig.canvas.mpl_connect(
            "close_event",
            lambda event: stop_event.set()
            if event.canvas is plot_fig.canvas
            else None,
        )

        def stop_from_plot_key(event):
            if event.key and event.key.lower() == "ctrl+c":
                print("Stopping execution.")
                stop_event.set()

        plot_fig.canvas.mpl_connect("key_press_event", stop_from_plot_key)
        plt.show(block=False)

        last_plot_update = time.monotonic()

        # main thread graphical user interface event loop
        while not stop_event.is_set():
            now = time.monotonic()
            if now - last_plot_update >= PLOT_REFRESH_SECONDS:
                # copy data from shared thread-safe buffers under lock protection to prevent race conditions
                with data_lock:
                    local_times = list(shared_state["times"])
                    local_values = [list(ch) for ch in shared_state["values"]]

                if local_times:
                    for line, channel_values in zip(plot_lines, local_values):
                        line.set_data(local_times, channel_values)

                    # update axis boundaries to maintain the short rolling time window for clean sine waves
                    right = max(PLOT_WINDOW_SECONDS, local_times[-1])
                    plot_ax.set_xlim(right - PLOT_WINDOW_SECONDS, right)
                    plot_ax.relim()
                    plot_ax.autoscale_view(scalex=False, scaley=True)

                    rms_rows = []
                    for channel, samples in enumerate(local_values):
                        ac_rms, total_rms = rms_from_volts(samples)
                        if ac_rms is not None:
                            rms_rows.append(
                                f"CH{channel}: AC {ac_rms:.4f}V | Total {total_rms:.4f}V"
                            )
                        else:
                            rms_rows.append(f"CH{channel}: Collecting...")

                    rms_text.set_text(
                        f"AC Input RMS (Rolling {PLOT_WINDOW_SECONDS}s Window)\n"
                        + "\n".join(rms_rows)
                    )

                    plot_fig.canvas.draw_idle()
                    plot_fig.canvas.flush_events()
                
                last_plot_update = now

            # short sleep to yield processing time within the gui event loop
            time.sleep(0.01)

        # wait for the background worker thread to finish execution upon shutdown signal
        acquisition_thread.join(timeout=1.0)
        print("Execution stopped cleanly.")
        return 0
            
    except KeyboardInterrupt:
        print("Execution interrupted by user.")
        return 130
        
    except Exception as exc:
        print(f"Uncaught error encountered: {exc}", file=sys.stderr)
        return 1
        
    finally:
        # comprehensive cleanup block to safely release hardware pins, close interfaces, and destroy plots
        if reset is not None:
            try:
                reset.off()
                time.sleep(0.001)
                reset.on()
                time.sleep(0.01)
            except Exception:
                logging.exception("Failed to toggle ADC reset line during application shutdown")

        if xen is not None:
            try:
                xen.off()
            except Exception:
                logging.exception("Failed to disable external ADC clock during shutdown")

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
                logging.exception("Failed to close Matplotlib waveform figure window")

        signal.signal(signal.SIGINT, previous_sigint_handler)


if __name__ == "__main__":
    raise SystemExit(main())