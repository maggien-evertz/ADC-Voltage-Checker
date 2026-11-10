#!/usr/bin/env python3
"""
Read ISO ADC 7 / AMC131M03 voltage data on Raspberry Pi SPI0.

A background thread polls SPI, tracks positive-going zero crossings, captures
100-sample waveform blocks, calibrates AC RMS baselines, and detects RMS
deviations. The main thread displays the rolling average of recent captures
and serves a live web dashboard featuring metrics and an embedded waveform plot.
"""

from collections import deque
from datetime import datetime
import io
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
from flask import Flask, jsonify, render_template_string, Response
from gpiozero import DigitalOutputDevice
import spidev

# ==============================================================================
# hardware and spi configuration parameters:
# this block defines the raspberry pi pin mapping, spi settings, adc register map,
# and timing values used to initialize the isolated adc acquisition path.
# ==============================================================================

# use spi bus 0 and device 0 as the adc interface on the raspberry pi.
SPI_BUS = 0
SPI_DEVICE = 0
# the reset and xen lines are connected to the adc control pins on the board.
RST_BCM = 24
XEN_BCM = 25

# set the adc spi mode to match the datasheet timing for the isolated converter.
SPI_MODE = 0b01
# keep the bus speed low enough for reliable communication with the adc frontend.
SPI_HZ = 500_000
# each adc frame contains five 24-bit words and therefore 15 total payload bytes.
FRAME_WORDS = 5
FRAME_BYTES = FRAME_WORDS * 3

# select the adc clock and power-control registers used during configuration.
REG_CLOCK = 0x03
CLOCK_600_SPS = 0x0716
REG_DCDC_CTRL = 0x31
DCDC_FREQ_4P096_MHZ_AND_ENABLE = 0x0001

# track supply fault and conversion-ready flags from the status register.
STATUS_SEC_FAIL = 1 << 6
STATUS_READY_MASK = 0b111

# convert the raw adc code to a measured differential voltage for each channel.
VOLTS_PER_CODE = 1.2 / (1 << 23)
# nominal sample rate used by the waveform and rms calculations.
SAMPLING_RATE_HZ = 600.0  # Nominal; verify against the ADC configuration.
# keep a rolling capture window and average across the last several trigger blocks.
TRIGGER_BUFFER_SIZE = 100
AVERAGE_BLOCK_COUNT = 10
# ignore the initial adc samples until startup settles.
STARTUP_DISCARD_SAMPLES = 10
PRINT_INTERVAL = 0.5
# raise an anomaly when the measured ac rms drifts outside the calibrated baseline.
ANOMALY_THRESHOLD_V = 0.10  # Difference in AC RMS from calibrated baseline.
# accumulate enough samples to estimate the baseline and detect slow drift.
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
# mathematical calculations and utility functions:
# these helpers convert raw adc values to volts, compute rms values, and package
# anomaly data so the live plotting loop can stay focused on waveform updates.
# ==============================================================================

def ac_rms_from_volts(samples):
    """Return mean-removed AC RMS for a sequence of voltage samples."""
    if len(samples) < 2:
        return None

    mean = sum(samples) / len(samples)
    return math.sqrt(
        sum((value - mean) ** 2 for value in samples) / len(samples)
    )

def synchronized_rms_from_volts(
    samples,
    crossing_times,
    target_cycles=RMS_CYCLES,
):
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
    """Print and log a structured EN 50160 power quality event record."""
    record = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "event": reason,
        **details,
    }
    message = json.dumps(record, default=str)
    print(f"POWER QUALITY EVENT: {message}", flush=True)
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

# background acquisition worker thread:
# this thread continuously reads the adc, watches for zero crossings, and maintains
# the rolling waveform history used for the averaged display and rms tracking.
# ==============================================================================

def adc_acquisition_worker(spi, stop_event, data_lock, shared_state):
    """Poll the ADC and update capture, calibration, and monitoring state."""
    try:
        def null_frame() -> list[int]:
            return spi.xfer2(command_frame(0x0000))

        null_frame()
        null_frame()

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
        event_active = [None, None]
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

            if is_zero_crossing:
                if prev_crossing_time is not None:
                    period = sample_time - prev_crossing_time
                    if 0.01 < period < 0.04:
                        measured_freq = 1.0 / period
                        current_frequency = (
                            0.8 * current_frequency + 0.2 * measured_freq
                        )

                prev_crossing_time = sample_time

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
                        shared_state["current_block"] = None

                    capturing_triggered = False
                    capture_start_time = None
                    temp_triggered_times = []
                    temp_triggered_vals = [[], [], []]

            prev_voltage = ch0_volts
            sample_wall_time = time.monotonic()

            with data_lock:
                shared_state["times"].append(sample_time)
                shared_state["frequency"] = current_frequency
                shared_state["last_sample_monotonic"] = sample_wall_time

                if is_zero_crossing:
                    shared_state["crossings"].append(sample_time)
                    shared_state["last_crossing_monotonic"] = sample_wall_time

                if capturing_triggered:
                    shared_state["current_block"] = {
                        "times": list(temp_triggered_times),
                        "values": [
                            list(channel_values)
                            for channel_values in temp_triggered_vals
                        ],
                    }

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

                    v_nom = baseline_rms[channel]
                    ratio = measured_rms / v_nom if v_nom > 0 else 1.0

                    current_event = None
                    if ratio < 0.10:
                        current_event = "INTERRUPTION"
                    elif ratio < 0.90:
                        current_event = "VOLT_SAG"
                    elif ratio > 1.10:
                        current_event = "VOLT_SWELL"

                    if current_event != event_active[channel]:
                        if current_event is not None:
                            report_anomaly(
                                f"CH{channel} {current_event} detected",
                                channel=channel,
                                measured_ac_rms=measured_rms,
                                nominal_rms=v_nom,
                                ratio=ratio,
                            )
                        event_active[channel] = current_event

                # Update live metrics for web dashboard consumption
                with data_lock:
                    shared_state["latest_metrics"] = {
                        "frequency": current_frequency,
                        "ch0_rms": ac_rms_from_volts(list(calibration_windows[0])) or 0.0,
                        "ch1_rms": ac_rms_from_volts(list(calibration_windows[1])) or 0.0,
                    }

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


# ==============================================================================
# LIGHTWEIGHT FLASK WEB DASHBOARD & PLOT STREAMING SERVER
# ==============================================================================

dashboard_app = Flask(__name__)
web_shared_state_ref = None

DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>ISO ADC 7 Power Quality Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; background: #f0f2f5; margin: 0; padding: 20px; color: #333; }
        .container { max-width: 900px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        h1 { color: #1d4ed8; font-size: 24px; border-bottom: 2px solid #e5e7eb; padding-bottom: 10px; margin-top: 0; }
        .metrics-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; margin-bottom: 20px; }
        .metric-card { background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 6px; padding: 15px; text-align: center; }
        .metric-title { font-size: 13px; color: #64748b; font-weight: bold; text-transform: uppercase; }
        .metric-value { font-size: 22px; color: #0f172a; font-weight: bold; margin-top: 5px; }
        .plot-container { text-align: center; background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 6px; padding: 15px; }
        .plot-container img { width: 100%; max-width: 800px; height: auto; border-radius: 4px; border: 1px solid #94a3b8; }
    </style>
</head>
<body>
    <div class="container">
        <h1>AMC131M03 Power Quality Dashboard</h1>
        <div class="metrics-grid">
            <div class="metric-card">
                <div class="metric-title">Grid Frequency</div>
                <div class="metric-value" id="freq-val">-- Hz</div>
            </div>
            <div class="metric-card">
                <div class="metric-title">Channel 0 AC RMS</div>
                <div class="metric-value" id="ch0-val">-- V</div>
            </div>
            <div class="metric-card">
                <div class="metric-title">Channel 1 AC RMS</div>
                <div class="metric-value" id="ch1-val">-- V</div>
            </div>
        </div>
        <div class="plot-container">
            <div class="metric-title" style="margin-bottom: 10px;">Live Averaged Waveform Monitor</div>
            <img id="waveform-img" src="/plot.png" alt="Live Waveform">
        </div>
    </div>
    <script>
        // Refresh metrics dynamically every second via API
        function updateMetrics() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    if (data.frequency !== undefined) {
                        document.getElementById('freq-val').innerText = data.frequency.toFixed(2) + ' Hz';
                        document.getElementById('ch0-val').innerText = data.ch0_rms.toFixed(4) + ' V';
                        document.getElementById('ch1-val').innerText = data.ch1_rms.toFixed(4) + ' V';
                    }
                }).catch(err => console.error(err));
        }
        setInterval(updateMetrics, 1000);

        // Refresh waveform plot image smoothly every 150ms
        setInterval(function() {
            var img = document.getElementById('waveform-img');
            img.src = '/plot.png?t=' + new Date().getTime();
        }, 150);
    </script>
</body>
</html>
"""

@dashboard_app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)

@dashboard_app.route("/api/status")
def api_status():
    global web_shared_state_ref
    if web_shared_state_ref:
        with web_shared_state_ref["lock"]:
            return jsonify(web_shared_state_ref["state"].get("latest_metrics", {}))
    return jsonify({"error": "Initializing"})

@dashboard_app.route("/plot.png")
def plot_png():
    global web_shared_state_ref
    if web_shared_state_ref:
        with web_shared_state_ref["lock"]:
            img_bytes = web_shared_state_ref["state"].get("plot_image_bytes")
            if img_bytes:
                return Response(img_bytes, mimetype="image/png")
    return "Generating...", 200

def run_web_server(shared_state, data_lock):
    global web_shared_state_ref
    web_shared_state_ref = {"state": shared_state, "lock": data_lock}
    dashboard_app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)


# ==============================================================================
# MAIN CONTROLLER
# ==============================================================================

def main() -> int:
    spi = spidev.SpiDev()
    reset = None
    xen = None
    plot_fig = None
    acquisition_thread = None
    web_thread = None

    stop_event = Event()
    data_lock = Lock()

    shared_state = {
        "times": deque(),
        "values": [deque(), deque(), deque()],
        "crossings": deque(),
        "triggered_blocks": deque(maxlen=AVERAGE_BLOCK_COUNT),
        "current_block": None,
        "last_sample_monotonic": None,
        "last_crossing_monotonic": None,
        "frequency": 60.0,
        "latest_metrics": {},
        "plot_image_bytes": None,
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

        time.sleep(0.05)
        xen.on()
        time.sleep(0.01)
        reset.off()
        time.sleep(0.001)
        reset.on()
        time.sleep(0.01)
        xen.off()
        time.sleep(0.005)

        spi.xfer2(wreg_frame(REG_CLOCK, CLOCK_600_SPS))
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

        # Start Flask web dashboard server in background thread
        web_thread = threading.Thread(
            target=run_web_server,
            args=(shared_state, data_lock),
            daemon=True,
        )
        web_thread.start()
        print("Web dashboard server started at http://172.17.223.251:5000")

        plt.ion()
        plot_fig, plot_ax = plt.subplots()
        plot_ax.set_xlabel("Time from Trigger (s)")
        plot_ax.set_ylabel("Voltage (V)")
        plot_ax.set_title("AMC131M03 Averaged Zero-Crossing Waveform")
        plot_ax.grid(True, color="#dfe6ee", linewidth=0.8)
        plot_ax.set_axisbelow(True)

        average_lines = []
        colors = ["tab:blue", "tab:orange", "tab:green"]

        for channel, color in enumerate(colors):
            line, = plot_ax.plot(
                [],
                [],
                color=color,
                linewidth=1.5,
                label=f"CH{channel}",
            )
            average_lines.append(line)

        channel_legend = plot_ax.legend(
            handles=average_lines,
            loc="lower right",
            bbox_to_anchor=(1.0, 0.0),
            ncol=1,
            frameon=True,
            fancybox=True,
            shadow=True,
            facecolor="white",
            edgecolor="#ccd5df",
            fontsize=9,
            title="Channels",
            title_fontsize=9,
            handlelength=1.8,
            handletextpad=0.5,
            columnspacing=1.2,
            borderpad=0.5,
        )
        plot_ax.add_artist(channel_legend)
        plot_ax.set_xlim(0.0, 0.10)
        plot_ax.set_ylim(-1.3, 1.3)

        rms_text = plot_ax.text(
            0.5,
            0.02,
            "Waiting for ADC samples...",
            transform=plot_ax.transAxes,
            ha="center",
            va="bottom",
            fontsize=8.5,
            linespacing=1.3,
            bbox={
                "boxstyle": "round,pad=0.45",
                "facecolor": "#f8fafc",
                "edgecolor": "#cbd5e1",
                "linewidth": 1.0,
                "alpha": 0.96,
            },
        )

        plot_fig.subplots_adjust(top=0.85, right=0.88, bottom=0.18)
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
                    local_current_block = shared_state["current_block"]

                    local_times = list(shared_state["times"])
                    local_values = [
                        list(channel_values)
                        for channel_values in shared_state["values"]
                    ]
                    local_crossings = list(shared_state["crossings"])
                    local_freq = shared_state["frequency"]

                    local_last_sample = shared_state["last_sample_monotonic"]
                    local_last_crossing = shared_state["last_crossing_monotonic"]
                    local_worker_error = shared_state["worker_error"]

                blocks_to_average = list(local_blocks)
                if local_current_block and local_current_block["times"]:
                    blocks_to_average.append(local_current_block)

                if blocks_to_average:
                    sample_count = max(
                        len(block["times"]) for block in blocks_to_average
                    )
                    average_times = []
                    average_values = [[], [], []]

                    for sample_index in range(sample_count):
                        contributors = [
                            block for block in blocks_to_average
                            if sample_index < len(block["times"])
                        ]

                        average_times.append(
                            sum(
                                block["times"][sample_index]
                                for block in contributors
                            ) / len(contributors)
                        )

                        for channel in range(3):
                            average_values[channel].append(
                                sum(
                                    block["values"][channel][sample_index]
                                    for block in contributors
                                ) / len(contributors)
                            )

                    for channel in range(3):
                        average_lines[channel].set_data(
                            average_times,
                            average_values[channel],
                        )

                    plot_ax.set_xlim(
                        0.0,
                        max(average_times[-1], 0.01),
                    )

                if local_worker_error:
                    input_status = (
                        f"Acquisition stopped: {local_worker_error}; "
                        "holding last average"
                    )
                elif local_last_sample is None:
                    input_status = "Waiting for ADC samples"
                elif now - local_last_sample > 1.0:
                    input_status = (
                        f"No ADC samples for {now - local_last_sample:.1f}s; "
                        "holding last average"
                    )
                elif (
                    local_last_crossing is None
                    or now - local_last_crossing > 0.2
                ):
                    input_status = "No recent zero crossing; holding last average"
                else:
                    input_status = "Live"

                rms_rows = [
                    f"Grid Frequency: {local_freq:.2f} Hz",
                    f"Input: {input_status}",
                    f"Web Dashboard: http://<pi-ip>:5000",
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
                        rms_rows.append(
                            f"CH{channel}: Waiting for a complete "
                            f"{RMS_CYCLES}-cycle window"
                        )

                rms_text.set_text("\n".join(rms_rows))
                plot_fig.canvas.draw_idle()
                plot_fig.canvas.flush_events()

                # Render the updated plot figure into in-memory bytes for the web dashboard
                buf = io.BytesIO()
                plot_fig.savefig(buf, format="png", bbox_inches="tight", dpi=110)
                buf.seek(0)
                with data_lock:
                    shared_state["plot_image_bytes"] = buf.getvalue()

                last_plot_update = now

            plt.pause(0.01)

        if acquisition_thread is not None:
            acquisition_thread.join()

        print("Execution stopped cleanly.")
        return 0

    except KeyboardInterrupt:
        print("Execution interrupted by user.")
        return 130

    except Exception as exc:
        print(f"Uncaught error encountered: {exc}", file=sys.stderr)
        return 1

    finally:
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