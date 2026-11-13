#!/usr/bin/env python3
# command to run dashboard in ssh terminal: python3 iso_adc_withVoltageDis_webview.py

"""
Read ISO ADC 7 / AMC131M03 voltage data on Raspberry Pi SPI0.

background thread polls SPI, tracks positive-going zero crossings, captures
100-sample waveform blocks, calibrates AC RMS baselines, detects EN 50160
power quality events. live web dashboard 172.17.223.251:5000.
"""

from collections import deque
from datetime import datetime
from flask import Flask, jsonify, render_template_string, Response, request
from gpiozero import DigitalOutputDevice

import io
import json
import logging
import math
from pathlib import Path
import signal
import os
import re
import secrets          #secure random token for time sync requests
import subprocess
import urllib.request
import sys
import threading
from threading import Event, Lock
import time
import spidev

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ==============================================================================
# hardware and spi configuration parameters:
# ==============================================================================

SPI_BUS = 0
SPI_DEVICE = 0
RST_BCM = 24
XEN_BCM = 25

SPI_MODE = 0b01
SPI_HZ = 500_000
FRAME_WORDS = 5                     #each adc frame has 5 words (3 bytes/24-bits each) for 15 bytes total
FRAME_BYTES = FRAME_WORDS * 3

REG_CLOCK = 0x03
CLOCK_600_SPS = 0x0716              #600 Samples/sec, 4.096 MHz internal clock, 50 Hz notch filter enabled
REG_DCDC_CTRL = 0x31
DCDC_FREQ_4P096_MHZ_AND_ENABLE = 0x0001

STATUS_SEC_FAIL = 1 << 6            #status register bit 6 indicates critical SEC_FAIL error
STATUS_READY_MASK = 0b011

VOLTS_PER_CODE = 1.2 / (1 << 23)    # converts signed 24-bit ADC code to voltage (V) for AMC131M03 with 1.2V reference
SAMPLING_RATE_HZ = 600.0
TRIGGER_BUFFER_SIZE = 100           # rolling sample buffer size for triggered waveform capture (100 samples at 600 SPS = 0.167 seconds)
AVERAGE_BLOCK_COUNT = 10            # number of triggered waveform blocks to average for live waveform display
STARTUP_DISCARD_SAMPLES = 10
PRINT_INTERVAL = 0.5                # seconds between console printouts of current status and voltage readings
BASELINE_SAMPLES = 100
RMS_CYCLES = 5                      # number of zero-crossing cycles to use for AC RMS calculation (5 cycles at 60 Hz = 0.083 seconds)
PLOT_REFRESH_SECONDS = 0.1
STALE_TIMEOUT_SECONDS = 2.0
CROSSING_TIMEOUT_SECONDS = 0.5
NOMINAL_RECALIBRATION_TIMEOUT_SECONDS = 5.0

# Internet probes use HTTP so an incorrect Pi clock cannot break a TLS check.
# Probe responses are never used as a time source; systemd-timesyncd uses NTP.
INTERNET_CHECK_INTERVAL = 60.0
INTERNET_PROBE_TIMEOUT = 4.0
INTERNET_PROBES = (
    ("http://connectivitycheck.gstatic.com/generate_204", 204, b""),
    ("http://www.msftconnecttest.com/connecttest.txt", 200, b"Microsoft Connect Test"),
)
TIME_SYNC_TIMEOUT = 45.0
TIME_SYNC_COOLDOWN = 30.0
SYSTEMCTL = "/usr/bin/systemctl"
TIME_SYNC_TOKEN = secrets.token_urlsafe(32)

LOG_FILE = Path(__file__).resolve().parent / "log.txt"

# logging level ERROR only
logging.basicConfig(
    filename=LOG_FILE,
    filemode="a",
    level=logging.ERROR,
    format="%(asctime)s %(levelname)s %(message)s",
)

# ==============================================================================
# mathematical calculations and utility functions:
# ==============================================================================

"""Return mean-removed AC RMS for sequence of voltage samples."""
def ac_rms_from_volts(samples):
    if len(samples) < 2:
        return None
    mean = sum(samples) / len(samples)
    return math.sqrt(sum((value - mean) ** 2 for value in samples) / len(samples))

"""print and log EN 50160 power quality event."""
def report_anomaly(reason, **details):
    record = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "event": reason,
        **details,
    }
    message = json.dumps(record, default=str)
    print(f"POWER QUALITY EVENT: {message}", flush=True)
    # Log anomalies as ERROR level so they are logged in log.txt
    logging.error(message)

# pack 16-bit word into 3-byte list for SPI transfer
def word24(word16: int) -> list[int]:
    return [(word16 >> 8) & 0xFF, word16 & 0xFF, 0]

# pack command frame for SPI 
def command_frame(command: int) -> list[int]:
    return word24(command) + [0] * (FRAME_BYTES - 3)

# pack write-register frame for SPI
def wreg_frame(address: int, value: int) -> list[int]:
    command = 0x6000 | ((address & 0x3F) << 7)
    return word24(command) + word24(value) + [0] * (FRAME_BYTES - 6)

# unpack signed 24-bit ADC code from 3-byte list
def signed24(data: list[int]) -> int:
    value = (data[0] << 16) | (data[1] << 8) | data[2]
    return value - (1 << 24) if value & 0x800000 else value

# unpack status register from received SPI frame
def status_from_frame(rx: list[int]) -> int:
    return (rx[0] << 8) | rx[1]

# unpack 3-channel signed 24-bit ADC codes from received SPI frame
def channels_from_frame(rx: list[int]) -> list[int]:
    return [signed24(rx[3:6]), signed24(rx[6:9]), signed24(rx[9:12])]

# ==============================================================================
# background acquisition worker thread:
# ==============================================================================

def update_calibration_status(shared_state, data_lock, **changes):
    with data_lock:
        shared_state["calibration_status"].update(changes)


"""Poll ADC and update capture, calibration, and monitoring state."""
def adc_acquisition_worker(spi, stop_event, data_lock, shared_state):
    try:
        def null_frame() -> list[int]:
            return spi.xfer2(command_frame(0x0000)) #send null command frame to ADC, check for response

        null_frame() # discard first frames after reset, as it may contain invalid data
        null_frame()

        deadline = time.monotonic() + 5.0 # wait up to 5 seconds for device to become ready
        status = 0
        device_ready = False

        # wait for ADC device to become ready (SEC_FAIL bit cleared)
        while not stop_event.is_set() and time.monotonic() < deadline:
            rx = null_frame()
            status = status_from_frame(rx)

            # check that STATUS register is valid (bits 8-9 should be 01)
            if ((status >> 8) & 0b11) != 0b01:
                raise RuntimeError(f"Unexpected STATUS 0x{status:04X}; check SPI wiring.")

            if not (status & STATUS_SEC_FAIL):
                device_ready = True
                break
            time.sleep(0.001)

        # if device is not ready after timeout, raise error
        if stop_event.is_set():
            return

        if not device_ready:
            raise TimeoutError(f"SEC_FAIL remained active (STATUS=0x{status:04X}).")

        print("ADC device ready; beginning background SPI polling.")

        # initialize acquisition state variables
        discard = STARTUP_DISCARD_SAMPLES
        calibration_windows = [deque(maxlen=BASELINE_SAMPLES), deque(maxlen=BASELINE_SAMPLES)]
        baseline_rms = None
        recalibration_active = False
        recalibration_started = None
        recalibration_windows = [deque(maxlen=BASELINE_SAMPLES), deque(maxlen=BASELINE_SAMPLES)]
        event_active = [None, None]
        last_print = 0.0
        last_ready_warning = 0.0
        last_metrics_update = float("-inf")
        last_capture_publish = float("-inf")
        last_accepted_monotonic = None
        valid_sample_count = 0
        last_ch2_voltage = math.nan

        prev_voltage = 0.0
        prev_crossing_time = None
        current_frequency = None

        capturing_triggered = False
        capture_start_time = None
        temp_triggered_times = []
        temp_triggered_vals = [[], [], []]

        start_time = time.monotonic()   # mark start time for sample timestamps

        # main acquisition loop: poll ADC, detect zero crossings, capture waveform blocks, calibrate RMS, and update shared state
        while not stop_event.is_set():
            # Only the acquisition worker owns the nominal values and sample windows.
            with data_lock:
                recalibration_requested = shared_state["nominal_recalibration_requested"]
                shared_state["nominal_recalibration_requested"] = False
            if recalibration_requested:
                recalibration_active = True
                recalibration_started = time.monotonic()
                for window in recalibration_windows:
                    window.clear()
                update_calibration_status(
                    shared_state, data_lock, busy=True, phase="collecting", error=None,
                    samples_collected=0,
                    message="Collecting fresh CH0/CH1 samples; event detection temporarily paused.",
                )
            if (recalibration_active and time.monotonic() - recalibration_started
                    >= NOMINAL_RECALIBRATION_TIMEOUT_SECONDS):
                recalibration_active = False
                message = "Recalibration timed out waiting for fresh samples. Previous nominal values kept."
                with data_lock:
                    shared_state["calibration_status"].update(
                        busy=False, phase="error", error=message, message=message,
                    )
                    shared_state["latest_metrics"]["calibrated"] = baseline_rms is not None

            rx = null_frame()
            status = status_from_frame(rx)

            with data_lock:
                shared_state["last_status"] = status

            if ((status >> 8) & 0b11) != 0b01:
                raise RuntimeError(f"Unexpected STATUS 0x{status:04X}; check SPI wiring.")

            if status & STATUS_SEC_FAIL:
                raise RuntimeError(f"ADC reported critical SEC_FAIL error: STATUS=0x{status:04X}")

            if (status & STATUS_READY_MASK) != STATUS_READY_MASK:
                now = time.monotonic()
                if now - last_ready_warning >= PRINT_INTERVAL:
                    print(
                        f"Waiting for CH0/CH1 data: STATUS=0x{status:04X}, "
                        f"ready bits=0b{status & 0b111:03b}",
                        flush=True,
                    )
                    last_ready_warning = now
                stop_event.wait(0.0005)
                continue

            sample_wall_time = time.monotonic()
            if (
                last_accepted_monotonic is not None
                and sample_wall_time - last_accepted_monotonic > CROSSING_TIMEOUT_SECONDS
            ):
                capturing_triggered = False
                capture_start_time = None
                temp_triggered_times = []
                temp_triggered_vals = [[], [], []]
                prev_voltage = 0.0
                prev_crossing_time = None
                current_frequency = None
                for window in calibration_windows:
                    window.clear()
                if recalibration_active:
                    for window in recalibration_windows:
                        window.clear()
                    update_calibration_status(
                        shared_state, data_lock, samples_collected=0,
                        message="Sample gap detected; waiting for 100 fresh consecutive samples per channel.",
                    )
                with data_lock:
                    shared_state["triggered_blocks"].clear()
                    shared_state["current_block"] = None
                    shared_state["last_crossing_monotonic"] = None
                    shared_state["times"].clear()
                    shared_state["crossings"].clear()
                    for channel_values in shared_state["values"]:
                        channel_values.clear()

            last_accepted_monotonic = sample_wall_time
            valid_sample_count += 1
            raw = channels_from_frame(rx)
            volts = [value * VOLTS_PER_CODE for value in raw]
            if status & 0b100:
                last_ch2_voltage = volts[2]
            volts[2] = last_ch2_voltage

            sample_time = sample_wall_time - start_time
            ch0_volts = volts[0]
            
            # detect positive-going zero crossing at 0.0V
            is_zero_crossing = prev_voltage < 0.0 and ch0_volts >= 0.0

            # update frequency estimate based on zero crossings
            if is_zero_crossing:
                if prev_crossing_time is not None:
                    period = sample_time - prev_crossing_time
                    if 0.01 < period < 0.04:
                        measured_freq = 1.0 / period
                        current_frequency = (
                            measured_freq if current_frequency is None
                            else 0.8 * current_frequency + 0.2 * measured_freq
                        )
                prev_crossing_time = sample_time

            # capture triggered waveform blocks (also fallback to continuous capture if zero-crossing takes a moment)
            if is_zero_crossing and not capturing_triggered:
                capturing_triggered = True
                capture_start_time = sample_time
                temp_triggered_times = [0.0]
                temp_triggered_vals = [[volts[0]], [volts[1]], [volts[2]]]

            elif capturing_triggered:
                temp_triggered_times.append(sample_time - capture_start_time)
                for channel, value in enumerate(volts):
                    temp_triggered_vals[channel].append(value)

                # once the triggered buffer is full, store it in shared state and reset capture state
                if len(temp_triggered_times) >= TRIGGER_BUFFER_SIZE:
                    with data_lock:
                        shared_state["triggered_blocks"].append({
                            "times": list(temp_triggered_times),
                            "values": [list(cv) for cv in temp_triggered_vals],
                        })
                        shared_state["current_block"] = None

                    capturing_triggered = False
                    capture_start_time = None
                    temp_triggered_times = []
                    temp_triggered_vals = [[], [], []]

            # update shared state with latest sample data, frequency, and zero crossing events
            prev_voltage = ch0_volts
            
            with data_lock:
                shared_state["times"].append(sample_time)
                shared_state["frequency"] = current_frequency
                shared_state["last_sample_monotonic"] = sample_wall_time
                shared_state["valid_sample_count"] = valid_sample_count

                if is_zero_crossing:
                    shared_state["crossings"].append(sample_time)
                    shared_state["last_crossing_monotonic"] = sample_wall_time

                if capturing_triggered and (
                    is_zero_crossing
                    or sample_wall_time - last_capture_publish >= PLOT_REFRESH_SECONDS
                ):
                    last_capture_publish = sample_wall_time
                    shared_state["current_block"] = {
                        "times": list(temp_triggered_times),
                        "values": [list(cv) for cv in temp_triggered_vals],
                    }

                for channel, value in enumerate(volts):
                    shared_state["values"][channel].append(value)

                while shared_state["times"] and sample_time - shared_state["times"][0] > 1.0:
                    shared_state["times"].popleft()
                    for channel_values in shared_state["values"]:
                        channel_values.popleft()

                while shared_state["crossings"] and sample_time - shared_state["crossings"][0] > 1.0:
                    shared_state["crossings"].popleft()

            # feed calibration windows
            if discard:
                discard -= 1
                continue

            for channel in range(2):
                calibration_windows[channel].append(volts[channel])
                if recalibration_active:
                    recalibration_windows[channel].append(volts[channel])

            if sample_wall_time - last_metrics_update < PLOT_REFRESH_SECONDS:
                continue
            last_metrics_update = sample_wall_time

            just_recalibrated = False
            if recalibration_active:
                collected = min(len(window) for window in recalibration_windows)
                update_calibration_status(shared_state, data_lock, samples_collected=collected)
                if collected >= BASELINE_SAMPLES:
                    candidate = [ac_rms_from_volts(list(window)) for window in recalibration_windows]
                    if all(value is not None and math.isfinite(value) for value in candidate):
                        baseline_rms = candidate
                        event_active = [None, None]
                        recalibration_active = False
                        just_recalibrated = True
                        update_calibration_status(
                            shared_state, data_lock, busy=False, phase="complete", error=None,
                            samples_collected=BASELINE_SAMPLES,
                            last_completed=datetime.now().astimezone().isoformat(timespec="seconds"),
                            message="CH0/CH1 nominal values recalculated. Event detection resumed; baselines are fixed.",
                        )
                        print(f"AC RMS baseline recalibrated: CH0={baseline_rms[0]:.4f}V, CH1={baseline_rms[1]:.4f}V")
                    else:
                        recalibration_active = False
                        message = "Invalid RMS samples. Previous nominal values kept."
                        update_calibration_status(
                            shared_state, data_lock, busy=False, phase="error", error=message, message=message,
                        )

            if baseline_rms is None:
                update_calibration_status(
                    shared_state, data_lock,
                    samples_collected=min(len(window) for window in calibration_windows),
                )
                if all(len(window) >= BASELINE_SAMPLES for window in calibration_windows):
                    baseline_rms = [ac_rms_from_volts(list(window)) for window in calibration_windows]
                    print(f"AC RMS baseline calibrated: CH0={baseline_rms[0]:.4f}V, CH1={baseline_rms[1]:.4f}V")
                    update_calibration_status(
                        shared_state, data_lock, busy=False, phase="complete", error=None,
                        samples_collected=BASELINE_SAMPLES,
                        last_completed=datetime.now().astimezone().isoformat(timespec="seconds"),
                        message="Startup calibration complete. Nominal values stay fixed until manually recalculated or restarted.",
                    )
            elif not recalibration_active and not just_recalibrated:
                for channel in range(2):
                    window = calibration_windows[channel]
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

            # continuously update dashboard metrics so numbers show up immediately
            ch0_rms_val = ac_rms_from_volts(list(calibration_windows[0])) or 0.0
            ch1_rms_val = ac_rms_from_volts(list(calibration_windows[1])) or 0.0

            with data_lock:
                shared_state["latest_metrics"] = {
                    "frequency": current_frequency,
                    "ch0_rms": ch0_rms_val,
                    "ch1_rms": ch1_rms_val,
                    "ch0_nom": baseline_rms[0] if baseline_rms else None,
                    "ch1_nom": baseline_rms[1] if baseline_rms else None,
                    "calibrated": baseline_rms is not None and not recalibration_active,
                }

            now = time.monotonic()
            if now - last_print >= PRINT_INTERVAL:
                print(
                    f"Status=0x{status:04X}  Freq="
                    f"{format(current_frequency, '.2f') if current_frequency is not None else '--'}Hz  "
                    f"CH0={volts[0]:+.4f}V  CH1={volts[1]:+.4f}V  CH2={volts[2]:+.4f}V"
                )
                last_print = now
    
    except Exception as exc:
        with data_lock:
            shared_state["worker_error"] = str(exc)
            shared_state["nominal_recalibration_requested"] = False
            shared_state["calibration_status"].update(
                busy=False, phase="error", error=str(exc),
                message="Acquisition error; nominal recalibration unavailable: " + str(exc),
            )
        print(f"Error in acquisition worker: {exc}", file=sys.stderr)
        logging.exception("Acquisition worker exception")


# ==============================================================================
# background internet connectivity and system clock synchronization:
# ==============================================================================

def check_internet_connection():
    """Check known responses, rejecting redirects and typical captive portals."""
    errors = []
    for url, expected_status, expected_body in INTERNET_PROBES:
        try:
            probe = urllib.request.Request(url, headers={"User-Agent": "ADC-Dashboard/1.0"})
            with urllib.request.urlopen(probe, timeout=INTERNET_PROBE_TIMEOUT) as response:
                body = response.read(128)
                if (response.geturl() == url and response.status == expected_status
                        and body == expected_body):
                    return True, None
                errors.append("Unexpected response or captive portal")
        except Exception as exc:
            errors.append(str(exc))
    logging.debug("Internet probe failures: %s", errors)
    return False, (
        "Internet check failed: neither connection-check endpoint was reachable with "
        "the expected response. Check Wi-Fi/Ethernet, DNS, firewall/proxy settings, "
        "or a captive portal. Date/time was not synchronized."
    )


def run_clock_command(arguments):
    """Run fixed OS commands without a shell, password prompt, or unbounded wait."""
    try:
        result = subprocess.run(
            arguments, capture_output=True, text=True, timeout=10,
            env={**os.environ, "LC_ALL": "C"},
        )
    except FileNotFoundError as exc:
        raise RuntimeError("Required system clock command is not installed.") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("The system clock command timed out.") from exc
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()[:350]
        raise RuntimeError(detail or "System clock command failed.")
    return result.stdout


def read_clock_status():
    output = run_clock_command([
        "timedatectl", "show", "--property=NTP", "--property=NTPSynchronized",
    ])
    properties = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
    if not {"NTP", "NTPSynchronized"}.issubset(properties):
        raise RuntimeError("Could not read the Pi's NTP synchronization status.")
    return properties


def force_system_time_sync(stop_event):
    """Restart timesyncd and verify a NEW NTP reply, not just an enabled service."""
    service = run_clock_command([
        SYSTEMCTL, "show", "systemd-timesyncd.service",
        "--property=LoadState", "--property=ActiveState",
    ])
    properties = dict(line.split("=", 1) for line in service.splitlines() if "=" in line)
    if properties.get("LoadState") != "loaded":
        raise RuntimeError(
            "systemd-timesyncd is unavailable. This feature requires Raspberry Pi OS "
            "with systemd-timesyncd; no other clock service was changed."
        )
    if properties.get("ActiveState") != "active":
        raise RuntimeError(
            "systemd-timesyncd is not running. Have an administrator enable it with "
            "'sudo timedatectl set-ntp true'. If using chrony/ntpd, keep that service "
            "and adapt this feature instead of running two NTP services."
        )
    if stop_event.is_set():
        raise RuntimeError("Time synchronization cancelled during shutdown.")

    # Grant ONLY this exact restart command via sudoers; do not run Flask as root.
    try:
        run_clock_command([
            "sudo", "-n", SYSTEMCTL, "restart", "systemd-timesyncd.service",
        ])
    except RuntimeError as exc:
        raise RuntimeError(
            "Cannot request an immediate NTP sync. Allow the dashboard's Linux user "
            "to run 'sudo -n /usr/bin/systemctl restart systemd-timesyncd.service' "
            "without a password. " + str(exc)
        ) from exc

    deadline = time.monotonic() + TIME_SYNC_TIMEOUT
    last_error = None
    while not stop_event.is_set() and time.monotonic() < deadline:
        try:
            clock = read_clock_status()
            report = run_clock_command(["timedatectl", "timesync-status", "--no-pager"])
            packets = re.search(r"^\s*Packet count:\s*(\d+)\s*$", report, re.MULTILINE)
            # Restart resets timesyncd's packet counter; NTP=yes alone proves nothing.
            if (clock["NTP"] == "yes" and clock["NTPSynchronized"] == "yes" and packets
                    and int(packets.group(1)) > 0):
                return
            last_error = None
        except RuntimeError as exc:
            last_error = str(exc)
        stop_event.wait(1.0)
    if stop_event.is_set():
        raise RuntimeError("Time synchronization cancelled during shutdown.")
    detail = " " + last_error if last_error else ""
    raise RuntimeError(
        "Internet check passed, but a fresh NTP synchronization could not be "
        "verified within 45 seconds. Check the configured NTP server and UDP port "
        "123; web access does not guarantee NTP access." + detail
    )


def update_network_state(shared_state, data_lock, **changes):
    with data_lock:
        shared_state["network_status"].update(changes)


def internet_time_worker(stop_event, data_lock, shared_state):
    """Keep network and NTP waits completely outside acquisition and Flask routes."""
    wake_event = shared_state["time_sync_request_event"]
    force_sync = True
    while not stop_event.is_set():
        wake_event.clear()
        with data_lock:
            previous = dict(shared_state["network_status"])
        update_network_state(
            shared_state, data_lock, busy=True, phase="checking",
            message="Checking the Pi's internet connection...", error=None,
        )
        try:
            online, error = check_internet_connection()
            update_network_state(
                shared_state, data_lock, internet_connected=online,
                last_checked=datetime.now().astimezone().isoformat(timespec="seconds"),
            )
            if not online:
                raise RuntimeError(error)
            if stop_event.is_set():
                break
            clock = read_clock_status()
            synchronized = clock["NTP"] == "yes" and clock["NTPSynchronized"] == "yes"
            update_network_state(shared_state, data_lock, ntp_synchronized=synchronized)
            if force_sync or previous["internet_connected"] is not True or not synchronized:
                update_network_state(
                    shared_state, data_lock, phase="syncing",
                    message="Internet reachable. Requesting a fresh NTP date/time sync...",
                )
                force_system_time_sync(stop_event)
                update_network_state(
                    shared_state, data_lock, ntp_synchronized=True,
                    last_sync=datetime.now().astimezone().isoformat(timespec="seconds"),
                )
            update_network_state(
                shared_state, data_lock, phase="complete", error=None,
                message="Internet reachable. Pi date/time is synchronized by NTP.",
            )
        except Exception as exc:
            message = str(exc)
            update_network_state(
                shared_state, data_lock, phase="error", error=message,
                message=message, ntp_synchronized=None,
            )
            if message != previous.get("error"):
                logging.error("Internet/time check: %s", message)
        finally:
            update_network_state(shared_state, data_lock, busy=False)

        deadline = time.monotonic() + INTERNET_CHECK_INTERVAL
        while not stop_event.is_set():
            if wake_event.wait(min(0.25, max(0.0, deadline - time.monotonic()))):
                force_sync = True
                break
            if time.monotonic() >= deadline:
                force_sync = False
                break


# ==============================================================================
# FLASK WEB DASHBOARD & LOG VIEWER SERVER
# ==============================================================================

dashboard_app = Flask(__name__)
web_shared_state_ref = None

DASHBOARD_HTML = r"""
<!DOCTYPE html>
<html data-theme="light">
<head>
    <title>ISO ADC 7 Power Quality Dashboard</title>
    <style>
        :root {
            --bg-color: #f0f2f5;
            --card-bg: #ffffff;
            --text-color: #333333;
            --heading-color: #1d4ed8;
            --border-color: #cbd5e1;
            --sub-text: #64748b;
            --val-color: #0f172a;
            --box-bg: #f8fafc;
            --info-bg: #eff6ff;
            --info-border: #bfdbfe;
            --info-text: #1e293b;
            --info-heading: #1e40af;
            --btn-bg: #e2e8f0;
            --btn-hover: #cbd5e1;
            --btn-text: #1e293b;
        }
        [data-theme="dark"] {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --text-color: #f1f5f9;
            --heading-color: #38bdf8;
            --border-color: #334155;
            --sub-text: #94a3b8;
            --val-color: #f8fafc;
            --box-bg: #090d16;
            --info-bg: #172554;
            --info-border: #1e40af;
            --info-text: #e2e8f0;
            --info-heading: #60a5fa;
            --btn-bg: #334155;
            --btn-hover: #475569;
            --btn-text: #f8fafc;
        }
        body { font-family: Arial, sans-serif; background: var(--bg-color); margin: 0; padding: 20px; color: var(--text-color); transition: background 0.3s, color 0.3s; }
        .container { max-width: 900px; margin: auto; background: var(--card-bg); padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); transition: background 0.3s; }
        .header-row { display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid var(--border-color); padding-bottom: 10px; margin-top: 0; margin-bottom: 20px; }
        h1 { color: var(--heading-color); font-size: 24px; margin: 0; }
        .theme-btn { background: var(--btn-bg); color: var(--btn-text); border: 1px solid var(--border-color); padding: 8px 14px; border-radius: 6px; cursor: pointer; font-weight: bold; font-size: 13px; transition: background 0.2s; }
        .theme-btn:hover { background: var(--btn-hover); }
        .metrics-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; margin-bottom: 20px; }
        .metric-card { background: var(--box-bg); border: 1px solid var(--border-color); border-radius: 6px; padding: 15px; text-align: center; }
        .metric-title { font-size: 13px; color: var(--sub-text); font-weight: bold; text-transform: uppercase; }
        .metric-value { font-size: 22px; color: var(--val-color); font-weight: bold; margin-top: 5px; }
        .metric-sub { font-size: 12px; color: var(--sub-text); margin-top: 4px; }
        .metrics-grid.stale .metric-value { opacity: 0.55; }
        .section-box { background: var(--box-bg); border: 1px solid var(--border-color); border-radius: 6px; padding: 15px; margin-bottom: 20px; text-align: center; }
        .section-box img { width: 100%; max-width: 800px; height: auto; border-radius: 4px; border: 1px solid var(--border-color); }
        .log-viewer { background: #090d16; color: #38bdf8; font-family: monospace; font-size: 12px; padding: 12px; border-radius: 6px; height: 180px; overflow-y: scroll; text-align: left; border: 1px solid #334155; white-space: pre-wrap; word-break: break-all; }
        .info-card { background: var(--info-bg); border: 1px solid var(--info-border); border-radius: 6px; padding: 15px; margin-bottom: 20px; text-align: left; }
        .info-card h3 { margin-top: 0; color: var(--info-heading); font-size: 15px; border-bottom: 1px solid var(--info-border); padding-bottom: 6px; }
        .info-card ul { margin: 0; padding-left: 20px; font-size: 13px; color: var(--info-text); line-height: 1.5; }
        .info-card li { margin-bottom: 6px; }
        .network-tools { text-align: left; }
        .network-tools p { margin: 8px 0; font-size: 13px; white-space: pre-wrap; overflow-wrap: anywhere; }
        .network-tools.error { border-width: 2px; }
        .theme-btn:disabled { opacity: 0.6; cursor: wait; }
        #network-error-dialog { background: var(--card-bg); color: var(--text-color); border: 2px solid var(--border-color); border-radius: 8px; padding: 22px; max-width: min(500px, 85vw); }
        #network-error-dialog::backdrop { background: rgba(0, 0, 0, 0.45); }
        #network-error-text { white-space: pre-wrap; overflow-wrap: anywhere; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header-row">
            <h1>AMC131M03 Power Quality Dashboard</h1>
            <button class="theme-btn" id="theme-toggle" onclick="toggleTheme()">Switch to Dark Mode</button>
        </div>
        <div class="section-box network-tools" id="network-panel">
            <div class="metric-title">Pi Internet &amp; Date/Time</div>
            <p id="pi-clock">Pi clock: waiting...</p>
            <p id="network-state" role="status" aria-live="polite">Waiting for connection check...</p>
            <p id="network-details"></p>
            <button class="theme-btn" id="sync-time-btn" onclick="checkInternetAndSync()">Check internet &amp; sync time</button>
        </div>
        <dialog id="network-error-dialog" aria-labelledby="network-error-title">
            <h2 id="network-error-title">Internet / time synchronization error</h2>
            <p id="network-error-text"></p>
            <button class="theme-btn" onclick="document.getElementById('network-error-dialog').close()">Dismiss</button>
        </dialog>
        <div class="metrics-grid" id="metrics-grid">
            <div class="metric-card">
                <div class="metric-title">Grid Frequency</div>
                <div class="metric-value" id="freq-val">-- Hz</div>
            </div>
            <div class="metric-card">
                <div class="metric-title">Channel 0 AC RMS</div>
                <div class="metric-value" id="ch0-val">-- V</div>
                <div class="metric-sub" id="ch0-nom">Nominal: Calibrating...</div>
            </div>
            <div class="metric-card">
                <div class="metric-title">Channel 1 AC RMS</div>
                <div class="metric-value" id="ch1-val">-- V</div>
                <div class="metric-sub" id="ch1-nom">Nominal: Calibrating...</div>
            </div>
        </div>
        <div class="section-box" style="text-align: left;">
            <button class="theme-btn" id="recalibrate-btn" onclick="recalculateNominal()" disabled>Recalculate nominal voltage (CH0 + CH1)</button>
            <p class="metric-sub">Use only when the expected nominal input is connected and stable. Uses 100 fresh samples per channel, not continuous recalibration.</p>
            <p class="metric-sub" id="calibration-status" role="status" aria-live="polite">Waiting for startup calibration...</p>
            <p class="metric-sub" id="calibration-request-error" role="alert"></p>
        </div>
        <div class="section-box">
            <div class="metric-title" style="margin-bottom: 10px;">Live Averaged Waveform Monitor</div>
            <img id="waveform-img" src="/plot.png" alt="Live Waveform">
        </div>
        <div class="section-box">
            <div class="metric-title" style="margin-bottom: 10px; text-align: left;">Event Log Viewer (log.txt)</div>
            <div class="log-viewer" id="log-box">Waiting for power quality events or anomalies...</div>
        </div>
        <div class="info-card">
            <h3>EN 50160 Power Quality Disruption Criteria & Live Thresholds</h3>
            <ul>
                <li><strong>Nominal Baseline (V<sub>nom</sub>):</strong> Calibrated automatically from the initial 100 samples per channel (displayed live above). Use the recalculation button after connecting a stable nominal input; the baseline does not continuously adapt.</li>
                <li><strong>Voltage Sag (Dip):</strong> Logged when measured AC RMS drops between <strong>10% and 90%</strong> of nominal (0.10 &middot; V<sub>nom</sub> &le; V<sub>RMS</sub> &lt; 0.90 &middot; V<sub>nom</sub>).</li>
                <li><strong>Voltage Swell:</strong> Logged when measured AC RMS rises above <strong>10%</strong> of nominal (V<sub>RMS</sub> &gt; 1.10 &middot; V<sub>nom</sub>).</li>
                <li><strong>Voltage Interruption:</strong> Logged when measured AC RMS drops below <strong>10%</strong> of nominal (&lt; 0.10 &middot; V<sub>nom</sub>).</li>
            </ul>
        </div>
    </div>
    <script>
        const TIME_SYNC_TOKEN = {{ time_sync_token | tojson }};
        let lastNetworkPopupError = null;
        let syncRequestInFlight = false;
        let nominalRequestInFlight = false;

        function updateCalibrationPanel(data) {
            const calibration = data.calibration;
            if (!calibration) return;
            const button = document.getElementById('recalibrate-btn');
            button.disabled = nominalRequestInFlight || calibration.busy || data.is_stale || Boolean(data.worker_error);
            button.innerText = calibration.busy
                ? 'Calibrating nominal voltage...'
                : 'Recalculate nominal voltage (CH0 + CH1)';
            let message = calibration.message;
            if (calibration.busy) message += ' (' + calibration.samples_collected + '/100 samples per channel)';
            if (data.worker_error) {
                message = 'Acquisition error: ' + data.worker_error + '. Recalibration unavailable.';
            } else if (data.is_stale) {
                message += ' No fresh ADC data; recalibration unavailable.';
            }
            document.getElementById('calibration-status').innerText = message;
        }

        async function recalculateNominal() {
            if (nominalRequestInFlight) return;
            nominalRequestInFlight = true;
            const button = document.getElementById('recalibrate-btn');
            const errorText = document.getElementById('calibration-request-error');
            button.disabled = true;
            errorText.innerText = '';
            const controller = new AbortController();
            const timeout = setTimeout(() => controller.abort(), 5000);
            try {
                const response = await fetch('/api/recalibrate-nominal', {
                    method: 'POST', cache: 'no-store', signal: controller.signal,
                    headers: {'X-Dashboard-Token': TIME_SYNC_TOKEN}
                });
                const result = await response.json();
                if (!response.ok) throw new Error(result.error || 'HTTP ' + response.status);
                document.getElementById('calibration-status').innerText = result.message;
            } catch (err) {
                errorText.innerText = 'Could not request recalibration: ' + err.message;
            } finally {
                clearTimeout(timeout);
                nominalRequestInFlight = false;
                // Status polling restores the button when acquisition/calibration is ready.
            }
        }

        function showNetworkError(message) {
            document.getElementById('network-error-text').innerText = message;
            const dialog = document.getElementById('network-error-dialog');
            if (typeof dialog.showModal === 'function') {
                if (!dialog.open) dialog.showModal();
            } else {
                window.alert(message);
            }
        }

        function updateNetworkPanel(network, piTime) {
            if (!network) return;
            document.getElementById('pi-clock').innerText = 'Pi clock: ' + piTime;
            document.getElementById('network-state').innerText = network.message;
            document.getElementById('network-panel').classList.toggle('error', Boolean(network.error));
            document.getElementById('network-details').innerText =
                'Last connection check (Pi time): ' + (network.last_checked || 'not checked') +
                '\nLast verified fresh NTP sync: ' + (network.last_sync || 'not verified this run');
            const button = document.getElementById('sync-time-btn');
            button.disabled = network.busy || syncRequestInFlight;
            button.innerText = network.busy ? 'Checking / syncing...' : 'Check internet & sync time';
            if (network.error && network.error !== lastNetworkPopupError) {
                lastNetworkPopupError = network.error;
                showNetworkError(network.error);
            } else if (network.phase === 'complete' && !network.error) {
                lastNetworkPopupError = null;
            }
        }

        async function checkInternetAndSync() {
            if (syncRequestInFlight) return;
            syncRequestInFlight = true;
            lastNetworkPopupError = null;
            const button = document.getElementById('sync-time-btn');
            const controller = new AbortController();
            const timeout = setTimeout(() => controller.abort(), 5000);
            button.disabled = true;
            try {
                const response = await fetch('/api/time-sync', {
                    method: 'POST', cache: 'no-store', signal: controller.signal,
                    headers: {'X-Time-Sync-Token': TIME_SYNC_TOKEN}
                });
                const result = await response.json();
                if (!response.ok) throw new Error(result.error || 'HTTP ' + response.status);
                document.getElementById('network-state').innerText = result.message;
            } catch (err) {
                showNetworkError('Could not request a check from the Pi: ' + err.message);
            } finally {
                clearTimeout(timeout);
                syncRequestInFlight = false;
                // Status polling will re-enable the button after the worker finishes.
            }
        }
        function toggleTheme() {
            const html = document.documentElement;
            const btn = document.getElementById('theme-toggle');
            if (html.getAttribute('data-theme') === 'light') {
                html.setAttribute('data-theme', 'dark');
                btn.innerText = 'Switch to Light Mode';
                try { localStorage.setItem('theme', 'dark'); } catch (_) {}
            } else {
                html.setAttribute('data-theme', 'light');
                btn.innerText = 'Switch to Dark Mode';
                try { localStorage.setItem('theme', 'light'); } catch (_) {}
            }
        }

        // Load saved theme preference on page load
        window.addEventListener('DOMContentLoaded', () => {
            let savedTheme = 'light';
            try { savedTheme = localStorage.getItem('theme') || 'light'; } catch (_) {}
            document.documentElement.setAttribute('data-theme', savedTheme);
            document.getElementById('theme-toggle').innerText = savedTheme === 'dark' ? 'Switch to Light Mode' : 'Switch to Dark Mode';
        });

        async function updateMetrics() {
            const grid = document.getElementById('metrics-grid');
            const controller = new AbortController();
            const timeout = setTimeout(() => controller.abort(), 5000);
            try {
                const response = await fetch('/api/status', {
                    cache: 'no-store', signal: controller.signal
                });
                if (!response.ok) throw new Error('HTTP ' + response.status);
                const data = await response.json();
                updateNetworkPanel(data.network, data.pi_time);
                updateCalibrationPanel(data);
                grid.classList.toggle('stale', data.is_stale || Boolean(data.worker_error));

                document.getElementById('freq-val').innerText =
                    Number.isFinite(data.frequency) ? data.frequency.toFixed(2) + ' Hz' : '-- Hz';
                for (const channel of [0, 1]) {
                    const rms = data['ch' + channel + '_rms'];
                    const nominal = data['ch' + channel + '_nom'];
                    if (Number.isFinite(rms)) {
                        document.getElementById('ch' + channel + '-val').innerText = rms.toFixed(4) + ' V';
                    }
                    document.getElementById('ch' + channel + '-nom').innerText =
                        Number.isFinite(nominal)
                            ? 'Nominal (Vnom): ' + nominal.toFixed(4) + ' V'
                            : 'Nominal: Calibrating...';
                }
            } catch (err) {
                grid.classList.add('stale');
                document.getElementById('recalibrate-btn').disabled = true;
                document.getElementById('calibration-status').innerText =
                    'Cannot contact the Pi; recalibration status is unknown.';
                document.getElementById('network-state').innerText =
                    'Cannot contact the Pi dashboard. Internet status is unknown; readings may be stale.';
            } finally {
                clearTimeout(timeout);
                setTimeout(updateMetrics, 500);
            }
        }

        function updateLogs() {
            fetch('/api/logs')
                .then(response => response.json())
                .then(data => {
                    if (data.logs && data.logs.length > 0) {
                        var logBox = document.getElementById('log-box');
                        logBox.innerText = data.logs.join('\n');
                        logBox.scrollTop = logBox.scrollHeight;
                    }
                }).catch(err => console.error('Logs fetch error:', err));
        }

        updateMetrics();
        updateLogs();
        setInterval(updateLogs, 2000);

        function refreshWaveform() {
            const next = new Image();
            next.onload = function() {
                next.id = 'waveform-img';
                next.alt = 'Live Waveform';
                document.getElementById('waveform-img').replaceWith(next);
                setTimeout(refreshWaveform, 100);
            };
            next.onerror = function() {
                setTimeout(refreshWaveform, 1000);
            };
            next.src = '/plot.png?t=' + Date.now();
        }
        refreshWaveform();
    </script>
</body>
</html>
"""
# flask routes for web dashboard and API endpoints
@dashboard_app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML, time_sync_token=TIME_SYNC_TOKEN)

@dashboard_app.route("/api/status")
def api_status():
    global web_shared_state_ref
    if web_shared_state_ref:
        with web_shared_state_ref["lock"]:
            payload = build_status_payload(web_shared_state_ref["state"], time.monotonic())
        response = jsonify(payload)
        response.headers["Cache-Control"] = "no-store"
        return response
    return jsonify({"error": "Initializing", "is_stale": True})

@dashboard_app.route("/api/recalibrate-nominal", methods=["POST"])
def api_recalibrate_nominal():
    token = request.headers.get("X-Dashboard-Token", "")
    if not secrets.compare_digest(token.encode("utf-8"), TIME_SYNC_TOKEN.encode("utf-8")):
        return jsonify({"error": "Invalid recalibration request token. Reload the dashboard."}), 403
    if web_shared_state_ref is None:
        return jsonify({"error": "Dashboard is initializing."}), 503
    state = web_shared_state_ref["state"]
    with web_shared_state_ref["lock"]:
        if state["worker_error"]:
            return jsonify({"error": "Acquisition has failed. Restart acquisition before recalibrating."}), 409
        last_sample = state["last_sample_monotonic"]
        if last_sample is None or time.monotonic() - last_sample > STALE_TIMEOUT_SECONDS:
            return jsonify({"error": "No fresh ADC data. Previous nominal values kept."}), 409
        if state["calibration_status"]["busy"] or state["nominal_recalibration_requested"]:
            return jsonify({"error": "Nominal voltage calibration is already running."}), 409
        state["nominal_recalibration_requested"] = True
        state["calibration_status"].update(
            busy=True, phase="queued", samples_collected=0, error=None,
            message="Nominal recalibration queued; previous values kept until fresh samples are collected.",
        )
    response = jsonify({"message": "Collecting 100 fresh CH0/CH1 samples to recalculate nominal voltage..."})
    response.headers["Cache-Control"] = "no-store"
    return response, 202


@dashboard_app.route("/api/time-sync", methods=["POST"])
def api_time_sync():
    # Token prevents cross-site requests; this LAN dashboard has no user login.
    token = request.headers.get("X-Time-Sync-Token", "")
    if not secrets.compare_digest(token.encode("utf-8"), TIME_SYNC_TOKEN.encode("utf-8")):
        return jsonify({"error": "Invalid time-sync request token. Reload the dashboard."}), 403
    if web_shared_state_ref is None:
        return jsonify({"error": "Dashboard is initializing."}), 503
    state = web_shared_state_ref["state"]
    with web_shared_state_ref["lock"]:
        if state["network_status"]["busy"] or state["time_sync_request_event"].is_set():
            return jsonify({"error": "A connection/time check is already running."}), 409
        now = time.monotonic()
        previous = state["last_manual_time_sync"]
        if previous is not None and now - previous < TIME_SYNC_COOLDOWN:
            return jsonify({"error": "Please wait 30 seconds between manual time-sync requests."}), 429
        state["last_manual_time_sync"] = now
        state["network_status"].update(
            busy=True, phase="queued", error=None, message="Connection/time check queued...",
        )
        state["time_sync_request_event"].set()
    response = jsonify({"message": "Checking internet and synchronizing the Pi clock..."})
    response.headers["Cache-Control"] = "no-store"
    return response, 202


@dashboard_app.route("/api/logs")
def api_logs():
    try:
        if LOG_FILE.exists():
            with open(LOG_FILE, "r") as f:
                lines = f.readlines()
                if not lines:
                    return jsonify({"logs": ["Waiting for power quality events or anomalies..."]})
                tail_lines = [line.strip() for line in lines[-50:]]
                return jsonify({"logs": tail_lines})
        else:
            return jsonify({"logs": ["Waiting for power quality events or anomalies..."]})
    except Exception as exc:
        return jsonify({"logs": [f"Error reading log file: {exc}"]})

@dashboard_app.route("/plot.png")
def plot_png():
    global web_shared_state_ref
    if web_shared_state_ref:
        with web_shared_state_ref["lock"]:
            img_bytes = web_shared_state_ref["state"].get("plot_image_bytes")
            if img_bytes:
                return Response(
                    img_bytes, mimetype="image/png",
                    headers={"Cache-Control": "no-store"},
                )
    return "Generating...", 503

def run_web_server(shared_state, data_lock):
    global web_shared_state_ref
    web_shared_state_ref = {"state": shared_state, "lock": data_lock}
    
    # silence Flask/Werkzeug HTTP access request logs in the console
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.ERROR)
    
    dashboard_app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)


# ==============================================================================
# MAIN, WEB SERVER & PLOT GENERATOR
# ==============================================================================

def create_shared_state():
    return {
        "nominal_recalibration_requested": False,
        "calibration_status": {
            "busy": True,
            "phase": "startup",
            "samples_collected": 0,
            "last_completed": None,
            "error": None,
            "message": "Collecting startup nominal voltage samples...",
        },
        "time_sync_request_event": Event(),
        "last_manual_time_sync": None,
        "network_status": {
            "busy": False,
            "phase": "waiting",
            "internet_connected": None,
            "ntp_synchronized": None,
            "last_checked": None,
            "last_sync": None,
            "message": "Waiting for internet/time check...",
            "error": None,
        },
        "times": deque(),
        "values": [deque(), deque(), deque()],
        "crossings": deque(),
        "triggered_blocks": deque(maxlen=AVERAGE_BLOCK_COUNT),
        "current_block": None,
        "last_sample_monotonic": None,
        "last_crossing_monotonic": None,
        "frequency": None,
        "latest_metrics": {},
        "plot_image_bytes": None,
        "worker_error": None,
        "plot_error": None,
        "last_status": None,
        "valid_sample_count": 0,
        "waveform_state": "Waiting for ADC data...",
    }


def build_status_payload(state, now):
    payload = dict(state.get("latest_metrics", {}))
    last_sample = state.get("last_sample_monotonic")
    last_crossing = state.get("last_crossing_monotonic")
    age = now - last_sample if last_sample is not None else None
    crossing_age = now - last_crossing if last_crossing is not None else None
    payload.update({
        "calibration": dict(state["calibration_status"]),
        "network": dict(state["network_status"]),
        "pi_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "sample_age_s": age,
        "is_stale": age is None or age > STALE_TIMEOUT_SECONDS,
        "worker_error": state.get("worker_error"),
        "plot_error": state.get("plot_error"),
        "last_status": state.get("last_status"),
        "valid_sample_count": state.get("valid_sample_count", 0),
        "waveform_state": state.get("waveform_state", "Waiting for ADC data..."),
    })
    if payload["is_stale"] or crossing_age is None or crossing_age > CROSSING_TIMEOUT_SECONDS:
        payload["frequency"] = None
    return payload


def update_waveform_plot(plot_ax, average_lines, snapshot, now):
    last_sample = snapshot["last_sample_monotonic"]
    if snapshot["worker_error"]:
        return "Acquisition error; holding last waveform"
    if last_sample is None or now - last_sample > STALE_TIMEOUT_SECONDS:
        return "Waiting for fresh CH0/CH1 samples; holding last waveform"

    last_crossing = snapshot["last_crossing_monotonic"]
    recent_crossing = last_crossing is not None and now - last_crossing <= CROSSING_TIMEOUT_SECONDS
    blocks = list(snapshot["triggered_blocks"])
    current = snapshot["current_block"]
    if current and current["times"]:
        blocks.append(current)
    blocks = blocks[-AVERAGE_BLOCK_COUNT:]

    if blocks and recent_crossing:
        sample_count = max(len(block["times"]) for block in blocks)
        average_times = []
        average_values = [[], [], []]
        for index in range(sample_count):
            contributors = [block for block in blocks if index < len(block["times"])]
            average_times.append(sum(block["times"][index] for block in contributors) / len(contributors))
            for channel in range(3):
                values = [block["values"][channel][index] for block in contributors]
                values = [value for value in values if math.isfinite(value)]
                average_values[channel].append(sum(values) / len(values) if values else math.nan)
        for channel in range(3):
            average_lines[channel].set_data(average_times, average_values[channel])
        plot_ax.set_xlabel("Time from Trigger (s)")
        plot_ax.set_xlim(0.0, max(average_times[-1], 0.01))
        return f"Zero-crossing average: {len(blocks)} blocks; CH2 held between its fresh readings"

    if blocks:
        return "No recent CH0 zero crossings; holding last triggered waveform (RMS still live)"

    times = snapshot["times"][-TRIGGER_BUFFER_SIZE:]
    if times:
        relative_times = [value - times[0] for value in times]
        for channel in range(3):
            values = snapshot["values"][channel][-len(times):]
            average_lines[channel].set_data(relative_times, values)
        plot_ax.set_xlabel("Time within live preview (s)")
        plot_ax.set_xlim(0.0, max(relative_times[-1], 0.01))
    return "Live preview (not averaged): waiting for a CH0 positive-going zero crossing"


def main() -> int:
    spi = spidev.SpiDev()
    reset = None
    xen = None
    acquisition_thread = None
    web_thread = None
    network_thread = None

    stop_event = Event()
    data_lock = Lock()

    # shared state dictionary to hold acquisition data, metrics, and plot image bytes for web dash
    shared_state = create_shared_state()

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
        spi.xfer2(wreg_frame(REG_DCDC_CTRL, DCDC_FREQ_4P096_MHZ_AND_ENABLE))

        xen.on()
        time.sleep(0.01)

        acquisition_thread = threading.Thread(
            target=adc_acquisition_worker,
            args=(spi, stop_event, data_lock, shared_state),
            daemon=False,
        )
        acquisition_thread.start()

        network_thread = threading.Thread(
            target=internet_time_worker,
            args=(stop_event, data_lock, shared_state),
            daemon=True,
        )
        network_thread.start()

        web_thread = threading.Thread(
            target=run_web_server,
            args=(shared_state, data_lock),
            daemon=True,
        )
        web_thread.start()
        print("Web dashboard server running at http://172.17.223.251:5000")

        plot_fig, plot_ax = plt.subplots(figsize=(8, 4.5))
        plot_ax.set_xlabel("Time from Trigger (s)")
        plot_ax.set_ylabel("Voltage (V)")
        plot_ax.set_title("AMC131M03 Averaged Zero-Crossing Waveform")
        plot_ax.grid(True, color="#dfe6ee", linewidth=0.8)
        plot_ax.set_axisbelow(True)

        average_lines = []
        colors = ["tab:blue", "tab:orange", "tab:green"]

        for channel, color in enumerate(colors):
            line, = plot_ax.plot([], [], color=color, linewidth=1.5, label=f"CH{channel}")
            average_lines.append(line)

        average_lines[2].set_label("CH2 (last received)")
        plot_status = plot_ax.text(
            0.01, 0.02, "Waiting for ADC data...", transform=plot_ax.transAxes,
            fontsize=8, va="bottom", wrap=True,
        )
        plot_ax.legend(loc="upper right", fontsize=9)
        plot_ax.set_xlim(0.0, 0.10)
        plot_ax.set_ylim(-1.3, 1.3)
        plot_fig.tight_layout()

        last_plot_update = time.monotonic()

        # main loop: periodically update waveform plot and store it in shared state for web dashboard
        while not stop_event.is_set():
            now = time.monotonic()

            if now - last_plot_update >= PLOT_REFRESH_SECONDS:
                with data_lock:
                    snapshot = {
                        "triggered_blocks": list(shared_state["triggered_blocks"]),
                        "current_block": shared_state["current_block"],
                        "times": list(shared_state["times"]),
                        "values": [list(cv) for cv in shared_state["values"]],
                        "last_sample_monotonic": shared_state["last_sample_monotonic"],
                        "last_crossing_monotonic": shared_state["last_crossing_monotonic"],
                        "worker_error": shared_state["worker_error"],
                    }

                try:
                    mode = update_waveform_plot(plot_ax, average_lines, snapshot, now)
                    plot_status.set_text(mode)
                    with io.BytesIO() as buf:
                        plot_fig.savefig(
                            buf, format="png", dpi=80,
                            pil_kwargs={"compress_level": 1},
                        )
                        image_bytes = buf.getvalue()
                    with data_lock:
                        shared_state["plot_image_bytes"] = image_bytes
                        shared_state["waveform_state"] = mode
                        shared_state["plot_error"] = None
                except Exception as exc:
                    with data_lock:
                        previous_error = shared_state["plot_error"]
                        shared_state["plot_error"] = str(exc)
                    if str(exc) != previous_error:
                        logging.exception("Waveform rendering exception")
                        print(f"Waveform rendering error: {exc}", file=sys.stderr)

                last_plot_update = now

            time.sleep(0.01)

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
        shared_state["time_sync_request_event"].set()
        if network_thread is not None:
            network_thread.join(timeout=2.0)
        if acquisition_thread is not None and acquisition_thread.is_alive():
            acquisition_thread.join()

        if reset is not None:
            try:
                reset.off()
                time.sleep(0.001)
                reset.on()
                time.sleep(0.01)
            except Exception:
                pass

        if xen is not None:
            try:
                xen.off()
            except Exception:
                pass

        try:
            spi.close()
        except Exception:
            pass

        signal.signal(signal.SIGINT, previous_sigint_handler)

if __name__ == "__main__":
    raise SystemExit(main())
