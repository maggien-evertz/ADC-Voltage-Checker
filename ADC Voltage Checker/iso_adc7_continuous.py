#!/usr/bin/env python3
"""ISO ADC 7 / AMC131M03 for Raspberry Pi SPI0.

script reads the AMC131M03 STATUS word over SPI and polls its per-channel
DRDY bits. Catches anolmalies in CH0/CH1 voltage relative to a baseline established at startup.
"""

import sys
import time

import spidev
from gpiozero import DigitalOutputDevice
#gpiozero lets the script to control the Pi's GPIO pins.

import json
import logging
from datetime import datetime

# BCM numbers for gpiozero:
# RST XEN physical pins 18 and 22
SPI_BUS = 0
SPI_DEVICE = 0  # GPIO8, CS physical pin 24
RST_BCM = 24    # GPIO 24, pin 18
XEN_BCM = 25    # GPIO 25, pin 22

# defaults to 24-bit words, SPI mode 1, and input CRC disabled.
SPI_MODE = 0b01
SPI_HZ = 500_000
FRAME_WORDS = 5     #each ADC frame has 5 words, each word is 24 bits (3 bytes)
FRAME_BYTES = FRAME_WORDS * 3  # 5 x 24-bit words, each frame is 15 bytes

# (1000 samples per second per channel at 8.192 MHz). Gain register reset is 0 => gain 1 (no amplification), +/-1.2 V differential.
REG_CLOCK = 0x03                         #selects the clock and data rate settings for the ADC
CLOCK_1000_SPS = 0x0716                  #enable all 3 channels, high resolution mode, divide clock by 2, set oversampling ratio to 16384 for 1000 samples per second
REG_DCDC_CTRL = 0x31                     #selects the DCDC converter frequency and enables it
DCDC_FREQ_4P096_MHZ_AND_ENABLE = 0x0001  # DCDC_FREQ=0000b, DCDC_EN=1

STATUS_SEC_FAIL = 1 << 6         #bit 6 of STATUS word indicates that the isolated-side supply is not valid
STATUS_READY_MASK = 0b111        #bits 0-2 of STATUS word indicate the ready status of each channel (CH0, CH1, CH2)

# With gain=1, the differential full-scale is +/-1.2 V.
VOLTS_PER_CODE = 1.2 / (1 << 23) # 1.2 V / 2^23

# Logging configuration
logging.basicConfig(
    filename="log.txt",
    filemode="a",  # append; don't overwrite earlier anomalies
    level=logging.WARNING,
    format="%(asctime)s %(levelname)s %(message)s",
)

# Function to report anomalies with timestamp and details
def report_anomaly(reason, **details):
    record = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "anomaly": reason,
        **details,
    }
    message = json.dumps(record, default=str)
    print(f"ANOMALY: {message}", flush=True)
    logging.warning(message)
    
# functions converts ADC's SPI frame format and python ints.
# SPI sends/receives bytes, ADC organizes into 24-bit words, and Python uses arbitrary-length ints.
def word24(word16: int) -> list[int]:
    # packs 16-bit command/register word in a 24-bit SPI word. MSB first
    return [(word16 >> 8) & 0xFF, word16 & 0xFF, 0]     
    #final byte is 0 because the ADC expects 24-bit words, but the command is only 16 bits.


def command_frame(command: int) -> list[int]:
    # build five-word frame, puts command in first word. Adds zero padding; input CRC disabled
    return word24(command) + [0] * (FRAME_BYTES - 3)


def wreg_frame(address: int, value: int) -> list[int]:
    # encodes one-register WREG frame plus required trailing words into command
    # WREG = 011 + 6-bit address + 7-bit count-minus-one.
    command = 0x6000 | ((address & 0x3F) << 7)
    # one command word, one register-data word, then three words to complete
    # the standard five-word frame
    return word24(command) + word24(value) + [0] * (FRAME_BYTES - 6)


def signed24(data: list[int]) -> int:
    # combines three received bytes into a signed 24-bit integer. MSB first.
    # The ADC's output is in two's complement format, so if MSB is set, we subtract 2^24 to get the negative value.
    value = (data[0] << 16) | (data[1] << 8) | data[2]
    return value - (1 << 24) if value & 0x800000 else value


def status_from_frame(rx: list[int]) -> int:
    # combines the first two bytes of the received frame into a 16-bit STATUS word. 
    return (rx[0] << 8) | rx[1]


def channels_from_frame(rx: list[int]) -> list[int]:
    # extracts the three 24-bit channel readings from the received frame and converts them to signed integers.
    return [signed24(rx[3:6]), signed24(rx[6:9]), signed24(rx[9:12])]


def main() -> int:
    spi = spidev.SpiDev()
    #GPIO OutputDevices
    reset = None
    xen = None

    try:
        # reset pulse requires CLKIN to be running; afterwards stop XEN while configuring
        # the ADC, then restart it only after DCDC_FREQ/DCDC_EN are set.
        reset = DigitalOutputDevice(RST_BCM, initial_value=True)
        xen = DigitalOutputDevice(XEN_BCM, initial_value=False)

        spi.open(SPI_BUS, SPI_DEVICE) #selects the SPI bus and device (chip select) to use for communication with the ADC
        spi.mode = SPI_MODE
        spi.max_speed_hz = SPI_HZ
        spi.bits_per_word = 8 # 8 bit

        time.sleep(0.05)  # allow board supplies to settle
        xen.on()
        time.sleep(0.01)  # allow the 8.192 MHz oscillator to start
        reset.off()       # SYNC/RESET is active low; CLKIN is now present
        time.sleep(0.001)  # >2048 CLKIN cycles required for a reset
        reset.on()
        time.sleep(0.01)  # allow reset/register acquisition to complete
        xen.off()
        time.sleep(0.005)

        # Keep ADC's default gain=1 and all channels enabled
        # data rate at 1000 SPS so Python can poll status
        spi.xfer2(wreg_frame(REG_CLOCK, CLOCK_1000_SPS))
        # 8.192 MHz CLKIN / default divider 2 = 4.096 MHz MOD_CLK.
        # DCDC_FREQ=0000 selects the range containing 4.096 MHz; then enable.
        spi.xfer2(wreg_frame(REG_DCDC_CTRL, 0x0000))
        spi.xfer2(wreg_frame(REG_DCDC_CTRL, DCDC_FREQ_4P096_MHZ_AND_ENABLE))

        xen.on()  # start the Click's 8.192 MHz oscillator
        time.sleep(0.01)

        def null_frame() -> list[int]:
            return spi.xfer2(command_frame(0x0000))

        # Responses are pipelined one SPI frame. Flush the last WREG response
        # and one NULL response before interpreting STATUS.
        null_frame()
        null_frame()

        # SEC_FAIL is latched at startup and clears when STATUS is read after
        # the isolated-side supply is valid. Poll it over SPI until it clears or timeout after 5 seconds.
        deadline = time.monotonic() + 5.0
        status = 0
        while time.monotonic() < deadline:
            rx = null_frame()
            status = status_from_frame(rx)
            if ((status >> 8) & 0b11) != 0b01:
                raise RuntimeError(
                    f"Unexpected STATUS 0x{status:04X}; check SPI wiring/mode."
                )
                
            # check if SEC_FAIL bit is cleared; if so, break the loop. Otherwise, sleep for 1 ms and try again.
            if not (status & STATUS_SEC_FAIL):
                break
            time.sleep(0.001)
        else:
            raise TimeoutError(
                f"SEC_FAIL stayed set (STATUS=0x{status:04X}). "
                "Check 3.3 V/GND, RST, XEN, and SPI wiring."
            )

        print("ADC ready; polling SPI STATUS bits.")
        print("Default gain is 1 (nominal differential range +/-1.2 V).")


        # Build baseline, then flag CH0/CH1 deviations from it.
        discard = 10 
        PRINT_INTERVAL = 0.5
        ANOMALY_THRESHOLD_V = 0.10
        BASELINE_SAMPLES = 100

        calibration_sums = [0.0, 0.0]
        calibration_count = 0
        baseline = None
        anomaly_active = [False, False]
        last_print = 0.0

        print(f"Calibrating CH0/CH1 baseline from {BASELINE_SAMPLES} samples... Deviations +/-0.10V will be flagged as anomalies.")

        # loops indefinitely until CTRL+C is pressed
        # reads ADC's status and channel data, checking for anomalies, and printing status information
        while True:
            rx = null_frame() # send NULL command frame to ADC and receive a response frame containing the STATUS word and channel data
            status = status_from_frame(rx)
            
            # check for SEC_FAIL bit in the STATUS word. If set, raise a RuntimeError indicating that the ADC reported SEC_FAIL.
            if status & STATUS_SEC_FAIL:
                raise RuntimeError(f"ADC reported SEC_FAIL: STATUS=0x{status:04X}")

            # check if all channels are ready by checking the STATUS_READY_MASK. 
            # If all channels ready, extract  channel data from the received frame, convertto volts, and process for baseline calibration or anomaly detection.
            if (status & STATUS_READY_MASK) == STATUS_READY_MASK:
                raw = channels_from_frame(rx)
                volts = [value * VOLTS_PER_CODE for value in raw]

                # discard counter used to skip first few samples after startup or reset to allow the ADC to stabilize.
                if discard:
                    discard -= 1

                # if baseline is None, accumulates the voltage readings for CH0 and CH1 to calculate the baseline
                elif baseline is None:
                    calibration_sums[0] += volts[0]
                    calibration_sums[1] += volts[1]
                    calibration_count += 1

                    if calibration_count == BASELINE_SAMPLES:
                        baseline = [
                            calibration_sums[0] / calibration_count,
                            calibration_sums[1] / calibration_count,
                        ]
                        print(
                            f"Baseline set: CH0={baseline[0]:+.4f} V, "
                            f"CH1={baseline[1]:+.4f} V"
                        )

                # if baseline not None, checks for anomalies in CH0 and CH1 by comparing the current voltage readings to baseline.
                else:
                    for channel in range(2):
                        deviation = volts[channel] - baseline[channel]
                        is_anomaly = abs(deviation) > ANOMALY_THRESHOLD_V

                        if is_anomaly and not anomaly_active[channel]:
                            print(
                                f"ANOMALY CH{channel}: "
                                f"{volts[channel]:+.4f} V "
                                f"(baseline {baseline[channel]:+.4f} V, "
                                f"change {deviation:+.4f} V)"
                            )

                            report_anomaly(
                                f"CH{channel} deviation exceeded threshold",
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

                    now = time.monotonic()
                    if now - last_print >= PRINT_INTERVAL:
                        print(
                            f"STATUS=0x{status:04X}  "
                            f"CH0={volts[0]:+.4f} V  "
                            f"CH1={volts[1]:+.4f} V  "
                            f"CH2={volts[2]:+.4f} V"
                        )
                        last_print = now

            
    # program shutdown and clean-up
    except KeyboardInterrupt:
        print("Stopped.")
        return 130
        # CTRL+C stops the program, returns exit code 130
        
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
        #catches other errors, prints to standard error
        
    finally:
        # cleanup: put the ADC in reset before stopping its clock.
        try:
            if reset is not None:
                reset.off()
            if xen is not None:
                xen.off()
            time.sleep(0.005)
            if reset is not None:
                reset.on()
        except Exception:
            pass
        try:
            spi.close()
        except Exception:
            pass
        if xen is not None:
            xen.close()
        if reset is not None:
            reset.close()


if __name__ == "__main__":
    raise SystemExit(main())