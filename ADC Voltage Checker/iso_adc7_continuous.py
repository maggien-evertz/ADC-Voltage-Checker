#!/usr/bin/env python3
"""Basic ISO ADC 7 / AMC131M03 test for Raspberry Pi SPI0.

script reads the AMC131M03 STATUS word over SPI and polls its per-channel
DRDY bits. 
"""

import sys
import time

import spidev
from gpiozero import DigitalOutputDevice
#gpiozero lets the script to control the Pi's GPIO pins.

# BCM numbers for gpiozero:
# RST physical 18 -> BCM24; XEN physical 22 -> BCM25.
SPI_BUS = 0
SPI_DEVICE = 0  # CS physical 24 -> CE0 / BCM8
RST_BCM = 24
XEN_BCM = 25

# defaults to 24-bit words, SPI mode 1, and input CRC disabled.
SPI_MODE = 0b01
SPI_HZ = 500_000
FRAME_WORDS = 5
FRAME_BYTES = FRAME_WORDS * 3  # 5 x 24-bit words

# Reset register values: enable all channels, HR mode, CLKIN/2, OSR=16384
# (250 SPS at 8.192 MHz). Gain register reset is 0 => gain 1, +/-1.2 V.
REG_CLOCK = 0x03
CLOCK_250_SPS = 0x071E
REG_DCDC_CTRL = 0x31
DCDC_FREQ_4P096_MHZ_AND_ENABLE = 0x0001  # DCDC_FREQ=0000b, DCDC_EN=1

STATUS_SEC_FAIL = 1 << 6
STATUS_READY_MASK = 0b111

# With gain=1, the differential full-scale is +/-1.2 V.
VOLTS_PER_CODE = 1.2 / (1 << 23)


def word24(word16: int) -> list[int]:
    # MSB-align a 16-bit command/register word in a 24-bit SPI word.
    return [(word16 >> 8) & 0xFF, word16 & 0xFF, 0]


def command_frame(command: int) -> list[int]:
    # build five-word frame; input CRC disabled
    return word24(command) + [0] * (3 * (FRAME_WORDS - 1))


def wreg_frame(address: int, value: int) -> list[int]:
    # build one-register WREG frame plus the required trailing words
    # WREG = 011 + 6-bit address + 7-bit count-minus-one.
    command = 0x6000 | ((address & 0x3F) << 7)
    # One command word, one register-data word, then three words to complete
    # the standard five-word frame (input CRC disabled).
    return word24(command) + word24(value) + [0] * (3 * 3)


def signed24(data: list[int]) -> int:
    value = (data[0] << 16) | (data[1] << 8) | data[2]
    return value - (1 << 24) if value & 0x800000 else value


def status_from_frame(rx: list[int]) -> int:
    # 16-bit STATUS word is MSB-aligned in the first 24-bit response word.
    return (rx[0] << 8) | rx[1]


def channels_from_frame(rx: list[int]) -> list[int]:
    return [signed24(rx[3:6]), signed24(rx[6:9]), signed24(rx[9:12])]


def main() -> int:
    spi = spidev.SpiDev()
    reset = None
    xen = None

    try:
        # GPIO outputs created before touching ADC. 
        # reset pulse requires CLKIN to be running; afterwards stop XEN while configuring
        # the ADC, then restart it only after DCDC_FREQ/DCDC_EN are set.
        reset = DigitalOutputDevice(RST_BCM, initial_value=True)
        xen = DigitalOutputDevice(XEN_BCM, initial_value=False)

        spi.open(SPI_BUS, SPI_DEVICE)
        spi.mode = SPI_MODE
        spi.max_speed_hz = SPI_HZ
        spi.bits_per_word = 8

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
        # data rate at 250 SPS so Python can poll status
        spi.xfer2(wreg_frame(REG_CLOCK, CLOCK_250_SPS))
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
        # the isolated-side supply is valid. Poll it over SPI, not the RDY pin.
        deadline = time.monotonic() + 5.0
        status = 0
        while time.monotonic() < deadline:
            rx = null_frame()
            status = status_from_frame(rx)
            if ((status >> 8) & 0b11) != 0b01:
                raise RuntimeError(
                    f"Unexpected STATUS 0x{status:04X}; check SPI wiring/mode."
                )
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


        # Build a baseline, then flag CH0/CH1 deviations from it.
        discard = 2
        PRINT_INTERVAL = 0.5
        ANOMALY_THRESHOLD_V = 0.10
        BASELINE_SAMPLES = 100

        calibration_sums = [0.0, 0.0]
        calibration_count = 0
        baseline = None
        anomaly_active = [False, False]
        last_print = 0.0

        print(f"Calibrating CH0/CH1 baseline from {BASELINE_SAMPLES} samples... Deviations -+0.10V will be flagged as anomalies.")

        while True:
            rx = null_frame()
            status = status_from_frame(rx)

            if status & STATUS_SEC_FAIL:
                raise RuntimeError(f"ADC reported SEC_FAIL: STATUS=0x{status:04X}")

            if (status & STATUS_READY_MASK) == STATUS_READY_MASK:
                raw = channels_from_frame(rx)
                volts = [value * VOLTS_PER_CODE for value in raw]

                if discard:
                    discard -= 1

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

            time.sleep(0.001)

            
    #program shutdown and clean-up
    except KeyboardInterrupt:
        print("Stopped.")
        return 130
        #stopped prints out when pressing CTRL+C
        
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
        #catches other erros, prints to standard error
        
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