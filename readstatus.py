import spidev

spi = spidev.SpiDev()
spi.open(0, 0)                 # SPI bus number and Chip select number
spi.mode = 1                   # AMC131M03 uses SPI mode 1
spi.bits_per_word = 8          # send each 24-bit device word as 3 bytes
spi.max_speed_hz = 100_000     # starting speed

def send_frame(command):
    # One command word + four more 24-bit words = a typical 5-word frame.
    tx = [(command >> 8) & 0xFF, command & 0xFF, 0] + [0] * 12
    return spi.xfer2(tx)

send_frame(0xA080)             # RREG: read one register at address 0x01 (STATUS)
rx = send_frame(0x0000)        # Next frame clocks out that register's response

status = int.from_bytes(bytes(rx[0:3]), "big") >> 8
print(f"STATUS = 0x{status:04X}")

spi.xfer([0b00000000, 0b00000000, 0b00000000])

spi.close()
