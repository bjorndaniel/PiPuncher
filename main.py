from machine import Pin, Timer, I2C, SPI
import max7219
import utime
import _thread
import sys
import time
import ustruct
from time import sleep

spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)
led = Pin(14, Pin.OUT)
button = Pin(15, Pin.IN, Pin.PULL_UP)
sLock = _thread.allocate_lock()
timer = Timer()
display = max7219.Matrix8x8(spi, ss, 4)
display.brightness(1)  # adjust brightness 1 to 15
display.fill(0)
display.show()
led.low()
# I2C address
ADXL343_ADDR = 0x53

# Registers
REG_DEVID = 0x00
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32

# Other constants
DEVID = 0xE5
SENSITIVITY_2G = 1.0 / 256  # (g/LSB)
EARTH_GRAVITY = 9.80665  # Earth's gravity in [m/s^2]

# Initialize I2C with pins
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)


def counter():
    print("Counter")
    global display
    msg = "3..."
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    display.fill(0)
    display.show()
    msg = "2..."
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    display.fill(0)
    display.show()
    msg = "1..."
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    display.fill(0)
    display.show()
    msg = "GO!!"
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)


def reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """

    # Construct message
    msg = bytearray()
    msg.append(data)

    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)


def reg_read(i2c, addr, reg, nbytes=1):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """

    # Check to make sure caller is asking for 1 or more bytes
    if nbytes < 1:
        return bytearray()

    # Request data from specified register(s) over I2C
    data = i2c.readfrom_mem(addr, reg, nbytes)

    return data


def debounce(pin):
    timer.init(mode=Timer.ONE_SHOT, period=200, callback=coretask)


def coretask(timer):
    global led
    global display
    readings = []
    readBase = reg_read(i2c, ADXL343_ADDR, REG_DATAX0, 6)
    baseVal = ustruct.unpack_from("<h", readBase, 0)[0]
    baseVal = baseVal * SENSITIVITY_2G * EARTH_GRAVITY
    delta = 0.0
    msg = "3..."
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    display.fill(0)
    display.show()
    msg = "2..."
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    display.fill(0)
    display.show()
    msg = "1..."
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    display.fill(0)
    display.show()
    msg = "GO!!"
    length = len(msg)
    length = length * 8
    display = max7219.Matrix8x8(spi, ss, 4)
    display.brightness(1)  # adjust brightness 1 to 15
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(msg, 1, 0, 1)
    display.show()
    sleep(1.0)
    led.high()
    t_end = time.time() + 5
    while time.time() < t_end:
        # Read X, Y, and Z values from registers (16 bits each)
        data = reg_read(i2c, ADXL343_ADDR, REG_DATAX0, 6)
        # Convert 2 bytes (little-endian) into 16-bit integer (signed)
        acc_x = ustruct.unpack_from("<h", data, 0)[0]
        # Convert measurements to [m/s^2]
        acc_x = acc_x * SENSITIVITY_2G * EARTH_GRAVITY
        readings.append(acc_x)
        utime.sleep(0.1)
    for i in readings:
        tempDel = abs(baseVal - i)
        if tempDel > delta:
            delta = tempDel
    score = round(delta * 25)
    print("Delta value: ", round(delta * 25))
    led.low()
    for i in range(score):
        display.fill(0)
        display.show()
        display.text(f'{i:03}', 4, 0, 1)
        display.show()
        sleep(0.01)
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(f'{score:03}', 4, 0, 1)
    display.show()
    sleep(0.5)
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(f'{score:03}', 4, 0, 1)
    display.show()
    sleep(0.5)
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(f'{score:03}', 4, 0, 1)
    display.show()
    sleep(0.5)
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(f'{score:03}', 4, 0, 1)
    display.show()
    sleep(0.5)
    display.fill(0)
    display.show()
    sleep(0.5)
    display.text(f'{score:03}', 4, 0, 1)
    display.show()

# Read device ID to make sure that we can communicate with the ADXL343
data = reg_read(i2c, ADXL343_ADDR, REG_DEVID)
if data != bytearray((DEVID,)):
    print("ERROR: Could not communicate with ADXL343")
    sys.exit()

# Read Power Control register
data = reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL)
print(data)

# Tell ADXL343 to start taking measurements by setting Measure bit to high
data = int.from_bytes(data, "big") | (1 << 3)
reg_write(i2c, ADXL343_ADDR, REG_POWER_CTL, data)

# Test: read Power Control register back to make sure Measure bit was set
data = reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL)
print(data)

# Wait before taking measurements
utime.sleep(2.0)

button.irq(trigger=Pin.IRQ_FALLING, handler=debounce)
