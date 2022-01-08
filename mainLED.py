import max7219
from machine import Pin, SPI
from time import sleep

spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)

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

# while True:
#     for x in range(32, -length, -1):
#         display.text(msg ,x,0,1)
#         display.show()
#         sleep(0.10)
#         display.fill(0)
