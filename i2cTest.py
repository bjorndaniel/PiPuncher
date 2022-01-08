from machine import I2C, Pin, Timer

# Create I2C object
i2c = I2C(0, scl=Pin(17), sda=Pin(16))

# Print out any addresses found
print("scanning...")
devices = i2c.scan()

if devices:
    for d in devices:
        print(hex(d))