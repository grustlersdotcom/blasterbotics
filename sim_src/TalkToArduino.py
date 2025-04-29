from smbus2 import SMBus
import time

class ArduinoI2C:
    def __init__(self, bus_id=1, address=0x08):
        self.bus = SMBus(bus_id)
        self.address = address

    def write_byte(self, data):
        """Write a single byte to the Arduino."""
        self.bus.write_byte(self.address, data)

    def write_bytes(self, data_list):
        """Write multiple bytes."""
        self.bus.write_i2c_block_data(self.address, 0, data_list)

    def read_byte(self):
        """Read a single byte from the Arduino."""
        return self.bus.read_byte(self.address)

    def read_bytes(self, num_bytes):
        """Read multiple bytes."""
        return self.bus.read_i2c_block_data(self.address, 0, num_bytes)

    def close(self):
        self.bus.close()


