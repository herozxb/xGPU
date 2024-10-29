# DEVICE CONTROL REGISTER in Python
# This is a Python representation of the device control register described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the DCR hardware module.

class DeviceControlRegister:
    def __init__(self):
        self.reset()

    def reset(self):
        """Resets the device control register state."""
        self.device_control_register = 0

    def execute(self, clk, reset, device_control_write_enable, device_control_data):
        """
        Simulates the behavior of the device control register during each clock cycle.
        """
        if reset:
            self.reset()
        elif device_control_write_enable:
            self.device_control_register = device_control_data

    def get_thread_count(self):
        """Returns the current thread count."""
        return self.device_control_register

# Example usage
dcr = DeviceControlRegister()
clk = True
reset = False
device_control_write_enable = True
device_control_data = 8

dcr.execute(clk, reset, device_control_write_enable, device_control_data)
print(f"Thread Count: {dcr.get_thread_count()}")

