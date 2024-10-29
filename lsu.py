# LOAD-STORE UNIT in Python
# This is a Python representation of the load-store unit described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the LSU hardware module.

class LSU:
    def __init__(self):
        self.reset()

    def reset(self):
        """Resets the load-store unit state."""
        self.lsu_state = 0b00  # IDLE state
        self.lsu_out = 0
        self.mem_read_valid = 0
        self.mem_read_address = 0
        self.mem_write_valid = 0
        self.mem_write_address = 0
        self.mem_write_data = 0

    def execute(self, clk, reset, enable, core_state, decoded_mem_read_enable, decoded_mem_write_enable,
                rs, rt, mem_read_ready, mem_read_data, mem_write_ready):
        """
        Simulates the behavior of the load-store unit during each clock cycle.
        """
        IDLE = 0b00
        REQUESTING = 0b01
        WAITING = 0b10
        DONE = 0b11

        if reset:
            self.reset()
        elif enable:
            # If memory read enable is triggered (LDR instruction)
            if decoded_mem_read_enable:
                if self.lsu_state == IDLE:
                    if core_state == 0b011:  # REQUEST state
                        self.lsu_state = REQUESTING
                elif self.lsu_state == REQUESTING:
                    self.mem_read_valid = 1
                    self.mem_read_address = rs
                    self.lsu_state = WAITING
                elif self.lsu_state == WAITING:
                    if mem_read_ready:
                        self.mem_read_valid = 0
                        self.lsu_out = mem_read_data
                        self.lsu_state = DONE
                elif self.lsu_state == DONE:
                    if core_state == 0b110:  # UPDATE state
                        self.lsu_state = IDLE

            # If memory write enable is triggered (STR instruction)
            if decoded_mem_write_enable:
                if self.lsu_state == IDLE:
                    if core_state == 0b011:  # REQUEST state
                        self.lsu_state = REQUESTING
                elif self.lsu_state == REQUESTING:
                    self.mem_write_valid = 1
                    self.mem_write_address = rs
                    self.mem_write_data = rt
                    self.lsu_state = WAITING
                elif self.lsu_state == WAITING:
                    if mem_write_ready:
                        self.mem_write_valid = 0
                        self.lsu_state = DONE
                elif self.lsu_state == DONE:
                    if core_state == 0b110:  # UPDATE state
                        self.lsu_state = IDLE

# Example usage
lsu = LSU()
clk = True
reset = False
enable = True
core_state = 0b011  # REQUEST state
decoded_mem_read_enable = True
decoded_mem_write_enable = False
rs = 10
rt = 5
mem_read_ready = True
mem_read_data = 20
mem_write_ready = True

lsu.execute(clk, reset, enable, core_state, decoded_mem_read_enable, decoded_mem_write_enable,
            rs, rt, mem_read_ready, mem_read_data, mem_write_ready)
print(f"LSU State: {lsu.lsu_state}")
print(f"LSU Output: {lsu.lsu_out}")

