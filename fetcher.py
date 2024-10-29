# INSTRUCTION FETCHER in Python
# This is a Python representation of the instruction fetcher described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the fetcher hardware module.

class Fetcher:
    def __init__(self, program_mem_addr_bits=8, program_mem_data_bits=16):
        self.PROGRAM_MEM_ADDR_BITS = program_mem_addr_bits
        self.PROGRAM_MEM_DATA_BITS = program_mem_data_bits
        self.reset()

    def reset(self):
        """Resets the instruction fetcher state."""
        self.fetcher_state = 0b000  # IDLE state
        self.mem_read_valid = 0
        self.mem_read_address = 0
        self.instruction = 0

    def execute(self, clk, reset, core_state, current_pc, mem_read_ready, mem_read_data):
        """
        Simulates the behavior of the instruction fetcher during each clock cycle.
        """
        IDLE = 0b000
        FETCHING = 0b001
        FETCHED = 0b010

        if reset:
            self.reset()
        else:
            if self.fetcher_state == IDLE:
                # Start fetching when core_state = FETCH (0b001)
                if core_state == 0b001:
                    self.fetcher_state = FETCHING
                    self.mem_read_valid = 1
                    self.mem_read_address = current_pc
            elif self.fetcher_state == FETCHING:
                # Wait for response from program memory
                if mem_read_ready:
                    self.fetcher_state = FETCHED
                    self.instruction = mem_read_data  # Store the instruction when received
                    self.mem_read_valid = 0
            elif self.fetcher_state == FETCHED:
                # Reset when core_state = DECODE (0b010)
                if core_state == 0b010:
                    self.fetcher_state = IDLE

# Example usage
fetcher = Fetcher()
clk = True
reset = False
core_state = 0b001  # FETCH state
current_pc = 5
mem_read_ready = True
mem_read_data = 42

fetcher.execute(clk, reset, core_state, current_pc, mem_read_ready, mem_read_data)
print(f"Fetcher State: {fetcher.fetcher_state}")
print(f"Instruction: {fetcher.instruction}")

