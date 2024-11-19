# INSTRUCTION FETCHER in Python
# This is a Python representation of the instruction fetcher described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the fetcher hardware module.



# A simple implementation of the Fetcher module to make the Core functional.
class Fetcher:
    def __init__(self, program_mem_addr_bits=8, program_mem_data_bits=16):
        self.PROGRAM_MEM_ADDR_BITS = program_mem_addr_bits
        self.PROGRAM_MEM_DATA_BITS = program_mem_data_bits
        self.fetcher_state = 'IDLE'
        self.instruction = 0
        self.mem_read_valid = False
        self.mem_read_address = 0

    def reset(self):
        self.fetcher_state = 'IDLE'
        self.instruction = 0

    def execute(self, clk, reset, core_state, current_pc, program_mem_read_ready, program_mem_read_data):
        print("===============Fetcher[1]================")
        print("===========self.fetcher_state==========")
        print(self.fetcher_state)
        if reset:
            self.reset()
        else:
            if self.fetcher_state == 'IDLE':
                if core_state == 0b001:  # FETCH
                    self.fetcher_state = 'FETCHING'
                    self.mem_read_valid = True
                    self.mem_read_address = current_pc
                    print("=========after_Fetcher[current_pc]===========")
                    print(self.mem_read_address)
                    print(current_pc)
            elif self.fetcher_state == 'FETCHING':
                if program_mem_read_ready:
                    self.fetcher_state = 'FETCHED'
                    self.instruction = program_mem_read_data
                    self.mem_read_valid = False
                    print(self.instruction)
            elif self.fetcher_state == 'FETCHED':
                if core_state == 0b010:  # DECODE
                    self.fetcher_state = 'IDLE'

