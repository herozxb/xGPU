# PROGRAM COUNTER in Python
# This is a Python representation of the program counter described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the PC hardware module.

class ProgramCounter:
    def __init__(self, data_mem_data_bits=8, program_mem_addr_bits=8):
        self.DATA_MEM_DATA_BITS = data_mem_data_bits
        self.PROGRAM_MEM_ADDR_BITS = program_mem_addr_bits
        self.reset()

    def reset(self):
        """Resets the program counter state."""
        self.nzp = 0b000
        self.next_pc = 0

    def execute(self, clk, reset, enable, core_state, decoded_nzp, decoded_immediate,
                decoded_nzp_write_enable, decoded_pc_mux, alu_out, current_pc):
        """
        Simulates the behavior of the program counter during each clock cycle.
        """
        if reset:
            self.reset()
        elif enable:
            # Update PC when core_state = EXECUTE (0b101)
            if core_state == 0b101:
                if decoded_pc_mux == 1:
                    if (self.nzp & decoded_nzp) != 0b000:
                        # On BRnzp instruction, branch to immediate if NZP case matches previous CMP
                        self.next_pc = decoded_immediate
                    else:
                        # Otherwise, just update to PC + 1 (next line)
                        self.next_pc = current_pc + 1
                else:
                    # By default update to PC + 1 (next line)
                    self.next_pc = current_pc + 1

            # Store NZP when core_state = UPDATE (0b110)
            if core_state == 0b110:
                # Write to NZP register on CMP instruction
                if decoded_nzp_write_enable:
                    self.nzp = (alu_out >> (self.DATA_MEM_DATA_BITS - 3)) & 0b111

# Example usage
pc = ProgramCounter()
clk = True
reset = False
enable = True
core_state = 0b101  # EXECUTE state
decoded_nzp = 0b011
decoded_immediate = 5
decoded_nzp_write_enable = True
decoded_pc_mux = 1
alu_out = 0b111
current_pc = 10

pc.execute(clk, reset, enable, core_state, decoded_nzp, decoded_immediate,
           decoded_nzp_write_enable, decoded_pc_mux, alu_out, current_pc)
print(f"Next PC: {pc.next_pc}")

