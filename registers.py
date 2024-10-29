# REGISTER FILE in Python
# This is a Python representation of the register file described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the register file hardware module.

class RegisterFile:
    def __init__(self, threads_per_block=4, thread_id=0, data_bits=8):
        self.THREADS_PER_BLOCK = threads_per_block
        self.THREAD_ID = thread_id
        self.DATA_BITS = data_bits
        self.reset()

    def reset(self):
        """Resets the register file state."""
        self.registers = [0] * 16  # 16 registers per thread (13 free registers and 3 read-only registers)
        self.registers[14] = self.THREADS_PER_BLOCK  # %blockDim
        self.registers[15] = self.THREAD_ID  # %threadIdx
        self.rs = 0
        self.rt = 0

    def execute(self, clk, reset, enable, block_id, core_state, decoded_rd_address, decoded_rs_address,
                decoded_rt_address, decoded_reg_write_enable, decoded_reg_input_mux, decoded_immediate,
                alu_out, lsu_out):
        """
        Simulates the behavior of the register file during each clock cycle.
        """
        ARITHMETIC = 0b00
        MEMORY = 0b01
        CONSTANT = 0b10

        if reset:
            self.reset()
        elif enable:
            # Update the block_id when a new block is issued from dispatcher
            self.registers[13] = block_id

            # Fill rs/rt when core_state = REQUEST (0b011)
            if core_state == 0b011:
                self.rs = self.registers[decoded_rs_address]
                self.rt = self.registers[decoded_rt_address]

            # Store rd when core_state = UPDATE (0b110)
            if core_state == 0b110:
                # Only allow writing to R0 - R12
                if decoded_reg_write_enable and decoded_rd_address < 13:
                    if decoded_reg_input_mux == ARITHMETIC:
                        # ADD, SUB, MUL, DIV
                        self.registers[decoded_rd_address] = alu_out
                    elif decoded_reg_input_mux == MEMORY:
                        # LDR
                        self.registers[decoded_rd_address] = lsu_out
                    elif decoded_reg_input_mux == CONSTANT:
                        # CONST
                        self.registers[decoded_rd_address] = decoded_immediate

# Example usage
register_file = RegisterFile()
clk = True
reset = False
enable = True
block_id = 1
core_state = 0b011  # REQUEST state
decoded_rd_address = 2
decoded_rs_address = 4
decoded_rt_address = 5
decoded_reg_write_enable = True
decoded_reg_input_mux = 0b00  # ARITHMETIC
decoded_immediate = 10
alu_out = 15
lsu_out = 20

register_file.execute(clk, reset, enable, block_id, core_state, decoded_rd_address, decoded_rs_address,
                     decoded_rt_address, decoded_reg_write_enable, decoded_reg_input_mux, decoded_immediate,
                     alu_out, lsu_out)
print(f"Register rs: {register_file.rs}")
print(f"Register rt: {register_file.rt}")

