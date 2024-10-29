# INSTRUCTION DECODER in Python
# This is a Python representation of the instruction decoder described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the decoder hardware module.

class Decoder:
    def __init__(self):
        self.reset()

    def reset(self):
        """Resets the instruction decoder state."""
        self.decoded_rd_address = 0
        self.decoded_rs_address = 0
        self.decoded_rt_address = 0
        self.decoded_immediate = 0
        self.decoded_nzp = 0
        self.decoded_reg_write_enable = 0
        self.decoded_mem_read_enable = 0
        self.decoded_mem_write_enable = 0
        self.decoded_nzp_write_enable = 0
        self.decoded_reg_input_mux = 0
        self.decoded_alu_arithmetic_mux = 0
        self.decoded_alu_output_mux = 0
        self.decoded_pc_mux = 0
        self.decoded_ret = 0

    def decode(self, clk, reset, core_state, instruction):
        """
        Simulates the behavior of the instruction decoder during each clock cycle.
        """
        if reset:
            self.reset()
        elif core_state == 0b010:  # DECODE state
            # Get instruction signals from the instruction
            self.decoded_rd_address = (instruction >> 8) & 0xF
            self.decoded_rs_address = (instruction >> 4) & 0xF
            self.decoded_rt_address = instruction & 0xF
            self.decoded_immediate = instruction & 0xFF
            self.decoded_nzp = (instruction >> 9) & 0x7

            # Control signals reset on every decode and set conditionally by instruction
            self.decoded_reg_write_enable = 0
            self.decoded_mem_read_enable = 0
            self.decoded_mem_write_enable = 0
            self.decoded_nzp_write_enable = 0
            self.decoded_reg_input_mux = 0
            self.decoded_alu_arithmetic_mux = 0
            self.decoded_alu_output_mux = 0
            self.decoded_pc_mux = 0
            self.decoded_ret = 0

            # Set the control signals for each instruction
            opcode = (instruction >> 12) & 0xF

            if opcode == 0b0000:  # NOP
                pass  # No operation
            elif opcode == 0b0001:  # BRnzp
                self.decoded_pc_mux = 1
            elif opcode == 0b0010:  # CMP
                self.decoded_alu_output_mux = 1
                self.decoded_nzp_write_enable = 1
            elif opcode == 0b0011:  # ADD
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b00
            elif opcode == 0b0100:  # SUB
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b01
            elif opcode == 0b0101:  # MUL
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b10
            elif opcode == 0b0110:  # DIV
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b11
            elif opcode == 0b0111:  # LDR
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b01
                self.decoded_mem_read_enable = 1
            elif opcode == 0b1000:  # STR
                self.decoded_mem_write_enable = 1
            elif opcode == 0b1001:  # CONST
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b10
            elif opcode == 0b1111:  # RET
                self.decoded_ret = 1

# Example usage
decoder = Decoder()
clk = True
reset = False
core_state = 0b010  # DECODE state
instruction = 0b0011000000001111  # Example instruction for ADD operation

decoder.decode(clk, reset, core_state, instruction)
print(f"Decoded RD Address: {decoder.decoded_rd_address}")
print(f"Decoded RS Address: {decoder.decoded_rs_address}")
print(f"Decoded RT Address: {decoder.decoded_rt_address}")
print(f"Decoded Immediate: {decoder.decoded_immediate}")
print(f"Decoded Reg Write Enable: {decoder.decoded_reg_write_enable}")

