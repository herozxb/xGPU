# ARITHMETIC-LOGIC UNIT in Python
# This is a Python representation of the ALU described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the ALU hardware module.

class ALU:
    def __init__(self):
        self.reset()

    def reset(self):
        """Resets the ALU state."""
        self.alu_out = 0

    def execute(self, clk, reset, enable, core_state, decoded_alu_arithmetic_mux, decoded_alu_output_mux, rs, rt):
        """
        Simulates the behavior of the ALU during each clock cycle.
        :param clk: Clock signal (not directly used in this Python simulation)
        :param reset: Reset signal to reset ALU state
        :param enable: Enable signal for the ALU
        :param core_state: Core state to determine execution stage
        :param decoded_alu_arithmetic_mux: Operation selector for arithmetic operations
        :param decoded_alu_output_mux: Output mux signal
        :param rs: First operand (8-bit integer)
        :param rt: Second operand (8-bit integer)
        """
        # Local parameter constants for operation codes
        ADD = 0b00
        SUB = 0b01
        MUL = 0b10
        DIV = 0b11

        # Reset ALU output if reset signal is active
        if reset:
            self.alu_out = 0
        elif enable:
            # Execute ALU operation when core_state is EXECUTE (represented by 0b101)
            if core_state == 0b101:
                if decoded_alu_output_mux == 1:
                    # Set values to compare with NZP register in alu_out[2:0]
                    nzp = [int(rs - rt > 0), int(rs - rt == 0), int(rs - rt < 0)]
                    self.alu_out = (nzp[0] << 2) | (nzp[1] << 1) | nzp[2]
                else:
                    # Execute the specified arithmetic instruction
                    if decoded_alu_arithmetic_mux == ADD:
                        self.alu_out = (rs + rt) & 0xFF  # Ensure 8-bit result
                    elif decoded_alu_arithmetic_mux == SUB:
                        self.alu_out = (rs - rt) & 0xFF  # Ensure 8-bit result
                    elif decoded_alu_arithmetic_mux == MUL:
                        self.alu_out = (rs * rt) & 0xFF  # Ensure 8-bit result
                    elif decoded_alu_arithmetic_mux == DIV:
                        self.alu_out = (rs // rt) & 0xFF if rt != 0 else 0  # Avoid division by zero, 8-bit result

    def get_output(self):
        """Returns the current output of the ALU."""
        return self.alu_out

# Example usage
alu = ALU()
clk = True
reset = False
enable = True
core_state = 0b101
decoded_alu_arithmetic_mux = 0b00  # ADD operation
decoded_alu_output_mux = 0
rs = 15
rt = 10

alu.execute(clk, reset, enable, core_state, decoded_alu_arithmetic_mux, decoded_alu_output_mux, rs, rt)
print(f"ALU Output: {alu.get_output()}")

