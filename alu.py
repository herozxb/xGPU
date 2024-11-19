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
        
        print("====================ALU[3]===============")
        
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
            
#		IDLE = 0b000			# 0
#		FETCH = 0b001			# 1
#		DECODE = 0b010			# 2
#		REQUEST = 0b011		# 3
#		WAIT = 0b100			# 4
#		EXECUTE = 0b101		# 5
#		UPDATE = 0b110			# 6
#		DONE = 0b111			# 7
            
                print("========core_state=EXECUTE==========")
                if decoded_alu_output_mux == 1:
                    print("============nzp===============")
                    # Set values to compare with NZP register in alu_out[2:0]
                    nzp = [int(rs - rt > 0), int(rs - rt == 0), int(rs - rt < 0)]
                    self.alu_out = (nzp[0] << 2) | (nzp[1] << 1) | nzp[2]
                else:
                    print("====decoded_alu_arithmetic_mux======")
                    print(bin(decoded_alu_arithmetic_mux))
                    print(rs)
                    print(rt)
                    # Execute the specified arithmetic instruction
                    if decoded_alu_arithmetic_mux == ADD:
                        print("ADD in AlU")
                        print(rs)
                        print(rt)
                        self.alu_out = (rs + rt) & 0xFF  # Ensure 8-bit result
                    elif decoded_alu_arithmetic_mux == SUB:
                        print("SUB in AlU")
                        self.alu_out = (rs - rt) & 0xFF  # Ensure 8-bit result
                    elif decoded_alu_arithmetic_mux == MUL:
                        print("MUL in AlU")
                        print(rs)
                        print(rt)
                        self.alu_out = (rs * rt) & 0xFF  # Ensure 8-bit result
                    elif decoded_alu_arithmetic_mux == DIV:
                        print("DIV in AlU")
                        self.alu_out = (rs // rt) & 0xFF if rt != 0 else 0  # Avoid division by zero, 8-bit result
        print("ALU output")
        print(bin(self.alu_out))
    def get_output(self):
        """Returns the current output of the ALU."""
        return self.alu_out

