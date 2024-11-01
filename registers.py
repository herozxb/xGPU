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
                
        self.registers[13] = 0 # %blockIdx
        self.registers[14] = self.THREADS_PER_BLOCK  # %blockDim
        self.registers[15] = 0 #3 #self.THREAD_ID  # %threadIdx
        
        
        self.rs = 0
        self.rt = 0

    def execute(self, clk, reset, enable, block_id, core_state, decoded_rd_address, decoded_rs_address,
                decoded_rt_address, decoded_reg_write_enable, decoded_reg_input_mux, decoded_immediate,
                alu_out, lsu_out):
                
        print("===============RegisterFile[5]================")
        print("===============core_state================")
        print(bin(core_state))
        
        """
        Simulates the behavior of the register file during each clock cycle.
        """
        ARITHMETIC = 0b00
        MEMORY = 0b01
        CONSTANT = 0b10

        if reset:
            self.reset()
        elif enable:
        
#        IDLE = 0b000			# 0
#        FETCH = 0b001			# 1
#        DECODE = 0b010			# 2
#        REQUEST = 0b011		# 3
#        WAIT = 0b100			# 4
#        EXECUTE = 0b101		# 5
#        UPDATE = 0b110			# 6
#        DONE = 0b111			# 7        
        
            print("==================block_id========================")
            print(block_id)
            print("==================self.registers[start]========================")
            print(self.registers)
            # Update the block_id when a new block is issued from dispatcher
            #self.registers[13] = block_id
            #self.registers[13] = 7
            self.registers[13] = 1
            #self.registers[1] = 100

            print("==================decoded_rs_address========================")
            print(decoded_rs_address)

            print("==================decoded_rt_address========================")
            print(decoded_rt_address)

            # Fill rs/rt when core_state = REQUEST (0b011)
            if core_state == 0b011:
                self.rs = self.registers[decoded_rs_address]
                self.rt = self.registers[decoded_rt_address]
                
                print("==================after_REQUEST========================")
                print(self.rs)
                print(self.rt)
            
            
            print("==================core_state[register]========================")
            print(bin(core_state) )           


            # Store rd when core_state = UPDATE (0b110)
            if core_state == 0b110:
                # Only allow writing to R0 - R12
                
                print("==================[UPDATE]decoded_reg_input_mux[register]========================")
                print(bin(decoded_reg_input_mux)) 
                print(bin(decoded_rd_address)) 
                print(bin(alu_out)) 
                
                if decoded_reg_write_enable and decoded_rd_address < 13:
                    if decoded_reg_input_mux == ARITHMETIC:
                        # ADD, SUB, MUL, DIV
                        self.registers[decoded_rd_address] = alu_out
                    elif decoded_reg_input_mux == MEMORY:
                        # LDR
                        print("=========LDR===========")
                        print(decoded_rd_address)
                        print(lsu_out)
                        print(self.registers)
                        self.registers[decoded_rd_address] = lsu_out
                    elif decoded_reg_input_mux == CONSTANT:
                        # CONST
                        self.registers[decoded_rd_address] = decoded_immediate
                print(self.registers) 

            print("==================self.registers[end]========================")
            print(self.registers)
