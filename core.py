# Import all the submodules
#from decoder import Decoder
#from scheduler import Scheduler
from alu import ALU
from lsu import LSU
from registers import RegisterFile
#from pc import ProgramCounter



class Scheduler:
    def __init__(self, threads_per_block=4):
        self.THREADS_PER_BLOCK = threads_per_block
        self.reset()

    def reset(self):
        """Resets the scheduler state."""
        self.current_pc = 0
        self.core_state = 0b000  # IDLE state
        self.done = False

    def execute(self, clk, reset, start, decoded_ret, fetcher_state, lsu_state, next_pc):
        """
        Simulates the behavior of the scheduler during each clock cycle.
        """
        
        #print("===============Scheduler=================")
        #print( next_pc )
        
        IDLE = 0b000			# 0
        FETCH = 0b001			# 1
        DECODE = 0b010			# 2
        REQUEST = 0b011		# 3
        WAIT = 0b100			# 4
        EXECUTE = 0b101		# 5
        UPDATE = 0b110			# 6
        DONE = 0b111			# 7

        if reset:
            self.reset()
        else:
            if self.core_state == IDLE:
                if start:
                    self.core_state = FETCH
            elif self.core_state == FETCH:
                if fetcher_state == 'FETCHED':  # FETCHED state
                    self.core_state = DECODE
            elif self.core_state == DECODE:
                self.core_state = REQUEST
            elif self.core_state == REQUEST:
                self.core_state = WAIT
            elif self.core_state == WAIT:
                any_lsu_waiting = any(lsu in [0b01, 0b10] for lsu in lsu_state)
                if not any_lsu_waiting:
                    self.core_state = EXECUTE
            elif self.core_state == EXECUTE:
                self.core_state = UPDATE
            elif self.core_state == UPDATE:
                if decoded_ret:
                    self.done = True
                    self.core_state = DONE
                else:
                    self.current_pc = next_pc[-1]  # Assume all next_pc converge
                    self.core_state = FETCH
            elif self.core_state == DONE:
                pass  # No operation
                
        #print("============current_pc===================")
        #print(self.current_pc)



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
        if reset:
            self.reset()
        else:
            if self.fetcher_state == 'IDLE':
                if core_state == 0b001:  # FETCH
                    self.fetcher_state = 'FETCHING'
                    self.mem_read_valid = True
                    self.mem_read_address = current_pc
            elif self.fetcher_state == 'FETCHING':
                if program_mem_read_ready:
                    self.fetcher_state = 'FETCHED'
                    self.instruction = program_mem_read_data
                    self.mem_read_valid = False
            elif self.fetcher_state == 'FETCHED':
                if core_state == 0b010:  # DECODE
                    self.fetcher_state = 'IDLE'

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

    def execute(self, clk, reset, core_state, instruction):
        """
        Simulates the behavior of the instruction decoder during each clock cycle.
        """
        if reset:
            self.reset()
        elif core_state == 0b010:  # DECODE state
            
            print(f"instruction state: {bin(instruction)}")
        
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
                print("# NOP")
                pass  # No operation
            elif opcode == 0b0001:  # BRnzp
                print("# BRnzp")
                self.decoded_pc_mux = 1
            elif opcode == 0b0010:  # CMP
                print("# CMP")
                self.decoded_alu_output_mux = 1
                self.decoded_nzp_write_enable = 1
            elif opcode == 0b0011:  # ADD
                print("# ADD")
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b00
            elif opcode == 0b0100:  # SUB
                print("# SUB")
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b01
            elif opcode == 0b0101:  # MUL
                print("# MUL")
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b10
            elif opcode == 0b0110:  # DIV
                print("# DIV")
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b00
                self.decoded_alu_arithmetic_mux = 0b11
            elif opcode == 0b0111:  # LDR
                print("# LDR")
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b01
                self.decoded_mem_read_enable = 1
            elif opcode == 0b1000:  # STR
                print("# STR")
                self.decoded_mem_write_enable = 1
            elif opcode == 0b1001:  # CONST
                print("# CONST")
                self.decoded_reg_write_enable = 1
                self.decoded_reg_input_mux = 0b10
            elif opcode == 0b1111:  # RET
                print("# RET")
                self.decoded_ret = 1


class ProgramCounter:
    def __init__(self, data_mem_data_bits=8, program_mem_addr_bits=8):
        self.DATA_MEM_DATA_BITS = data_mem_data_bits
        self.PROGRAM_MEM_ADDR_BITS = program_mem_addr_bits
        self.reset()

    def reset(self):
        """Resets the program counter state."""
        self.nzp = 0b000
        self.next_pc = 0

    def execute(self, clk, reset, enable, core_state, decoded_nzp, decoded_immediate,decoded_nzp_write_enable, decoded_pc_mux, alu_out, current_pc):
        """
        Simulates the behavior of the program counter during each clock cycle.
        """
        if reset:
            self.reset()
        elif enable:
            #print( "====== core_state ======" )
            #print( core_state )
       
            #print( "====== decoded_pc_mux ======" )
            #print( decoded_pc_mux )
            
            #print( "====== self.next_pc[0] ======" )
            #print( self.next_pc )            
            
            # Update PC when core_state = EXECUTE (0b101)
            if core_state == 0b101:
                if decoded_pc_mux == 1:
                    if (self.nzp & decoded_nzp) != 0b000:
                        # On BRnzp instruction, branch to immediate if NZP case matches previous CMP
                        
                        print( "====== BRnzp ======" )
                        self.next_pc = decoded_immediate
                    else:
                        # Otherwise, just update to PC + 1 (next line)
                        print( "====== next ======" )
                        self.next_pc = current_pc + 1
                else:
                    # By default update to PC + 1 (next line)
                    print( "====== default next ======" )
                    self.next_pc = current_pc + 1

            # Store NZP when core_state = UPDATE (0b110)
            if core_state == 0b110:
                # Write to NZP register on CMP instruction
                if decoded_nzp_write_enable:
                    self.nzp = (alu_out >> (self.DATA_MEM_DATA_BITS - 3)) & 0b111

            #print( "====== self.next_pc[1] ======" )
            #print( self.next_pc )   


from typing import List

class Memory:
    def __init__(self, dut, addr_bits, data_bits, channels, name):
        self.dut = dut									# dut as the Device Under Test that it interacts with.
        self.addr_bits = addr_bits							# addr_bits and data_bits represent the address and data bus sizes.
        self.data_bits = data_bits
        self.memory = [0] * (2**addr_bits)						# memory is the actual storage, modeled as a list with 2^addr_bits entries.
        self.channels = channels							# channels indicates the number of parallel memory channels.
        self.name = name

        self.mem_read_valid = getattr(dut, f"{name}_mem_read_valid")			# whether the read is valid (mem_read_valid)
        self.mem_read_address = getattr(dut, f"{name}_mem_read_address")		# the address (mem_read_address)
        self.mem_read_ready = getattr(dut, f"{name}_mem_read_ready")			# whether the data is ready (mem_read_ready),
        self.mem_read_data = getattr(dut, f"{name}_mem_read_data")			# the data itself (mem_read_data).

        if name != "program":
            self.mem_write_valid = getattr(dut, f"{name}_mem_write_valid")		# whether the write is valid (mem_read_valid)
            self.mem_write_address = getattr(dut, f"{name}_mem_write_address")	# the address (mem_read_address)
            self.mem_write_data = getattr(dut, f"{name}_mem_write_data")		# the data itself (mem_read_data).
            self.mem_write_ready = getattr(dut, f"{name}_mem_write_ready")		# whether the data is write (mem_read_ready),

    def run(self):
        mem_read_valid = [
            int(str(self.mem_read_valid.value)[i:i+1], 2)
            for i in range(0, len(str(self.mem_read_valid.value)), 1)
        ]
        
        # The list comprehension iterates over the indices from 0 to the length of the string representation of self.mem_read_valid.value.
        # For each index i, it:
        # Extracts the character at position i.
        # Converts this character from a binary string ('0' or '1') to an integer (0 or 1).
        # This produces a list where each element is either 0 or 1, corresponding to the binary bits of self.mem_read_valid.value.

        mem_read_address = [
            int(str(self.mem_read_address.value)[i:i+self.addr_bits], 2)
            for i in range(0, len(str(self.mem_read_address.value)), self.addr_bits)
        ]
        
        mem_read_ready = [0] * self.channels
        mem_read_data = [0] * self.channels

        for i in range(self.channels):
            if mem_read_valid[i] == 1:
                mem_read_data[i] = self.memory[mem_read_address[i]]
                mem_read_ready[i] = 1
            else:
                mem_read_ready[i] = 0

        self.mem_read_data.value = int(''.join(format(d, '0' + str(self.data_bits) + 'b') for d in mem_read_data), 2)
        
        # For example, if self.data_bits is 8, the format string would be '08b', which means an 8-bit binary number with leading zeros.
        # For example, if mem_read_data = [3, 5] and data_bits = 8, the output would be '0000001100000101'   00000011 = 3, 00000101 = 5 
        
        
        self.mem_read_ready.value = int(''.join(format(r, '01b') for r in mem_read_ready), 2)
        
        #For example, if mem_read_ready = [1, 0, 1, 1], the output would be '1011'.
        


        if self.name != "program":
        
            mem_write_valid = [
                int(str(self.mem_write_valid.value)[i:i+1], 2)
                for i in range(0, len(str(self.mem_write_valid.value)), 1)
            ]
            
            mem_write_address = [
                int(str(self.mem_write_address.value)[i:i+self.addr_bits], 2)
                for i in range(0, len(str(self.mem_write_address.value)), self.addr_bits)
            ]
            
            mem_write_data = [
                int(str(self.mem_write_data.value)[i:i+self.data_bits], 2)
                for i in range(0, len(str(self.mem_write_data.value)), self.data_bits)
            ]
            
            mem_write_ready = [0] * self.channels

            for i in range(self.channels):
                if mem_write_valid[i] == 1:
                    self.memory[mem_write_address[i]] = mem_write_data[i]
                    mem_write_ready[i] = 1
                else:
                    mem_write_ready[i] = 0

            self.mem_write_ready.value = int(''.join(format(w, '01b') for w in mem_write_ready), 2)

    def write(self, address, data):
        if address < len(self.memory):
            self.memory[address] = data

    def load(self, rows: List[int]):
        for address, data in enumerate(rows):
            self.write(address, data)

    def display(self, rows, decimal=True):
        print("\n")
        print(f"{self.name.upper()} MEMORY")
        
        table_size = (8 * 2) + 3
        print("+" + "-" * (table_size - 3) + "+")

        header = "| Addr | Data "
        print(header + " " * (table_size - len(header) - 1) + "|")

        print("+" + "-" * (table_size - 3) + "+")
        for i, data in enumerate(self.memory):
            if i < rows:
                if decimal:
                    row = f"| {i:<4} | {data:<4}"
                    print(row + " " * (table_size - len(row) - 1) + "|")
                else:
                    data_bin = format(data, f'0{16}b')
                    row = f"| {i:<4} | {data_bin} |"
                    print(row + " " * (table_size - len(row) - 1) + "|")
        print("+" + "-" * (table_size - 3) + "+")




# Python Representation of GPU Compute Core Modules

# --- CORE MODULE ---
# Manages the overall execution of a compute core including fetcher, decoder, scheduler, ALU, LSU, registers, and program counter
class Core:
    def __init__(self, threads_per_block=4, data_mem_addr_bits=8, data_mem_data_bits=8, program_mem_addr_bits=8, program_mem_data_bits=16):
        self.THREADS_PER_BLOCK = threads_per_block
        self.DATA_MEM_ADDR_BITS = data_mem_addr_bits
        self.DATA_MEM_DATA_BITS = data_mem_data_bits
        self.PROGRAM_MEM_ADDR_BITS = program_mem_addr_bits
        self.PROGRAM_MEM_DATA_BITS = program_mem_data_bits

        # Initialize submodules
        self.fetcher = Fetcher(program_mem_addr_bits, program_mem_data_bits)
        self.decoder = Decoder()
        self.scheduler = Scheduler(threads_per_block)
        self.alu = [ALU() for _ in range(threads_per_block)]
        self.lsu = [LSU() for _ in range(threads_per_block)]
        self.register_file = [RegisterFile(threads_per_block, i, data_mem_data_bits) for i in range(threads_per_block)]
        self.pc = [ProgramCounter(data_mem_data_bits, program_mem_addr_bits) for _ in range(threads_per_block)]

    def reset(self):
        self.fetcher.reset()
        self.decoder.reset()
        self.scheduler.reset()
        for i in range(self.THREADS_PER_BLOCK):
            self.alu[i].reset()
            self.lsu[i].reset()
            self.register_file[i].reset()
            self.pc[i].reset()
            
    def execute(self, clk, reset, start, block_id, thread_count, program_memory, data_memory):
        if reset:
            self.reset()
        else:
            #print( " ============= self.scheduler.core_state [0] =============" )
            #print( self.scheduler.core_state )
            
            # Fetcher retrieves instruction from program memory
            if self.scheduler.core_state == 0b001:  # FETCH state
                #print("=======2======")
                self.fetcher.mem_read_address = self.pc[0].next_pc
                program_mem_read_ready = True
                program_mem_read_data = program_memory.memory[self.fetcher.mem_read_address]
                #print("=======program_memory.memory[0]======")
                #print(program_mem_read_data)
            else:
                #print("=======3======")
                program_mem_read_ready = False
                program_mem_read_data = 0

            #print("=======4======")
            # Scheduler manages control flow
            self.scheduler.execute(clk, reset, start, self.decoder.decoded_ret, self.fetcher.fetcher_state, [lsu.lsu_state for lsu in self.lsu], [pc.next_pc for pc in self.pc])
	    
            #print( " ============= self.scheduler.core_state [1] =============" )
            #print( self.scheduler.core_state )

            # Fetcher retrieves instruction
            self.fetcher.execute(clk, reset, self.scheduler.core_state, self.pc[0].next_pc, program_mem_read_ready, program_mem_read_data)

            #print( " ============= self.fetcher.fetcher_state [0] =============" )
            #print( self.fetcher.fetcher_state )

            # Decoder decodes instruction
            self.decoder.execute(clk, reset, self.scheduler.core_state, self.fetcher.instruction)

            # Execute ALU, LSU, Register File, and Program Counter for each thread
            for i in range(thread_count):
                self.pc[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_nzp, self.decoder.decoded_immediate, self.decoder.decoded_nzp_write_enable,self.decoder.decoded_pc_mux, self.alu[i].alu_out, self.pc[i].next_pc)
                #print("=======program_memory.memory[1]======")
                #print(self.pc[i].next_pc)
                #print(self.fetcher.mem_read_address)
                #print(bin(program_memory.memory[self.fetcher.mem_read_address]))
 

# Example usage of Core
if __name__ == "__main__":
    clk = True  # Clock placeholder
    reset = False
    start = True

    # Initialize the core with 4 threads per block
    core = Core(threads_per_block=1)

    # Placeholder values for the required inputs
    block_id = 0  # Block ID
    thread_count = 1  # Number of threads per block
    program_mem_read_ready = True  # Program memory read is ready (placeholder)
    program_mem_read_data = 0x0  # Placeholder for program memory read data

    # Data memory read/write readiness and data for each thread (assuming 4 threads per block)
    data_mem_read_ready = [True for _ in range(4)]  # Placeholder for data memory read readiness
    data_mem_read_data = [0 for _ in range(4)]  # Placeholder for data memory read data
    data_mem_write_ready = [True for _ in range(4)]  # Placeholder for data memory write readiness

    # Create a dummy dut object with the required properties for simulation
    class DummyDut:
        def __init__(self):
            self.program_mem_read_valid = DummySignal()
            self.program_mem_read_address = DummySignal()
            self.program_mem_read_ready = DummySignal()
            self.program_mem_read_data = DummySignal()

            self.data_mem_read_valid = DummySignal()
            self.data_mem_read_address = DummySignal()
            self.data_mem_read_ready = DummySignal()
            self.data_mem_read_data = DummySignal()
            self.data_mem_write_valid = DummySignal()
            self.data_mem_write_address = DummySignal()
            self.data_mem_write_data = DummySignal()
            self.data_mem_write_ready = DummySignal()

    # Dummy Signal class to simulate hardware signals
    class DummySignal:
        def __init__(self):
            self.value = 0

    dut = DummyDut()

    # Initialize Program Memory
    program_memory = Memory(dut=dut, addr_bits=8, data_bits=16, channels=1, name="program")
    program = [
        0b0101000011011110, # MUL R0, %blockIdx, %blockDim
        0b0011000000001111, # ADD R0, R0, %threadIdx         ; i = blockIdx * blockDim + threadIdx
        0b1001000100000000, # CONST R1, #0                   ; baseA (matrix A base address)
        0b1001001000001000, # CONST R2, #8                   ; baseB (matrix B base address)
        0b1001001100010000, # CONST R3, #16                  ; baseC (matrix C base address)
        0b0011010000010000, # ADD R4, R1, R0                 ; addr(A[i]) = baseA + i
        0b0111010001000000, # LDR R4, R4                     ; load A[i] from global memory
        0b0011010100100000, # ADD R5, R2, R0                 ; addr(B[i]) = baseB + i
        0b0111010101010000, # LDR R5, R5                     ; load B[i] from global memory
        0b0011011001000101, # ADD R6, R4, R5                 ; C[i] = A[i] + B[i]
        0b0011011100110000, # ADD R7, R3, R0                 ; addr(C[i]) = baseC + i
        0b1000000001110110, # STR R7, R6                     ; store C[i] in global memory
        0b1111000000000000, # RET                            ; end of kernel
    ]
    
    program_memory.load(program)

    # Initialize Data Memory
    data_memory = Memory(dut=dut, addr_bits=8, data_bits=8, channels=4, name="data")
    data = [
        0, 1, 2, 3, 4, 5, 6, 7,  # Matrix A (1 x 8)
        0, 1, 2, 3, 4, 5, 6, 7  # Matrix B (1 x 8)
    ]
    data_memory.load(data)

    # Display loaded memories
    program_memory.display(rows=13, decimal=False)
    data_memory.display(rows=10, decimal=True)

    # Simulate clock cycles
    for cycle in range(91):
    
        core.execute(clk, reset, start, block_id=0, thread_count=1, program_memory=program_memory, data_memory=data_memory)
        
        # Toggle clock
        clk = not clk
        
        # Display the current PC, core state, and whether done
        print(f"Cycle {cycle}:")
        print(f"Current PC: {core.scheduler.current_pc}")
        print(f"Core State: {bin(core.scheduler.core_state)}")
        print(f"Done: {core.scheduler.done}")
        print(f"Fetcher state: {core.fetcher.fetcher_state}")
        print("-" * 30)

