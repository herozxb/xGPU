# Import all the submodules
from scheduler import Scheduler
from fetcher import Fetcher
from decoder import Decoder
from memory import Memory

from alu import ALU
from lsu import LSU
from registers import RegisterFile

from pc import ProgramCounter

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

        print("len")
        print(threads_per_block)
        print(len(self.alu))
    
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
        print("========================================core============================================")   
        if reset:
            self.reset()
        else:
            # Fetcher retrieves instruction from program memory
            if self.scheduler.core_state == 0b001:  # FETCH state
                self.fetcher.mem_read_address = self.pc[0].next_pc
                program_mem_read_ready = True
                program_mem_read_data = program_memory.memory[self.fetcher.mem_read_address]
            else:
                program_mem_read_ready = False
                program_mem_read_data = 0

            # Scheduler manages control flow
            self.scheduler.execute(clk, reset, start, self.decoder.decoded_ret, self.fetcher.fetcher_state, [lsu.lsu_state for lsu in self.lsu], [pc.next_pc for pc in self.pc])
	    
            # Fetcher retrieves instruction
            self.fetcher.execute(clk, reset, self.scheduler.core_state, self.pc[0].next_pc, program_mem_read_ready, program_mem_read_data)

            # Decoder decodes instruction
            self.decoder.execute(clk, reset, self.scheduler.core_state, self.fetcher.instruction)

            # Execute ALU, LSU, Register File, and Program Counter for each thread
            for i in range(thread_count):
            
                self.alu[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_alu_arithmetic_mux, self.decoder.decoded_alu_output_mux, self.register_file[i].rs, self.register_file[i].rt)
                self.lsu[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_mem_read_enable, self.decoder.decoded_mem_write_enable, self.register_file[i].rs, self.register_file[i].rt, data_mem_read_ready[i], data_mem_read_data[i], data_mem_write_ready[i],data_memory)
                self.register_file[i].execute(clk, reset, True, block_id, self.scheduler.core_state, self.decoder.decoded_rd_address, self.decoder.decoded_rs_address, self.decoder.decoded_rt_address, self.decoder.decoded_reg_write_enable, self.decoder.decoded_reg_input_mux, self.decoder.decoded_immediate, self.alu[i].alu_out, self.lsu[i].lsu_out)
            
            
            
            
                self.pc[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_nzp, self.decoder.decoded_immediate, self.decoder.decoded_nzp_write_enable,self.decoder.decoded_pc_mux, self.alu[i].alu_out, self.pc[i].next_pc)
                #print("=======program_memory.memory[1]======")
                #print(self.pc[i].next_pc)
                #print(self.fetcher.mem_read_address)
                #print(bin(program_memory.memory[self.fetcher.mem_read_address]))
 

# Example usage of Core
if __name__ == "__main__":

    core_number = 1

    clk = True  # Clock placeholder
    reset = False
    start = True

    # Initialize the core with 4 threads per block
    core = Core(threads_per_block=core_number)

    # Placeholder values for the required inputs
    block_id = 0  # Block ID
    thread_count = core_number  # Number of threads per block
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
    
    #'''
    program = [
        0b0101000011011110, # MUL R0, %blockIdx, %blockDim           # self.registers[13] = 0 				# %blockIdx
                                                                     # self.registers[14] = self.THREADS_PER_BLOCK  	# %blockDim   
                                                                     # self.registers[15] = self.THREAD_ID 		# %threadIdx
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

    '''
    program = [
        0b0101000011011110, # MUL R0, %blockIdx, %blockDim
        0b0011000000001111, # ADD R0, R0, %threadIdx         ; i = blockIdx * blockDim + threadIdx
        0b1001000100000001, # CONST R1, #1                   ; increment
        0b1001001000000010, # CONST R2, #2                   ; N (matrix inner dimension)
        0b1001001100000000, # CONST R3, #0                   ; baseA (matrix A base address)
        0b1001010000000100, # CONST R4, #4                   ; baseB (matrix B base address)
        0b1001010100001000, # CONST R5, #8                   ; baseC (matrix C base address)
        0b0110011000000010, # DIV R6, R0, R2                 ; row = i // N
        0b0101011101100010, # MUL R7, R6, R2
        0b0100011100000111, # SUB R7, R0, R7                 ; col = i % N
        0b1001100000000000, # CONST R8, #0                   ; acc = 0
        0b1001100100000000, # CONST R9, #0                   ; k = 0
                            # LOOP:
        0b0101101001100010, #   MUL R10, R6, R2
        0b0011101010101001, #   ADD R10, R10, R9
        0b0011101010100011, #   ADD R10, R10, R3             ; addr(A[i]) = row * N + k + baseA
        0b0111101010100000, #   LDR R10, R10                 ; load A[i] from global memory
        0b0101101110010010, #   MUL R11, R9, R2
        0b0011101110110111, #   ADD R11, R11, R7
        0b0011101110110100, #   ADD R11, R11, R4             ; addr(B[i]) = k * N + col + baseB
        0b0111101110110000, #   LDR R11, R11                 ; load B[i] from global memory
        0b0101110010101011, #   MUL R12, R10, R11
        0b0011100010001100, #   ADD R8, R8, R12              ; acc = acc + A[i] * B[i]
        0b0011100110010001, #   ADD R9, R9, R1               ; increment k
        0b0010000010010010, #   CMP R9, R2
        0b0001100000001100, #   BRn LOOP                     ; loop while k < N
        0b0011100101010000, # ADD R9, R5, R0                 ; addr(C[i]) = baseC + i 
        0b1000000010011000, # STR R9, R8                     ; store C[i] in global memory
        0b1111000000000000  # RET                            ; end of kernel
    ]
    '''
    program_memory.load(program)

    # Initialize Data Memory
    data_memory = Memory(dut=dut, addr_bits=8, data_bits=8, channels=1, name="data")
    data = [
        0, 1, 2, 3, 4, 5, 6, 7,  # Matrix A (1 x 8)
        0, 1, 2, 3, 4, 5, 6, 7,  # Matrix B (1 x 8)
        0, 0, 0, 0, 0, 0, 0, 0  # Matrix C (1 x 8)
    ]
    data_memory.load(data)

    # Display loaded memories
    program_memory.display(rows=13, decimal=False)
    data_memory.display(rows=24, decimal=True)

#PROGRAM MEMORY
#+----------------+
#| Addr | Data     |
#+----------------+
#| 0    | 0101000011011110 ||
#| 1    | 0011000000001111 ||
#| 2    | 1001000100000000 ||
#| 3    | 1001001000001000 ||
#| 4    | 1001001100010000 ||
#| 5    | 0011010000010000 ||
#| 6    | 0111010001000000 ||
#| 7    | 0011010100100000 ||
#| 8    | 0111010101010000 ||
#| 9    | 0011011001000101 ||
#| 10   | 0011011100110000 ||
#| 11   | 1000000001110110 ||
#| 12   | 1111000000000000 ||
#+----------------+


#DATA MEMORY
#+----------------+
#| Addr | Data     |
#+----------------+
#| 0    | 0        |
#| 1    | 11       |
#| 2    | 22       |
#| 3    | 33       |
#| 4    | 44       |
#| 5    | 55       |
#| 6    | 66       |
#| 7    | 77       |
#| 8    | 0        |
#| 9    | 111      |
#| 10   | 222      |
#| 11   | 333      |
#| 12   | 444      |
#| 13   | 555      |
#| 14   | 666      |
#| 15   | 777      |
#+----------------+


    # Simulate clock cycles
    for cycle in range(860): # 9, 16, 23, 30, 37, 44, 51, 58, 65, 72, 79, 86
    
        core.execute(clk, reset, start, block_id=0, thread_count=core_number, program_memory=program_memory, data_memory=data_memory)
        
        # Toggle clock
        clk = not clk
        
        # Display the current PC, core state, and whether done
        print(f"Cycle {cycle}:")
        print(f"Current PC: {core.scheduler.current_pc}")
        print(f"Core State: {bin(core.scheduler.core_state)}")
        print(f"Done: {core.scheduler.done}")
        print(f"Fetcher state: {core.fetcher.fetcher_state}")
        print("-" * 30)
        
        if core.scheduler.core_state == 0b111:
            break
        
    data_memory.display(rows=24, decimal=True)
