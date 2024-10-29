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
    core = Core(threads_per_block=4)

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
    
        core.execute(clk, reset, start, block_id=0, thread_count=4, program_memory=program_memory, data_memory=data_memory)
        
        # Toggle clock
        clk = not clk
        
        # Display the current PC, core state, and whether done
        print(f"Cycle {cycle}:")
        print(f"Current PC: {core.scheduler.current_pc}")
        print(f"Core State: {bin(core.scheduler.core_state)}")
        print(f"Done: {core.scheduler.done}")
        print(f"Fetcher state: {core.fetcher.fetcher_state}")
        print("-" * 30)

