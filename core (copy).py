from fetcher import Fetcher
from decoder import Decoder
from scheduler import Scheduler
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

    def execute(self, clk, reset, start, block_id, thread_count, program_mem_read_ready, program_mem_read_data,
                data_mem_read_ready, data_mem_read_data, data_mem_write_ready):
        if reset:
            self.reset()
        else:
            # Scheduler manages control flow
            self.scheduler.execute(clk, reset, start, self.fetcher.fetcher_state, self.decoder.decoded_mem_read_enable,self.decoder.decoded_mem_write_enable, self.decoder.decoded_ret, [lsu.lsu_state for lsu in self.lsu])

            # Fetcher retrieves instruction
            self.fetcher.execute(clk, reset, self.scheduler.core_state, self.pc[0].next_pc, program_mem_read_ready, program_mem_read_data)

            # Decoder decodes instruction
            self.decoder.execute(clk, reset, self.scheduler.core_state, self.fetcher.instruction)

            # Execute ALU, LSU, Register File, and Program Counter for each thread
            for i in range(thread_count):
                self.alu[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_alu_arithmetic_mux,
                                    self.decoder.decoded_alu_output_mux, self.register_file[i].rs, self.register_file[i].rt)
                self.lsu[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_mem_read_enable,
                                    self.decoder.decoded_mem_write_enable, self.register_file[i].rs, self.register_file[i].rt,
                                    data_mem_read_ready[i], data_mem_read_data[i], data_mem_write_ready[i])
                self.register_file[i].execute(clk, reset, True, block_id, self.scheduler.core_state,
                                              self.decoder.decoded_rd_address, self.decoder.decoded_rs_address,
                                              self.decoder.decoded_rt_address, self.decoder.decoded_reg_write_enable,
                                              self.decoder.decoded_reg_input_mux, self.decoder.decoded_immediate,
                                              self.alu[i].alu_out, self.lsu[i].lsu_out)
                self.pc[i].execute(clk, reset, True, self.scheduler.core_state, self.decoder.decoded_nzp,
                                   self.decoder.decoded_immediate, self.decoder.decoded_nzp_write_enable,
                                   self.decoder.decoded_pc_mux, self.alu[i].alu_out, self.pc[i].next_pc)



# Example usage of Core
if __name__ == "__main__":
    clk = 1  # Clock placeholder
    reset = False
    start = True

    # Initialize the core with 4 threads per block
    core = Core(threads_per_block=4)

    # Placeholder values for the required inputs
    block_id = 0  # Block ID
    thread_count = 4  # Number of threads per block
    program_mem_read_ready = True  # Program memory read is ready (placeholder)
    program_mem_read_data = 0  # Placeholder for program memory read data

    # Data memory read/write readiness and data for each thread (assuming 4 threads per block)
    data_mem_read_ready = [True for _ in range(4)]  # Placeholder for data memory read readiness
    data_mem_read_data = [0 for _ in range(4)]  # Placeholder for data memory read data
    data_mem_write_ready = [True for _ in range(4)]  # Placeholder for data memory write readiness

    # Simulate clock cycles
    for cycle in range(10):
        core.execute(clk, reset, start, block_id, thread_count, program_mem_read_ready, program_mem_read_data, data_mem_read_ready, data_mem_read_data, data_mem_write_ready)
        # Toggle clock
        clk = not clk
