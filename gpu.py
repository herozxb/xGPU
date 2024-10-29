# GPU in Python
# This is a Python representation of the GPU described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the GPU hardware module.

class GPU:
    def __init__(self, data_mem_addr_bits=8, data_mem_data_bits=8, data_mem_num_channels=4,
                 program_mem_addr_bits=8, program_mem_data_bits=16, program_mem_num_channels=1,
                 num_cores=2, threads_per_block=4):
        self.DATA_MEM_ADDR_BITS = data_mem_addr_bits
        self.DATA_MEM_DATA_BITS = data_mem_data_bits
        self.DATA_MEM_NUM_CHANNELS = data_mem_num_channels
        self.PROGRAM_MEM_ADDR_BITS = program_mem_addr_bits
        self.PROGRAM_MEM_DATA_BITS = program_mem_data_bits
        self.PROGRAM_MEM_NUM_CHANNELS = program_mem_num_channels
        self.NUM_CORES = num_cores
        self.THREADS_PER_BLOCK = threads_per_block
        self.reset()

    def reset(self):
        """Resets the GPU state."""
        self.done = 0
        self.thread_count = 0
        self.core_start = [0] * self.NUM_CORES
        self.core_reset = [1] * self.NUM_CORES
        self.core_done = [0] * self.NUM_CORES
        self.core_block_id = [0] * self.NUM_CORES
        self.core_thread_count = [self.THREADS_PER_BLOCK] * self.NUM_CORES
        self.lsu_read_valid = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_read_address = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_read_ready = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_read_data = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_write_valid = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_write_address = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_write_data = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.lsu_write_ready = [0] * (self.NUM_CORES * self.THREADS_PER_BLOCK)
        self.fetcher_read_valid = [0] * self.NUM_CORES
        self.fetcher_read_address = [0] * self.NUM_CORES
        self.fetcher_read_ready = [0] * self.NUM_CORES
        self.fetcher_read_data = [0] * self.NUM_CORES

    def execute(self, clk, reset, start, device_control_write_enable, device_control_data,
                program_mem_read_ready, program_mem_read_data, data_mem_read_ready, data_mem_read_data,
                data_mem_write_ready):
        """
        Simulates the behavior of the GPU during each clock cycle.
        """
        if reset:
            self.reset()
        else:
            # Device Control Register Update
            if device_control_write_enable:
                self.thread_count = device_control_data

            # Dispatcher Logic
            total_blocks = (self.thread_count + self.THREADS_PER_BLOCK - 1) // self.THREADS_PER_BLOCK

            for i in range(self.NUM_CORES):
                if self.core_reset[i]:
                    self.core_reset[i] = 0
                    if self.core_block_id[i] < total_blocks:
                        self.core_start[i] = 1
                        self.core_block_id[i] = self.core_block_id[i] + 1
                        self.core_thread_count[i] = (
                            self.thread_count - (self.core_block_id[i] * self.THREADS_PER_BLOCK)
                            if self.core_block_id[i] == total_blocks - 1
                            else self.THREADS_PER_BLOCK
                        )

            for i in range(self.NUM_CORES):
                if self.core_done[i]:
                    self.core_reset[i] = 1
                    self.core_start[i] = 0

# Example usage
gpu = GPU()
clk = True
reset = False
start = True
device_control_write_enable = True
device_control_data = 10
program_mem_read_ready = [1]
program_mem_read_data = [42]
data_mem_read_ready = [1, 1, 1, 1]
data_mem_read_data = [10, 20, 30, 40]
data_mem_write_ready = [1, 1, 1, 1]

gpu.execute(clk, reset, start, device_control_write_enable, device_control_data,
            program_mem_read_ready, program_mem_read_data, data_mem_read_ready, data_mem_read_data,
            data_mem_write_ready)
print(f"Core Start: {gpu.core_start}")
print(f"Core Block ID: {gpu.core_block_id}")
print(f"Core Thread Count: {gpu.core_thread_count}")
print(f"Done: {gpu.done}")

