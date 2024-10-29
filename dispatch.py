# BLOCK DISPATCH in Python
# This is a Python representation of the block dispatch unit described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the dispatch hardware module.

class BlockDispatch:
    def __init__(self, num_cores=2, threads_per_block=4):
        self.NUM_CORES = num_cores
        self.THREADS_PER_BLOCK = threads_per_block
        self.reset()

    def reset(self):
        """Resets the block dispatch state."""
        self.done = 0
        self.blocks_dispatched = 0
        self.blocks_done = 0
        self.start_execution = False
        self.core_start = [0] * self.NUM_CORES
        self.core_reset = [1] * self.NUM_CORES
        self.core_block_id = [0] * self.NUM_CORES
        self.core_thread_count = [self.THREADS_PER_BLOCK] * self.NUM_CORES

    def execute(self, clk, reset, start, thread_count, core_done):
        """
        Simulates the behavior of the block dispatch unit during each clock cycle.
        """
        total_blocks = (thread_count + self.THREADS_PER_BLOCK - 1) // self.THREADS_PER_BLOCK

        if reset:
            self.reset()
        elif start:
            # EDA: Indirect way to get @(posedge start) without driving from 2 different clocks
            if not self.start_execution:
                self.start_execution = True
                for i in range(self.NUM_CORES):
                    self.core_reset[i] = 1

            # If the last block has finished processing, mark this kernel as done executing
            if self.blocks_done == total_blocks:
                self.done = 1

            for i in range(self.NUM_CORES):
                if self.core_reset[i]:
                    self.core_reset[i] = 0

                    # If this core was just reset, check if there are more blocks to be dispatched
                    if self.blocks_dispatched < total_blocks:
                        self.core_start[i] = 1
                        self.core_block_id[i] = self.blocks_dispatched
                        self.core_thread_count[i] = (
                            thread_count - (self.blocks_dispatched * self.THREADS_PER_BLOCK)
                            if self.blocks_dispatched == total_blocks - 1
                            else self.THREADS_PER_BLOCK
                        )
                        self.blocks_dispatched += 1

            for i in range(self.NUM_CORES):
                if self.core_start[i] and core_done[i]:
                    # If a core just finished executing its current block, reset it
                    self.core_reset[i] = 1
                    self.core_start[i] = 0
                    self.blocks_done += 1

# Example usage
dispatch = BlockDispatch()
clk = True
reset = False
start = True
thread_count = 10
core_done = [0, 0]

# Simulate clock cycles
dispatch.execute(clk, reset, start, thread_count, core_done)
print(f"Core Start: {dispatch.core_start}")
print(f"Core Block ID: {dispatch.core_block_id}")
print(f"Core Thread Count: {dispatch.core_thread_count}")
print(f"Done: {dispatch.done}")

