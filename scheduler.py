# SCHEDULER in Python
# This is a Python representation of the scheduler described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the scheduler hardware module.

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
        
        print("==============Scheduler[0]===============")
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
                print("IDLE")
                if start:
                    self.core_state = FETCH
                print("IDLE->FETCH")
            elif self.core_state == FETCH:
                print("FETCH")
                if fetcher_state == 'FETCHED':  # FETCHED state
                    print("fetcher_state == 'FETCHED'")
                    print("FETCH->DECODE")
                    self.core_state = DECODE
            elif self.core_state == DECODE:
                print("REQUEST")
                self.core_state = REQUEST
            elif self.core_state == REQUEST:
                print("WAIT")
                self.core_state = WAIT
            elif self.core_state == WAIT:
                any_lsu_waiting = any(lsu in [0b01, 0b10] for lsu in lsu_state)
                print("any_lsu_waiting")
                if not any_lsu_waiting:
                    print("EXECUTE")
                    self.core_state = EXECUTE
            elif self.core_state == EXECUTE:
                print("UPDATE")
                self.core_state = UPDATE
            elif self.core_state == UPDATE:
                if decoded_ret:
                    self.done = True
                    self.core_state = DONE
                else:
                    print("FETCH and current_pc")
                    self.current_pc = next_pc[-1]  # Assume all next_pc converge
                    self.core_state = FETCH
            elif self.core_state == DONE:
                pass  # No operation
                
        #print("============current_pc===================")
        #print(self.current_pc)


