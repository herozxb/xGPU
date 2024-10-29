# MEMORY CONTROLLER in Python
# This is a Python representation of the memory controller described in the SystemVerilog code.
# The Python implementation uses a class to emulate the behavior of the memory controller hardware module.

class MemoryController:
    def __init__(self, addr_bits=8, data_bits=16, num_consumers=4, num_channels=1, write_enable=True):
        self.ADDR_BITS = addr_bits
        self.DATA_BITS = data_bits
        self.NUM_CONSUMERS = num_consumers
        self.NUM_CHANNELS = num_channels
        self.WRITE_ENABLE = write_enable
        self.reset()

    def reset(self):
        """Resets the memory controller state."""
        self.mem_read_valid = [0] * self.NUM_CHANNELS
        self.mem_read_address = [0] * self.NUM_CHANNELS
        self.mem_write_valid = [0] * self.NUM_CHANNELS
        self.mem_write_address = [0] * self.NUM_CHANNELS
        self.mem_write_data = [0] * self.NUM_CHANNELS

        self.consumer_read_ready = [0] * self.NUM_CONSUMERS
        self.consumer_read_data = [0] * self.NUM_CONSUMERS
        self.consumer_write_ready = [0] * self.NUM_CONSUMERS

        self.current_consumer = [0] * self.NUM_CHANNELS
        self.controller_state = [0] * self.NUM_CHANNELS
        self.channel_serving_consumer = [0] * self.NUM_CONSUMERS

    def execute(self, clk, reset, consumer_read_valid, consumer_read_address, consumer_write_valid, consumer_write_address, consumer_write_data, mem_read_ready, mem_read_data, mem_write_ready):
        """
        Simulates the behavior of the memory controller during each clock cycle.
        """
        IDLE = 0b000
        READ_WAITING = 0b010
        WRITE_WAITING = 0b011
        READ_RELAYING = 0b100
        WRITE_RELAYING = 0b101

        if reset:
            self.reset()
        else:
            for i in range(self.NUM_CHANNELS):
                if self.controller_state[i] == IDLE:
                    for j in range(self.NUM_CONSUMERS):
                        if consumer_read_valid[j] and not self.channel_serving_consumer[j]:
                            self.channel_serving_consumer[j] = 1
                            self.current_consumer[i] = j
                            self.mem_read_valid[i] = 1
                            self.mem_read_address[i] = consumer_read_address[j]
                            self.controller_state[i] = READ_WAITING
                            break
                        elif consumer_write_valid[j] and not self.channel_serving_consumer[j] and self.WRITE_ENABLE:
                            self.channel_serving_consumer[j] = 1
                            self.current_consumer[i] = j
                            self.mem_write_valid[i] = 1
                            self.mem_write_address[i] = consumer_write_address[j]
                            self.mem_write_data[i] = consumer_write_data[j]
                            self.controller_state[i] = WRITE_WAITING
                            break

                elif self.controller_state[i] == READ_WAITING:
                    if mem_read_ready[i]:
                        self.mem_read_valid[i] = 0
                        consumer_index = self.current_consumer[i]
                        self.consumer_read_ready[consumer_index] = 1
                        self.consumer_read_data[consumer_index] = mem_read_data[i]
                        self.controller_state[i] = READ_RELAYING

                elif self.controller_state[i] == WRITE_WAITING:
                    if mem_write_ready[i]:
                        self.mem_write_valid[i] = 0
                        consumer_index = self.current_consumer[i]
                        self.consumer_write_ready[consumer_index] = 1
                        self.controller_state[i] = WRITE_RELAYING

                elif self.controller_state[i] == READ_RELAYING:
                    consumer_index = self.current_consumer[i]
                    if not consumer_read_valid[consumer_index]:
                        self.channel_serving_consumer[consumer_index] = 0
                        self.consumer_read_ready[consumer_index] = 0
                        self.controller_state[i] = IDLE

                elif self.controller_state[i] == WRITE_RELAYING:
                    consumer_index = self.current_consumer[i]
                    if not consumer_write_valid[consumer_index]:
                        self.channel_serving_consumer[consumer_index] = 0
                        self.consumer_write_ready[consumer_index] = 0
                        self.controller_state[i] = IDLE

# Example usage
controller = MemoryController()
clk = True
reset = False
consumer_read_valid = [1, 0, 0, 0]
consumer_read_address = [15, 0, 0, 0]
consumer_write_valid = [0, 0, 0, 0]
consumer_write_address = [0, 0, 0, 0]
consumer_write_data = [0, 0, 0, 0]
mem_read_ready = [1]
mem_read_data = [25]
mem_write_ready = [1]

controller.execute(clk, reset, consumer_read_valid, consumer_read_address, consumer_write_valid, consumer_write_address, consumer_write_data, mem_read_ready, mem_read_data, mem_write_ready)
print(f"Consumer Read Ready: {controller.consumer_read_ready}")
print(f"Consumer Read Data: {controller.consumer_read_data}")

