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



