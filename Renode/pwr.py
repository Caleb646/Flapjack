# from dataclasses import dataclass
# @dataclass
# class RenodeRequest:
#     isInit bool        # Is true during construction of the peripheral if its marked as initable
#     isRead bool        # Is true when the CPU is trying to read from a peripheral
#     isWrite bool       # Is true when the CPU is trying to write to a peripheral
#     value int          # When isWrite == true, this is the value to be written, when isRead == true this is the return value
#     offset int         # Offset within the peripheral
#     type int           # Width of the access (8bit, 16bit, 32bit)
#     self                # The peripheral itself
#     size int           # Size of the peripheral as a hexadecimal value


cache = {}
if request.isRead: 
    if request.offset == 0x4: # PWR Status Register
        request.value = 0x2000
    if request.offset in cache:
        request.value = cache[request.offset]
elif request.isWrite:
    cache[request.offset] = request.value


# pwr_csr1: Python.PythonPeripheral @ sysbus 0x58024804
#     size: 0x4
#     initable: false
#     script: "request.value = 0x2000"
#     // filename: "my_pyscript.py"