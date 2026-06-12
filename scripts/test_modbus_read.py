# pip install pyserial
# pip install pymodbus
import time
from pymodbus.client import ModbusSerialClient

slave_id = 126  # Device ID

print("modbus read test")
client = ModbusSerialClient(
    "/dev/ttyUSB0",
    baudrate=460800,
    parity="N",
    stopbits=1,
    bytesize=8,
    timeout=1,
)
client.connect()
time.sleep(1.0)
print(f"sleep 1.0 s")

def read_and_print(read_func, label, desc, **kwargs):
    """Read registers and print the result."""
    result = read_func(**kwargs)
    if result.isError():
        print(f"  [{label}] {desc} - Read failed: {result}")
    else:
        print(f"  [{label}] {desc}: {result.registers}")


# Read task list: (read_func, label, desc, kwargs)
read_tasks = [
    (client.read_input_registers,   "Input@901",   "Hand Type",    {"address": 901,  "count": 1,  "device_id": slave_id}),
    # (client.read_holding_registers, "Holding@3000", "FW Version",   {"address": 3000, "count": 20,  "device_id": slave_id}),
    # (client.read_input_registers,   "Input@2000",   "Motor Status", {"address": 2000, "count": 24, "device_id": slave_id}),
]

for i in range(10):
    print(f"Read #{i+1}...")
    start_time = time.time()
    for read_func, label, desc, kwargs in read_tasks:
        read_and_print(read_func, label, desc, **kwargs)
    elapsed_ms = (time.time() - start_time) * 1000
    print(f"Cost: {elapsed_ms:.2f} ms\n")

client.close()
