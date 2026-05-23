"""Diagnostic: sweep baudrates and send a get_system_version frame. Useful when no response."""
import time
from serial import Serial
from serial.tools import list_ports

FRAME = b"\xfe\xfe\x02\x02\xfa"  # get_system_version

ports = [p.device for p in list_ports.comports()]
if not ports:
    raise SystemExit("no serial ports detected")
port = ports[0]
print("port:", port)

for baud in (115200, 1000000, 921600, 460800, 230400, 57600, 9600):
    try:
        s = Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"  {baud}: open err {e}"); time.sleep(0.3); continue
    s.dtr = False; s.rts = False
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(FRAME); s.flush()
    time.sleep(0.4)
    r = s.read(64)
    print(f"  {baud}: {r.hex() or '(no response)'}")
    s.close(); time.sleep(0.2)
