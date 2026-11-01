"""Stride CAN host: talk to the leg through the CANable.

Mirrors src/stride_can.h. Sends a heartbeat at 10 Hz, decodes STATUS/FEEDBACK, takes commands on stdin.

Setup (pip install python-can):
  candleLight firmware on Linux -> native SocketCAN:
      sudo ip link set can0 up type can bitrate 500000
      python can_host.py --interface socketcan --channel can0
  slcan firmware (shows up as a serial port):
      python can_host.py --interface slcan --channel /dev/ttyACM1
  Windows + candleLight: --interface gs_usb --channel 0   (needs pyusb + libusb)
Use whichever worked for your Arduino test.

Commands: enable | disable | estop | t <rad> | vlim <volts> | fb (toggle feedback print) | quit
"""
import argparse, struct, sys, threading, time
import can

BITRATE = 500_000
NODES = {"knee": 1, "hip": 2}
ID_ESTOP = 0x000
CMD = dict(heartbeat=0x00, disable=0x01, enable=0x02, set_target=0x03, set_vlimit=0x04)
MSG_STATUS, MSG_FEEDBACK = 0x10, 0x11
STATES = {0: "IDLE", 1: "OPEN_LOOP", 2: "CLOSED_LOOP", 3: "ENCODER"}
FAULTS = ["ENCODER_LOST", "DEADMAN", "HOST_TIMEOUT", "CALIB_FAILED", "BAD_FRAME"]

def make_id(node, cmd): return (node << 5) | (cmd & 0x1F)

def decode_status(d):
    state, faults, last_cmd, rx = d[0], d[1], d[2], d[3]
    i2c_err, age = struct.unpack_from("<HH", d, 4)
    names = [n for i, n in enumerate(FAULTS) if faults & (1 << i)] or ["none"]
    return (f"STATUS {STATES.get(state, state)}  faults={','.join(names)}  "
            f"last_cmd=0x{last_cmd:02X}  rx={rx}  i2c_err={i2c_err}  host_age={age}ms")

def decode_feedback(d):
    angle, vel = struct.unpack("<ff", bytes(d[:8]))
    return angle, vel

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--node", default="knee", choices=NODES)
    a = ap.parse_args()
    node = NODES[a.node]
    bus = can.Bus(interface=a.interface, channel=a.channel, bitrate=BITRATE)
    send_lock = threading.Lock()
    show_fb = [False]
    running = [True]

    def send(cmd, payload=b""):
        msg = can.Message(arbitration_id=make_id(node, cmd), data=payload, is_extended_id=False)
        with send_lock:
            bus.send(msg)

    def heartbeat():
        while running[0]:
            try: send(CMD["heartbeat"])
            except can.CanError as e: print("heartbeat send failed:", e)
            time.sleep(0.1)

    def rx():
        last_fb_print = 0.0
        while running[0]:
            m = bus.recv(0.2)
            if m is None or m.is_extended_id: continue
            src, cmd = m.arbitration_id >> 5, m.arbitration_id & 0x1F
            if src != node: continue
            if cmd == MSG_STATUS and len(m.data) == 8:
                print(decode_status(m.data))
            elif cmd == MSG_FEEDBACK and len(m.data) == 8 and show_fb[0]:
                if time.time() - last_fb_print > 0.1:          # leg sends 50 Hz, print 10 Hz
                    angle, vel = decode_feedback(m.data)
                    print(f"FEEDBACK angle={angle:+.3f} rad  vel={vel:+.2f} rad/s")
                    last_fb_print = time.time()

    threading.Thread(target=heartbeat, daemon=True).start()
    threading.Thread(target=rx, daemon=True).start()
    print(f"connected ({a.interface} {a.channel}), node {a.node}={node}. Commands: enable disable estop t <rad> vlim <V> fb quit")
    try:
        for line in sys.stdin:
            p = line.split()
            if not p: continue
            c = p[0].lower()
            try:
                if c == "enable":   send(CMD["enable"])
                elif c == "disable": send(CMD["disable"])
                elif c == "estop":
                    with send_lock:
                        bus.send(can.Message(arbitration_id=ID_ESTOP, data=b"", is_extended_id=False))
                elif c == "t" and len(p) == 2:    send(CMD["set_target"], struct.pack("<f", float(p[1])))
                elif c == "vlim" and len(p) == 2: send(CMD["set_vlimit"], struct.pack("<f", float(p[1])))
                elif c == "fb":   show_fb[0] = not show_fb[0]
                elif c in ("quit", "q"): break
                else: print("? enable | disable | estop | t <rad> | vlim <V> | fb | quit")
            except (ValueError, can.CanError) as e:
                print("error:", e)
    finally:
        running[0] = False
        try: send(CMD["disable"])
        except Exception: pass
        bus.shutdown()

if __name__ == "__main__":
    main()
