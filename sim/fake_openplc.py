"""A stand-in for OpenPLC's Modbus server, backed by the compiled ST program.

It scans the PLC every 10 ms in real time and serves holding registers 1024+ (%MW0...) over
Modbus TCP, so live.py can be tested without Docker. With OpenPLC running, live.py talks to the
real runtime instead.

    python sim/fake_openplc.py --port 5020
"""
import argparse
import socketserver
import struct
import threading
import time
from plc import CompiledPLC

LOCK = threading.Lock()
PLC = None


def scanner():
    while True:
        with LOCK:
            PLC.scan()
        time.sleep(PLC.scan_s)


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        while True:
            head = self.request.recv(7)
            if len(head) < 7:
                return
            tid, proto, length, unit = struct.unpack(">HHHB", head)
            pdu = self.request.recv(length - 1)
            fc = pdu[0]
            if fc == 3:
                start, count = struct.unpack(">HH", pdu[1:5])
                with LOCK:
                    vals = [PLC.lib.plc_get(start - 1024 + i) for i in range(count)]
                body = struct.pack(">BB", 3, 2 * count) + struct.pack(f">{count}h", *vals)
            elif fc == 16:
                start, count, nbytes = struct.unpack(">HHB", pdu[1:6])
                vals = struct.unpack(f">{count}h", pdu[6:6 + nbytes])
                with LOCK:
                    for i, v in enumerate(vals):
                        PLC.lib.plc_set(start - 1024 + i, v)
                body = struct.pack(">BHH", 16, start, count)
            else:
                body = struct.pack(">BB", fc | 0x80, 1)
            self.request.sendall(struct.pack(">HHHB", tid, 0, len(body) + 1, unit) + body)


def serve(port):
    global PLC
    PLC = CompiledPLC()
    threading.Thread(target=scanner, daemon=True).start()
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--port", type=int, default=5020)
    a = ap.parse_args()
    serve(a.port)
    print(f"fake OpenPLC Modbus server on 127.0.0.1:{a.port}; Ctrl+C to stop")
    while True:
        time.sleep(1)
