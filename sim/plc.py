"""Two ways to talk to the E84 PLC program, with the same interface.

CompiledPLC  the ST program compiled by matiec into build/libe84.so; each scan() runs one
             10 ms PLC scan, so tests are exact and fast.
ModbusPLC    the same ST program running in OpenPLC; inputs and outputs travel over
             Modbus TCP (holding registers 1024 + n are the PLC's %MWn words).
"""
import ctypes
import socket
import struct
import time
from pathlib import Path

IN_BITS = ["VALID", "CS_0", "TR_REQ", "BUSY", "COMPT", "PRESENCE", "PLACEMENT",
           "IN_SERVICE", "UNLOAD_READY", "ESTOP_OK", "RESET"]
OUT_BITS = ["L_REQ", "U_REQ", "READY", "HO_AVBL", "ES", "CLAMP", "ALARM"]
STATES = {0: "idle", 10: "wait TR_REQ", 20: "wait BUSY", 30: "transferring", 40: "wait COMPT",
          50: "wait VALID off", 99: "ALARM"}
ALARMS = {0: "", 1: "TP1 timeout: no TR_REQ", 2: "TP2 timeout: no BUSY",
          3: "TP3 timeout: carrier not placed/removed", 4: "TP4 timeout: no COMPT",
          5: "TP5 timeout: VALID not released", 6: "carrier present but not seated", 11: "VALID or CS_0 lost during hand-off",
          12: "TR_REQ dropped before BUSY", 13: "BUSY dropped before carrier placed/removed",
          14: "carrier sensors changed after transfer", 80: "AMHS signal without VALID", 90: "emergency stop"}


def pack(inputs):
    return sum(1 << i for i, name in enumerate(IN_BITS) if inputs.get(name))


def unpack(word):
    return {name: bool((word >> i) & 1) for i, name in enumerate(OUT_BITS)}


class CompiledPLC:
    def __init__(self, lib=Path(__file__).resolve().parent.parent / "build" / "libe84.so"):
        self.lib = ctypes.CDLL(str(lib))
        self.lib.plc_get.restype = ctypes.c_int16
        self.lib.plc_set.argtypes = [ctypes.c_int, ctypes.c_int16]
        self.lib.plc_scan_ns.restype = ctypes.c_ulonglong
        self.scan_s = self.lib.plc_scan_ns() / 1e9
        self.lib.plc_init()
        self.t = 0.0

    def write(self, inputs):
        self.lib.plc_set(0, pack(inputs))

    def read(self):
        out = unpack(self.lib.plc_get(1) & 0xFFFF)
        out.update(ALARM_CODE=self.lib.plc_get(2), STATE=self.lib.plc_get(3), HANDOFFS=self.lib.plc_get(4))
        return out

    def scan(self):
        self.lib.plc_scan()
        self.t += self.scan_s


class ModbusPLC:
    """Tiny Modbus TCP client: function 3 (read holding registers) and 16 (write registers)."""
    BASE = 1024                                    # %MW0

    def __init__(self, host="127.0.0.1", port=502, unit=1, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.unit, self.tid = unit, 0
        self.t0 = time.monotonic()

    def _request(self, pdu):
        self.tid = (self.tid + 1) & 0xFFFF
        self.sock.sendall(struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, self.unit) + pdu)
        head = self._recv(7)
        tid, _, length, _ = struct.unpack(">HHHB", head)
        body = self._recv(length - 1)
        if body[0] & 0x80:
            raise IOError(f"Modbus exception code {body[1]}")
        return body

    def _recv(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("PLC closed the connection")
            buf += chunk
        return buf

    def write(self, inputs):
        self._request(struct.pack(">BHHBH", 16, self.BASE, 1, 2, pack(inputs)))

    def read_words(self, start, count):
        body = self._request(struct.pack(">BHH", 3, self.BASE + start, count))
        return list(struct.unpack(f">{count}h", body[2:2 + 2 * count]))

    def read(self):
        w = self.read_words(1, 4)
        out = unpack(w[0] & 0xFFFF)
        out.update(ALARM_CODE=w[1], STATE=w[2], HANDOFFS=w[3])
        return out

    @property
    def t(self):
        return time.monotonic() - self.t0

    def scan(self):
        time.sleep(0.02)                           # the PLC scans on its own every 10 ms
