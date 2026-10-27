"""Quick checks on the compiled ST program (a few seconds). Run after `make`: python sim/tests.py"""
from plc import CompiledPLC, pack, unpack, IN_BITS
from oht import Vehicle, Port
from runner import run, settle

passed = 0


def test(name, cond):
    global passed
    assert cond, name
    passed += 1
    print(f"ok  {name}")


def idle(port):
    d = dict(VALID=False, CS_0=False, TR_REQ=False, BUSY=False, COMPT=False); d.update(port.inputs())
    return d


test("input bits pack in the documented order", pack({"VALID": 1, "ESTOP_OK": 1}) == 1 + 512 and
     pack({n: 1 for n in IN_BITS}) == 2047)
test("output word unpacks", unpack(1 + 4 + 64) == {"L_REQ": True, "U_REQ": False, "READY": True, "HO_AVBL": False,
                                                   "ES": False, "CLAMP": False, "ALARM": True})

plc = CompiledPLC()
test("task interval is 10 ms", abs(plc.scan_s - 0.01) < 1e-12)
out = settle(plc, {}, 0.05)
test("a dead E-stop input at power-up latches alarm 90", out["ALARM"] and out["ALARM_CODE"] == 90 and not out["ES"])
port = Port()
settle(plc, idle(port), 0.05)
test("alarm stays latched until RESET", plc.read()["ALARM"])
port.reset = True; settle(plc, idle(port), 0.05); port.reset = False
out = settle(plc, idle(port), 0.05)
test("RESET with healthy inputs returns to idle and offers hand-offs", out["STATE"] == 0 and out["HO_AVBL"])

res, ev = run(plc, Vehicle(port, "load"), 30)
test("a nominal load completes and clamps the carrier", res["done"] and plc.read()["CLAMP"] and plc.read()["HANDOFFS"] == 1)

# TP1: raise VALID + CS_0 and never send TR_REQ
plc = CompiledPLC(); port = Port(); settle(plc, idle(port), 0.05)
inp = idle(port); inp.update(VALID=True, CS_0=True)
t_req = None
while plc.t < 5:
    plc.write(inp); plc.scan(); o = plc.read()
    if o["L_REQ"] and t_req is None:
        t_req = plc.t
    if o["ALARM"]:
        break
test("TP1 fires 2 s after L_REQ (within one scan)", o["ALARM_CODE"] == 1 and 2.0 <= plc.t - t_req <= 2.02)
test("L_REQ and HO_AVBL are already off in the alarm scan", not o["L_REQ"] and not o["HO_AVBL"])
print(f"\n{passed} tests passed")
