"""Drive the E84 program running in OpenPLC (or fake_openplc.py) over Modbus TCP, in real time.

    python sim/live.py --host 127.0.0.1 --port 502

Runs a load, an unload, then a fault (BUSY dropped early) followed by an operator reset,
and prints every signal edge with its time, like a logic analyser.
"""
import argparse
from plc import ModbusPLC, ALARMS, STATES
from oht import Vehicle, Port
from runner import run, settle


def show(events, t0):
    for t, side, sig, val in events:
        print(f"    {t - t0:7.2f} s  {side:4s}  {sig:8s} {'ON' if val else 'off'}")


def idle(port):
    d = dict(VALID=False, CS_0=False, TR_REQ=False, BUSY=False, COMPT=False); d.update(port.inputs())
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=502)
    a = ap.parse_args()
    plc = ModbusPLC(a.host, a.port)
    port = Port(carrier=False)
    # start from a clean state: sensors healthy, alarm (if any) reset
    settle(plc, idle(port), 0.3)
    port.reset = True; settle(plc, idle(port), 0.3); port.reset = False; settle(plc, idle(port), 0.3)
    out = plc.read()
    print(f"connected to {a.host}:{a.port}: state {STATES.get(out['STATE'])}, HO_AVBL {out['HO_AVBL']}, "
          f"hand-offs so far {out['HANDOFFS']}")
    ok = True
    for job in ("load", "unload"):
        if job == "unload":
            settle(plc, idle(port), 0.5); port.unload_ready = True
        t0 = plc.t
        res, ev = run(plc, Vehicle(port, job), 30)
        settle(plc, idle(port), 0.2)
        out = plc.read()
        print(f"\n  {job}: {'complete' if res['done'] else 'FAILED: ' + str(res['error'])}; CLAMP {out['CLAMP']}")
        show(ev, t0)
        ok &= res["done"]
    print("\n  fault: BUSY dropped before the carrier was placed")
    t0 = plc.t
    v = Vehicle(port, "load", fault="busy_early")
    res, ev = run(plc, v, 10)
    out = plc.read()
    show(ev, t0)
    print(f"    PLC alarm {out['ALARM_CODE']} ({ALARMS.get(out['ALARM_CODE'], '?')}), "
          f"{1000 * (res['alarm_t'] - v.fault_t):.0f} ms after the fault (includes the Modbus round trip)")
    ok &= out["ALARM_CODE"] == 13
    v.release(plc.t)
    settle(plc, idle(port), 0.3)
    port.reset = True; settle(plc, idle(port), 0.3); port.reset = False; settle(plc, idle(port), 0.3)
    out = plc.read()
    print(f"    after RESET: state {STATES.get(out['STATE'])}, HO_AVBL {out['HO_AVBL']}")
    ok &= out["STATE"] == 0 and out["HO_AVBL"]
    print(f"\nlive run {'passed' if ok else 'FAILED'}; PLC hand-off counter {out['HANDOFFS']}")
    return 0 if ok else 1


HINT = """
Could not talk to the PLC's Modbus server. OpenPLC only serves Modbus while a program is RUNNING:
  1. http://localhost:8080 (openplc / openplc) -> Programs -> Upload Program -> openplc/e84_sim.st -> Launch program
  2. wait for "Compilation finished successfully", then Dashboard -> Start PLC (status: Running)
  3. Settings -> Modbus Server enabled on port 502 (it is by default)
Then run this again."""

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConnectionError, OSError) as e:
        print(f"{type(e).__name__}: {e}{HINT}")
        raise SystemExit(2)
