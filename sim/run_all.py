"""Every E84 scenario against the compiled ST program: nominal hand-offs, 12 injected faults,
recovery, an out-of-service port and a 100-hand-off endurance run. Prints the requirement table.

    make && python sim/run_all.py | tee results.txt
Exit code 1 if any requirement fails.
"""
import sys
from pathlib import Path
from plc import CompiledPLC, ALARMS, STATES
from oht import Vehicle, Port, FAULTS, HOIST_S
from runner import run, settle

req = []
SCAN = None


def check(rid, text, value, limit, ok):
    req.append((rid, text, value, limit, ok))


EXPECTED = {
    "load": ["CS_0+", "VALID+", "L_REQ+", "TR_REQ+", "READY+", "BUSY+", "L_REQ-", "BUSY-", "TR_REQ-",
             "COMPT+", "READY-", "COMPT-", "VALID-", "CS_0-"],
    "unload": ["CS_0+", "VALID+", "U_REQ+", "TR_REQ+", "READY+", "BUSY+", "U_REQ-", "BUSY-", "TR_REQ-",
               "COMPT+", "READY-", "COMPT-", "VALID-", "CS_0-"],
}
HANDSHAKE = {"CS_0", "VALID", "TR_REQ", "BUSY", "COMPT", "L_REQ", "U_REQ", "READY"}


def sequence(events):
    return [f"{s}{'+' if v else '-'}" for _, _, s, v in events if s in HANDSHAKE]


def new_plc():
    plc = CompiledPLC()
    return plc


def idle_inputs(port):
    d = dict(VALID=False, CS_0=False, TR_REQ=False, BUSY=False, COMPT=False); d.update(port.inputs())
    return d


plc = new_plc()
SCAN = plc.scan_s
print(f"PLC: E84_LOAD_PORT compiled from IEC 61131-3 ST by matiec; task interval {1000 * SCAN:.0f} ms")

# ------------------------------------------------------------------ 1. nominal
print("\n[1  Nominal hand-offs]")
port = Port(carrier=False)
settle(plc, idle_inputs(port), 0.2)
nominal = {}
for job in ("load", "unload"):
    if job == "unload":
        port.unload_ready = True
    v = Vehicle(port, job)
    t0 = plc.t
    res, ev = run(plc, v, 30)
    seq = sequence(ev)
    ok = res["done"] and seq == EXPECTED[job]
    nominal[job] = (ok, ev, t0)
    cs_on = next(t for t, _, s, val in ev if s == "CS_0" and val)
    cs_off = max(t for t, _, s, val in ev if s == "CS_0" and not val)
    out = plc.read()
    print(f"  {job:6s}: {'complete' if res['done'] else 'FAILED'}, {len(seq)} signal edges in E84 order: "
          f"{'yes' if seq == EXPECTED[job] else 'NO'}, hand-off took {cs_off - cs_on:.2f} s "
          f"({HOIST_S:.0f} s of it hoist travel), CLAMP after: {out['CLAMP']}")
    print("    " + " ".join(seq))
check("R1", "Load and unload complete with E84 signal order", "2/2" if all(n[0] for n in nominal.values()) else "fail",
      "2/2", all(n[0] for n in nominal.values()))

# ------------------------------------------------------------------ 2. faults
print("\n[2  Injected faults: what the PLC flagged and how fast]")
TIMEOUT = {"no_tr_req": (1, 2.0), "no_busy": (2, 2.0), "hoist_jam": (3, 60.0), "tilted": (6, 60.0),
           "no_compt": (4, 60.0), "valid_stuck": (5, 2.0)}
PROTOCOL = {"valid_drop": 11, "tr_req_drop": 12, "busy_early": 13, "sensor_flicker": 14, "no_valid": 80, "estop": 90}
print("  timeouts: time from entering the waiting state to the alarm; violations: time from the bad signal to the alarm")
print(f"  {'fault':16s} {'expected':>8s} {'got':>5s} {'detected after':>15s} {'safe outputs':>13s}  recovery")
fault_ok, timing_ok, viol_ok, recov_ok, safe_ok = [], [], [], [], []
estop_latency = None
traces = {}
for name in list(TIMEOUT) + list(PROTOCOL):
    plc = new_plc()
    port = Port(carrier=False)
    settle(plc, idle_inputs(port), 0.2)
    v = Vehicle(port, "load", fault=name)
    tr = []
    res, ev = run(plc, v, 90, trace=tr)
    traces[name] = (tr, ev)
    out = plc.read()
    code = out["ALARM_CODE"]
    want = TIMEOUT[name][0] if name in TIMEOUT else PROTOCOL[name]
    lat = res["alarm_t"] - v.fault_t if res["alarm_t"] is not None and v.fault_t is not None else None
    safe = not (out["L_REQ"] or out["U_REQ"] or out["READY"] or out["HO_AVBL"])
    if name == "estop":
        safe = safe and not out["ES"]
        estop_latency = lat
    if name in TIMEOUT:
        # a timeout is measured from the moment the PLC entered the state whose timer ran out
        states = [(x[0], x[2]["STATE"]) for x in tr]
        waiting = [st for _, st in states if st != 99][-1]
        k = len(states) - 1
        while k > 0 and states[k][1] == 99:
            k -= 1
        while k > 0 and states[k - 1][1] == waiting:
            k -= 1
        lat = res["alarm_t"] - states[k][0]
        tp = TIMEOUT[name][1]
        ok_t = tp <= lat <= tp + 3 * SCAN
        timing_ok.append(ok_t)
        lat_s = f"{lat:.2f} s (TP {tp:.0f})"
    else:
        ok_t = lat is not None and lat <= 2 * SCAN + 1e-9
        viol_ok.append(ok_t)
        lat_s = f"{1000 * lat:.0f} ms" if lat is not None else "never"
    # recovery: vehicle releases, operator clears the port and presses RESET, then a clean load
    v.release(plc.t)
    port.present = port.seated = False; port.unload_ready = False; port.estop_ok = True
    settle(plc, idle_inputs(port), 0.3)
    port.reset = True; settle(plc, idle_inputs(port), 0.1); port.reset = False
    settle(plc, idle_inputs(port), 0.1)
    v2 = Vehicle(port, "load")
    r2, _ = run(plc, v2, 30)
    rec = r2["done"] and plc.read()["HANDOFFS"] == 1
    fault_ok.append(code == want); safe_ok.append(safe); recov_ok.append(rec)
    print(f"  {name:16s} {want:8d} {code:5d} {lat_s:>15s} {'yes' if safe else 'NO':>13s}  {'ok' if rec else 'FAILED'}"
          f"   {ALARMS.get(code, '?')}")
n = len(fault_ok)
check("R2", "Each injected fault raises the right alarm code", f"{sum(fault_ok)}/{n}", f"{n}/{n}", all(fault_ok))
check("R3", "Timeout faults flagged at their timer (within +30 ms)", f"{sum(timing_ok)}/{len(timing_ok)}",
      f"{len(timing_ok)}/{len(timing_ok)}", all(timing_ok))
check("R4", "Protocol violations flagged within 2 scans (20 ms)", f"{sum(viol_ok)}/{len(viol_ok)}",
      f"{len(viol_ok)}/{len(viol_ok)}", all(viol_ok))
check("R5", "L_REQ, U_REQ, READY, HO_AVBL off on every alarm; ES off on E-stop", f"{sum(safe_ok)}/{n}", f"{n}/{n}", all(safe_ok))
check("R6", "Reset, then a clean hand-off, after every fault", f"{sum(recov_ok)}/{n}", f"{n}/{n}", all(recov_ok))

# ------------------------------------------------------------------ 3. out of service
print("\n[3  Port out of service]")
plc = new_plc()
port = Port(carrier=False); port.in_service = False
settle(plc, idle_inputs(port), 0.2)
v = Vehicle(port, "load")
res, ev = run(plc, v, 10)
out = plc.read()
oos = (not out["HO_AVBL"]) and not any(s == "L_REQ" for _, side, s, _ in ev if side == "PORT") and not out["ALARM"] \
      and res["error"] is not None
print(f"  HO_AVBL {out['HO_AVBL']}, L_REQ raised: {any(s == 'L_REQ' for _, _, s, _ in ev)}, PLC alarm: {out['ALARM']}; "
      f"vehicle: {res['error']}")
check("R7", "Out-of-service port refuses hand-offs without alarming", "yes" if oos else "no", "yes", oos)

# ------------------------------------------------------------------ 4. endurance
print("\n[4  Endurance: 100 alternating load / unload hand-offs]")
plc = new_plc()
port = Port(carrier=False)
settle(plc, idle_inputs(port), 0.2)
clamp_ok = True
done = 0
for k in range(100):
    job = "load" if k % 2 == 0 else "unload"
    if job == "unload":
        settle(plc, idle_inputs(port), 0.5)            # tool processes the carrier
        port.unload_ready = True
        if not plc.read()["CLAMP"]:
            clamp_ok = False
    res, _ = run(plc, Vehicle(port, job), 30)
    out = plc.read()
    if job == "unload" and out["CLAMP"]:
        clamp_ok = False
    if res["done"] and not out["ALARM"]:
        done += 1
    else:
        break
    settle(plc, idle_inputs(port), 0.3)
cnt = plc.read()["HANDOFFS"]
print(f"  completed {done}/100, PLC hand-off counter {cnt}, alarms 0, CLAMP held while loaded and released for unload: {clamp_ok}, "
      f"simulated time {plc.t / 60:.1f} min")
check("R8", "100 alternating hand-offs, no alarm, counter matches", f"{done}/100", "100/100", done == 100 and cnt == 100)
check("R9", "Carrier clamped after load, unclamped for unload", "yes" if clamp_ok else "no", "yes", clamp_ok)

# ------------------------------------------------------------------ table
print("\nID   Requirement                                                            Result   Target  Status")
for rid, text, val, lim, ok in req:
    print(f"{rid:4s} {text:70s} {str(val):>7s} {str(lim):>7s}  {'PASS' if ok else 'FAIL'}")
n_ok = sum(r[-1] for r in req)
print(f"{n_ok}/{len(req)} requirements pass")

# ------------------------------------------------------------------ timing chart
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    lanes = ["CS_0", "VALID", "TR_REQ", "BUSY", "COMPT", "L_REQ", "READY", "HO_AVBL", "PRESENCE", "PLACEMENT", "ALARM"]
    fig, axes = plt.subplots(1, 2, figsize=(15, 6), sharey=True)
    def chart_run(fault, max_s):
        plc = new_plc(); port = Port(carrier=False)
        tr = []
        for _ in range(30):                                  # 0.3 s idle before the vehicle arrives
            plc.write(idle_inputs(port)); plc.scan(); tr.append((plc.t, idle_inputs(port), plc.read()))
        run(plc, Vehicle(port, "load", fault=fault), max_s, stop_on_alarm=False, trace=tr)
        for _ in range(30):
            plc.scan(); tr.append((plc.t, tr[-1][1], plc.read()))
        return tr
    nominal_tr, fault_tr = chart_run(None, 30), chart_run("busy_early", 1.8)
    for ax, (title, trace) in zip(axes, (("Nominal load (port empty, carrier delivered)", nominal_tr),
                                         ("Fault: BUSY dropped before the carrier was placed", fault_tr))):
        t = [x[0] - trace[0][0] for x in trace]
        for i, name in enumerate(lanes):
            src = 1 if name in ("L_REQ", "READY", "HO_AVBL", "ALARM") else 0
            val = [float(x[1 + src][name]) for x in trace]
            col = "#d62728" if name == "ALARM" else ("#1f77b4" if src == 0 else "#2ca02c")
            ax.step(t, [1.2 * (len(lanes) - 1 - i) + 0.7 * v for v in val], where="post", color=col, lw=1.6)
        ax.set_yticks([1.2 * (len(lanes) - 1 - i) + 0.35 for i in range(len(lanes))])
        ax.set_yticklabels(lanes)
        ax.set(xlabel="time (s)", title=title)
        ax.grid(axis="x", alpha=0.3)
    axes[0].text(0.01, -0.12, "blue: vehicle and port sensors (PLC inputs)   green: PLC outputs   red: alarm",
                 transform=axes[0].transAxes, fontsize=9)
    fig.tight_layout(); fig.savefig("timing.png", dpi=110)
    print("saved timing.png")
except ImportError:
    print("matplotlib not installed; skipped timing.png")
sys.exit(0 if n_ok == len(req) else 1)
