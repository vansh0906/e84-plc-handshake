"""Run a vehicle against the PLC tick by tick and record every signal change."""
from plc import OUT_BITS


def run(plc, vehicle, max_s, stop_on_alarm=True, trace=None):
    """Step until the job is done, the PLC alarms, the vehicle gives up, or max_s passes.

    Returns (result, events). events = [(t, side, signal, value)] for both sides.
    """
    events = []
    last = None
    t_end = plc.t + max_s
    alarm_t = None
    while plc.t < t_end:
        out = plc.read()
        if last is not None:
            for k in OUT_BITS:
                if out[k] != last[k]:
                    events.append((plc.t, "PORT", k, out[k]))
        if out["ALARM"] and alarm_t is None:
            alarm_t = plc.t
            if stop_on_alarm:
                last = out
                break
        last = out
        vehicle.step(plc.t, out)
        if trace is not None:
            trace.append((plc.t, dict(vehicle.inputs()), dict(out)))
        plc.write(vehicle.inputs())
        plc.scan()
        if vehicle.done or vehicle.error:
            plc.scan()                     # let the PLC see the vehicle's last signal change
            break
    events += vehicle.events
    events.sort(key=lambda e: e[0])
    return {"done": vehicle.done, "error": vehicle.error, "alarm_t": alarm_t, "out": last}, events


def settle(plc, inputs, seconds):
    """Hold the given inputs for a while (operator actions, idle time)."""
    t_end = plc.t + seconds
    while plc.t < t_end:
        plc.write(inputs)
        plc.scan()
    return plc.read()
