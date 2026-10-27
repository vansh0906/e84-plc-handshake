"""Active side (overhead hoist vehicle) and the load port's carrier sensors.

The vehicle follows the E84 sequence with its own TA1-TA3 timeouts. A fault name makes it
(or the port hardware) misbehave in one specific way, to check that the PLC catches it.
"""
CS_TO_VALID = 0.15       # s, recommended delay between CS_0 and VALID
HOIST_S = 3.0            # s, hoist travel down or up
SEAT_S = 0.3             # s, carrier present -> seated on the kinematic pins
TA = 2.0                 # s, vehicle-side timeouts TA1-TA3
STEP = 0.1               # s, vehicle controller reaction time between its own signal changes

FAULTS = {
    "no_tr_req": "vehicle never raises TR_REQ",
    "no_busy": "vehicle never raises BUSY",
    "hoist_jam": "hoist stops; carrier never reaches the port",
    "tilted": "carrier lands tilted: present but never seated",
    "no_compt": "vehicle never raises COMPT",
    "valid_stuck": "vehicle never drops VALID",
    "valid_drop": "VALID drops in the middle of the transfer",
    "tr_req_drop": "TR_REQ drops before BUSY",
    "busy_early": "BUSY drops before the carrier is placed",
    "sensor_flicker": "carrier sensor drops out after placement",
    "no_valid": "TR_REQ raised without VALID",
    "estop": "emergency stop pressed during the transfer",
}


class Port:
    """Carrier sensors and the tool's inputs to the PLC."""

    def __init__(self, carrier=False):
        self.present = self.seated = carrier
        self.in_service = True
        self.unload_ready = carrier
        self.estop_ok = True
        self.reset = False

    def inputs(self):
        return {"PRESENCE": self.present, "PLACEMENT": self.seated, "IN_SERVICE": self.in_service,
                "UNLOAD_READY": self.unload_ready, "ESTOP_OK": self.estop_ok, "RESET": self.reset}


class Vehicle:
    def __init__(self, port, job="load", fault=None):
        self.port, self.job, self.fault = port, job, fault
        self.sig = dict(VALID=False, CS_0=False, TR_REQ=False, BUSY=False, COMPT=False)
        self.phase, self.t_phase = "start", 0.0
        self.done = False
        self.error = None
        self.fault_t = None              # when the injected fault took effect
        self.events = []

    def _go(self, phase, t):
        self.phase, self.t_phase = phase, t

    def _set(self, name, val, t):
        if self.sig[name] != val:
            self.sig[name] = val
            self.events.append((t, "AMHS", name, val))

    def step(self, t, out):
        """Advance the vehicle by one tick, given the PLC's latest outputs."""
        req = out["L_REQ"] if self.job == "load" else out["U_REQ"]
        dt = t - self.t_phase
        f, p = self.fault, self.phase
        if self.done:
            return
        if p == "start":
            if f == "no_valid":
                self._set("TR_REQ", True, t); self.fault_t = t; self._go("abort", t); return
            if not out["HO_AVBL"]:
                if dt > TA:
                    self.error = "TA1: port never offered a hand-off (HO_AVBL off)"; self._go("abort", t)
                return
            self._set("CS_0", True, t); self._go("cs", t)
        elif p == "cs" and dt >= CS_TO_VALID:
            self._set("VALID", True, t); self._go("wait_req", t)
        elif p == "wait_req":
            if req:
                self._go("trq", t)
            elif dt > TA:
                self.error = "TA1 timeout"; self._go("abort", t)
        elif p == "trq" and dt >= STEP:
            if f == "no_tr_req":
                self.fault_t = t; self._go("hang", t); return
            self._set("TR_REQ", True, t); self._go("wait_ready", t)
        elif p == "wait_ready":
            if out["READY"]:
                self._go("bsy", t)
            elif dt > TA:
                self.error = "TA2 timeout"; self._go("abort", t)
        elif p == "bsy" and dt >= STEP:
            if f == "no_busy":
                self.fault_t = t; self._go("hang", t); return
            if f == "tr_req_drop":
                self._set("TR_REQ", False, t); self.fault_t = t; self._go("hang", t); return
            self._set("BUSY", True, t); self._go("hoist", t)
        elif p == "hoist":
            if f == "valid_drop" and dt >= 1.0:
                self._set("VALID", False, t); self.fault_t = t; self._go("hang", t); return
            if f == "busy_early" and dt >= 1.0:
                self._set("BUSY", False, t); self.fault_t = t; self._go("hang", t); return
            if f == "estop" and dt >= 1.0:
                self.port.estop_ok = False; self.fault_t = t; self._go("hang", t); return
            if f == "hoist_jam":
                if self.fault_t is None:
                    self.fault_t = t
                return
            if dt >= HOIST_S:
                if self.job == "load":
                    self.port.present = True
                    if f == "tilted":
                        self.fault_t = t; self._go("hang", t); return
                else:
                    self.port.seated = False
                self._go("sensors", t)
        elif p == "sensors" and dt >= SEAT_S:
            if self.job == "load":
                self.port.seated = True
            else:
                self.port.present = False; self.port.unload_ready = False
            self._go("wait_req_off", t)
        elif p == "wait_req_off":
            if not req:
                if f == "sensor_flicker":
                    self.port.seated = False; self.fault_t = t; self._go("hang", t); return
                self._go("end1", t)
        elif p == "end1" and dt >= 2 * STEP:
            self._set("BUSY", False, t); self._go("end2", t)
        elif p == "end2" and dt >= STEP:
            self._set("TR_REQ", False, t); self._go("end3", t)
        elif p == "end3" and dt >= STEP:
            if f == "no_compt":
                self.fault_t = t; self._go("hang", t); return
            self._set("COMPT", True, t); self._go("wait_ready_off", t)
        elif p == "wait_ready_off":
            if not out["READY"]:
                self._go("rel1", t)
            elif dt > TA:
                self.error = "TA3 timeout"; self._go("abort", t)
        elif p == "rel1" and dt >= STEP:
            self._set("COMPT", False, t); self._go("rel2", t)
        elif p == "rel2" and dt >= STEP:
            if f == "valid_stuck":
                self.fault_t = t; self._go("hang", t); return
            self._set("VALID", False, t); self._go("rel3", t)
        elif p == "rel3" and dt >= STEP:
            self._set("CS_0", False, t)
            self.done = True
        elif p == "abort":
            pass

    def release(self, t):
        """Drop every AMHS signal (what a vehicle does after an error)."""
        for k in self.sig:
            self._set(k, False, t)

    def inputs(self):
        d = dict(self.sig); d.update(self.port.inputs())
        return d
