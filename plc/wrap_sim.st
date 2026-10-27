(* Simulation wrapper for OpenPLC: the vehicle simulator writes and reads
   16-bit memory words over Modbus TCP (holding registers 1024 + n = %MWn).
   %MW0 inputs, one bit each: 0 VALID, 1 CS_0, 2 TR_REQ, 3 BUSY, 4 COMPT,
        5 PRESENCE, 6 PLACEMENT, 7 IN_SERVICE, 8 UNLOAD_READY, 9 ESTOP_OK, 10 RESET
   %MW1 outputs: 0 L_REQ, 1 U_REQ, 2 READY, 3 HO_AVBL, 4 ES, 5 CLAMP, 6 ALARM
   %MW2 alarm code, %MW3 state, %MW4 completed hand-offs, %MW5 scan counter *)
PROGRAM E84_SIM
  VAR
    in_word AT %MW0 : INT;
    out_word AT %MW1 : INT;
    alarm_code AT %MW2 : INT;
    state AT %MW3 : INT;
    handoffs AT %MW4 : INT;
    scans AT %MW5 : INT;
  END_VAR
  VAR
    port : E84_LOAD_PORT;
  END_VAR
  port(VALID := (in_word MOD 2) = 1,
       CS_0 := ((in_word / 2) MOD 2) = 1,
       TR_REQ := ((in_word / 4) MOD 2) = 1,
       BUSY := ((in_word / 8) MOD 2) = 1,
       COMPT := ((in_word / 16) MOD 2) = 1,
       PRESENCE := ((in_word / 32) MOD 2) = 1,
       PLACEMENT := ((in_word / 64) MOD 2) = 1,
       IN_SERVICE := ((in_word / 128) MOD 2) = 1,
       UNLOAD_READY := ((in_word / 256) MOD 2) = 1,
       ESTOP_OK := ((in_word / 512) MOD 2) = 1,
       RESET := ((in_word / 1024) MOD 2) = 1);
  out_word := BOOL_TO_INT(port.L_REQ) + 2 * BOOL_TO_INT(port.U_REQ) + 4 * BOOL_TO_INT(port.READY)
            + 8 * BOOL_TO_INT(port.HO_AVBL) + 16 * BOOL_TO_INT(port.ES) + 32 * BOOL_TO_INT(port.CLAMP)
            + 64 * BOOL_TO_INT(port.ALARM);
  alarm_code := port.ALARM_CODE;
  state := port.STATE;
  handoffs := port.HANDOFFS;
  scans := (scans + 1) MOD 30000;
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM instance0 WITH task0 : E84_SIM;
  END_RESOURCE
END_CONFIGURATION
