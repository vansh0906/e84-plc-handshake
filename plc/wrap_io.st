(* Hardware wrapper: E84 parallel I/O and port sensors on real PLC I/O.
   Inputs  %IX0.0 VALID, %IX0.1 CS_0, %IX0.2 TR_REQ, %IX0.3 BUSY, %IX0.4 COMPT,
           %IX0.5 PRESENCE, %IX0.6 PLACEMENT, %IX0.7 IN_SERVICE,
           %IX1.0 UNLOAD_READY, %IX1.1 ESTOP_OK, %IX1.2 RESET
   Outputs %QX0.0 L_REQ, %QX0.1 U_REQ, %QX0.2 READY, %QX0.3 HO_AVBL, %QX0.4 ES,
           %QX0.5 CLAMP, %QX0.6 ALARM; %QW0 alarm code, %QW1 state *)
PROGRAM E84_IO
  VAR
    i_valid AT %IX0.0 : BOOL;
    i_cs0 AT %IX0.1 : BOOL;
    i_tr_req AT %IX0.2 : BOOL;
    i_busy AT %IX0.3 : BOOL;
    i_compt AT %IX0.4 : BOOL;
    i_presence AT %IX0.5 : BOOL;
    i_placement AT %IX0.6 : BOOL;
    i_in_service AT %IX0.7 : BOOL;
    i_unload_ready AT %IX1.0 : BOOL;
    i_estop_ok AT %IX1.1 : BOOL;
    i_reset AT %IX1.2 : BOOL;
    q_l_req AT %QX0.0 : BOOL;
    q_u_req AT %QX0.1 : BOOL;
    q_ready AT %QX0.2 : BOOL;
    q_ho_avbl AT %QX0.3 : BOOL;
    q_es AT %QX0.4 : BOOL;
    q_clamp AT %QX0.5 : BOOL;
    q_alarm AT %QX0.6 : BOOL;
    q_alarm_code AT %QW0 : INT;
    q_state AT %QW1 : INT;
  END_VAR
  VAR
    port : E84_LOAD_PORT;
  END_VAR
  port(VALID := i_valid, CS_0 := i_cs0, TR_REQ := i_tr_req, BUSY := i_busy, COMPT := i_compt,
       PRESENCE := i_presence, PLACEMENT := i_placement, IN_SERVICE := i_in_service,
       UNLOAD_READY := i_unload_ready, ESTOP_OK := i_estop_ok, RESET := i_reset);
  q_l_req := port.L_REQ; q_u_req := port.U_REQ; q_ready := port.READY; q_ho_avbl := port.HO_AVBL;
  q_es := port.ES; q_clamp := port.CLAMP; q_alarm := port.ALARM;
  q_alarm_code := port.ALARM_CODE; q_state := port.STATE;
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM instance0 WITH task0 : E84_IO;
  END_RESOURCE
END_CONFIGURATION
