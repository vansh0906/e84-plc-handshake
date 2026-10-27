(* =====================================================================
   SEMI E84 load-port (passive equipment) handshake, IEC 61131-3 ST
   One function block, E84_LOAD_PORT, runs the carrier hand-off with an
   overhead hoist vehicle (the active equipment):

     AMHS  : CS_0 -> VALID                      (vehicle selects this port)
     PORT  : L_REQ (empty port) or U_REQ (carrier ready to leave)
     AMHS  : TR_REQ                             (transfer request)
     PORT  : READY
     AMHS  : BUSY                               (hoist moving)
     PORT  : L_REQ / U_REQ off once the sensors see the carrier placed / gone
     AMHS  : BUSY off, TR_REQ off, COMPT on
     PORT  : READY off
     AMHS  : COMPT off, VALID off, CS_0 off     (port clamps a delivered carrier)

   Passive timeouts (defaults as in common E84 test tools):
     TP1 L_REQ/U_REQ on  -> TR_REQ on      2 s
     TP2 READY on        -> BUSY on        2 s
     TP3 BUSY on         -> carrier placed / removed   60 s
     TP4 L_REQ/U_REQ off -> BUSY off, TR_REQ off, COMPT on   60 s
     TP5 READY off       -> VALID off      2 s
   Any timeout, protocol violation or E-stop drops L_REQ, U_REQ, READY and
   HO_AVBL at once and latches an alarm until RESET with the AMHS signals off.

   ALARM_CODE: 1..5 TP1..TP5 timeout
               6  carrier present but not seated when TP3 ran out
               11 VALID or CS_0 lost during a hand-off
               12 TR_REQ dropped before BUSY
               13 BUSY dropped before the carrier was placed / removed
               14 carrier sensors changed after the transfer finished
               80 TR_REQ / BUSY / COMPT on without VALID
               90 emergency stop
   ===================================================================== *)

FUNCTION_BLOCK E84_LOAD_PORT
  VAR_INPUT
    VALID : BOOL;          (* from AMHS *)
    CS_0 : BOOL;
    TR_REQ : BOOL;
    BUSY : BOOL;
    COMPT : BOOL;
    PRESENCE : BOOL;       (* port sensors: carrier present *)
    PLACEMENT : BOOL;      (*               carrier seated on the kinematic pins *)
    IN_SERVICE : BOOL;     (* port in automatic transfer mode *)
    UNLOAD_READY : BOOL;   (* tool finished with the carrier on the port *)
    ESTOP_OK : BOOL;       (* emergency-stop chain healthy *)
    RESET : BOOL;
    TP1 : TIME := T#2s;
    TP2 : TIME := T#2s;
    TP3 : TIME := T#60s;
    TP4 : TIME := T#60s;
    TP5 : TIME := T#2s;
  END_VAR
  VAR_OUTPUT
    L_REQ : BOOL;
    U_REQ : BOOL;
    READY : BOOL;
    HO_AVBL : BOOL;
    ES : BOOL;
    CLAMP : BOOL;
    ALARM : BOOL;
    ALARM_CODE : INT;
    STATE : INT;
    HANDOFFS : INT;
  END_VAR
  VAR
    tmr : TON;
    pt : TIME;
    last_state : INT := -1;
    loading : BOOL;
  END_VAR

  (* one timer per waiting state; it restarts whenever the state changes *)
  CASE STATE OF
    10: pt := TP1;
    20: pt := TP2;
    30: pt := TP3;
    40: pt := TP4;
    50: pt := TP5;
  ELSE
    pt := T#1h;
  END_CASE;
  tmr(IN := (STATE = last_state) AND (STATE >= 10) AND (STATE <= 50), PT := pt);
  last_state := STATE;

  ES := ESTOP_OK;
  IF NOT ESTOP_OK AND STATE <> 99 THEN
    ALARM_CODE := 90;
    STATE := 99;
  END_IF;

  CASE STATE OF
    0: (* idle: offer a hand-off when the vehicle selects this port *)
      L_REQ := FALSE; U_REQ := FALSE; READY := FALSE;
      HO_AVBL := IN_SERVICE AND ESTOP_OK;
      IF (TR_REQ OR BUSY OR COMPT) AND NOT VALID THEN
        ALARM_CODE := 80; STATE := 99;
      ELSIF VALID AND CS_0 AND HO_AVBL THEN
        IF NOT PRESENCE AND NOT PLACEMENT THEN
          loading := TRUE; L_REQ := TRUE; STATE := 10;
        ELSIF PRESENCE AND PLACEMENT AND UNLOAD_READY THEN
          loading := FALSE; CLAMP := FALSE; U_REQ := TRUE; STATE := 10;
        END_IF;
      END_IF;

    10: (* wait for TR_REQ *)
      IF NOT VALID OR NOT CS_0 THEN ALARM_CODE := 11; STATE := 99;
      ELSIF TR_REQ THEN READY := TRUE; STATE := 20;
      ELSIF tmr.Q THEN ALARM_CODE := 1; STATE := 99;
      END_IF;

    20: (* wait for BUSY *)
      IF NOT VALID OR NOT CS_0 THEN ALARM_CODE := 11; STATE := 99;
      ELSIF NOT TR_REQ THEN ALARM_CODE := 12; STATE := 99;
      ELSIF BUSY THEN STATE := 30;
      ELSIF tmr.Q THEN ALARM_CODE := 2; STATE := 99;
      END_IF;

    30: (* hoist moving: wait for the carrier to be placed or removed *)
      IF NOT VALID OR NOT CS_0 THEN ALARM_CODE := 11; STATE := 99;
      ELSIF loading AND PRESENCE AND PLACEMENT THEN L_REQ := FALSE; STATE := 40;
      ELSIF NOT loading AND NOT PRESENCE AND NOT PLACEMENT THEN U_REQ := FALSE; STATE := 40;
      ELSIF NOT BUSY THEN ALARM_CODE := 13; STATE := 99;
      ELSIF tmr.Q THEN
        ALARM_CODE := 3;
        IF loading AND PRESENCE AND NOT PLACEMENT THEN ALARM_CODE := 6; END_IF;
        STATE := 99;
      END_IF;

    40: (* wait for BUSY off, TR_REQ off and COMPT on *)
      IF NOT VALID OR NOT CS_0 THEN ALARM_CODE := 11; STATE := 99;
      ELSIF (loading AND NOT (PRESENCE AND PLACEMENT)) OR (NOT loading AND (PRESENCE OR PLACEMENT)) THEN
        ALARM_CODE := 14; STATE := 99;
      ELSIF NOT BUSY AND NOT TR_REQ AND COMPT THEN READY := FALSE; STATE := 50;
      ELSIF tmr.Q THEN ALARM_CODE := 4; STATE := 99;
      END_IF;

    50: (* wait for the vehicle to release the port *)
      IF NOT VALID AND NOT COMPT THEN
        IF loading THEN CLAMP := TRUE; END_IF;
        HANDOFFS := HANDOFFS + 1;
        STATE := 0;
      ELSIF tmr.Q THEN ALARM_CODE := 5; STATE := 99;
      END_IF;

    99: (* alarm: stop offering hand-offs until reset *)
      L_REQ := FALSE; U_REQ := FALSE; READY := FALSE; HO_AVBL := FALSE;
      IF RESET AND ESTOP_OK AND NOT VALID AND NOT TR_REQ AND NOT BUSY AND NOT COMPT THEN
        ALARM_CODE := 0; STATE := 0;
      END_IF;
  END_CASE;

  (* outputs go safe in the same scan that raises an alarm *)
  IF STATE = 99 THEN
    L_REQ := FALSE; U_REQ := FALSE; READY := FALSE; HO_AVBL := FALSE;
  END_IF;
  ALARM := STATE = 99;
END_FUNCTION_BLOCK
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
