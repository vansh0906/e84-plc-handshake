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
