(* ---- 1. Heartbeat and plant session ---- *)
IF HEARTBEAT_OUT >= 2000000000 THEN
    HEARTBEAT_OUT := 0;
ELSE
    HEARTBEAT_OUT := HEARTBEAT_OUT + 1;
END_IF;

DoClear := FALSE;
NewSession := FALSE;
IF PLANT_SESSION <> Session THEN
    (* the simulation was restarted: forget everything that belongs to the previous run *)
    Session := PLANT_SESSION;
    SESSION_ACK := Session;
    NewSession := TRUE;
    DoClear := TRUE;
    Run := FALSE;
    PLC_FAULT := 0;
    LastCam := CAM_ID;
    ActiveA := 0; ActiveB := 0; ActiveC := 0;
    DoneA := 0; DoneB := 0; DoneC := 0;
END_IF;

KpiEdge(CLK := KPI_RESET);
IF KpiEdge.Q AND NOT Run THEN
    DoClear := TRUE;
END_IF;
IF DoClear THEN
    FOR i := 0 TO 15 DO
        Route[i] := 4;
    END_FOR;
    COUNT_A := 0; COUNT_B := 0; COUNT_C := 0; COUNT_REJECT := 0; Total := 0;
    RejectPct := 0; RATE_PM := 0; RateBase := 0;
    TESTED := 0; Correct := 0; ACCURACY_PCT := 0;
    LastGrade := 0; LastDiameter := 0.0;
END_IF;

(* ---- 2. Faults: E-stop, belt jam (gauge beam blocked 5 s), plant link lost (no heartbeat for 10 s) ----
   The plant is a simulation and it freezes now and then (robot start, new crates, a Windows hiccup). A freeze is not a fault:
   while the heartbeat is stale for more than 1 s (PlantStalled) the timers that watch the plant are held at zero, and a lost
   link clears itself as soon as the heartbeat is back (the operator then presses Start again). *)
LinkTimer(IN := (HEARTBEAT_IN = LastHB) AND (Session <> 0), PT := T#10s);
PlantStalled := LinkTimer.ET >= T#1s;
LastHB := HEARTBEAT_IN;
JamTimer(IN := BEAM_BLOCKED AND Run AND NOT PlantStalled, PT := T#5s);
ResetEdge(CLK := RESET_PB);
IF (PLC_FAULT = 1) AND NOT ESTOP THEN
    PLC_FAULT := 0;
END_IF;
IF (PLC_FAULT = 4) AND NOT LinkTimer.Q THEN
    PLC_FAULT := 0;
END_IF;
IF ESTOP THEN
    PLC_FAULT := 1;
ELSIF PLC_FAULT = 0 THEN
    IF JamTimer.Q THEN
        PLC_FAULT := 2;
    ELSIF LinkTimer.Q THEN
        PLC_FAULT := 4;
    END_IF;
END_IF;
IF ResetEdge.Q AND NOT ESTOP AND NOT LinkTimer.Q THEN
    PLC_FAULT := 0;
END_IF;

(* ---- 3. Line run latch: Start sets, Stop / any fault resets ---- *)
StartEdge(CLK := START_PB);
IF STOP_PB OR ESTOP OR (PLC_FAULT <> 0) THEN
    Run := FALSE;
ELSIF StartEdge.Q AND NOT NewSession THEN
    Run := TRUE;
END_IF;
RUNNING := Run;
CONVEYOR_RUN := Run;

(* ---- 4. Automated feeder: the gate opens for 0.5 s at the end of every feed interval ---- *)
IF FEED_INTERVAL < 15 THEN
    IntervalDs := 15;
ELSIF FEED_INTERVAL > 100 THEN
    IntervalDs := 100;
ELSE
    IntervalDs := FEED_INTERVAL;
END_IF;
(* PT = whole seconds + tenths; a single MULTIME(T#100ms, n) would overflow in matiec's time library for n >= 22 *)
PtSeconds := MULTIME(T#1s, IntervalDs / 10);
PtTenths := MULTIME(T#100ms, IntervalDs MOD 10);
FeedTimer(IN := Run AND NOT FeedTimer.Q AND NOT PlantStalled, PT := ADD_TIME(PtSeconds, PtTenths));
IF FeedTimer.Q THEN
    GateOpen := TRUE;
END_IF;
GateTimer(IN := GateOpen, PT := T#500ms);
IF GateTimer.Q OR NOT Run THEN
    GateOpen := FALSE;
END_IF;
FEED_GATE := GateOpen AND Run;

(* ---- 5. Classification: one new CAM_ID = one new onion measured by gauge (size) and camera (sprout) ---- *)
IF CAM_ID <> LastCam THEN
    LastCam := CAM_ID;
    IF SPROUTED OR (DIAMETER_MM < 40.0) OR (DIAMETER_MM >= 90.0) THEN
        Grade := 4;
    ELSIF DIAMETER_MM >= 70.0 THEN
        Grade := 1;
    ELSIF DIAMETER_MM >= 55.0 THEN
        Grade := 2;
    ELSE
        Grade := 3;
    END_IF;
    Idx := DINT_TO_INT(((CAM_ID MOD 16) + 16) MOD 16);   (* always 0..15, an out-of-range index would stop the PLC task *)
    Route[Idx] := Grade;
    LastGrade := Grade;
    LastDiameter := DIAMETER_MM;
    CASE Grade OF
        1: COUNT_A := COUNT_A + 1;
        2: COUNT_B := COUNT_B + 1;
        3: COUNT_C := COUNT_C + 1;
    ELSE
        COUNT_REJECT := COUNT_REJECT + 1;
    END_CASE;
    TESTED := TESTED + 1;
    IF Grade = TRUE_GRADE THEN
        Correct := Correct + 1;
    END_IF;
END_IF;

(* ---- 6. KPIs: totals, reject share, accuracy, onions per minute ---- *)
Total := COUNT_A + COUNT_B + COUNT_C + COUNT_REJECT;
IF Total > 0 THEN
    RejectPct := DINT_TO_INT((COUNT_REJECT * 100) / Total);
END_IF;
IF TESTED > 0 THEN
    ACCURACY_PCT := DINT_TO_INT((Correct * 100) / TESTED);
END_IF;
RateTimer(IN := Run AND NOT RateTimer.Q, PT := T#60s);
IF RateTimer.Q THEN
    RATE_PM := DINT_TO_INT(Total - RateBase);
    RateBase := Total;
END_IF;

(* ---- 7A. Pusher A: act when an onion routed to grade A stands in front of the pusher ---- *)
IF Run AND (ActiveA = 0) AND (PE_A_ID <> 0) AND (PE_A_ID <> DoneA) THEN
    Idx := DINT_TO_INT(((PE_A_ID MOD 16) + 16) MOD 16);
    IF Route[Idx] = 1 THEN
        ActiveA := PE_A_ID;
    END_IF;
END_IF;
PushTimerA(IN := Run AND (ActiveA <> 0) AND NOT PlantStalled, PT := T#4s);
IF ActiveA <> 0 THEN
    IF (PUSHED_A_ID = ActiveA) OR PushTimerA.Q OR ((PE_A_ID <> ActiveA) AND (PE_A_ID <> 0)) THEN
        DoneA := ActiveA;
        ActiveA := 0;
    END_IF;
END_IF;
PUSH_A := Run AND (ActiveA <> 0);

(* ---- 7B. Pusher B: act when an onion routed to grade B stands in front of the pusher ---- *)
IF Run AND (ActiveB = 0) AND (PE_B_ID <> 0) AND (PE_B_ID <> DoneB) THEN
    Idx := DINT_TO_INT(((PE_B_ID MOD 16) + 16) MOD 16);
    IF Route[Idx] = 2 THEN
        ActiveB := PE_B_ID;
    END_IF;
END_IF;
PushTimerB(IN := Run AND (ActiveB <> 0) AND NOT PlantStalled, PT := T#4s);
IF ActiveB <> 0 THEN
    IF (PUSHED_B_ID = ActiveB) OR PushTimerB.Q OR ((PE_B_ID <> ActiveB) AND (PE_B_ID <> 0)) THEN
        DoneB := ActiveB;
        ActiveB := 0;
    END_IF;
END_IF;
PUSH_B := Run AND (ActiveB <> 0);

(* ---- 7C. Pusher C: act when an onion routed to grade C stands in front of the pusher ---- *)
IF Run AND (ActiveC = 0) AND (PE_C_ID <> 0) AND (PE_C_ID <> DoneC) THEN
    Idx := DINT_TO_INT(((PE_C_ID MOD 16) + 16) MOD 16);
    IF Route[Idx] = 3 THEN
        ActiveC := PE_C_ID;
    END_IF;
END_IF;
PushTimerC(IN := Run AND (ActiveC <> 0) AND NOT PlantStalled, PT := T#4s);
IF ActiveC <> 0 THEN
    IF (PUSHED_C_ID = ActiveC) OR PushTimerC.Q OR ((PE_C_ID <> ActiveC) AND (PE_C_ID <> 0)) THEN
        DoneC := ActiveC;
        ActiveC := 0;
    END_IF;
END_IF;
PUSH_C := Run AND (ActiveC <> 0);

(* ---- 8. Packaging: call the robot for every full crate waiting at the pick-up spot ---- *)
CALL_A := CRATE_A_READY AND NOT PALLET_FULL AND NOT ESTOP;
CALL_B := CRATE_B_READY AND NOT PALLET_FULL AND NOT ESTOP;
CALL_C := CRATE_C_READY AND NOT PALLET_FULL AND NOT ESTOP;

(* ---- 9. Safe state ---- *)
IF NOT Run THEN
    FEED_GATE := FALSE;
    CONVEYOR_RUN := FALSE;
    PUSH_A := FALSE; PUSH_B := FALSE; PUSH_C := FALSE;
END_IF;
