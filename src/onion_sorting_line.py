#!/usr/bin/env python3
"""
ONION SORTING LINE - PLC logic + CoppeliaSim 3D scene in ONE file
=================================================================
Workflow modelled:  hopper/feeder -> layer 1 SIZE gauge (beam, timing) -> layer 2 SPROUT camera -> separation
(3 pushers A/B/C push onions into crates beside the belt, rejects fall into a bin) -> full crates roll to a pick-up spot -> one robot palletizes them.

What is inside (sections of this file):
  1. PLC PROGRAM   - the SIMPLE ladder (about 50 rungs, default) and the older FULL ladder (126 rungs, --ladder full),
                     both run by a small Mitsubishi-FX5U-style interpreter (device names X/Y/M/D/T as in the ladder document).
  2. LINE MODEL    - belt, onions, sensors, pushers, crates, robot, pallet (the "plant" the PLC controls).
  3. SCENE VIEW    - builds the factory cell in CoppeliaSim (fence, conveyor, sensors, crates, pallet) and loads a REAL
                     robot from the CoppeliaSim library (UR10 palletizer). Their joints are driven by a
                     built-in inverse-kinematics solver (section 2b), so no IK plugin is needed.
  4. RUNNER        - command line, KPI printout, keyboard control.

HOW TO RUN
  1. Install CoppeliaSim (4.6 or newer, EDU/PRO). Start it and open a NEW EMPTY scene (File > New scene).
  2. pip install coppeliasim-zmqremoteapi-client
  3. python onion_sorting_line.py            (old objects are removed, the scene is built, the simulation starts)
  First time?  python onion_sorting_line.py --check         (shows whether the robot library is found)
               python onion_sorting_line.py --robot-test    (moves the robot through every working point)
  No CoppeliaSim?  python onion_sorting_line.py --offline --duration 120   (prints the KPIs only)

OPTIONS  (python onion_sorting_line.py --help)
  --feed-interval 2.0   seconds between onions      --p-sprout 0.12      share of sprouted onions
  --sprout-detect 0.96  camera detection rate       --crate-size 10      onions per crate
  --onions 200          stop feeding after N        --save-scene line.ttt   save the built scene
  --build-only          just build + save the scene
  --no-models           use a simple primitive arm instead of the library robot
  --coppelia-dir DIR    CoppeliaSim install folder if the model library is not found automatically
  --palletizer-model F.ttm   use another robot model from the library (e.g. UR5)
  --keep-scene          do not delete old objects from the open scene first
  --no-hmi              do not open the operator panel (HMI) window
OPENPLC (optional): the control logic can run in an external OpenPLC program instead of the built-in ladder.
  python onion_sorting_line.py --write-plc          writes OnionSorting.st (the PLC program) and io_map.csv (tag list) to this folder
  python onion_sorting_line.py --plc demo           tries the whole OPC UA chain with a stand-in PLC (no OpenPLC needed; pip install asyncua)
  python onion_sorting_line.py --opc-check          checks the connection to OpenPLC and that every tag is exposed
  python onion_sorting_line.py --plc opcua          runs the line controlled by OpenPLC   (--opc-endpoint, --opc-user, --opc-password)
KEYS in the terminal:  e = E-stop   r = release + fault reset   s = start   t = stop   x = reset counters   q = quit

NOTE: onions, crates and tooling are plain shapes moved by the script (physics is switched off), so the behaviour is
deterministic and the PLC logic, not a physics engine, decides where every onion goes. The robot is a real library
models whose joints are moved kinematically.
"""

_BUNDLED = True

# ====================================================================================================
#  1. PLC PROGRAM
# ====================================================================================================

import re

# ------------------------------------------------------------------ DSL
def NO(a): return ('NO', a)
def NC(a): return ('NC', a)
def RISE(a): return ('RISE', a)
def FALL(a): return ('FALL', a)
def CMP(op, a, b): return ('CMP', op, a, b)
def OR(*branches): return ('OR', list(branches))      # branch = contact or tuple of contacts

def OUT(d): return ('OUT', d)
def SET(d): return ('SET', d)
def RST(d): return ('RST', d)
def MOV(s, d): return ('MOV', s, d)
def INC(d): return ('INC', d)
def ADD(a, b, d): return ('ADD', a, b, d)
def SUB(a, b, d): return ('SUB', a, b, d)
def MUL(a, b, d): return ('MUL', a, b, d)
def DIV(a, b, d): return ('DIV', a, b, d)
def ZRST(a, b): return ('ZRST', a, b)
def WAND(a, b, d): return ('WAND', a, b, d)      # word AND (used as modulo 16 for the ring buffer)
def TMR(kind, name, preset): return ('TMR', kind, name, preset)   # kind OUT (100 ms) | OUTH (10 ms)

RUNGS = []
SECTIONS = []

def section(title, note=''):
    SECTIONS.append((len(RUNGS), title, note))

def rung(title, cond, acts):
    RUNGS.append(dict(title=title, cond=cond, acts=acts))

ALWAYS = [NO('SM400')]
FAULT_BITS = ['M50', 'M51', 'M52', 'M53', 'M54', 'M55', 'M56', 'M57']

# ------------------------------------------------------------------ program
section('1. Initialisation and parameter conversion',
        'Runs once at RUN (SM402), then the size-timer presets are recalculated every scan from the HMI parameters.')
rung('Default parameters (first scan only)', [NO('SM402')], [
    MOV('K100', 'D200'), MOV('K100', 'D201'), MOV('K70', 'D210'), MOV('K55', 'D211'),
    MOV('K40', 'D212'), MOV('K90', 'D213'), MOV('K40', 'D215'), MOV('K10', 'D230'),
    MOV('K13', 'D231'), MOV('K20', 'D240'), MOV('K3', 'D241'), MOV('K3', 'D242'),
    ZRST('D20', 'D27'), ZRST('D40', 'D69')])
rung('Size thresholds converted to timer presets (ticks = mm x ticks/s / belt mm/s)', [CMP('>', 'D200', 'K0')], [
    MUL('D212', 'D201', 'D260'), DIV('D260', 'D200', 'D250'),
    MUL('D211', 'D201', 'D260'), DIV('D260', 'D200', 'D252'),
    MUL('D210', 'D201', 'D260'), DIV('D260', 'D200', 'D254'),
    MUL('D213', 'D201', 'D260'), DIV('D260', 'D200', 'D256')])
rung('HMI "reset counters and queue" (only while stopped)', [RISE('M310'), NC('M0')], [
    ZRST('D40', 'D69'), ZRST('D20', 'D27'), ZRST('M30', 'M32'), ZRST('M40', 'M42'),
    RST('Y5'), RST('Y6'), RST('Y7')])

section('2. Faults, start/stop and conveyor',
        'Fault bits are latched (SET) except the E-stop bit, which follows the input. X3 clears them once the cause is gone.')
rung('Fault reset button (E-stop circuit must be healthy)', [RISE('X3'), NO('X2')], [
    ZRST('M50', 'M57'), ZRST('M30', 'M32'), ZRST('M40', 'M42')])
rung('F1  E-stop pressed (X2 is wired normally closed)', [NC('X2')], [OUT('M50')])
rung('F7  Reject bin full sensor', [NO('X15')], [SET('M56')])
rung('F2  Jam: beam blocked longer than D215 x 0.1 s - timer', [NO('X5')], [TMR('OUT', 'T24', 'D215')])
rung('F2  Jam: latch fault', [NO('T24')], [SET('M51')])
rung('Any fault', [OR(*[NO(b) for b in FAULT_BITS])], [OUT('M1')])
rung('System run latch (start / stop / e-stop / fault)',
     [OR(NO('X0'), NO('M0')), NC('X1'), NO('X2'), NC('M1')], [OUT('M0')])
rung('Conveyor motor', [NO('M0')], [OUT('Y0')])

section('3. Automated feeding',
        'One item is released at the end of every D240 x 0.1 s interval (default 2.0 s) of running time. The gate stays open 0.5 s. After a stop the interval starts again, so items never bunch up.')
rung('Feed enable', [NO('M0'), NC('X4'), NC('M58'), NC('M1')], [OUT('M3')])
rung('Feed cycle timer (self-resetting, runs only while feeding is enabled)', [NO('M3'), NC('T1')], [TMR('OUT', 'T1', 'D240')])
rung('Cycle time elapsed: open the gate', [NO('T1')], [SET('M4')])
rung('Gate-open timer (0.5 s)', [NO('M4')], [TMR('OUT', 'T2', 'K5')])
rung('Close the gate', [NO('T2')], [RST('M4')])
rung('Feeding stopped: close the gate', [NC('M3')], [RST('M4')])
rung('Feeder gate / vibratory feeder', [NO('M4'), NO('M3')], [OUT('Y1')])

section('4. Measurement, classification and grading',
        'Size is measured by beam-break timing: four 10 ms timers (T20-T23) run while the beam is blocked. At the '
        'falling edge their contacts tell which size band the item belongs to. Test mode (X4) injects the size from the HMI instead.')
rung('Belt stopped while item in beam: measurement invalid', [NO('X5'), NC('Y0')], [SET('M8')])
rung('Classification trigger: beam falling edge (auto) or HMI inject (test mode)',
     [OR((FALL('X5'), NC('X4')), (RISE('M300'), NO('X4'), NO('M0')))], [OUT('M6')])
rung('Test mode: injected defect flag', [NO('M6'), NO('X4'), NO('M301')], [SET('M7')])
rung('Clear size flags', [NO('M6')], [ZRST('M14', 'M17')])
rung('Auto: size >= C minimum (T20)', [NO('M6'), NC('X4'), NO('T20')], [SET('M14')])
rung('Auto: size >= B minimum (T21)', [NO('M6'), NC('X4'), NO('T21')], [SET('M15')])
rung('Auto: size >= A minimum (T22)', [NO('M6'), NC('X4'), NO('T22')], [SET('M16')])
rung('Auto: size >= maximum, oversize (T23)', [NO('M6'), NC('X4'), NO('T23')], [SET('M17')])
rung('Test: injected size >= C minimum', [NO('M6'), NO('X4'), CMP('>=', 'D300', 'D212')], [SET('M14')])
rung('Test: injected size >= B minimum', [NO('M6'), NO('X4'), CMP('>=', 'D300', 'D211')], [SET('M15')])
rung('Test: injected size >= A minimum', [NO('M6'), NO('X4'), CMP('>=', 'D300', 'D210')], [SET('M16')])
rung('Test: injected size >= maximum', [NO('M6'), NO('X4'), CMP('>=', 'D300', 'D213')], [SET('M17')])
rung('Grade code = 3 (C) by default', [NO('M6')], [MOV('K3', 'D16')])
rung('Grade code = 2 (B)', [NO('M6'), NO('M15')], [MOV('K2', 'D16')])
rung('Grade code = 1 (A)', [NO('M6'), NO('M16')], [MOV('K1', 'D16')])
rung('Grade code = 4 (Reject): too small, oversize, defect or invalid measurement',
     [NO('M6'), OR(NC('M14'), NO('M17'), NO('M7'), NO('M8'))], [MOV('K4', 'D16')])
rung('Auto mode: write grade into the queue - set write index', [NO('M6'), NC('X4')], [MOV('D20', 'Z0')])
rung('Auto mode: store grade code', [NO('M6'), NC('X4')], [MOV('D16', 'D100Z0')])
rung('Auto mode: advance write pointer', [NO('M6'), NC('X4')], [INC('D20')])
rung('Wrap write pointer (16-entry ring)', [CMP('=', 'D20', 'K16')], [MOV('K0', 'D20')])
rung('Clear measurement latches for next item', [NO('M6')], [RST('M7'), RST('M8')])
rung('Newest queue entry index (D26 = write pointer - 1)', ALWAYS, [SUB('D20', 'K1', 'D26')])
rung('Newest queue entry index: wrap', [CMP('<', 'D26', 'K0')], [ADD('D26', 'K16', 'D26')])
rung('Layer 2 (sprout camera, downstream of the size gauge): sprout seen, overwrite the newest queue entry with Reject',
     [RISE('X6'), NC('X4')], [MOV('D26', 'Z0'), MOV('K4', 'D100Z0')])
rung('Item leaves the camera zone: capture its final grade in D18', [FALL('X16'), NC('X4')], [MOV('D26', 'Z0'), MOV('D100Z0', 'D18')])
rung('Item leaves the camera zone: its queue entry is final, advance the station-visible pointer D27', [FALL('X16'), NC('X4')], [INC('D27')])
rung('Wrap D27 (16-entry ring)', [CMP('=', 'D27', 'K16')], [MOV('K0', 'D27')])
rung('Accuracy (test mode): count tested items', [NO('M6'), NO('X4'), NO('M302')], [INC('D62')])
rung('Accuracy (test mode): count correct (D16 = true grade D302)', [NO('M6'), NO('X4'), NO('M302'), CMP('=', 'D16', 'D302')], [INC('D63')])
rung('Accuracy (auto mode): count tested items at camera exit', [FALL('X16'), NC('X4'), NO('M302')], [INC('D62')])
rung('Accuracy (auto mode): count correct (final grade D18 = true grade D302)', [FALL('X16'), NC('X4'), NO('M302'), CMP('=', 'D18', 'D302')], [INC('D63')])
rung('Accuracy % = correct x 100 / tested', [CMP('>', 'D62', 'K0')], [MUL('D63', 'K100', 'D66'), DIV('D66', 'D62', 'D64')])
rung('Size timer T20 (C minimum)', [NO('X5'), NO('Y0')], [TMR('OUTH', 'T20', 'D250')])
rung('Size timer T21 (B minimum)', [NO('X5'), NO('Y0')], [TMR('OUTH', 'T21', 'D252')])
rung('Size timer T22 (A minimum)', [NO('X5'), NO('Y0')], [TMR('OUTH', 'T22', 'D254')])
rung('Size timer T23 (maximum / oversize)', [NO('X5'), NO('Y0')], [TMR('OUTH', 'T23', 'D256')])

section('5. Queue depth and overflow',
        'D25 = items stored but not yet passed station C. 15 or more means the 16-entry ring is about to overwrite itself.')
rung('Queue depth', ALWAYS, [SUB('D20', 'D23', 'D25')])
rung('Queue depth wrap', [CMP('<', 'D25', 'K0')], [ADD('D25', 'K16', 'D25')])
rung('F6  Queue overflow', [CMP('>=', 'D25', 'K15')], [SET('M55')])


def station(letter, sensor, ptr, ev, pend, tdel, y, tpulse, await_, twd, flt, bin_sensor, grade, skip, zskip, first=False):
    n = letter
    section(f'6{ {"A":"a","B":"b","C":"c"}[n] }. Sorting station {n} (grade {grade})',
            {'A': 'Sensor X7 sees an item arrive; the grade at the head of the queue decides whether pusher A fires. Station A never skips queue entries.',
             'B': 'Items already diverted at A never reach B, so grade-A entries are skipped (one per scan) before an arrival is matched.',
             'C': 'Grade-A and grade-B entries are skipped. A grade-4 item passes straight on to the reject bin and is counted as a reject here.'}[n])
    if skip:
        rung(f'Station {n}: point Z1 at the queue head', [CMP('<>', ptr, 'D27')], [MOV(ptr, 'Z1')])
        rung(f'Station {n}: skip entries diverted upstream', [CMP('<>', ptr, 'D27'), skip], [INC(ptr)])
        rung(f'Station {n}: wrap read pointer', [CMP('=', ptr, 'K16')], [MOV('K0', ptr)])
    rung(f'F8  Item seen at station {n} but queue is empty', [RISE(sensor), CMP('=', ptr, 'D27')], [SET('M57')])
    rung(f'Station {n}: arrival event (1 scan)', [RISE(sensor), CMP('<>', ptr, 'D27')], [OUT(ev)])
    rung(f'Station {n}: point Z0 at the queue head', [NO(ev)], [MOV(ptr, 'Z0')])
    rung(f'Station {n}: grade matches, request push', [NO(ev), CMP('=', 'D100Z0', f'K{grade}')], [SET(pend)])
    rung(f'Station {n}: consume queue entry', [NO(ev)], [INC(ptr)])
    rung(f'Station {n}: wrap read pointer', [CMP('=', ptr, 'K16')], [MOV('K0', ptr)])
    rung(f'Station {n}: push delay timer (D241 x 0.1 s, only while running)', [NO(pend), NO('M0')], [TMR('OUT', tdel, 'D241')])
    rung(f'Station {n}: extend pusher', [NO(tdel)], [SET(y), RST(pend), SET(await_)])
    rung(f'Station {n}: pusher dwell timer (D242 x 0.1 s)', [NO(y)], [TMR('OUT', tpulse, 'D242')])
    rung(f'Station {n}: retract pusher', [NO(tpulse)], [RST(y)])
    rung(f'Station {n}: bin entry sensor confirms the item', [RISE(bin_sensor), NO(await_)], [RST(await_)])
    rung(f'Station {n}: watchdog timer 2.0 s', [NO(await_), NC('X4')], [TMR('OUT', twd, 'K20')])
    rung(f'{flt[0]}  Item lost at station {n}: no bin confirmation', [NO(twd)], [SET(flt[1]), RST(await_)])

station('A', 'X7', 'D21', 'M20', 'M40', 'T10', 'Y2', 'T3', 'M30', 'T13', ('F3', 'M52'), 'X12', 1, None, None)
station('B', 'X10', 'D22', 'M22', 'M41', 'T11', 'Y3', 'T4', 'M31', 'T14', ('F4', 'M53'), 'X13', 2, CMP('=', 'D100Z1', 'K1'), 'Z1')
station('C', 'X11', 'D23', 'M24', 'M42', 'T12', 'Y4', 'T5', 'M32', 'T15', ('F5', 'M54'), 'X14', 3, CMP('<=', 'D100Z1', 'K2'), 'Z1')

section('7. Quantity monitoring and packaging',
        'Count events M60-M63 come from the pusher firing (auto) or from the classification (test mode). '
        'A crate is "ready" at D230 items; the robot / operator confirms with M320-M322.')
rung('Count event: grade A', [OR(NO('T10'), (NO('M6'), NO('X4'), CMP('=', 'D16', 'K1')))], [OUT('M60')])
rung('Count event: grade B', [OR(NO('T11'), (NO('M6'), NO('X4'), CMP('=', 'D16', 'K2')))], [OUT('M61')])
rung('Count event: grade C', [OR(NO('T12'), (NO('M6'), NO('X4'), CMP('=', 'D16', 'K3')))], [OUT('M62')])
rung('Count event: reject (grade 4 passing station C)',
     [OR((NO('M24'), CMP('=', 'D100Z0', 'K4')), (NO('M6'), NO('X4'), CMP('=', 'D16', 'K4')))], [OUT('M63')])
rung('Count grade A (total and current crate)', [NO('M60')], [INC('D40'), INC('D50')])
rung('Count grade B (total and current crate)', [NO('M61')], [INC('D41'), INC('D51')])
rung('Count grade C (total and current crate)', [NO('M62')], [INC('D42'), INC('D52')])
rung('Count rejects', [NO('M63')], [INC('D43')])
rung('Total processed', ALWAYS, [ADD('D40', 'D41', 'D44'), ADD('D44', 'D42', 'D44'), ADD('D44', 'D43', 'D44')])
rung('Reject rate %', [CMP('>', 'D44', 'K0')], [MUL('D43', 'K100', 'D47'), DIV('D47', 'D44', 'D45')])
rung('Crate A full: pack-ready signal (robot trigger)', [CMP('>=', 'D50', 'D230')], [SET('Y5')])
rung('Crate B full: pack-ready signal (robot trigger)', [CMP('>=', 'D51', 'D230')], [SET('Y6')])
rung('Crate C full: pack-ready signal (robot trigger)', [CMP('>=', 'D52', 'D230')], [SET('Y7')])
rung('Crate A packed (robot done, M320)', [RISE('M320'), NO('Y5')], [RST('Y5'), SUB('D50', 'D230', 'D50'), INC('D54')])
rung('Crate B packed (robot done, M321)', [RISE('M321'), NO('Y6')], [RST('Y6'), SUB('D51', 'D230', 'D51'), INC('D55')])
rung('Crate C packed (robot done, M322)', [RISE('M322'), NO('Y7')], [RST('Y7'), SUB('D52', 'D230', 'D52'), INC('D56')])
rung('Hold the feeder if any crate is nearly overfull (>= D231)',
     [OR(CMP('>=', 'D50', 'D231'), CMP('>=', 'D51', 'D231'), CMP('>=', 'D52', 'D231'))], [OUT('M58')])

section('8. Processing-rate KPI',
        'Rate = items completed in the last window, scaled to items per minute. Updated every 10 s and every 60 s while running.')
rung('10 s window timer', [NO('M0'), NC('T30')], [TMR('OUT', 'T30', 'K100')])
rung('Rate over last 10 s, scaled to items/min (D68)', [NO('T30')],
     [SUB('D44', 'D69', 'D70'), MUL('D70', 'K6', 'D72'), MOV('D72', 'D68'), MOV('D44', 'D69')])
rung('60 s window timer', [NO('M0'), NC('T31')], [TMR('OUT', 'T31', 'K600')])
rung('Rate over last 60 s = items/min (D60)', [NO('T31')], [SUB('D44', 'D61', 'D60'), MOV('D44', 'D61')])

section('9. Outputs and safe state',
        'Pushers retract whenever the line is not running.')
rung('Safe state: retract all pushers when stopped', [NC('M0')], [RST('Y2'), RST('Y3'), RST('Y4')])
rung('Green lamp: running without fault', [NO('M0'), NC('M1')], [OUT('Y10')])
rung('Red lamp: fault present', [NO('M1')], [OUT('Y11')])


# ------------------------------------------------------------------ interpreter
def s16(x):
    return ((int(x) + 32768) & 0xFFFF) - 32768


class PLC:
    def __init__(self, dt_ms=1.0, rungs=None):
        self.rungs = RUNGS if rungs is None else rungs
        self.tkind = {}
        self.bits = {}
        self.words = {}
        self.prev = {}
        self.tacc = {}
        self.dt = dt_ms
        self.first = True
        self.scan_count = 0

    # --- device access
    def _word_name(self, dev):
        m = re.match(r'^D(\d+)Z(\d+)$', dev)
        if m:
            return f'D{int(m.group(1)) + self.words.get("Z" + m.group(2), 0)}'
        return dev

    def get(self, dev):
        if isinstance(dev, int):
            return dev
        if dev.startswith('K'):
            return int(dev[1:])
        if dev.startswith('TC'):                       # timer current value (ticks)
            n = 'T' + dev[2:]
            return int(self.tacc.get(n, 0) / (100 if self.tkind.get(n, 'OUT') == 'OUT' else 10) + 1e-6)
        if dev[0] in 'DZ':
            return self.words.get(self._word_name(dev), 0)
        if dev == 'SM400':
            return True
        if dev == 'SM402':
            return self.first
        return self.bits.get(dev, False)

    def setw(self, dev, v):
        self.words[self._word_name(dev)] = s16(v)

    def setb(self, dev, v):
        self.bits[dev] = bool(v)

    # --- condition evaluation
    def contact(self, c, key):
        t = c[0]
        if t == 'NO':
            return bool(self.get(c[1]))
        if t == 'NC':
            return not self.get(c[1])
        if t in ('RISE', 'FALL'):
            cur = bool(self.get(c[1]))
            prev = self.prev.get(key, False)
            self.prev[key] = cur
            return (cur and not prev) if t == 'RISE' else (prev and not cur)
        if t == 'CMP':
            _, op, a, b = c
            a, b = self.get(a), self.get(b)
            return {'=': a == b, '<>': a != b, '>': a > b, '>=': a >= b, '<': a < b, '<=': a <= b}[op]
        raise ValueError(c)

    def cond(self, conds, ri):
        res = True
        for ci, c in enumerate(conds):
            if c[0] == 'OR':
                any_on = False
                for bi, br in enumerate(c[1]):
                    items = br if isinstance(br, tuple) and br and isinstance(br[0], tuple) else (br,)
                    v = True
                    for k, it in enumerate(items):
                        v = self.contact(it, (ri, ci, bi, k)) and v
                    any_on = any_on or v
                res = res and any_on
            else:
                res = self.contact(c, (ri, ci)) and res
        return res

    def act(self, a, on, ri, ai):
        op = a[0]
        if op == 'OUT':
            self.setb(a[1], on)
        elif op == 'TMR':
            _, kind, name, preset = a
            unit = 100 if kind == 'OUT' else 10
            self.tkind[name] = kind
            if on:
                self.tacc[name] = self.tacc.get(name, 0) + self.dt
                self.bits[name] = self.tacc[name] >= self.get(preset) * unit
            else:
                self.tacc[name] = 0
                self.bits[name] = False
        elif not on:
            return
        elif op == 'SET':
            self.setb(a[1], True)
        elif op == 'RST':
            self.setb(a[1], False)
        elif op == 'MOV':
            self.setw(a[2], self.get(a[1]))
        elif op == 'INC':
            self.setw(a[1], self.get(a[1]) + 1)
        elif op == 'ADD':
            self.setw(a[3], self.get(a[1]) + self.get(a[2]))
        elif op == 'SUB':
            self.setw(a[3], self.get(a[1]) - self.get(a[2]))
        elif op == 'MUL':
            p = self.get(a[1]) * self.get(a[2])
            self.setw(a[3], p & 0xFFFF)
            nxt = re.sub(r'(\d+)$', lambda m: str(int(m.group(1)) + 1), a[3])
            self.setw(nxt, (p >> 16) & 0xFFFF)
        elif op == 'DIV':
            x, y = self.get(a[1]), self.get(a[2])
            if y == 0:
                raise ZeroDivisionError(f'rung {ri}')
            q = int(x / y)
            self.setw(a[3], q)
            nxt = re.sub(r'(\d+)$', lambda m: str(int(m.group(1)) + 1), a[3])
            self.setw(nxt, x - q * y)
        elif op == 'WAND':
            self.setw(a[3], self.get(a[1]) & self.get(a[2]))
        elif op == 'ZRST':
            lo, hi = a[1], a[2]
            kind = lo[0]
            for n in range(int(lo[1:]), int(hi[1:]) + 1):
                if kind == 'D':
                    self.words[f'D{n}'] = 0
                else:
                    self.bits[f'{kind}{n}'] = False
        else:
            raise ValueError(a)

    def scan(self):
        for ri, r in enumerate(self.rungs):
            on = self.cond(r['cond'], ri)
            for ai, a in enumerate(r['acts']):
                self.act(a, on, ri, ai)
        self.first = False
        self.scan_count += 1


if __name__ == '__main__' and not globals().get('_BUNDLED'):
    print(len(RUNGS), 'rungs;', len(SECTIONS), 'sections')


# ====================================================================================================
#  1b. SIMPLE PLC PROGRAM (default, about 50 rungs)
# ====================================================================================================


RUNGS_S = []
SECTIONS_S = []


def section(title, note=''):
    SECTIONS_S.append((len(RUNGS_S), title, note))


def rung(title, cond, acts):
    RUNGS_S.append(dict(title=title, cond=cond, acts=acts))


ALWAYS = [NO('SM400')]

section('1. Start-up and counter reset',
        'Default parameters are loaded on the first scan. The HMI "Reset counters" button (M310, only while stopped) clears all KPIs and the grade queue.')
rung('First scan: clear KPIs and queue, default feed interval (0.1 s units) and crate size', [NO('SM402')],
     [ZRST('D40', 'D69'), ZRST('D20', 'D23'), MOV('K20', 'D240'), MOV('K10', 'D230')])
rung('HMI reset: clear counters, KPIs and queue pointers', [RISE('M310'), NC('M0')],
     [ZRST('D40', 'D69'), ZRST('D20', 'D23'), RST('Y5'), RST('Y6'), RST('Y7')])

section('2. Start / stop, conveyor, faults',
        'M0 = line running. Two faults: E-stop (M50, X2 is wired normally closed) and belt jam (M51, size beam blocked 5 s). X3 clears the jam once it is physically cleared.')
rung('Fault F1: E-stop pressed', [NC('X2')], [OUT('M50')])
rung('Fault F2: jam timer, belt running but size beam blocked for 5 s', [NO('X5'), NO('Y0')], [TMR('OUT', 'T24', 'K50')])
rung('Fault F2: latch jam', [NO('T24')], [SET('M51')])
rung('Fault reset button', [RISE('X3'), NO('X2')], [RST('M51')])
rung('Any fault', [OR(NO('M50'), NO('M51'))], [OUT('M1')])
rung('Line run latch (start, stop, fault)', [OR(NO('X0'), NO('M0')), NC('X1'), NC('M1')], [OUT('M0')])
rung('Conveyor motor', [NO('M0')], [OUT('Y0')])
rung('Green lamp: running', [NO('M0')], [OUT('Y10')])
rung('Red lamp: fault', [NO('M1')], [OUT('Y11')])

section('3. Automated feeder',
        'While running, the gate opens for 0.5 s at the end of every D240 x 0.1 s (default 2 s).')
rung('Feed interval timer', [NO('M0'), NC('T1')], [TMR('OUT', 'T1', 'D240')])
rung('Feeder gate: opens when the interval ends, stays open until the 0.5 s timer ends', [OR(NO('T1'), NO('Y1')), NC('T2')], [OUT('Y1')])
rung('Gate-open timer (0.5 s)', [NO('Y1')], [TMR('OUT', 'T2', 'K5')])

section('4. Layer 1: size gauge (beam break timing)',
        'Timer T20 counts 10 ms ticks while the beam is blocked. At 100 mm/s one tick = 1 mm, so the timer value is the onion diameter in mm. At the falling edge the grade code is set: 1 = A (70-<90), 2 = B (55-<70), 3 = C (40-<55), 4 = reject (<40 or >=90).')
rung('Beam clear: show measured size on HMI, default grade = reject', [FALL('X5')], [MOV('TC20', 'D11'), MOV('K4', 'D10')])
rung('Size >= 40 mm: grade C', [FALL('X5'), CMP('>=', 'D11', 'K40')], [MOV('K3', 'D10')])
rung('Size >= 55 mm: grade B', [FALL('X5'), CMP('>=', 'D11', 'K55')], [MOV('K2', 'D10')])
rung('Size >= 70 mm: grade A', [FALL('X5'), CMP('>=', 'D11', 'K70')], [MOV('K1', 'D10')])
rung('Size >= 90 mm: oversize, reject', [FALL('X5'), CMP('>=', 'D11', 'K90')], [MOV('K4', 'D10')])
rung('Size timer (10 ms ticks; TC20 = diameter in mm). Must stay AFTER the rungs above', [NO('X5')], [TMR('OUTH', 'T20', 'K999')])

section('5. Layer 2: sprout camera and grade queue',
        'The camera result (X6) is remembered while the onion is in view (X16). When the onion leaves the view its final grade is stored in the 16-entry ring D100-D115 (write pointer D20). Sprouted = reject.')
rung('Sprout seen while onion is in camera view', [NO('X6')], [SET('M7')])
rung('Onion leaves camera view: store size grade in the queue', [FALL('X16')], [MOV('D20', 'Z0'), MOV('D10', 'D100Z0')])
rung('...sprouted: overwrite with reject', [FALL('X16'), NO('M7')], [MOV('K4', 'D100Z0')])
rung('...advance write pointer (ring of 16), clear sprout flag', [FALL('X16')], [INC('D20'), WAND('D20', 'K15', 'D20'), RST('M7')])

section('6. Separation: pushers A, B, C and reject',
        'Each station has its own read pointer (D21, D22, D23). Station A reads every onion. B skips onions already pushed at A (grade 1), C also skips grade 2. Grade 4 passes all stations and falls into the reject bin at the belt end. Pointer Z1..Z3 select the queue entry.')
rung('Point index registers at the stations\' queue entries', ALWAYS, [MOV('D21', 'Z1'), MOV('D22', 'Z2'), MOV('D23', 'Z3')])
rung('Pusher A: onion passed X7 and is grade 1 (held 0.3 s)',
     [OR(((FALL('X7'), CMP('<>', 'D21', 'D20'), CMP('=', 'D100Z1', 'K1'))), NO('Y2')), NC('T3')], [OUT('Y2')])
rung('Pusher A dwell timer (0.3 s)', [NO('Y2')], [TMR('OUT', 'T3', 'K3')])
rung('Station A: onion consumed from queue', [FALL('X7'), CMP('<>', 'D21', 'D20')], [INC('D21'), WAND('D21', 'K15', 'D21')])
rung('Station B: skip onions pushed at A', [CMP('<>', 'D22', 'D20'), CMP('=', 'D100Z2', 'K1')], [INC('D22'), WAND('D22', 'K15', 'D22')])
rung('Pusher B: onion passed X10 and is grade 2 (held 0.3 s)',
     [OR(((FALL('X10'), CMP('<>', 'D22', 'D20'), CMP('=', 'D100Z2', 'K2'))), NO('Y3')), NC('T4')], [OUT('Y3')])
rung('Pusher B dwell timer (0.3 s)', [NO('Y3')], [TMR('OUT', 'T4', 'K3')])
rung('Station B: onion consumed from queue', [FALL('X10'), CMP('<>', 'D22', 'D20')], [INC('D22'), WAND('D22', 'K15', 'D22')])
rung('Station C: skip onions pushed at A or B', [CMP('<>', 'D23', 'D20'), CMP('<=', 'D100Z3', 'K2')], [INC('D23'), WAND('D23', 'K15', 'D23')])
rung('Pusher C: onion passed X11 and is grade 3 (held 0.3 s)',
     [OR(((FALL('X11'), CMP('<>', 'D23', 'D20'), CMP('=', 'D100Z3', 'K3'))), NO('Y4')), NC('T5')], [OUT('Y4')])
rung('Pusher C dwell timer (0.3 s)', [NO('Y4')], [TMR('OUT', 'T5', 'K3')])
rung('Station C: onion consumed from queue', [FALL('X11'), CMP('<>', 'D23', 'D20')], [INC('D23'), WAND('D23', 'K15', 'D23')])

section('7. KPIs: quantity per grade, reject %, rate, accuracy',
        'All counters are counted when the grade is decided (camera exit). D40-D43 = A, B, C, reject; D44 = total; D45 = reject %; D60 = items per minute (60 s window); D64 = classification accuracy %. The accuracy reference D302 is the true grade (manual audit / reference system; supplied by the simulation).')
rung('Count grade A (total and current crate)', [FALL('X16'), CMP('=', 'D100Z0', 'K1')], [INC('D40'), INC('D50')])
rung('Count grade B (total and current crate)', [FALL('X16'), CMP('=', 'D100Z0', 'K2')], [INC('D41'), INC('D51')])
rung('Count grade C (total and current crate)', [FALL('X16'), CMP('=', 'D100Z0', 'K3')], [INC('D42'), INC('D52')])
rung('Count rejects', [FALL('X16'), CMP('=', 'D100Z0', 'K4')], [INC('D43')])
rung('Total processed', ALWAYS, [ADD('D40', 'D41', 'D44'), ADD('D44', 'D42', 'D44'), ADD('D44', 'D43', 'D44')])
rung('Reject rate %', [CMP('>', 'D44', 'K0')], [MUL('D43', 'K100', 'D47'), DIV('D47', 'D44', 'D45')])
rung('60 s window timer (runs only while the line runs)', [NO('M0'), NC('T31')], [TMR('OUT', 'T31', 'K600')])
rung('Processing rate: items finished in the last 60 s = items/min', [NO('T31')], [SUB('D44', 'D61', 'D60'), MOV('D44', 'D61')])
rung('Accuracy: count checked items', [FALL('X16')], [INC('D62')])
rung('Accuracy: count correct (assigned grade = true grade D302)', [FALL('X16'), CMP('=', 'D100Z0', 'D302')], [INC('D63')])
rung('Accuracy % = correct x 100 / checked', [CMP('>', 'D62', 'K0')], [MUL('D63', 'K100', 'D66'), DIV('D66', 'D62', 'D64')])

section('8. Packaging: crates and robot',
        'A crate is full at D230 onions (default 10). The PLC raises "crate ready" (Y5/Y6/Y7); the robot palletizes it and answers with M320/M321/M322; the PLC then clears the signal and counts the packed crate (D54-D56).')
rung('Crate A ready for the robot', [CMP('>=', 'D50', 'D230')], [SET('Y5')])
rung('Crate B ready for the robot', [CMP('>=', 'D51', 'D230')], [SET('Y6')])
rung('Crate C ready for the robot', [CMP('>=', 'D52', 'D230')], [SET('Y7')])
rung('Crate A palletized (robot answer M320)', [RISE('M320'), NO('Y5')], [RST('Y5'), SUB('D50', 'D230', 'D50'), INC('D54')])
rung('Crate B palletized (robot answer M321)', [RISE('M321'), NO('Y6')], [RST('Y6'), SUB('D51', 'D230', 'D51'), INC('D55')])
rung('Crate C palletized (robot answer M322)', [RISE('M322'), NO('Y7')], [RST('Y7'), SUB('D52', 'D230', 'D52'), INC('D56')])

if __name__ == '__main__' and not globals().get('_BUNDLED'):
    print(len(RUNGS_S), 'rungs;', len(SECTIONS_S), 'sections')


# ====================================================================================================
#  2a. ARM KINEMATICS (pure Python inverse kinematics for library robots)
# ====================================================================================================

import math
import random


def vadd(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def vsub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def vmul(a, s): return [a[0] * s, a[1] * s, a[2] * s]
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def norm(a): return math.sqrt(dot(a, a))
def unit(a):
    n = norm(a)
    return [a[0] / n, a[1] / n, a[2] / n]


def quat_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def mat_col(m, j): return [m[0][j], m[1][j], m[2][j]]
def mat_vec(m, v): return [dot(m[0], v), dot(m[1], v), dot(m[2], v)]


def rotate_about(point, axis_p, axis_w, theta):
    """Rotate `point` by theta about the line through axis_p with unit direction axis_w (Rodrigues)."""
    v = vsub(point, axis_p)
    c, s = math.cos(theta), math.sin(theta)
    k = cross(axis_w, v)
    d = dot(axis_w, v)
    return [axis_p[i] + v[i] * c + k[i] * s + axis_w[i] * d * (1 - c) for i in range(3)]


def rotate_dir(vec, axis_w, theta):
    c, s = math.cos(theta), math.sin(theta)
    k = cross(axis_w, vec)
    d = dot(axis_w, vec)
    return [vec[i] * c + k[i] * s + axis_w[i] * d * (1 - c) for i in range(3)]


def solve_linear(a, b):
    """Solve a x = b for a small dense square matrix (Gaussian elimination with partial pivoting)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-14:
            continue
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            if f:
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = m[i][n] - sum(m[i][k] * x[k] for k in range(i + 1, n))
        x[i] = s / m[i][i] if abs(m[i][i]) > 1e-14 else 0.0
    return x


class ArmKin:
    """joints: list of dict(p=[x,y,z], w=[unit axis]) in the world at configuration q_ref.
    tool_p / tool_a: tool point and outward tool axis (unit) in the world at q_ref."""

    def __init__(self, joints, q_ref, tool_p, tool_a, limits=None):
        self.n = len(joints)
        self.jp = [list(j['p']) for j in joints]
        self.jw = [unit(j['w']) for j in joints]
        self.q_ref = list(q_ref)
        self.tp = list(tool_p)
        self.ta = unit(tool_a)
        self.limits = limits or [(-2 * math.pi, 2 * math.pi)] * self.n
        self.q_home = list(q_ref)

    # ---- forward kinematics (product of exponentials about the reference screw axes)
    def frames(self, q):
        """World origin and axis of every joint, plus tool point and axis, at joint values q."""
        d = [q[i] - self.q_ref[i] for i in range(self.n)]
        ps, ws = [], []
        for i in range(self.n):
            p, w = self.jp[i], self.jw[i]
            for k in range(i - 1, -1, -1):
                p = rotate_about(p, self.jp[k], self.jw[k], d[k])
                w = rotate_dir(w, self.jw[k], d[k])
            ps.append(p)
            ws.append(w)
        tp, ta = self.tp, self.ta
        for k in range(self.n - 1, -1, -1):
            tp = rotate_about(tp, self.jp[k], self.jw[k], d[k])
            ta = rotate_dir(ta, self.jw[k], d[k])
        return ps, ws, tp, ta

    def tool(self, q):
        _, _, tp, ta = self.frames(q)
        return tp, ta

    # ---- inverse kinematics: position + tool axis direction (roll about the tool axis is free)
    def solve(self, q0, target_p, target_a, iters=12, damping=0.02, posture=0.02, w_axis=0.35, max_dq=0.2, tol=0.0008):
        """Damped least squares on [position error, w_axis * axis error] with a null-space pull towards q_home."""
        q = list(q0)
        ta_des = unit(target_a)
        n = self.n
        for _ in range(iters):
            ps, ws, tp, ta = self.frames(q)
            ep = vsub(target_p, tp)
            ea = cross(ta, ta_des)
            if norm(ep) < tol and norm(ea) < 0.004:
                break
            e = [ep[0], ep[1], ep[2], w_axis * ea[0], w_axis * ea[1], w_axis * ea[2]]
            Jm = [[0.0] * n for _ in range(6)]
            for i in range(n):
                jv = cross(ws[i], vsub(tp, ps[i]))
                for r in range(3):
                    Jm[r][i] = jv[r]
                    Jm[r + 3][i] = w_axis * ws[i][r]
            JtJ = [[sum(Jm[r][a] * Jm[r][b] for r in range(6)) for b in range(n)] for a in range(n)]
            lam = damping * damping
            A = [[JtJ[a][b] + (lam if a == b else 0.0) for b in range(n)] for a in range(n)]
            Jte = [sum(Jm[r][a] * e[r] for r in range(6)) for a in range(n)]
            dq = solve_linear(A, Jte)
            if posture:
                # null-space projector N = I - A^-1 JtJ  (so the posture pull does not fight the task)
                cols = [solve_linear(A, [JtJ[r][c] for r in range(n)]) for c in range(n)]       # A^-1 JtJ, column c
                pull = [posture * (self.q_home[i] - q[i]) for i in range(n)]
                for i in range(n):
                    dq[i] += pull[i] - sum(cols[c][i] * pull[c] for c in range(n))
            m = max(abs(x) for x in dq)
            if m > max_dq:
                dq = [x * max_dq / m for x in dq]
            q = [min(max(q[i] + dq[i], self.limits[i][0]), self.limits[i][1]) for i in range(n)]
        ps, ws, tp, ta = self.frames(q)
        return q, norm(vsub(target_p, tp)), math.degrees(math.acos(max(-1.0, min(1.0, dot(ta, ta_des)))))

    def solve_global(self, target_p, target_a, tries=60, seed=1, prefer_elbow_up=True):
        """Multi-start search for a good starting posture (used once, at start-up)."""
        rnd = random.Random(seed)
        best = None
        for t in range(tries):
            q = [min(max(self.q_home[i] + (0.0 if t == 0 else rnd.uniform(-1.6, 1.6)), self.limits[i][0]), self.limits[i][1]) for i in range(self.n)]
            q, ep, ea = self.solve(q, target_p, target_a, iters=80)
            if ep > 0.003 or ea > 1.5:
                continue
            ps, _, _, _ = self.frames(q)
            elbow = ps[min(2, self.n - 1)][2] if prefer_elbow_up else 0.0
            cost = -elbow * 5 + sum(abs(q[i] - self.q_home[i]) for i in range(self.n)) * 0.2
            if best is None or cost < best[0]:
                best = (cost, q, ep, ea)
        return None if best is None else (best[1], best[2], best[3])


# ====================================================================================================
#  2. LINE MODEL (the plant)
# ====================================================================================================

import math
import random


# ------------------------------------------------------------------ geometry (mm)
DT = 0.010                     # model step, s
SCANS_PER_STEP = 5             # PLC scans per step (2 ms each)
BELT_X0, BELT_X1 = -150.0, 1850.0
BELT_HALF_W = 110.0
BELT_TOP = 800.0
X_FEED = 0.0
X_BEAM = 150.0                 # layer 1: size gauge (beam X5)
X_CAM = 270.0                  # layer 2: sprout inspection camera (X16 zone, X6 result)
XS = {'A': 700.0, 'B': 1100.0, 'C': 1500.0}      # pusher stations, 400 mm apart (one crate station each)
PUSH_OFF, PUSH_HALF = 20.0, 40.0
PE_LEAD = 60.0                 # external PLC: the 'onion at pusher' sensor starts this far upstream of the pusher centre
PUSH_RETRACT, PUSH_EXTEND, PUSH_SPEED = -130.0, 150.0, 1400.0   # front face y, mm and mm/s
BIN_X0, BIN_X1, BIN_HALF_Y, BIN_FLOOR = 1870.0, 2120.0, 140.0, 300.0
REJECT_VISIBLE = 40
SLIDE_S = 0.45                 # time an onion needs to slide from the belt into its crate, s

TABLE_TOP = 400.0                              # crate stands, roller track and pallet top
CRATE_X, CRATE_Y, CRATE_H = 240.0, 340.0, 80.0 # crate footprint 240 mm (x) by 340 mm (y), 2 x 3 cells
CRATE_POS = {g: (XS[g] + PUSH_OFF, 340.0) for g in 'ABC'}      # station beside the belt (receives the pushed onions)
READY_POS = {g: (XS[g] + PUSH_OFF, 800.0) for g in 'ABC'}      # a full crate rolls here and waits for the robot
CRATE_TOP = TABLE_TOP + CRATE_H
CELL_DX = (-52.0, 52.0)
CELL_DY = (-100.0, 0.0, 100.0)
CRATE_CAPACITY = 6
CRATE_SPEED = 250.0            # roller track speed, mm/s
PALLET_XY = (1100.0, 1950.0)
PALLET_TOP = TABLE_TOP
PALLET_SLOTS = [(PALLET_XY[0] + dx, PALLET_XY[1] + dy, layer) for layer in range(2)
                for dy in (-190.0, 190.0) for dx in (-270.0, 0.0, 270.0)]
CARRY_OFFSET = 150.0           # tool z above the centre of a carried crate
Z_SAFE = {'palletizer': 900.0}
ARM_BASE = {'palletizer': (1100.0, 1350.0, 600.0)}     # robot base (pedestal top), mm
ARM_HOME = {'palletizer': (1450.0, 1350.0, 900.0)}
ARM_SPEED = {'palletizer': (800.0, 600.0)}             # xy, z speed in mm/s
Z_CRATE_GRIP = TABLE_TOP + CRATE_H / 2 + CARRY_OFFSET
ROUTE = {'palletizer': (260.0, 330.0, 180.0)}   # keep-out distance from base axis (mm), detour radius (mm), 'cut' direction the base joint never crosses (deg)
ONION_H = 0.85                 # onion height / diameter
DWELL = 0.20                   # pause of an arm after gripping / releasing, s
PALLET_CLEAR_S = 4.0
GRADE_OF = {'A': 1, 'B': 2, 'C': 3}


def true_grade(d, sprouted):
    if sprouted or d < 40 or d >= 90:
        return 4
    if d >= 70:
        return 1
    if d >= 55:
        return 2
    return 3


class OnionSource:
    def __init__(self, seed=1, p_sprout=0.12, p_odd=0.07, limit=None):
        self.rnd = random.Random(seed)
        self.p_sprout, self.p_odd = p_sprout, p_odd
        self.limit, self.count = limit, 0

    def next(self):
        if self.limit is not None and self.count >= self.limit:
            return None
        self.count += 1
        r = self.rnd.random()
        if r < self.p_sprout:
            return float(self.rnd.randint(45, 88)), True
        if r < self.p_sprout + self.p_odd:
            return float(self.rnd.choice([30, 34, 38, 91, 95, 104])), False
        return float(self.rnd.randint(41, 88)), False


class Onion:
    _n = 0

    def __init__(self, d, sprouted, x, y, z):
        Onion._n += 1
        self.id = Onion._n
        self.d, self.r, self.sprouted = d, d / 2.0, sprouted
        self.grade = true_grade(d, sprouted)
        self.x, self.y, self.z = x, y, z
        self.state = 'belt'
        self.cam_checked = False
        self.sprout_seen = False
        self.dest = None          # 'A','B','C','R'
        self.lane = None
        self.crate = None
        self.cell = None
        self.vz = 0.0
        self.gone = False         # visual object should be removed

    @property
    def h(self):
        return self.d * ONION_H


class Crate:
    _n = 0

    def __init__(self, g):
        Crate._n += 1
        self.id = Crate._n
        self.g = g
        self.x, self.y = CRATE_POS[g]
        self.z = TABLE_TOP + CRATE_H / 2
        self.onions = [None] * CRATE_CAPACITY
        self.res = set()           # cells reserved by onions that are still sliding in
        self.extra = []            # onions that arrived while the crate was already full (move to the next crate)
        self.state = 'station'     # station | moving | ready | carried | pallet
        self.gone = False

    @property
    def count(self):
        return sum(1 for o in self.onions if o is not None)

    def cell_xy(self, i):
        return self.x + CELL_DX[i % 2], self.y + CELL_DY[(i // 2) % 3]


class LineModel:
    def __init__(self, seed=1, source=None, p_detect=1.0, p_false=0.0, feed_interval_s=3.0,
                 auto_start=True, crate_capacity=CRATE_CAPACITY, reject_capacity=10_000, ladder='simple', plc=None):
        # plc=None: the built-in ladder interpreter controls the line.  plc=<ExtPlc>: an external PLC (OpenPLC over OPC UA) does.
        self.external = plc is not None
        self.ladder = 'simple' if self.external else ladder
        self.plc = plc if self.external else PLC(dt_ms=DT * 1000 / SCANS_PER_STEP, rungs=RUNGS_S if ladder == 'simple' else None)
        self.plc.bits['X2'] = True
        if not self.external:
            self.plc.scan()                               # first scan loads the defaults
        self.plc.words['D240'] = int(round(feed_interval_s * 10))
        self.plc.words['D230'] = crate_capacity
        self.plc.words['D231'] = crate_capacity + 3
        self.source = source or OnionSource(seed)
        self.rnd = random.Random(seed + 1000)
        self.p_detect, self.p_false = p_detect, p_false
        self.crate_capacity = crate_capacity
        self.reject_capacity = reject_capacity
        self.t = 0.0
        self.auto_start = auto_start
        self.started = False
        self.onions = []
        self.ready = {g: None for g in 'ABC'}
        self.reject_bin = []
        self.crates = {g: Crate(g) for g in 'ABC'}
        self.crate_list = list(self.crates.values())     # every crate that is currently visible
        self.pallet_crates = []
        self.pallet_slot = 0
        self.pallet_full_t = None
        self.stats = dict(spawned=0, packed={g: 0 for g in 'ABC'}, crates_done={g: 0 for g in 'ABC'},
                          pallets_done=0, misrouted=0, rejected=0, onions_on_pallet=0)
        self.plate = {g: PUSH_RETRACT for g in 'ABC'}
        self.bin_pulse = {g: 0.0 for g in 'ABC'}
        self.prev_y1 = False
        self.stripe = 0.0
        self.gate_open = False
        self.fail_pusher = set()
        self.beam_stuck = False
        self.pending_inputs = []         # (device, release_time)
        self.ack_pulse = {}
        self.last_cam_grade = 1
        self.cam_info = dict(id=0, d=0.0, sprouted=False, grade=0)     # last onion seen by the camera (for an external PLC)
        self.pushed = {g: 0 for g in 'ABC'}                              # id of the last onion pushed into each crate
        if self.external:
            self.plc.attach(self)
        # robot
        self.arms = {'palletizer': dict(name='palletizer', x=ARM_HOME['palletizer'][0], y=ARM_HOME['palletizer'][1],
                                        z=ARM_HOME['palletizer'][2], plan=[], wait=0.0, carrying_crate=None, busy=False, cycles=0,
                                        vxy=ARM_SPEED['palletizer'][0], vz=ARM_SPEED['palletizer'][1])}
        self.log = []
        self.trash = []            # onions whose visuals the view should delete
        self.hist = []             # total processed, sampled once per simulated second (for the 10 s rate)
        self._k = 0
        self.events = []           # one record per onion once it is routed (used by the dashboard)
        self.trash_crates = []     # crates whose visuals the view should delete

    # ---------------------------------------------------------------- operator inputs
    def press(self, dev, seconds=0.1):
        self.plc.bits[dev] = True
        self.pending_inputs.append((dev, self.t + seconds))

    def set_estop(self, pressed):
        self.plc.bits['X2'] = not pressed

    # ---------------------------------------------------------------- sensors -> PLC inputs
    def _blocked(self, x0, o):
        # half-open interval: an item of integer width d blocks a sensor for exactly d samples at 1 mm/step,
        # so the 10 ms size timers read exactly d ticks (no off-by-one at the grade boundaries)
        return o.state == 'belt' and (x0 - o.d / 2.0) <= o.x < (x0 + o.d / 2.0)

    def update_sensors(self):
        b, w = self.plc.bits, self.plc.words
        on_belt = [o for o in self.onions if o.state == 'belt']
        b['X5'] = self.beam_stuck or any(self._blocked(X_BEAM, o) for o in on_belt)
        cam = [o for o in on_belt if self._blocked(X_CAM, o)]
        for o in cam:
            if not o.cam_checked:
                o.cam_checked = True
                o.sprout_seen = (self.rnd.random() < self.p_detect) if o.sprouted else (self.rnd.random() < self.p_false)
                self.cam_info = dict(id=o.id, d=o.d, sprouted=o.sprout_seen, grade=o.grade)
        b['X16'] = bool(cam)
        b['X6'] = any(o.sprout_seen for o in cam)
        if cam:
            self.last_cam_grade = cam[0].grade
        w['D302'] = self.last_cam_grade
        for dev, g in (('X7', 'A'), ('X10', 'B'), ('X11', 'C')):
            b[dev] = any(self._blocked(XS[g], o) for o in on_belt)
        for dev, g in (('X12', 'A'), ('X13', 'B'), ('X14', 'C')):
            b[dev] = self.bin_pulse[g] > 0
        b['X15'] = len(self.reject_bin) >= self.reject_capacity
        b['X4'] = bool(b.get('X4', False))

    def plc_status(self):
        if self.external:
            return self.plc.status()
        return dict(mode='Built-in ladder (' + ('simple' if self.ladder == 'simple' else 'full') + ')', ext=False, link=True, ms=0)

    def ext_inputs(self):
        """Sensor values for an external PLC (OPC UA tags, see plc_tags.py): ids and windows, not 10 ms beam timing."""
        on_belt = [o for o in self.onions if o.state == 'belt']
        ci = self.cam_info
        d = dict(CAM_ID=ci['id'], DIAMETER_MM=float(ci['d']), SPROUTED=bool(ci['sprouted']), TRUE_GRADE=int(ci['grade']),
                 BEAM_BLOCKED=bool(self.beam_stuck or any(self._blocked(X_BEAM, o) for o in on_belt)),
                 PALLET_FULL=self.pallet_full_t is not None or self.pallet_slot >= len(PALLET_SLOTS))
        for g in 'ABC':
            xp = XS[g] + PUSH_OFF
            near = [o for o in on_belt if xp - PE_LEAD <= o.x <= xp + PUSH_HALF and o.y < BELT_HALF_W]
            near.sort(key=lambda o: abs(o.x - xp))
            d[f'PE_{g}_ID'] = near[0].id if near else 0
            d[f'PUSHED_{g}_ID'] = self.pushed[g]
            c = self.ready[g]
            d[f'CRATE_{g}_READY'] = bool(c is not None and c.state == 'ready')
        return d

    # ---------------------------------------------------------------- actuators
    def step(self):
        # operator inputs that expire
        for item in list(self.pending_inputs):
            if self.t >= item[1]:
                self.plc.bits[item[0]] = False
                self.pending_inputs.remove(item)
        for dev in list(self.ack_pulse):
            self.ack_pulse[dev] -= 1
            if self.ack_pulse[dev] <= 0:
                self.plc.bits[dev] = False
                del self.ack_pulse[dev]
        if self.auto_start and not self.started and self.t >= 0.5 and (not self.external or self.plc.ready()):
            self.started = True
            self.press('X0', 0.1)

        self.update_sensors()
        for _ in range(1 if self.external else SCANS_PER_STEP):
            self.plc.scan()
        b = self.plc.bits
        y = lambda n: bool(b.get(n, False))

        # feeder: a rising edge of Y1 drops one onion
        self.gate_open = y('Y1')
        if y('Y1') and not self.prev_y1:
            nxt = self.source.next()
            if nxt is not None:                      # None = hopper empty
                d, sp = nxt
                self.onions.append(Onion(d, sp, X_FEED, 0.0, BELT_TOP + d * ONION_H / 2 + 120.0))
                self.stats['spawned'] += 1
        self.prev_y1 = y('Y1')

        v = self.plc.words.get('D200', 100)        # belt speed, mm/s
        belt_on = y('Y0')
        if belt_on:
            self.stripe = (self.stripe + v * DT) % 100.0

        self._move_onions(belt_on, v)
        self._pushers()
        self._slides()
        self._crates_move()
        self._robot()
        for g in 'ABC':
            if self.bin_pulse[g] > 0:
                self.bin_pulse[g] -= DT
        gone = [o for o in self.onions if o.gone and o.state == 'removed']
        if gone:
            self.trash.extend(gone)
            self.onions = [o for o in self.onions if not (o.gone and o.state == 'removed')]
        self.t += DT
        self._k += 1
        if self._k % 100 == 0:
            self.hist.append(self.plc.words.get('D44', 0))
            del self.hist[:-12]

    # ---------------------------------------------------------------- belt, falling, rejects
    def _move_onions(self, belt_on, v):
        for o in self.onions:
            if o.state == 'belt':
                if o.z > BELT_TOP + o.h / 2:                 # still dropping from the feeder
                    o.vz += -9810.0 * DT
                    o.z = max(BELT_TOP + o.h / 2, o.z + o.vz * DT)
                    if o.z <= BELT_TOP + o.h / 2:
                        o.vz = 0.0
                if belt_on:
                    o.x += v * DT
                if o.x > BELT_X1:
                    o.state = 'fall'
                    o.vz = 0.0
            elif o.state == 'fall':
                o.x += v * DT
                o.vz += -9810.0 * DT
                o.z += o.vz * DT
                if o.z <= BIN_FLOOR + o.h / 2 + 5:
                    o.state = 'reject'
                    o.dest = 'R'
                    o.x = BIN_X0 + 20 + (len(self.reject_bin) * 53) % (BIN_X1 - BIN_X0 - 40)
                    o.y = -BIN_HALF_Y + 25 + ((len(self.reject_bin) * 37) % int(2 * BIN_HALF_Y - 50))
                    o.z = BIN_FLOOR + o.h / 2 + 5 + (len(self.reject_bin) // 12) * 40
                    self.reject_bin.append(o)
                    self.stats['rejected'] += 1
                    self.events.append(dict(t=self.t, id=o.id, d=o.d, sp=o.sprouted, tg=o.grade, dest='R', seen=o.sprout_seen))
                    if o.grade != 4:
                        self.stats['misrouted'] += 1
                    if len(self.reject_bin) > REJECT_VISIBLE:
                        old = self.reject_bin[len(self.reject_bin) - REJECT_VISIBLE - 1]
                        old.gone, old.state = True, 'removed'

    def _pushers(self):
        b = self.plc.bits
        for g, out in (('A', 'Y2'), ('B', 'Y3'), ('C', 'Y4')):
            target = PUSH_EXTEND if (b.get(out) and g not in self.fail_pusher) else PUSH_RETRACT
            step = PUSH_SPEED * DT
            f = self.plate[g]
            f = min(target, f + step) if target > f else max(target, f - step)
            self.plate[g] = f
            xp = XS[g] + PUSH_OFF
            for o in self.onions:
                if o.state == 'belt' and abs(o.x - xp) <= PUSH_HALF and f + o.r > o.y:
                    o.y = f + o.r
                    if o.y >= BELT_HALF_W + 10:
                        self._start_slide(o, g)
                        o.dest = g
                        self.pushed[g] = o.id
                        self.bin_pulse[g] = 0.3          # crate entry sensor
                        self.events.append(dict(t=self.t, id=o.id, d=o.d, sp=o.sprouted, tg=o.grade, dest=g, seen=o.sprout_seen))
                        if o.grade != GRADE_OF[g]:
                            self.stats['misrouted'] += 1

    def _route(self, name, a, b, z):
        """Via points (at height z) that carry the tool from xy point a to b around the robot's own base column.

        The base joint turns only inside a fixed 360-degree window (it never crosses the 'cut' direction), so it can never wind
        up turn after turn. A straight move is used only when it neither passes near the base nor would cross the cut."""
        bx, by, _ = ARM_BASE[name]
        thr, R, cut_deg = ROUTE[name]
        cut = math.radians(cut_deg)
        ax, ay, cx, cy = a[0] - bx, a[1] - by, b[0] - bx, b[1] - by
        dx, dy = cx - ax, cy - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / L2))
        near = math.hypot(ax + t * dx, ay + t * dy) < thr
        un = lambda ang: (ang - cut) % (2 * math.pi) + cut             # angle in [cut, cut + 2 pi)
        a0, a1 = un(math.atan2(ay, ax)), un(math.atan2(cy, cx))
        d = a1 - a0
        short = (d + math.pi) % (2 * math.pi) - math.pi
        if not near and abs(short - d) < 1e-6:
            return [('move', b[0], b[1], z, None)]
        n = max(1, int(math.ceil(abs(d) / math.radians(40))))
        pts = [(bx + R * math.cos(a0 + d * k / n), by + R * math.sin(a0 + d * k / n)) for k in range(n + 1)]
        return [('move', x, y, z, None) for x, y in pts] + [('move', b[0], b[1], z, None)]

    # ---------------------------------------------------------------- onions slide from the belt into the crate of their grade
    def _free_slot(self, crate):
        for i in range(self.crate_capacity):
            if crate.onions[i] is None and i not in crate.res:
                return i
        return None

    def _start_slide(self, o, g):
        crate = self.crates[g]
        slot = self._free_slot(crate)
        o.crate, o.lane = crate, g
        if slot is None:                       # crate full and still waiting for the roller track: stack on top (rare)
            slot = len(crate.extra) % self.crate_capacity
            crate.extra.append(o)
            o.cell, extra_lvl = slot, len(crate.extra)
        else:
            crate.res.add(slot)
            o.cell, extra_lvl = slot, 0
        cx, cy = crate.cell_xy(slot)
        o.from_, o.dest_xyz = (o.x, o.y, o.z), (cx, cy, CRATE_TOP + o.h / 2 + 4 + extra_lvl * o.h)
        o.prog, o.state = 0.0, 'slide'

    def _slides(self):
        for o in self.onions:
            if o.state != 'slide':
                continue
            o.prog = min(1.0, o.prog + DT / SLIDE_S)
            f = o.prog
            (x0, y0, z0), (x1, y1, z1) = o.from_, o.dest_xyz
            o.x, o.y = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
            o.z = z0 + (z1 - z0) * f * f + 40.0 * math.sin(math.pi * f)       # a little hop over the belt edge
            if o.prog >= 1.0:
                crate = o.crate
                crate.res.discard(o.cell)
                o.state = 'crate'
                if o in crate.extra:
                    pass                                          # it travels with the crate; re-slotted at the next crate change
                else:
                    crate.onions[o.cell] = o
                self.stats['packed'][crate.g] += 1
                self._try_shuttle(crate.g)

    def _place_items(self, c):
        for i, oo in enumerate(c.onions):
            if oo is not None:
                oo.x, oo.y = c.cell_xy(i)
                oo.z = c.z + CRATE_H / 2 + oo.h / 2 + 4
        for k, oo in enumerate(c.extra):
            if oo.state == 'crate':
                oo.x, oo.y = c.cell_xy(oo.cell)
                oo.z = c.z + CRATE_H / 2 + oo.h / 2 + 4 + (k + 1) * oo.h

    def _try_shuttle(self, g):
        """A full station crate rolls to the ready spot (if it is free) and an empty crate takes its place."""
        c = self.crates[g]
        if c.state != 'station' or c.count < self.crate_capacity or c.res or self.ready[g] is not None:
            return
        c.state = 'moving'
        self.ready[g] = c
        new = Crate(g)
        self.crates[g] = new
        self.crate_list.append(new)
        for oo in c.extra:                                          # onions that arrived too late join the new crate
            slot = self._free_slot(new)
            if slot is None:
                break
            new.onions[slot] = oo
            oo.crate, oo.cell = new, slot
        c.extra = []
        self._place_items(new)

    def _crates_move(self):
        for g, c in self.ready.items():
            if c is not None and c.state == 'moving':
                tx, ty = READY_POS[g]
                step = CRATE_SPEED * DT
                c.y = min(ty, c.y + step)
                if c.y >= ty - 1e-6:
                    c.state = 'ready'
                self._place_items(c)
        for g in 'ABC':
            self._try_shuttle(g)

    # ---------------------------------------------------------------- robot (one UR10 palletizing full crates)
    def _plan_palletize(self, arm, g):
        crate = self.ready[g]
        px, py, layer = PALLET_SLOTS[self.pallet_slot]
        zs = Z_SAFE['palletizer']
        z_rel = PALLET_TOP + CRATE_H / 2 + layer * CRATE_H + CARRY_OFFSET + 5
        here = (arm['x'], arm['y'])
        arm['plan'] = (self._route('palletizer', here, (crate.x, crate.y), zs) + [
            ('move', crate.x, crate.y, Z_CRATE_GRIP, None),
            ('grip_crate', crate, None, None, None),
            ('move', crate.x, crate.y, zs, None)]
            + self._route('palletizer', (crate.x, crate.y), (px, py), zs) + [
            ('move', px, py, z_rel, None),
            ('release_crate', crate, (px, py, layer), None, None),
            ('move', px, py, zs, None)])
        self.pallet_slot += 1
        arm['busy'] = True

    def _choose_palletize(self, arm):
        b = self.plc.bits
        if self.pallet_full_t is None and self.pallet_slot < len(PALLET_SLOTS):
            for g, lamp in (('A', 'Y5'), ('B', 'Y6'), ('C', 'Y7')):
                c = self.ready[g]
                if c is not None and c.state == 'ready' and b.get(lamp):
                    self._plan_palletize(arm, g)
                    return

    def _carry_update(self, arm):
        c = arm['carrying_crate']
        if c is not None:
            c.x, c.y, c.z = arm['x'], arm['y'], arm['z'] - CARRY_OFFSET
            self._place_items(c)

    def _robot(self):
        if self.pallet_full_t is not None and self.t >= self.pallet_full_t:
            self._clear_pallet()
        self._arm_step(self.arms['palletizer'])

    def arms_idle(self):
        return all(not a['plan'] and a['wait'] <= 0 and a['carrying_crate'] is None for a in self.arms.values())

    def waiting_crates(self):
        return {g: (1 if self.ready[g] is not None else 0) for g in 'ABC'}

    def _arm_step(self, rb):
        if rb['wait'] > 0:
            rb['wait'] -= DT
            self._carry_update(rb)
            return
        if not rb['plan']:
            rb['busy'] = False
            self._choose_palletize(rb)
            if not rb['plan']:
                return
        step = rb['plan'][0]
        kind = step[0]
        if kind == 'move':
            _, tx, ty, tz, _u = step
            done = True
            for key, tgt, vmax in (('x', tx, rb['vxy']), ('y', ty, rb['vxy']), ('z', tz, rb['vz'])):
                d = tgt - rb[key]
                if abs(d) > 1e-6:
                    mv = min(abs(d), vmax * DT)
                    rb[key] += math.copysign(mv, d)
                    if abs(tgt - rb[key]) > 1e-6:
                        done = False
            if done:
                rb['plan'].pop(0)
        elif kind == 'grip_crate':
            c = step[1]
            c.state = 'carried'
            self.ready[c.g] = None
            rb['carrying_crate'] = c
            rb['wait'] = DWELL
            rb['plan'].pop(0)
        elif kind == 'release_crate':
            _, c, (px, py, layer), _u, _w = step
            c.state = 'pallet'
            c.x, c.y = px, py
            c.z = PALLET_TOP + CRATE_H / 2 + layer * CRATE_H
            for oo in c.onions:
                if oo is not None:
                    oo.state = 'pallet'
                    self.stats['onions_on_pallet'] += 1
            self._place_items(c)
            rb['carrying_crate'] = None
            rb['cycles'] += 1
            self.pallet_crates.append(c)
            self.stats['crates_done'][c.g] += 1
            self.ack_pulse[{'A': 'M320', 'B': 'M321', 'C': 'M322'}[c.g]] = 2
            self.plc.bits[{'A': 'M320', 'B': 'M321', 'C': 'M322'}[c.g]] = True
            if self.pallet_slot >= len(PALLET_SLOTS):
                self.pallet_full_t = self.t + PALLET_CLEAR_S
            rb['wait'] = DWELL
            rb['plan'].pop(0)
        self._carry_update(rb)

    def _clear_pallet(self):
        for c in self.pallet_crates:
            c.gone = True
            for oo in c.onions:
                if oo is not None:
                    oo.gone, oo.state = True, 'removed'
        self.trash_crates.extend(self.pallet_crates)
        self.crate_list = [c for c in self.crate_list if c not in self.pallet_crates]
        self.pallet_crates = []
        self.pallet_slot = 0
        self.pallet_full_t = None
        self.stats['pallets_done'] += 1

    # ---------------------------------------------------------------- reporting
    def kpis(self):
        w = self.plc.words
        r10 = w.get('D68', 0)
        if self.ladder == 'simple' and len(self.hist) >= 10:
            r10 = (w.get('D44', 0) - self.hist[-10]) * 6      # items in the last 10 s, scaled to per minute
        return dict(A=w.get('D40', 0), B=w.get('D41', 0), C=w.get('D42', 0), reject=w.get('D43', 0), total=w.get('D44', 0),
                    rate10=r10, rate60=w.get('D60', 0), accuracy=w.get('D64', 0), tested=w.get('D62', 0),
                    crates=(w.get('D54', 0), w.get('D55', 0), w.get('D56', 0)), pallets=self.stats['pallets_done'])

    def run(self, seconds):
        for _ in range(int(round(seconds / DT))):
            self.step()


# ====================================================================================================
#  3. SCENE VIEW (CoppeliaSim)
# ====================================================================================================

import glob
import math
import os
import sys


M = 0.001   # mm -> m

GREEN = (0.25, 0.70, 0.30)
BLUE = (0.25, 0.45, 0.85)
ORANGE = (0.95, 0.60, 0.15)
RED = (0.85, 0.20, 0.20)
GREY = (0.62, 0.64, 0.66)
DARK = (0.22, 0.23, 0.25)
YELLOW = (0.95, 0.80, 0.15)
STEEL = (0.75, 0.78, 0.82)
LIGHT = (0.88, 0.88, 0.86)
BLACK = (0.08, 0.08, 0.09)
GRADE_COL = {'A': GREEN, 'B': BLUE, 'C': ORANGE}
S45 = math.sqrt(0.5)
Q_ROLL90 = (S45, 0.0, 0.0, S45)          # rotates a cylinder's axis from z to y


def quat_z(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """Quaternion (x, y, z, w) of a camera at `eye` looking at `target` (camera looks along +z, +y is up)."""
    z = unit([target[i] - eye[i] for i in range(3)])
    x = unit(cross(list(up), z))
    y = cross(z, x)
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


class Api:
    """Small wrapper that hides CoppeliaSim version differences (4.5 vs 4.6+ argument order, constants)."""

    def __init__(self, sim):
        self.sim = sim
        self.new_pos_api = None
        self.new_pose_api = None
        self.cache = {}
        self.warned = set()
        self.world = getattr(sim, 'handle_world', -1)

    def warn(self, msg):
        if msg not in self.warned:
            self.warned.add(msg)
            print('  [warning]', msg)

    def prepare(self):
        sim = self.sim
        for fn in (lambda: sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, False),
                   lambda: sim.setBoolProperty(sim.handle_scene, 'dynamicsEnabled', False)):
            try:
                fn()
                break
            except Exception:
                continue
        else:
            self.warn('could not switch the physics engine off; objects are also marked static individually')

    def shape(self, kind, size, pos, color, alias=None, quat=None):
        sim = self.sim
        k = {'box': sim.primitiveshape_cuboid, 'cyl': sim.primitiveshape_cylinder,
             'sph': sim.primitiveshape_spheroid, 'cone': sim.primitiveshape_cone}[kind]
        h = sim.createPrimitiveShape(k, [float(v) for v in size], 0)
        for fn in (lambda: sim.setObjectInt32Param(h, sim.shapeintparam_static, 1),
                   lambda: sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)):
            try:
                fn()
            except Exception:
                self.warn('shape static/respondable parameter not available in this version (harmless with physics off)')
        self.set_color(h, color)
        if quat is not None:
            self.set_pose(h, pos, quat, force=True)
        else:
            self.set_pos(h, pos, force=True)
        if alias:
            try:
                sim.setObjectAlias(h, alias)
            except Exception:
                pass
        return h

    def set_pos(self, h, pos, force=False):
        p = (round(pos[0], 4), round(pos[1], 4), round(pos[2], 4))
        if not force and self.cache.get(('p', h)) == p:
            return
        self.cache[('p', h)] = p
        sim = self.sim
        if self.new_pos_api is None:
            try:
                sim.setObjectPosition(h, list(p))
                self.new_pos_api = True
                return
            except Exception:
                self.new_pos_api = False
        if self.new_pos_api:
            sim.setObjectPosition(h, list(p))
        else:
            sim.setObjectPosition(h, -1, list(p))      # older argument order: (handle, relativeTo=-1 world, position)

    def set_pose(self, h, pos, quat, force=False):
        key = (round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)) + tuple(round(c, 4) for c in quat)
        if not force and self.cache.get(('q', h)) == key:
            return
        self.cache[('q', h)] = key
        self.cache[('p', h)] = key[:3]
        pose = [float(v) for v in key]
        sim = self.sim
        if self.new_pose_api is None:
            try:
                sim.setObjectPose(h, pose)
                self.new_pose_api = True
                return
            except Exception:
                self.new_pose_api = False
        if self.new_pose_api:
            sim.setObjectPose(h, pose)
        else:
            sim.setObjectPose(h, -1, pose)

    def get_pose(self, h):
        try:
            return list(self.sim.getObjectPose(h, self.world))
        except Exception:
            return list(self.sim.getObjectPose(h))

    def set_color(self, h, rgb, force=False):
        rgb = tuple(round(c, 3) for c in rgb)
        if not force and self.cache.get(('c', h)) == rgb:
            return
        self.cache[('c', h)] = rgb
        sim = self.sim
        try:
            sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, list(rgb))
        except Exception:
            sim.setShapeColor(h, '', sim.colorcomponent_ambient_diffuse, list(rgb))

    def alpha(self, h, a):
        try:
            self.sim.setShapeColor(h, None, self.sim.colorcomponent_transparency, [a])
        except Exception:
            self.warn('transparency is not available; fence panels are drawn opaque')

    def remove(self, h):
        try:
            self.sim.removeObject(h)
        except Exception:
            pass
        for k in [k for k in self.cache if k[1] == h]:
            del self.cache[k]


# =====================================================================================================
#  library discovery
# =====================================================================================================
def candidate_install_dirs(sim, hint=None):
    c = []
    if hint:
        c.append(hint)
    try:
        c.append(sim.getStringParam(sim.stringparam_application_path))
    except Exception:
        pass
    for ev in ('COPPELIASIM_ROOT_DIR', 'COPPELIASIM_ROOT'):
        if os.environ.get(ev):
            c.append(os.environ[ev])
    pats = []
    for pf in (os.environ.get('ProgramFiles'), os.environ.get('ProgramFiles(x86)'), 'C:/Program Files', 'D:/Program Files'):
        if pf:
            pats += [os.path.join(pf, 'CoppeliaRobotics', '*'), os.path.join(pf, 'CoppeliaSim*')]
    home = os.path.expanduser('~')
    pats += [os.path.join(home, 'CoppeliaSim*'), os.path.join(home, 'Downloads', 'CoppeliaSim*'), '/opt/CoppeliaSim*',
             '/Applications/coppeliaSim.app/Contents/Resources', os.path.join(home, 'Applications', 'CoppeliaSim*')]
    for p in pats:
        c += sorted(glob.glob(p))
    return c


def find_models_dir(sim, hint=None):
    seen = []
    for d in candidate_install_dirs(sim, hint):
        if not d or d in seen:
            continue
        seen.append(d)
        for cand in (os.path.join(d, 'models'), d):
            if os.path.isdir(cand) and os.path.basename(cand.rstrip('/\\')).lower() == 'models':
                return cand, seen
    return None, seen


def _key(s):
    return ''.join(ch for ch in s.lower() if ch.isalnum())


def find_robot_models(models_dir):
    """{normalised name: path} for every .ttm below models/robots (e.g. 'ur5' -> .../UR5.ttm)."""
    out = {}
    if not models_dir:
        return out
    base = os.path.join(models_dir, 'robots')
    for root, _dirs, files in os.walk(base if os.path.isdir(base) else models_dir):
        for f in files:
            if f.lower().endswith('.ttm'):
                out.setdefault(_key(os.path.splitext(f)[0]), os.path.join(root, f))
    return out


def pick_model(found, names):
    for n in names:
        if _key(n) in found:
            return found[_key(n)]
    for n in names:
        for k, p in found.items():
            if k.startswith(_key(n)):
                return p
    return None


# =====================================================================================================
#  robots
# =====================================================================================================
class ScaraArm:
    """Fallback robot made of primitive shapes: column, two horizontal links and a vertical quill (always works)."""
    kind = 'primitive SCARA (fallback)'

    def __init__(self, api, name, base_mm, color):
        self.api, self.name = api, name
        self.bx, self.by, self.bz = base_mm[0] * M, base_mm[1] * M, base_mm[2] * M
        self.L1 = self.L2 = 0.60
        self.z1, self.z2 = 1.34, 1.24
        self.sgn = 1 if name == 'picker' else -1
        b = api.shape
        b('cyl', (0.16, 0.16, self.z1 - self.bz + 0.05), (self.bx, self.by, (self.z1 + self.bz) / 2), GREY, f'{name}_column')
        self.l1 = b('box', (self.L1, 0.09, 0.06), (self.bx, self.by, self.z1), color, f'{name}_link1')
        self.l2 = b('box', (self.L2, 0.07, 0.05), (self.bx, self.by, self.z2), color, f'{name}_link2')
        self.elb = b('cyl', (0.10, 0.10, 0.16), (self.bx, self.by, self.z1 - 0.05), DARK, f'{name}_elbow')
        self.quill = b('cyl', (0.04, 0.04, 0.75), (self.bx, self.by, 1.0), STEEL, f'{name}_quill')

    def update(self, x, y, z):
        x, y, z = x * M, y * M, z * M
        dx, dy = x - self.bx, y - self.by
        d = min(max(math.hypot(dx, dy), 0.08), self.L1 + self.L2 - 0.01)
        ang = math.atan2(dy, dx)
        b = math.acos(max(-1.0, min(1.0, (self.L1 ** 2 + d ** 2 - self.L2 ** 2) / (2 * self.L1 * d))))
        a1 = ang + self.sgn * b
        ex, ey = self.bx + self.L1 * math.cos(a1), self.by + self.L1 * math.sin(a1)
        tx, ty = self.bx + d * math.cos(ang), self.by + d * math.sin(ang)
        a2 = math.atan2(ty - ey, tx - ex)
        A = self.api
        A.set_pose(self.l1, ((self.bx + ex) / 2, (self.by + ey) / 2, self.z1), quat_z(a1))
        A.set_pose(self.l2, ((ex + tx) / 2, (ey + ty) / 2, self.z2), quat_z(a2))
        A.set_pos(self.elb, (ex, ey, self.z1 - 0.05))
        A.set_pos(self.quill, (tx, ty, z + 0.375))


class LibRobot:
    """A robot model from CoppeliaSim's library, driven joint by joint through arm_kin inverse kinematics."""

    def __init__(self, api, name, base_mm, home_mm, tool_len, path, waypoints):
        self.api, self.sim, self.name = api, api.sim, name
        self.path, self.tool_len = path, tool_len
        self.base_mm, self.home_mm, self.waypoints = base_mm, home_mm, waypoints
        self.kin = None
        self.q = None
        self.root = None
        self.max_err = 0.0
        self.bad_frames = 0
        self.frames = 0
        self.fails = []
        self.info = {}
        self.kind = 'library model ' + os.path.basename(path)

    def _depth(self, h):
        d = 0
        while h != self.root and h != -1 and d < 60:
            h = self.sim.getObjectParent(h)
            d += 1
        return d

    def _strip_scripts(self):
        sim = self.sim
        n = 0
        objs = [self.root] + list(sim.getObjectsInTree(self.root, getattr(sim, 'handle_all', -2), 0))
        for st_name in ('scripttype_childscript', 'scripttype_simulation', 'scripttype_customization'):
            st = getattr(sim, st_name, None)
            if st is None:
                continue
            for h in objs:
                try:
                    s = sim.getScript(st, h)
                    if s != -1:
                        sim.removeScript(s)
                        n += 1
                except Exception:
                    pass
        self.info['scripts_removed'] = n

    def _static(self):
        sim = self.sim
        for h in sim.getObjectsInTree(self.root, sim.object_shape_type, 0):
            for fn in (lambda: sim.setObjectInt32Param(h, sim.shapeintparam_static, 1),
                       lambda: sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)):
                try:
                    fn()
                except Exception:
                    pass
        for j in self.joints:
            try:
                sim.setJointMode(j, sim.jointmode_kinematic, 0)
            except Exception:
                pass

    def _world_frame(self, h):
        p = self.api.get_pose(h)
        return p[:3], mat_col(quat_to_matrix(p[3:7]), 2)

    def _limits(self, j, q):
        try:
            r = self.sim.getJointInterval(j)
            cyc, iv = (r[0], r[1]) if isinstance(r[1], (list, tuple)) else (False, r)
            if cyc:
                return (q - 2 * math.pi, q + 2 * math.pi)
            return (iv[0], iv[0] + iv[1])
        except Exception:
            return (q - 2 * math.pi, q + 2 * math.pi)

    def setup(self):
        sim = self.sim
        self.root = sim.loadModel(self.path)
        self.api.set_pos(self.root, [v * M for v in self.base_mm], force=True)
        revolute = getattr(sim, 'joint_revolute_subtype', None)
        allj = [j for j in sim.getObjectsInTree(self.root, sim.object_joint_type, 0)
                if revolute is None or sim.getJointType(j) == revolute]
        if len(allj) < 4:
            raise RuntimeError(f'only {len(allj)} revolute joints found in the model')
        tip = max(allj, key=self._depth)
        chain, h = [], tip
        while h != self.root and h != -1:
            if h in allj:
                chain.append(h)
            h = sim.getObjectParent(h)
        chain.reverse()
        if len(chain) > 7:
            chain = chain[-7:]
        self.joints = chain
        self._strip_scripts()
        self._static()
        self.q_ref = [sim.getJointPosition(j) for j in chain]
        frames = [self._world_frame(j) for j in chain]
        base_p, tool_a = frames[-1]
        for d in sim.getObjectsInTree(self.root, sim.object_dummy_type, 0):     # tool point: a connection/tip dummy if present
            try:
                al = sim.getObjectAlias(d).lower()
            except Exception:
                al = ''
            if any(k in al for k in ('connection', 'tip', 'tcp', 'flange')):
                dp = self.api.get_pose(d)[:3]
                if math.dist(dp, base_p) < 0.3:
                    base_p = dp
                    self.info['tip_dummy'] = al
                    break
        jp, jw = frames[-1]
        direction = None                                           # which way does the flange point along the last joint axis?
        if 'tip_dummy' in self.info:
            dd = sum((base_p[i] - jp[i]) * jw[i] for i in range(3))
            direction = (1.0 if dd > 0 else -1.0) if abs(dd) > 0.02 else None
        if direction is None:
            try:
                shp = sim.getObjectsInTree(chain[-1], sim.object_shape_type, 0)
                if shp:
                    cen = [sum(self.api.get_pose(h)[i] for h in shp) / len(shp) for i in range(3)]
                    dd = sum((cen[i] - jp[i]) * jw[i] for i in range(3))
                    direction = (1.0 if dd > 0 else -1.0) if abs(dd) > 0.01 else None
            except Exception:
                direction = None
        self.info['tool_direction'] = 'from model geometry' if direction else 'unknown (chosen by reach)'
        extra = self.tool_len + (0.0 if 'tip_dummy' in self.info else 0.08)
        self.real_lim = [self._limits(j, q) for j, q in zip(chain, self.q_ref)]
        # a joint that can turn a full circle is tracked without limits (the value is wrapped back into the real range when applied)
        lim = [(-60.0, 60.0) if hi - lo >= 2 * math.pi - 1e-3 else (lo, hi) for lo, hi in self.real_lim]
        self.applied = list(self.q_ref)
        home = [v * M for v in self.home_mm]
        down = [0.0, 0.0, -1.0]
        best = None
        for s in ((direction,) if direction else (1.0, -1.0)):
            ta = [s * c for c in tool_a]
            tp = [base_p[i] + ta[i] * extra for i in range(3)]
            kin = ArmKin([dict(p=f[0], w=f[1]) for f in frames], self.q_ref, tp, ta, lim)
            g = kin.solve_global(home, down, tries=80)
            if g is None:
                self.info[f'tool_dir_{int(s):+d}'] = 'no home posture'
                continue
            qw = []                                                             # posture nearest to the start, same pose
            for i, v in enumerate(g[0]):
                w = v - 2 * math.pi * round((v - self.q_ref[i]) / (2 * math.pi)) if lim[i][1] - lim[i][0] > 100 else v
                qw.append(w)
            kin.q_home = list(qw)
            g = (qw, g[1], g[2])
            q, fails = list(qw), []
            for w in self.waypoints:
                q, ep, ea = kin.solve(q, [v * M for v in w], down, iters=80)
                if ep > 0.003 or ea > 3.0:
                    fails.append(w)
            self.info[f'tool_dir_{int(s):+d}'] = f'{len(fails)} of {len(self.waypoints)} targets unreachable'
            cost = (len(fails), sum(abs(a - b) for a, b in zip(g[0], self.q_ref)))
            if best is None or cost < best[0]:
                best = (cost, kin, g[0], fails)
        if best is None:
            raise RuntimeError('inverse kinematics found no tool-down posture at the home point')
        _, self.kin, q_home, self.fails = best
        self.q = list(q_home)
        self._apply(self.q, force=True)
        ps, ws, _tp, _ta = self.kin.frames(self.q)                              # compare my kinematics with the simulator
        err = 0.0
        for k, j in enumerate(chain):
            p, w = self._world_frame(j)
            err = max(err, math.dist(p, ps[k]), 0.2 * math.dist(w, ws[k]))
        self.info['calibration_error_mm'] = round(err * 1000, 2)
        if err > 0.01:
            raise RuntimeError(f'kinematic model does not match the simulator (error {err * 1000:.1f} mm)')
        if len(self.fails) > max(2, len(self.waypoints) // 4):
            raise RuntimeError(f'{len(self.fails)} of {len(self.waypoints)} working targets are out of reach')
        return True

    def _wrap(self, i, v):
        lo, hi = self.real_lim[i]
        if hi - lo < 2 * math.pi - 1e-3:
            return min(max(v, lo), hi)
        c = v - 2 * math.pi * round((v - self.applied[i]) / (2 * math.pi))
        while c < lo:
            c += 2 * math.pi
        while c > hi:
            c -= 2 * math.pi
        return c

    def _apply(self, q, force=False):
        sim = self.sim
        for i, (j, v) in enumerate(zip(self.joints, q)):
            c = self._wrap(i, v)
            if not force and abs(self.applied[i] - c) < 1e-5:
                continue
            self.applied[i] = c
            sim.setJointPosition(j, float(c))

    def update(self, x, y, z):
        tgt = [x * M, y * M, z * M]
        down = [0.0, 0.0, -1.0]
        q, ep, ea = self.kin.solve(self.q, tgt, down, iters=8)
        if ep > 0.004 or ea > 4.0:
            q, ep, ea = self.kin.solve(q, tgt, down, iters=60)
        self.q = q
        self.max_err = max(self.max_err, ep)
        self.bad_frames += 1 if ep > 0.01 else 0
        self.frames += 1
        self._apply(q)


# =====================================================================================================
#  scene
# =====================================================================================================
class Hud:
    """Status text in a CoppeliaSim auxiliary console (silently disabled if the call is not available)."""

    def __init__(self, sim):
        self.sim, self.h, self.ok, self.last = sim, None, True, -1
        try:
            self.h = sim.auxiliaryConsoleOpen('Onion sorting line - live status', 30, 1, [20, 40], [430, 330],
                                              [0.1, 0.1, 0.1], [0.92, 0.94, 0.92])
        except Exception:
            self.ok = False

    def show(self, text, t):
        if not self.ok or int(t) == self.last:
            return
        self.last = int(t)
        try:
            self.sim.auxiliaryConsolePrint(self.h, None)
            self.sim.auxiliaryConsolePrint(self.h, text)
        except Exception:
            self.ok = False


class SceneView:
    def __init__(self, api, lm, use_models=True, models_dir=None, palletizer_model=None, **_ignored):
        self.api, self.lm = api, lm
        self.use_models, self.models_dir = use_models, models_dir
        self.model_paths = {'palletizer': palletizer_model}
        self.onion_h, self.extra_h, self.crate_h = {}, {}, {}
        self.stripes, self.parts, self.drivers = [], {}, {}
        self.hud = None
        self.robot_report = {}

    def _b(self, size, pos, col, alias=None, quat=None):
        return self.api.shape('box', size, pos, col, alias, quat)

    def _c(self, r, h, pos, col, alias=None, quat=None):
        return self.api.shape('cyl', (2 * r, 2 * r, h), pos, col, alias, quat)

    def _beam(self, x, alias, col):
        self._b((0.05, 0.05, 0.10), (x, -0.17, 0.86), DARK, alias + '_emitter')
        self._b((0.05, 0.05, 0.10), (x, 0.17, 0.86), DARK, alias + '_receiver')
        return self.api.shape('box', (0.004, 0.34, 0.004), (x, 0, 0.835), col, alias)

    def waypoints(self, name='palletizer'):
        """Every tool target the line model can generate for the robot (used for the reach check)."""
        lm = self.lm
        zs = lm.Z_SAFE['palletizer']
        w = []
        for g in 'ABC':
            cx, cy = lm.READY_POS[g]
            w += [(cx, cy, zs), (cx, cy, lm.Z_CRATE_GRIP)]
        for px, py, layer in lm.PALLET_SLOTS:
            w += [(px, py, zs), (px, py, lm.PALLET_TOP + lm.CRATE_H / 2 + layer * lm.CRATE_H + lm.CARRY_OFFSET + 5)]
        return w

    # ------------------------------------------------------------------ static scene
    def build_static(self):
        a, lm = self.api, self.lm
        b, c = self._b, self._c
        P = self.parts

        # ---- floor and a light safety fence (posts, two rails, yellow floor line)
        b((5.6, 4.6, 0.02), (0.9, 1.0, -0.01), (0.60, 0.62, 0.64), 'floor')
        fx0, fx1, fy0, fy1 = -0.75, 2.55, -0.75, 2.75
        for (x0, y0, x1, y1) in ((fx0, fy0, fx1, fy0), (fx0, fy1, fx1, fy1), (fx0, fy0, fx0, fy1), (fx1, fy0, fx1, fy1)):
            horiz = abs(y1 - y0) < 1e-9
            b((abs(x1 - x0) + 0.06, 0.06, 0.003) if horiz else (0.06, abs(y1 - y0) + 0.06, 0.003), ((x0 + x1) / 2, (y0 + y1) / 2, 0.002), YELLOW, 'safety_line')
        for (x0, y0, x1, y1) in ((fx0, fy1, fx1, fy1), (fx0, fy0, fx0, fy1), (fx1, fy0, fx1, fy1)):
            L = math.hypot(x1 - x0, y1 - y0)
            ang = math.atan2(y1 - y0, x1 - x0)
            for zr in (0.5, 1.0):
                b((L, 0.025, 0.025), ((x0 + x1) / 2, (y0 + y1) / 2, zr), YELLOW, 'fence_rail', quat_z(ang))
            n = int(L / 1.1) + 1
            for k in range(n + 1):
                f = k / n
                c(0.025, 1.1, (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, 0.55), DARK, 'fence_post')
        for px in (fx0, fx1):
            c(0.025, 1.1, (px, fy0, 0.55), DARK, 'fence_post')

        # ---- infeed conveyor
        x0, x1 = lm.BELT_X0 * M, lm.BELT_X1 * M
        xm, L = (x0 + x1) / 2, x1 - x0
        b((L, 0.22, 0.04), (xm, 0, 0.78), (0.15, 0.15, 0.17), 'belt')
        for s in (-1, 1):
            b((L + 0.06, 0.025, 0.07), (xm, s * 0.125, 0.775), STEEL, 'belt_side_rail')
        for rx in (x0 - 0.02, x1 + 0.02):
            c(0.04, 0.26, (rx, 0, 0.78), STEEL, 'belt_end_roller', Q_ROLL90)
        for k in range(5):
            lx = x0 + 0.1 + k * (L - 0.2) / 4
            for s in (-1, 1):
                c(0.022, 0.74, (lx, s * 0.13, 0.37), STEEL, 'belt_leg')
            b((0.03, 0.30, 0.03), (lx, 0, 0.30), STEEL, 'belt_brace')
        b((0.20, 0.16, 0.16), (x0 - 0.15, 0.0, 0.75), (0.2, 0.35, 0.6), 'belt_motor')
        for k in range(int(L / 0.1)):
            self.stripes.append(b((0.012, 0.20, 0.002), (x0 + k * 0.1, 0, 0.801), (0.35, 0.35, 0.38), 'belt_stripe'))
        self.belt_len = int(L / 0.1) * 100

        # ---- feeder: hopper, chute, gate
        b((0.40, 0.40, 0.12), (-0.14, 0, 1.30), GREY, 'hopper_top')
        b((0.26, 0.26, 0.10), (-0.14, 0, 1.19), (0.55, 0.57, 0.60), 'hopper_mid')
        b((0.14, 0.14, 0.10), (-0.14, 0, 1.09), (0.50, 0.52, 0.55), 'hopper_neck')
        b((0.30, 0.14, 0.02), (-0.05, 0, 1.00), STEEL, 'feeder_tray')
        for sx in (-0.30, 0.02):
            for sy in (-0.19, 0.19):
                c(0.018, 1.24, (sx, sy, 0.62), STEEL, 'hopper_leg')
        P['gate'] = b((0.10, 0.01, 0.10), (0.0, 0.0, 0.91), RED, 'feeder_gate')

        # ---- layer 1: size gauge, layer 2: sprout camera, station sensors
        P['beam'] = self._beam(lm.X_BEAM * M, 'size_gauge_beam', (0.2, 0.9, 0.2))
        P['stn'] = {g: self._beam(lm.XS[g] * M, f'sensor_station_{g}', YELLOW) for g in 'ABC'}
        xc = lm.X_CAM * M
        for s in (-1, 1):
            c(0.014, 0.40, (xc, s * 0.19, 0.99), DARK, 'camera_post')
        b((0.18, 0.42, 0.04), (xc, 0, 1.19), DARK, 'camera_bar')
        b((0.10, 0.10, 0.08), (xc, 0, 1.12), GREY, 'camera_head')
        P['lens'] = c(0.022, 0.02, (xc, 0, 1.07), BLACK, 'camera_lens')
        P['ring'] = c(0.07, 0.01, (xc, 0, 1.03), DARK, 'camera_light_ring')

        # ---- pushers, crate stands with roller track
        P['plate'] = {}
        for g in 'ABC':
            xp = (lm.XS[g] + lm.PUSH_OFF) * M
            b((0.11, 0.10, 0.09), (xp, -0.24, 0.84), DARK, f'pusher_{g}_housing')
            P['plate'][g] = b((0.08, 0.02, 0.05), (xp, (lm.PUSH_RETRACT - 10) * M, 0.83), GRADE_COL[g], f'pusher_{g}_plate')
            ty0, ty1 = 0.17, 0.97
            b((0.30, ty1 - ty0, 0.02), (xp, (ty0 + ty1) / 2, lm.TABLE_TOP * M - 0.01), (0.35, 0.37, 0.40), f'track_{g}')
            for s in (-1, 1):
                b((0.014, ty1 - ty0, 0.03), (xp + s * 0.152, (ty0 + ty1) / 2, lm.TABLE_TOP * M + 0.005), GRADE_COL[g], f'track_{g}_rail')
            for yy in (ty0 + 0.03, ty1 - 0.03):
                for sx in (-0.12, 0.12):
                    c(0.018, lm.TABLE_TOP * M - 0.02, (xp + sx, yy, (lm.TABLE_TOP * M - 0.02) / 2), STEEL, f'track_{g}_leg')
            rx, ry = lm.READY_POS[g]
            b((0.28, 0.01, 0.004), (rx * M, ry * M - 0.18, lm.TABLE_TOP * M + 0.002), YELLOW, 'ready_mark')
        # ---- reject bin
        bx = (lm.BIN_X0 + lm.BIN_X1) / 2 * M
        bw = (lm.BIN_X1 - lm.BIN_X0) * M
        b((bw, 0.30, 0.30), (bx, 0.0, 0.15), (0.45, 0.12, 0.12), 'reject_bin')
        for s in (-1, 1):
            b((bw, 0.012, 0.12), (bx, s * 0.156, 0.36), RED, 'reject_wall')
            b((0.012, 0.30, 0.12), (bx + s * (bw / 2 + 0.006), 0.0, 0.36), RED, 'reject_wall')

        # ---- robot pedestal and pallet
        bx_, by_, bz_ = lm.ARM_BASE['palletizer']
        b((0.24, 0.24, bz_ * M), (bx_ * M, by_ * M, bz_ * M / 2), (0.30, 0.32, 0.36), 'robot_pedestal')
        b((0.30, 0.30, 0.03), (bx_ * M, by_ * M, bz_ * M - 0.015), YELLOW, 'robot_pedestal_plate')
        px, py = lm.PALLET_XY[0] * M, lm.PALLET_XY[1] * M
        top = lm.PALLET_TOP * M
        b((0.90, 0.84, 0.03), (px, py, top - 0.015), (0.72, 0.56, 0.34), 'pallet_deck')
        for dx in (-0.36, 0.0, 0.36):
            b((0.10, 0.84, 0.05), (px + dx, py, top - 0.055), (0.62, 0.48, 0.30), 'pallet_runner')
        for dx in (-0.38, 0.38):
            for dy in (-0.36, 0.36):
                b((0.12, 0.12, top - 0.08), (px + dx, py + dy, (top - 0.08) / 2), (0.45, 0.45, 0.48), 'pallet_stand')

        # ---- control cabinet with HMI screen, tower light
        b((0.50, 0.34, 1.70), (-0.50, 0.75, 0.85), (0.82, 0.83, 0.82), 'control_cabinet')
        P['hmi'] = b((0.30, 0.012, 0.20), (-0.50, 0.75 - 0.176, 1.30), (0.2, 0.4, 0.7), 'hmi_screen')
        c(0.025, 0.04, (-0.62, 0.75 - 0.175, 1.05), RED, 'estop_button', Q_ROLL90)
        c(0.02, 0.03, (-0.50, 0.75 - 0.175, 1.05), GREEN, 'start_button', Q_ROLL90)
        c(0.015, 0.40, (-0.32, -0.25, 0.20), STEEL, 'tower_post')
        P['lamp_g'] = c(0.035, 0.05, (-0.32, -0.25, 0.425), DARK, 'lamp_green')
        P['lamp_a'] = c(0.035, 0.05, (-0.32, -0.25, 0.48), DARK, 'lamp_amber')
        P['lamp_r'] = c(0.035, 0.05, (-0.32, -0.25, 0.535), DARK, 'lamp_red')

        self._build_robots()
        A = self.api
        P['clamp_stem'] = A.shape('cyl', (0.05, 0.05, 0.20), (0, 0, -1), STEEL, 'palletizer_stem')
        P['clamp_bar'] = A.shape('box', (0.30, 0.40, 0.022), (0, 0, -1), DARK, 'palletizer_clamp_bar')
        P['clamp_l'] = A.shape('box', (0.012, 0.34, 0.19), (0, 0, -1), YELLOW, 'palletizer_clamp_jaw')
        P['clamp_r'] = A.shape('box', (0.012, 0.34, 0.19), (0, 0, -1), YELLOW, 'palletizer_clamp_jaw')
        self._camera()
        self.hud = Hud(A.sim)

    def _build_robots(self):
        a, lm = self.api, self.lm
        found = {}
        if self.use_models:
            mdir, _tried = find_models_dir(a.sim, self.models_dir)
            self.models_dir = mdir
            if mdir:
                found = find_robot_models(mdir)
                print(f'  CoppeliaSim model library: {mdir}  ({len(found)} robot models)')
            else:
                a.warn('CoppeliaSim model library not found (use --coppelia-dir "C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu"); using a primitive arm')
        name = 'palletizer'
        path = self.model_paths.get(name) or pick_model(found, ('UR10', 'UR5', 'UR3'))
        drv = None
        if path:
            try:
                drv = LibRobot(a, name, lm.ARM_BASE[name], lm.ARM_HOME[name], 0.20, path, self.waypoints(name))
                drv.setup()
                print(f'  {name}: {drv.kind}, {len(drv.joints)} joints, calibration error {drv.info["calibration_error_mm"]} mm, '
                      f'{len(drv.waypoints) - len(drv.fails)}/{len(drv.waypoints)} working targets reachable')
            except Exception as e:
                print(f'  [warning] {name}: could not use the library robot ({e}); using the primitive arm instead')
                if drv is not None and drv.root is not None:
                    try:
                        a.sim.removeModel(drv.root)
                    except Exception:
                        for h in a.sim.getObjectsInTree(drv.root, a.sim.handle_all, 0) + [drv.root]:
                            a.remove(h)
                drv = None
        if drv is None:
            drv = ScaraArm(a, name, lm.ARM_BASE[name], YELLOW)
        self.drivers[name] = drv
        self.robot_report[name] = dict(kind=drv.kind, **getattr(drv, 'info', {}))

    def _camera(self):
        a = self.api
        try:
            cam = a.sim.getObject('/DefaultCamera')
            eye, tgt = (2.6, -2.3, 2.5), (0.95, 0.95, 0.5)
            a.set_pose(cam, eye, look_at_quat(eye, tgt), force=True)
        except Exception:
            a.warn('could not set the default camera (use the mouse to orbit)')

    # ------------------------------------------------------------------ dynamic objects
    def _onion_color(self, o):
        t = (o.id * 37 % 100) / 100.0
        if o.sprouted:
            return (0.88 + 0.05 * t, 0.80 + 0.08 * t, 0.55 + 0.1 * t)
        return (0.80 + 0.12 * t, 0.50 + 0.12 * t, 0.20 + 0.10 * t)

    def _ensure_onion(self, o):
        if o.id in self.onion_h:
            return
        a = self.api
        h = a.shape('sph', (o.d * M, o.d * M, o.h * M), (o.x * M, o.y * M, o.z * M), self._onion_color(o), f'onion_{o.id}')
        self.onion_h[o.id] = h
        parts = [a.shape('cone', (0.012, 0.012, 0.016), (o.x * M, o.y * M, (o.z + o.h / 2 + 6) * M), (0.62, 0.45, 0.2), f'onion_{o.id}_neck')]
        if o.sprouted:
            parts.append(a.shape('cone', (0.014, 0.014, 0.045), (o.x * M, o.y * M, (o.z + o.h / 2 + 20) * M), (0.25, 0.80, 0.2), f'onion_{o.id}_sprout'))
        self.extra_h[o.id] = parts

    def _crate_parts(self, c):
        lm = self.lm
        col = GRADE_COL[c.g]
        dark = tuple(0.8 * v for v in col)
        W, Ln, H = lm.CRATE_X * M, lm.CRATE_Y * M, lm.CRATE_H * M
        return [((W, Ln, 0.008), (0, 0, -H / 2 + 0.004), dark),
                ((0.008, Ln, H - 0.01), (-W / 2 + 0.004, 0, 0.003), col), ((0.008, Ln, H - 0.01), (W / 2 - 0.004, 0, 0.003), col),
                ((W, 0.008, H - 0.01), (0, -Ln / 2 + 0.004, 0.003), col), ((W, 0.008, H - 0.01), (0, Ln / 2 - 0.004, 0.003), col)]

    def sync(self):
        a, lm, m = self.api, self.lm, self.model
        P = self.parts
        for k, h in enumerate(self.stripes):
            a.set_pos(h, (lm.BELT_X0 * M + ((k * 100 + m.stripe) % self.belt_len) * M, 0, 0.801))
        b = m.plc.bits
        a.set_color(P['beam'], RED if b.get('X5') else (0.2, 0.9, 0.2))
        for g, dev in (('A', 'X7'), ('B', 'X10'), ('C', 'X11')):
            a.set_color(P['stn'][g], RED if b.get(dev) else YELLOW)
        busy = b.get('X6') or b.get('X16')
        a.set_color(P['lens'], (0.9, 0.1, 0.8) if b.get('X6') else ((0.2, 0.9, 0.2) if b.get('X16') else BLACK))
        a.set_color(P['ring'], (1.0, 0.95, 0.7) if busy else DARK)
        a.set_pos(P['gate'], (0.0, 0.12 if m.gate_open else 0.0, 0.91))
        run, flt = bool(b.get('Y10')), bool(b.get('Y11'))
        a.set_color(P['lamp_g'], (0.1, 0.9, 0.1) if run else DARK)
        a.set_color(P['lamp_a'], (1.0, 0.7, 0.1) if not run and not flt else DARK)
        a.set_color(P['lamp_r'], (1.0, 0.1, 0.1) if flt else DARK)
        a.set_color(P['hmi'], (0.9, 0.15, 0.15) if flt else ((0.15, 0.75, 0.25) if run else (0.9, 0.65, 0.1)))
        for g in 'ABC':
            a.set_pos(P['plate'][g], ((lm.XS[g] + lm.PUSH_OFF) * M, (m.plate[g] - 10) * M, 0.83))
        r = m.arms['palletizer']
        self.drivers['palletizer'].update(r['x'], r['y'], r['z'])
        x, y, z = r['x'] * M, r['y'] * M, r['z'] * M
        a.set_pos(P['clamp_stem'], (x, y, z + 0.10))
        a.set_pos(P['clamp_bar'], (x, y, z + 0.005))
        a.set_pos(P['clamp_l'], (x - 0.131, y, z - 0.09))
        a.set_pos(P['clamp_r'], (x + 0.131, y, z - 0.09))
        for o in m.onions:
            if o.state == 'removed':
                continue
            self._ensure_onion(o)
            a.set_pos(self.onion_h[o.id], (o.x * M, o.y * M, o.z * M))
            ex = self.extra_h[o.id]
            a.set_pos(ex[0], (o.x * M, o.y * M, (o.z + o.h / 2 + 6) * M))
            if len(ex) > 1:
                a.set_pos(ex[1], (o.x * M, o.y * M, (o.z + o.h / 2 + 20) * M))
        for o in m.trash:
            for h in self.extra_h.pop(o.id, []):
                a.remove(h)
            h = self.onion_h.pop(o.id, None)
            if h is not None:
                a.remove(h)
        m.trash.clear()
        for c in m.crate_list:
            if c.id not in self.crate_h:
                self.crate_h[c.id] = [(a.shape('box', s, (c.x * M + off[0], c.y * M + off[1], c.z * M + off[2]), col, f'crate_{c.g}_{c.id}'), off)
                                      for s, off, col in self._crate_parts(c)]
            for h, off in self.crate_h[c.id]:
                a.set_pos(h, (c.x * M + off[0], c.y * M + off[1], c.z * M + off[2]))
        for c in m.trash_crates:
            for h, _off in self.crate_h.pop(c.id, []):
                a.remove(h)
        m.trash_crates.clear()
        if self.hud:
            k = m.kpis()
            state = 'FAULT' if flt else ('RUNNING' if run else ('E-STOP' if not b.get('X2', True) else 'STOPPED'))
            self.hud.show(
                f"STATE  {state}      t = {m.t:7.1f} s\n"
                f"GRADE A {k['A']:4d}   B {k['B']:4d}   C {k['C']:4d}   REJECT {k['reject']:4d}\n"
                f"TOTAL  {k['total']:5d}   rate {k['rate10']:3d}/min (10 s)  {k['rate60']:3d}/min (60 s)\n"
                f"ACCURACY {k['accuracy']} %  (n = {k['tested']})\n"
                f"CRATES done A/B/C {k['crates'][0]}/{k['crates'][1]}/{k['crates'][2]}   PALLETS {k['pallets']}\n"
                f"FULL CRATES waiting {sum(m.waiting_crates().values())}   ROBOT cycles {r['cycles']}", m.t)

    def attach(self, model):
        self.model = model

    def robot_test(self):
        """--robot-test: move the robot to every working target and report how accurately the IK arrives."""
        for name in ('palletizer',):
            d, pts = self.drivers[name], self.waypoints(name)
            x, y, z = self.lm.ARM_HOME[name]
            worst = 0.0
            for (px, py, pz) in pts:
                for k in range(1, 13):
                    f = k / 12.0
                    d.update(x + (px - x) * f, y + (py - y) * f, z + (pz - z) * f)
                if isinstance(d, LibRobot):
                    tp, _ = d.kin.tool(d.q)
                    worst = max(worst, math.dist(tp, [px * M, py * M, pz * M]))
                x, y, z = px, py, pz
            print(f'  robot test {name}: {len(pts)} targets visited' + (f', worst error at a target {worst * 1000:.2f} mm' if isinstance(d, LibRobot) else ''))
        for n, r in self.robot_report.items():
            print('  ', n, r)

    def robot_summary(self):
        out = []
        for n, d in self.drivers.items():
            if isinstance(d, LibRobot):
                out.append(f'{n}: {d.kind}; worst IK error {d.max_err * 1000:.2f} mm; frames over 10 mm: {d.bad_frames}/{d.frames}')
            else:
                out.append(f'{n}: {d.kind}')
        return out


def diagnose(sim, models_dir=None):
    """--check: print what the connected CoppeliaSim offers (version, model library, robot models, API probes)."""
    print('--- CoppeliaSim diagnostic ---')
    try:
        print('version code      :', sim.getInt32Param(sim.intparam_program_version))
    except Exception as e:
        print('version code      : unavailable', e)
    mdir, tried = find_models_dir(sim, models_dir)
    print('model library     :', mdir or 'NOT FOUND')
    if not mdir:
        print('  searched:', *tried[:8], sep='\n    ')
    found = find_robot_models(mdir)
    print('robot models found:', len(found))
    for n in ('UR3', 'UR5', 'UR10', 'Franka', 'KUKA', 'ABB'):
        for k, p in found.items():
            if k.startswith(_key(n)):
                print(f'  {n:7s} ->', p)
                break
    for name in ('getObjectPose', 'loadModel', 'getJointInterval', 'setJointPosition', 'auxiliaryConsoleOpen', 'getObjectsInTree'):
        try:
            getattr(sim, name)
            print(f'api {name:22s}: ok')
        except Exception as e:
            print(f'api {name:22s}: {str(e)[:60]}')


# ====================================================================================================
#  2c. EXTERNAL PLC: OpenPLC over OPC UA (tag list, ST program text, gateway, stand-in PLC)
# ====================================================================================================

from collections import namedtuple

Tag = namedtuple('Tag', 'name type owner init note')

TAGS = [
    # ---- operator (HMI / dashboard)
    Tag('START_PB', 'BOOL', 'operator', 'FALSE', 'Start pushbutton (rising edge starts the line)'),
    Tag('STOP_PB', 'BOOL', 'operator', 'FALSE', 'Stop pushbutton'),
    Tag('ESTOP', 'BOOL', 'operator', 'FALSE', 'Emergency stop pressed = TRUE'),
    Tag('RESET_PB', 'BOOL', 'operator', 'FALSE', 'Fault reset pushbutton'),
    Tag('KPI_RESET', 'BOOL', 'operator', 'FALSE', 'Clears the counters (rising edge, only while stopped)'),
    Tag('FEED_INTERVAL', 'INT', 'operator', '20', 'Seconds between onions in 0.1 s units (20 = 2.0 s, limited to 15..100)'),
    # ---- plant (sensors)
    Tag('PLANT_SESSION', 'DINT', 'plant', '0', 'New value every time the simulation starts (the PLC then clears its state)'),
    Tag('HEARTBEAT_IN', 'DINT', 'plant', '0', 'Counter written by the plant every cycle (the PLC watches it)'),
    Tag('CAM_ID', 'DINT', 'plant', '0', 'Id of the onion that was just measured; changes once per onion'),
    Tag('DIAMETER_MM', 'REAL', 'plant', '0.0', 'Measured diameter of that onion (size gauge), mm'),
    Tag('SPROUTED', 'BOOL', 'plant', 'FALSE', 'Camera result for that onion'),
    Tag('TRUE_GRADE', 'INT', 'plant', '0', 'Audit reference grade for the accuracy KPI (1..4)'),
    Tag('PE_A_ID', 'DINT', 'plant', '0', 'Id of the onion in front of pusher A (0 = none)'),
    Tag('PE_B_ID', 'DINT', 'plant', '0', 'Id of the onion in front of pusher B (0 = none)'),
    Tag('PE_C_ID', 'DINT', 'plant', '0', 'Id of the onion in front of pusher C (0 = none)'),
    Tag('PUSHED_A_ID', 'DINT', 'plant', '0', 'Id of the last onion that entered crate A'),
    Tag('PUSHED_B_ID', 'DINT', 'plant', '0', 'Id of the last onion that entered crate B'),
    Tag('PUSHED_C_ID', 'DINT', 'plant', '0', 'Id of the last onion that entered crate C'),
    Tag('BEAM_BLOCKED', 'BOOL', 'plant', 'FALSE', 'Size-gauge light beam is interrupted'),
    Tag('CRATE_A_READY', 'BOOL', 'plant', 'FALSE', 'Full crate A waits at the pick-up spot'),
    Tag('CRATE_B_READY', 'BOOL', 'plant', 'FALSE', 'Full crate B waits at the pick-up spot'),
    Tag('CRATE_C_READY', 'BOOL', 'plant', 'FALSE', 'Full crate C waits at the pick-up spot'),
    Tag('PALLET_FULL', 'BOOL', 'plant', 'FALSE', 'Pallet has no free slot'),
    # ---- plc (outputs and KPIs)
    Tag('HEARTBEAT_OUT', 'DINT', 'plc', '0', 'Counter incremented every PLC scan'),
    Tag('SESSION_ACK', 'DINT', 'plc', '0', 'Copy of PLANT_SESSION after the PLC reset itself'),
    Tag('PLC_FAULT', 'INT', 'plc', '0', '0 = ok, 1 = E-stop, 2 = belt jam, 4 = plant link lost'),
    Tag('RUNNING', 'BOOL', 'plc', 'FALSE', 'Line is running'),
    Tag('CONVEYOR_RUN', 'BOOL', 'plc', 'FALSE', 'Belt motor'),
    Tag('FEED_GATE', 'BOOL', 'plc', 'FALSE', 'Feeder gate (an onion drops at the rising edge)'),
    Tag('PUSH_A', 'BOOL', 'plc', 'FALSE', 'Pusher A (grade A)'),
    Tag('PUSH_B', 'BOOL', 'plc', 'FALSE', 'Pusher B (grade B)'),
    Tag('PUSH_C', 'BOOL', 'plc', 'FALSE', 'Pusher C (grade C)'),
    Tag('CALL_A', 'BOOL', 'plc', 'FALSE', 'Robot: take the full crate A to the pallet'),
    Tag('CALL_B', 'BOOL', 'plc', 'FALSE', 'Robot: take the full crate B to the pallet'),
    Tag('CALL_C', 'BOOL', 'plc', 'FALSE', 'Robot: take the full crate C to the pallet'),
    Tag('COUNT_A', 'DINT', 'plc', '0', 'Graded onions: A (70 to under 90 mm)'),
    Tag('COUNT_B', 'DINT', 'plc', '0', 'Graded onions: B (55 to under 70 mm)'),
    Tag('COUNT_C', 'DINT', 'plc', '0', 'Graded onions: C (40 to under 55 mm)'),
    Tag('COUNT_REJECT', 'DINT', 'plc', '0', 'Rejected onions (sprouted, under 40 mm, 90 mm or more)'),
    Tag('RATE_PM', 'INT', 'plc', '0', 'Onions per minute (last full minute)'),
    Tag('TESTED', 'DINT', 'plc', '0', 'Onions checked against the audit grade'),
    Tag('ACCURACY_PCT', 'INT', 'plc', '0', 'Classification accuracy, %'),
]
BY_NAME = {t.name: t for t in TAGS}
OWNERS = ('operator', 'plant', 'plc')


def tag_names(owner):
    return [t.name for t in TAGS if t.owner == owner]


def initial(owner):
    out = {}
    for t in TAGS:
        if t.owner == owner:
            out[t.name] = {'BOOL': False, 'INT': 0, 'DINT': 0, 'REAL': 0.0}[t.type]
            if t.name == 'FEED_INTERVAL':
                out[t.name] = 20
    return out


def st_var_block():
    lines = ['VAR']
    for t in TAGS:
        lines.append(f'    {t.name} : {t.type} := {t.init}; (* {t.owner}: {t.note} *)')
    return '\n'.join(lines)


def csv_text():
    rows = ['name,iec_type,opc_ua_type,owner,access_for_gateway,description']
    ua = {'BOOL': 'Boolean', 'INT': 'Int16', 'DINT': 'Int32', 'REAL': 'Float'}
    for t in TAGS:
        acc = 'read' if t.owner == 'plc' else 'write'
        rows.append(f'{t.name},{t.type},{ua[t.type]},{t.owner},{acc},"{t.note}"')
    return '\n'.join(rows) + '\n'


ST_SOURCE = r'''(* ======================================================================================================
   ONION SORTING LINE - OpenPLC program (Structured Text, IEC 61131-3)
   The plant (CoppeliaSim + Python) writes the sensor tags and reads the output tags over OPC UA.
   Grades:  1 = A (70 to under 90 mm)   2 = B (55 to under 70 mm)   3 = C (40 to under 55 mm)   4 = reject
            (sprouted, under 40 mm or 90 mm and more)
   Tag owners:  operator = HMI buttons,  plant = sensors,  plc = outputs and KPIs (see io_map.csv).
   Timing: the OPC UA link adds up to ~0.3 s, so the PLC decides on IDs and windows, never on 10 ms beam timing.
   ====================================================================================================== *)

PROGRAM OnionSorting
VAR
    START_PB : BOOL := FALSE; (* operator: Start pushbutton (rising edge starts the line) *)
    STOP_PB : BOOL := FALSE; (* operator: Stop pushbutton *)
    ESTOP : BOOL := FALSE; (* operator: Emergency stop pressed = TRUE *)
    RESET_PB : BOOL := FALSE; (* operator: Fault reset pushbutton *)
    KPI_RESET : BOOL := FALSE; (* operator: Clears the counters (rising edge, only while stopped) *)
    FEED_INTERVAL : INT := 20; (* operator: Seconds between onions in 0.1 s units (20 = 2.0 s, limited to 15..100) *)
    PLANT_SESSION : DINT := 0; (* plant: New value every time the simulation starts (the PLC then clears its state) *)
    HEARTBEAT_IN : DINT := 0; (* plant: Counter written by the plant every cycle (the PLC watches it) *)
    CAM_ID : DINT := 0; (* plant: Id of the onion that was just measured; changes once per onion *)
    DIAMETER_MM : REAL := 0.0; (* plant: Measured diameter of that onion (size gauge), mm *)
    SPROUTED : BOOL := FALSE; (* plant: Camera result for that onion *)
    TRUE_GRADE : INT := 0; (* plant: Audit reference grade for the accuracy KPI (1..4) *)
    PE_A_ID : DINT := 0; (* plant: Id of the onion in front of pusher A (0 = none) *)
    PE_B_ID : DINT := 0; (* plant: Id of the onion in front of pusher B (0 = none) *)
    PE_C_ID : DINT := 0; (* plant: Id of the onion in front of pusher C (0 = none) *)
    PUSHED_A_ID : DINT := 0; (* plant: Id of the last onion that entered crate A *)
    PUSHED_B_ID : DINT := 0; (* plant: Id of the last onion that entered crate B *)
    PUSHED_C_ID : DINT := 0; (* plant: Id of the last onion that entered crate C *)
    BEAM_BLOCKED : BOOL := FALSE; (* plant: Size-gauge light beam is interrupted *)
    CRATE_A_READY : BOOL := FALSE; (* plant: Full crate A waits at the pick-up spot *)
    CRATE_B_READY : BOOL := FALSE; (* plant: Full crate B waits at the pick-up spot *)
    CRATE_C_READY : BOOL := FALSE; (* plant: Full crate C waits at the pick-up spot *)
    PALLET_FULL : BOOL := FALSE; (* plant: Pallet has no free slot *)
    HEARTBEAT_OUT : DINT := 0; (* plc: Counter incremented every PLC scan *)
    SESSION_ACK : DINT := 0; (* plc: Copy of PLANT_SESSION after the PLC reset itself *)
    PLC_FAULT : INT := 0; (* plc: 0 = ok, 1 = E-stop, 2 = belt jam, 4 = plant link lost *)
    RUNNING : BOOL := FALSE; (* plc: Line is running *)
    CONVEYOR_RUN : BOOL := FALSE; (* plc: Belt motor *)
    FEED_GATE : BOOL := FALSE; (* plc: Feeder gate (an onion drops at the rising edge) *)
    PUSH_A : BOOL := FALSE; (* plc: Pusher A (grade A) *)
    PUSH_B : BOOL := FALSE; (* plc: Pusher B (grade B) *)
    PUSH_C : BOOL := FALSE; (* plc: Pusher C (grade C) *)
    CALL_A : BOOL := FALSE; (* plc: Robot: take the full crate A to the pallet *)
    CALL_B : BOOL := FALSE; (* plc: Robot: take the full crate B to the pallet *)
    CALL_C : BOOL := FALSE; (* plc: Robot: take the full crate C to the pallet *)
    COUNT_A : DINT := 0; (* plc: Graded onions: A (70 to under 90 mm) *)
    COUNT_B : DINT := 0; (* plc: Graded onions: B (55 to under 70 mm) *)
    COUNT_C : DINT := 0; (* plc: Graded onions: C (40 to under 55 mm) *)
    COUNT_REJECT : DINT := 0; (* plc: Rejected onions (sprouted, under 40 mm, 90 mm or more) *)
    RATE_PM : INT := 0; (* plc: Onions per minute (last full minute) *)
    TESTED : DINT := 0; (* plc: Onions checked against the audit grade *)
    ACCURACY_PCT : INT := 0; (* plc: Classification accuracy, % *)
END_VAR

VAR
    Session : DINT := 0;
    Total : DINT := 0;
    Correct : DINT := 0;
    RejectPct : INT := 0;
    LastGrade : INT := 0;
    LastDiameter : REAL := 0.0;
    LastHB : DINT := 0;
    LastCam : DINT := 0;
    Run : BOOL := FALSE;
    NewSession : BOOL := FALSE;
    DoClear : BOOL := FALSE;
    IntervalDs : INT := 20;
    PtSeconds : TIME;
    PtTenths : TIME;
    GateOpen : BOOL := FALSE;
    Grade : INT := 4;
    i : INT := 0;
    Idx : INT := 0;
    PlantStalled : BOOL := FALSE;
    RateBase : DINT := 0;
    ActiveA : DINT := 0;
    ActiveB : DINT := 0;
    ActiveC : DINT := 0;
    DoneA : DINT := 0;
    DoneB : DINT := 0;
    DoneC : DINT := 0;
    Route : ARRAY[0..15] OF INT;
    StartEdge : R_TRIG;
    ResetEdge : R_TRIG;
    KpiEdge : R_TRIG;
    LinkTimer : TON;
    JamTimer : TON;
    FeedTimer : TON;
    GateTimer : TON;
    RateTimer : TON;
    PushTimerA : TON;
    PushTimerB : TON;
    PushTimerC : TON;
END_VAR

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

END_PROGRAM

CONFIGURATION Config0
    RESOURCE Res0 ON PLC
        TASK MainTask(INTERVAL := T#50ms, PRIORITY := 0);
        PROGRAM Inst0 WITH MainTask : OnionSorting;
    END_RESOURCE
END_CONFIGURATION
'''

import asyncio
import json
import random
import struct
import threading
import time


PERIOD = 0.05                  # model seconds between two exchanges with the PLC
OPERATOR_HOLD = 0.5            # a button pulse is kept at least this long (the OPC UA link samples about every 0.1 s)
GRADE_BITS = {'CONVEYOR_RUN': 'Y0', 'FEED_GATE': 'Y1', 'PUSH_A': 'Y2', 'PUSH_B': 'Y3', 'PUSH_C': 'Y4',
              'CALL_A': 'Y5', 'CALL_B': 'Y6', 'CALL_C': 'Y7', 'RUNNING': 'M0'}
KPI_WORDS = {'COUNT_A': 'D40', 'COUNT_B': 'D41', 'COUNT_C': 'D42', 'COUNT_REJECT': 'D43',
             'RATE_PM': 'D60', 'TESTED': 'D62', 'ACCURACY_PCT': 'D64'}


def f32(x):
    return struct.unpack('f', struct.pack('f', float(x)))[0]


class ExtPlc:
    """Drop-in replacement for the built-in PLC object (bits / words / scan)."""

    def __init__(self, backend):
        self.backend = backend
        self.bits, self.words = {}, {}
        self.rungs = []
        self.model = None
        self.session = random.randint(1, 2_000_000_000)
        self.hb = 0
        self.t_next = 0.0
        self.hold = {}
        self.out = {}
        self.link_ok = True
        self._seen_fault, self._seen_link = 0, True

    def attach(self, model):
        self.model = model

    def ready(self):
        return bool(self.out) and self.out.get('SESSION_ACK') == self.session and self.link_ok

    def status(self):
        be = self.backend
        return dict(mode=getattr(be, 'label', 'External PLC'), ext=True, link=bool(self.link_ok), ms=int(getattr(be, 'cycle_ms', 0)))

    def _button(self, dev, t):
        if self.bits.get(dev):
            self.hold[dev] = t + OPERATOR_HOLD
        return t < self.hold.get(dev, -1.0)

    def scan(self):
        m = self.model
        if m.t + 1e-9 < self.t_next:
            return
        self.t_next = m.t + PERIOD
        t = m.t
        inp = m.ext_inputs()
        self.hb = (self.hb + 1) % 2_000_000_000
        inp.update(PLANT_SESSION=self.session, HEARTBEAT_IN=self.hb,
                   START_PB=self._button('X0', t), STOP_PB=self._button('X1', t), RESET_PB=self._button('X3', t),
                   KPI_RESET=self._button('M310', t), ESTOP=not self.bits.get('X2', True),
                   FEED_INTERVAL=int(self.words.get('D240', 20)))
        out = self.backend.exchange(inp, t)
        self.link_ok = self.backend.healthy()
        if out:
            self.out = out
        self._apply(m)

    PLC_FAULT_TEXT = {1: 'E-stop', 2: 'belt jam (size beam blocked 5 s while running)', 4: 'PLC did not see the plant heartbeat for 10 s'}

    def _report(self, m, fault):
        if fault != self._seen_fault:
            if fault:
                o = self.out
                print(f'  [PLC] FAULT {fault}: {self.PLC_FAULT_TEXT.get(fault, "?")}  (t={m.t:.1f}s, beam={self.bits.get("X5")},'
                      f' link={self.link_ok}, plc cycle={getattr(self.backend, "cycle_ms", 0):.0f} ms)')
            else:
                print(f'  [PLC] fault {self._seen_fault} cleared (t={m.t:.1f}s)')
            self._seen_fault = fault
        if self.link_ok != self._seen_link:
            print(f'  [PLC] link to the PLC {"restored" if self.link_ok else "LOST (no PLC heartbeat for 4 s, outputs frozen)"} (t={m.t:.1f}s)')
            self._seen_link = self.link_ok

    def _apply(self, m):
        o, b, w = self.out, self.bits, self.words
        live = self.link_ok and bool(o)
        for tag, dev in GRADE_BITS.items():
            b[dev] = bool(o.get(tag)) if live else False          # PLC silent -> every output off (plant frozen, fail-safe)
        fault = int(o.get('PLC_FAULT', 0))
        self._report(m, fault)
        b['M1'] = fault != 0 or not self.link_ok
        b['Y10'] = bool(b['M0'])                               # green lamp: running
        b['Y11'] = b['M1']                                     # red lamp: fault
        b['M50'] = fault == 1
        b['M51'] = fault == 2
        b['M57'] = fault == 4 or not self.link_ok
        for tag, dev in KPI_WORDS.items():
            w[dev] = int(o.get(tag, 0))
        tot = sum(w[d] for d in ('D40', 'D41', 'D42', 'D43'))
        w['D44'] = tot
        w['D45'] = (100 * w['D43']) // tot if tot else 0
        w['D63'] = (w['D64'] * w['D62']) // 100
        for g, d1, d2 in (('A', 'D50', 'D54'), ('B', 'D51', 'D55'), ('C', 'D52', 'D56')):
            w[d1] = m.crates[g].count
            w[d2] = m.stats['crates_done'][g]


class LocalBackend:
    """Runs a PLC implementation in-process (SoftPlc or the compiled ST from the test harness) on model time.
    delay = number of exchanges the outputs are held back, to imitate the OPC UA synchronisation latency."""

    def __init__(self, plc, delay=2):
        self.plc, self.delay, self.q, self.last_t = plc, delay, [], None
        self.out_names = tag_names('plc')

    def exchange(self, inp, t):
        plc = self.plc
        plc.write(inp)
        ms = int(round((t - (self.last_t if self.last_t is not None else t - plc.tick_ms / 1000.0)) * 1000))
        self.last_t = t
        for _ in range(max(1, ms // plc.tick_ms)):
            plc.scan()
        self.q.append(plc.read(self.out_names))
        return self.q.pop(0) if len(self.q) > self.delay else None

    def healthy(self):
        return True


# ===================================================================================================== Python copy of the ST program
class SoftPlc:
    """Python mirror of plc/OnionSorting.st (same scan order). Used by --plc demo and checked against the compiled ST by the tests."""

    def __init__(self, tick_ms=50):
        self.tick_ms, self.now = tick_ms, 0
        self.v = {t.name: {'BOOL': False, 'INT': 0, 'DINT': 0, 'REAL': 0.0}[t.type] for t in TAGS}
        self.v['FEED_INTERVAL'] = 20
        s = self.s = dict(Session=0, LastHB=0, LastCam=0, Run=False, NewSession=False, DoClear=False, IntervalDs=20, GateOpen=False,
                          Grade=4, RateBase=0, Route=[0] * 16, Total=0, Correct=0)
        for g in 'ABC':
            s['Active' + g], s['Done' + g] = 0, 0
        self.edge = {k: False for k in ('Start', 'Reset', 'Kpi')}
        self.ton = {k: [None, False] for k in ('Link', 'Jam', 'Feed', 'Gate', 'Rate', 'PushA', 'PushB', 'PushC')}

    def write(self, d):
        for k, x in d.items():
            self.v[k] = f32(x) if BY_NAME[k].type == 'REAL' else (bool(x) if BY_NAME[k].type == 'BOOL' else int(x))

    def read(self, names):
        return {n: self.v[n] for n in names}

    def _ton(self, k, IN, pt_ms):
        st = self.ton[k]
        if IN:
            if st[0] is None:
                st[0] = self.now
            st[1] = (self.now - st[0]) >= pt_ms
        else:
            st[0], st[1] = None, False
        return st[1]

    def _rtrig(self, k, clk):
        q = clk and not self.edge[k]
        self.edge[k] = clk
        return q

    def scan(self, advance_ms=None):
        self.now += self.tick_ms if advance_ms is None else advance_ms
        v, s = self.v, self.s
        v['HEARTBEAT_OUT'] = 0 if v['HEARTBEAT_OUT'] >= 2000000000 else v['HEARTBEAT_OUT'] + 1
        s['DoClear'] = s['NewSession'] = False
        if v['PLANT_SESSION'] != s['Session']:
            s['Session'] = v['SESSION_ACK'] = v['PLANT_SESSION']
            s['NewSession'] = s['DoClear'] = True
            s['Run'], v['PLC_FAULT'], s['LastCam'] = False, 0, v['CAM_ID']
            for g in 'ABC':
                s['Active' + g] = s['Done' + g] = 0
        if self._rtrig('Kpi', v['KPI_RESET']) and not s['Run']:
            s['DoClear'] = True
        if s['DoClear']:
            s['Route'] = [4] * 16
            for k in ('COUNT_A', 'COUNT_B', 'COUNT_C', 'COUNT_REJECT', 'RATE_PM', 'TESTED', 'ACCURACY_PCT'):
                v[k] = 0
            s['RateBase'], s['Total'], s['Correct'] = 0, 0, 0
        link = self._ton('Link', v['HEARTBEAT_IN'] == s['LastHB'] and s['Session'] != 0, 10000)
        t0 = self.ton['Link'][0]
        stalled = t0 is not None and (self.now - t0) >= 1000        # plant frozen: hold the plant watchers
        s['LastHB'] = v['HEARTBEAT_IN']
        jam = self._ton('Jam', v['BEAM_BLOCKED'] and s['Run'] and not stalled, 5000)
        reset = self._rtrig('Reset', v['RESET_PB'])
        if v['PLC_FAULT'] == 1 and not v['ESTOP']:
            v['PLC_FAULT'] = 0
        if v['PLC_FAULT'] == 4 and not link:
            v['PLC_FAULT'] = 0
        if v['ESTOP']:
            v['PLC_FAULT'] = 1
        elif v['PLC_FAULT'] == 0:
            if jam:
                v['PLC_FAULT'] = 2
            elif link:
                v['PLC_FAULT'] = 4
        if reset and not v['ESTOP'] and not link:
            v['PLC_FAULT'] = 0
        start = self._rtrig('Start', v['START_PB'])
        if v['STOP_PB'] or v['ESTOP'] or v['PLC_FAULT'] != 0:
            s['Run'] = False
        elif start and not s['NewSession']:
            s['Run'] = True
        run = s['Run']
        v['RUNNING'] = v['CONVEYOR_RUN'] = run
        s['IntervalDs'] = min(100, max(15, v['FEED_INTERVAL']))
        if self._ton('Feed', run and not self.ton['Feed'][1] and not stalled, (s['IntervalDs'] // 10) * 1000 + (s['IntervalDs'] % 10) * 100):
            s['GateOpen'] = True
        if self._ton('Gate', s['GateOpen'], 500) or not run:
            s['GateOpen'] = False
        v['FEED_GATE'] = s['GateOpen'] and run
        if v['CAM_ID'] != s['LastCam']:
            s['LastCam'] = v['CAM_ID']
            d = v['DIAMETER_MM']
            if v['SPROUTED'] or d < 40.0 or d >= 90.0:
                gr = 4
            elif d >= 70.0:
                gr = 1
            elif d >= 55.0:
                gr = 2
            else:
                gr = 3
            s['Route'][v['CAM_ID'] % 16] = gr
            v[{1: 'COUNT_A', 2: 'COUNT_B', 3: 'COUNT_C'}.get(gr, 'COUNT_REJECT')] += 1
            v['TESTED'] += 1
            if gr == v['TRUE_GRADE']:
                s['Correct'] += 1
        s['Total'] = v['COUNT_A'] + v['COUNT_B'] + v['COUNT_C'] + v['COUNT_REJECT']
        if v['TESTED'] > 0:
            v['ACCURACY_PCT'] = s['Correct'] * 100 // v['TESTED']
        if self._ton('Rate', run and not self.ton['Rate'][1], 60000):
            v['RATE_PM'] = s['Total'] - s['RateBase']
            s['RateBase'] = s['Total']
        for g, grade in (('A', 1), ('B', 2), ('C', 3)):
            pe = v[f'PE_{g}_ID']
            if run and s['Active' + g] == 0 and pe != 0 and pe != s['Done' + g] and s['Route'][pe % 16] == grade:
                s['Active' + g] = pe
            tq = self._ton('Push' + g, run and s['Active' + g] != 0 and not stalled, 4000)
            act = s['Active' + g]
            if act != 0 and (v[f'PUSHED_{g}_ID'] == act or tq or (pe != act and pe != 0)):
                s['Done' + g], s['Active' + g] = act, 0
            v['PUSH_' + g] = run and s['Active' + g] != 0
        for g in 'ABC':
            v['CALL_' + g] = v[f'CRATE_{g}_READY'] and not v['PALLET_FULL'] and not v['ESTOP']
        if not s['Run']:
            v['FEED_GATE'] = v['CONVEYOR_RUN'] = v['PUSH_A'] = v['PUSH_B'] = v['PUSH_C'] = False


# ===================================================================================================== OPC UA
def _vtype(ua, tag):
    return {'BOOL': ua.VariantType.Boolean, 'INT': ua.VariantType.Int16, 'DINT': ua.VariantType.Int32, 'REAL': ua.VariantType.Float}[tag.type]


def _py(vt_name, v):
    if vt_name == 'Boolean':
        return bool(v)
    if vt_name in ('Float', 'Double'):
        return float(v)
    return int(v)


async def discover_nodes(client, mapping=None, log=print):
    """Find every tag of plc_tags.py in the PLC's address space by browse name. A JSON map {TAG: 'ns=2;s=PLC.Onion.TAG'} overrides it."""
    from asyncua import ua
    found = {}
    if mapping:
        for k, ident in mapping.items():
            found[k] = client.get_node(ident)
    else:
        seen, todo = set(), [(client.nodes.objects, 0)]
        while todo and len(seen) < 20000:
            node, depth = todo.pop()
            for child in await node.get_children():
                if child.nodeid.NamespaceIndex == 0 or child.nodeid.to_string() in seen:
                    continue
                seen.add(child.nodeid.to_string())
                cls = await child.read_node_class()
                if cls == ua.NodeClass.Variable:
                    nm = (await child.read_browse_name()).Name.rsplit('.', 1)[-1]
                    if nm in BY_NAME:
                        if nm in found:
                            raise ValueError(f'tag {nm} exists twice in the OPC UA address space; give a node map with --opc-map')
                        found[nm] = child
                elif cls == ua.NodeClass.Object and depth < 10:
                    todo.append((child, depth + 1))
    missing = [t.name for t in TAGS if t.name not in found]
    if missing:
        raise ValueError('These tags are not exposed by the OPC UA server: ' + ', '.join(missing) +
                         '. Add them in the OpenPLC OPC UA configuration (see plc/io_map.csv).')
    return found


class OpcUaBackend:
    """asyncua client in a background thread. exchange() never blocks the simulation: it hands over the newest inputs
    and returns the newest outputs received from the PLC."""

    def __init__(self, endpoint, user=None, password=None, security=None, mapping=None, period=0.1, log=print):
        self.endpoint, self.user, self.password, self.security, self.mapping = endpoint, user, password, security, mapping
        self.period, self.log = period, log
        self.lock = threading.Lock()
        self.pending, self.latest = {}, {}
        self.ready, self.stop_flag = threading.Event(), False
        self.error = None
        self.last_hb, self.last_hb_change = None, time.monotonic()
        self.cycles, self.cycle_ms = 0, 0.0
        self.label = 'OpenPLC via OPC UA'
        self.th = threading.Thread(target=lambda: asyncio.run(self._main()), daemon=True)

    def start(self, timeout=20):
        self.th.start()
        if not self.ready.wait(timeout):
            raise RuntimeError(f'OPC UA: no connection to {self.endpoint} ({self.error or "timeout"})')
        if self.error:
            raise RuntimeError(self.error)

    def close(self):
        self.stop_flag = True

    def exchange(self, inp, t):
        with self.lock:
            self.pending = dict(inp)
            return dict(self.latest)

    def healthy(self):
        return self.error is None and (time.monotonic() - self.last_hb_change) < 4.0

    async def _main(self):
        from asyncua import Client, ua
        first = True
        while not self.stop_flag:
            client = None
            try:
                client = Client(self.endpoint, timeout=3)
                if self.security:
                    await client.set_security_string(self.security)
                if self.user:
                    client.set_user(self.user)
                    client.set_password(self.password or '')
                await client.connect()
                nodes = await discover_nodes(client, self.mapping)
                types = {}
                for n, node in nodes.items():
                    types[n] = await node.read_data_type_as_variant_type()
                    want = _vtype(ua, BY_NAME[n])
                    if types[n] != want:
                        raise ValueError(f'{n}: the server exposes OPC UA type {types[n].name}, expected {want.name} (IEC {BY_NAME[n].type})')
                outs = [n for n in tag_names('plc')]
                ins = [n for n in BY_NAME if BY_NAME[n].owner != 'plc']
                self.error = None
                self.last_hb_change = time.monotonic()
                if first:
                    self.log(f'  OPC UA connected: {self.endpoint}  ({len(nodes)} tags found)')
                    first = False
                self.ready.set()
                sent, last_full = {}, 0.0
                while not self.stop_flag:
                    t0 = time.monotonic()
                    with self.lock:
                        pend = dict(self.pending)
                    if pend:
                        # Write only what changed (the PLC runtime queues every write in a 128-entry journal, so writing all 36
                        # inputs ten times a second is needless load), everything once a second so a restarted PLC catches up,
                        # and CAM_ID last so the PLC never sees a new onion id together with the previous onion's diameter.
                        full = t0 - last_full >= 1.0
                        if full:
                            last_full = t0
                        names = [n for n in ins if n in pend and (full or n == 'HEARTBEAT_IN' or sent.get(n) != pend[n])]
                        names.sort(key=lambda n: n == 'CAM_ID')
                        vals = [ua.DataValue(ua.Variant(_py(types[n].name, pend[n]), types[n])) for n in names]
                        if names:
                            await client.write_values([nodes[n] for n in names], vals)
                            sent.update({n: pend[n] for n in names})
                    dvs = await client.read_attributes([nodes[n] for n in outs], ua.AttributeIds.Value)
                    cur = {}
                    for n, dv in zip(outs, dvs):
                        dv.StatusCode.check()
                        cur[n] = _py(types[n].name, dv.Value.Value)
                    if cur['HEARTBEAT_OUT'] != self.last_hb:
                        self.last_hb, self.last_hb_change = cur['HEARTBEAT_OUT'], time.monotonic()
                    with self.lock:
                        self.latest = cur
                    self.cycles += 1
                    self.cycle_ms = 0.9 * self.cycle_ms + 0.1 * (time.monotonic() - t0) * 1000
                    await asyncio.sleep(max(0.0, self.period - (time.monotonic() - t0)))
            except Exception as e:
                self.error = f'{type(e).__name__}: {e}'
                self.log('  OPC UA problem:', self.error)
                self.ready.set()
                await asyncio.sleep(1.0)
                if first:
                    return                     # never connected: report to the caller instead of retrying forever
            finally:
                if client is not None:
                    try:
                        await client.disconnect()
                    except Exception:
                        pass


# ----------------------------------------------------------------------------- demo server (stands in for OpenPLC)
def serve_demo(endpoint, plc=None, ready=None, stop=None, log=print, sync_s=0.1):
    """Run a local OPC UA server that hosts every tag and executes the PLC program (SoftPlc by default) in wall time.
    It behaves like the OpenPLC OPC UA plugin: PLC tags are read-only, inputs are writable, and data is copied between the OPC UA
    nodes and the PLC only every sync_s seconds (the plugin's cycle_time_ms), while the PLC scans every 50 ms."""
    from asyncua import Server, ua
    plc = plc or SoftPlc()

    async def main():
        srv = Server()
        await srv.init()
        srv.set_endpoint(endpoint)
        srv.set_server_name('Onion line DEMO PLC (stands in for OpenPLC)')
        srv.set_security_policy([ua.SecurityPolicyType.NoSecurity])
        ns = await srv.register_namespace('urn:onion-line:demo-plc')
        obj = await srv.nodes.objects.add_object(ua.NodeId('PLC.Onion', ns), 'Onion')
        nodes = {}
        for t in TAGS:
            n = await obj.add_variable(ua.NodeId('PLC.Onion.' + t.name, ns), t.name, ua.Variant(initial(t.owner).get(t.name, 0) if t.type != 'BOOL' else False, _vtype(ua, t)))
            if t.owner != 'plc':
                await n.set_writable()
            nodes[t.name] = n
        ins = [n for n in nodes if BY_NAME[n].owner != 'plc']
        outs = tag_names('plc')
        async with srv:
            if ready:
                ready.set()
            log(f'  demo PLC (SoftPlc) serving {endpoint}')
            t0 = time.monotonic()
            last_sync = 0.0
            while not (stop and stop.is_set()):
                now = time.monotonic() - t0
                if now - last_sync >= sync_s - 1e-3:
                    last_sync = now
                    vals = [await nodes[n].read_data_value() for n in ins]
                    plc.write({n: dv.Value.Value for n, dv in zip(ins, vals)})
                    out = plc.read(outs)
                    for n in outs:
                        await nodes[n].write_value(ua.DataValue(ua.Variant(out[n], _vtype(ua, BY_NAME[n]))))
                plc.scan(int(now * 1000) - plc.now)
                await asyncio.sleep(0.05)
    asyncio.run(main())


def start_demo_server(endpoint, plc=None, log=print):
    ready, stop = threading.Event(), threading.Event()
    th = threading.Thread(target=serve_demo, args=(endpoint, plc, ready, stop, log), daemon=True)
    th.start()
    if not ready.wait(15):
        raise RuntimeError('the stand-in OPC UA server did not start (port busy? try --opc-endpoint opc.tcp://127.0.0.1:4851/onion/)')
    return stop


DEFAULT_OPC_ENDPOINT = 'opc.tcp://127.0.0.1:4840/openplc/opcua'


def check_server(endpoint, user=None, password=None, security=None, mapping=None):
    """--opc-check: connect, find every tag, compare data types, show the PLC values. Returns 0 when everything is in order."""
    try:
        from asyncua import Client, ua
    except ImportError:
        print('The OPC UA client library is missing:  pip install asyncua')
        return 2

    async def run():
        client = Client(endpoint, timeout=4)
        if security:
            await client.set_security_string(security)
        if user:
            client.set_user(user)
            client.set_password(password or '')
        print(f'Connecting to {endpoint} ...')
        await client.connect()
        bad = 0
        try:
            print('  connected. Namespaces:', await client.get_namespace_array())
            found = {}
            try:
                found = await discover_nodes(client, mapping)
            except ValueError as e:
                print('  PROBLEM:', e)
                return 1
            print(f'  {"tag":<16}{"IEC":<6}{"expected":<9}{"server":<9}{"owner":<9}value')
            for t in TAGS:
                node = found[t.name]
                vt = await node.read_data_type_as_variant_type()
                want = _vtype(ua, t)
                val = await node.read_value() if t.owner == 'plc' else ''
                flag = '' if vt == want else '   <-- TYPE MISMATCH'
                bad += vt != want
                print(f'  {t.name:<16}{t.type:<6}{want.name:<9}{vt.name:<9}{t.owner:<9}{val}{flag}')
            hb1 = await found['HEARTBEAT_OUT'].read_value()
            await asyncio.sleep(0.6)
            hb2 = await found['HEARTBEAT_OUT'].read_value()
            print('  PLC heartbeat:', hb1, '->', hb2, '(program is scanning)' if hb2 != hb1 else '  <-- NOT CHANGING: is the program running?')
            bad += hb2 == hb1
            print('All', len(TAGS), 'tags found.' if not bad else 'tags found, but see the problems above.')
        finally:
            await client.disconnect()
        return 1 if bad else 0
    try:
        return asyncio.run(run())
    except Exception as e:
        print('Connection failed:', type(e).__name__, e)
        return 1


def connect_external(a, mapping=None):
    """Start the OPC UA link for --plc opcua / --plc demo and return the ExtPlc object."""
    try:
        import asyncua  # noqa: F401
    except ImportError:
        raise SystemExit('The OPC UA client library is missing.  Install it with:  pip install asyncua')
    ep = a.opc_endpoint
    if a.plc == 'demo':
        ep = ep or 'opc.tcp://127.0.0.1:4841/onion/'
        print('Starting the stand-in PLC (a Python copy of OnionSorting.st) at', ep)
        start_demo_server(ep, log=lambda *x: None)
        mapping = None
    else:
        ep = ep or DEFAULT_OPC_ENDPOINT
    be = OpcUaBackend(ep, a.opc_user, a.opc_password, a.opc_security, mapping)
    if a.plc == 'demo':
        be.label = 'Demo PLC (Python copy) via OPC UA'
    try:
        be.start()
    except RuntimeError as e:
        raise SystemExit(f'\n{e}\n\nCheck that OpenPLC is running with the OPC UA server enabled and the program loaded, then try:\n'
                         f'    python onion_sorting_line.py --opc-check --opc-endpoint {ep}')
    return ExtPlc(be)


# ====================================================================================================
#  4. DASHBOARD (local web page, standard library only)
# ====================================================================================================

import json
import math
import queue
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

FAULT_NAMES = {50: 'F1 E-stop pressed', 51: 'F2 Belt jam (size beam blocked too long)', 52: 'F3 Pusher A / lane A entry failed',
               53: 'F4 Pusher B / lane B entry failed', 54: 'F5 Pusher C / lane C entry failed', 55: 'F6 Grade queue overflow',
               56: 'F7 Reject bin full', 57: 'F8 PLC link lost (no heartbeat)'}


class Telemetry:
    """Collects everything the dashboard shows. Called from the simulation loop (single thread)."""

    SAMPLE_S = 1.0

    def __init__(self, model, keep=1800):
        self.m, self.keep = model, keep
        self.series = []
        self.ev_seen = 0
        self.events = []
        self.run_t = self.fault_t = self.idle_t = 0.0
        self.faults_log, self.prev_faults = [], set()
        self.next_sample = 0.0
        self.snapshot = {}
        self.last_dt = 0.01
        self.k = 0
        self.spawn_base = 0

    # ---- called every model step (cheap)
    def on_step(self):
        m = self.m
        b = m.plc.bits
        dt = 0.01
        if any(b.get(f'M{n}') for n in range(50, 58)):
            self.fault_t += dt
        elif b.get('Y0'):
            self.run_t += dt
        else:
            self.idle_t += dt
        if len(m.events) > self.ev_seen:
            self.events.extend(m.events[self.ev_seen:])
            self.ev_seen = len(m.events)
        self.k += 1
        if self.k % 100 == 0:                  # once per simulated second: fault edge log
            now = {n for n in range(50, 58) if b.get(f'M{n}')}
            for n in sorted(now - self.prev_faults):
                self.faults_log.append(dict(t=round(m.t, 1), code=n, text=FAULT_NAMES[n], state='RAISED'))
            for n in sorted(self.prev_faults - now):
                self.faults_log.append(dict(t=round(m.t, 1), code=n, text=FAULT_NAMES[n], state='CLEARED'))
            self.prev_faults = now
            self.faults_log = self.faults_log[-60:]
        if m.t >= self.next_sample:
            self.next_sample = m.t + self.SAMPLE_S
            self.sample()

    # ---- derived numbers
    @staticmethod
    def mass_kg(d_mm):
        d = d_mm / 10.0
        return math.pi / 6.0 * d * d * (0.85 * d) * 0.95 / 1000.0     # ellipsoid volume (cm3) x 0.95 g/cm3

    def stats(self):
        ev = self.events
        conf = [[0] * 4 for _ in range(4)]          # rows: true grade 1..4, cols: assigned grade 1..4 (4 = rejected)
        sizes = {g: [] for g in 'ABCR'}
        mass = {g: 0.0 for g in 'ABCR'}
        reasons = dict(sprouted=0, oversize=0, undersize=0, false_reject=0, sprout_missed=0, mis_sized=0)
        tp = fn = fp = tn = 0
        for e in ev:
            a = {'A': 1, 'B': 2, 'C': 3, 'R': 4}[e['dest']]
            conf[e['tg'] - 1][a - 1] += 1
            sizes[e['dest']].append(e['d'])
            mass[e['dest']] += self.mass_kg(e['d'])
            if e['sp'] and e['seen']:
                tp += 1
            elif e['sp']:
                fn += 1
            elif e['seen']:
                fp += 1
            else:
                tn += 1
            if e['dest'] == 'R':
                if e['sp']:
                    reasons['sprouted'] += 1
                elif e['d'] >= 90:
                    reasons['oversize'] += 1
                elif e['d'] < 40:
                    reasons['undersize'] += 1
                else:
                    reasons['false_reject'] += 1
            elif e['sp']:
                reasons['sprout_missed'] += 1
            elif e['tg'] != a:
                reasons['mis_sized'] += 1
        n = len(ev)
        correct = sum(conf[i][i] for i in range(4))
        ms = {}
        for g, v in sizes.items():
            if v:
                mu = sum(v) / len(v)
                sd = math.sqrt(sum((x - mu) ** 2 for x in v) / len(v))
                ms[g] = [round(mu, 1), round(sd, 1), min(v), max(v)]
            else:
                ms[g] = [0, 0, 0, 0]
        hist = {}
        for e in ev:
            k = int(e['d'] // 5) * 5
            hist.setdefault(k, [0, 0, 0, 0])[e['tg'] - 1] += 1
        return dict(n=n, conf=conf, acc_phys=round(100.0 * correct / n, 1) if n else 0, reasons=reasons,
                    sprout=dict(tp=tp, fn=fn, fp=fp, tn=tn,
                                recall=round(100.0 * tp / (tp + fn), 1) if tp + fn else None,
                                precision=round(100.0 * tp / (tp + fp), 1) if tp + fp else None,
                                false_alarm=round(100.0 * fp / (fp + tn), 1) if fp + tn else None),
                    size=ms, hist=[[k] + v for k, v in sorted(hist.items())],
                    mass={g: round(v, 2) for g, v in mass.items()})

    def sample(self):
        m = self.m
        k = m.kpis()
        st = self.stats()
        tot = max(1e-9, self.run_t + self.fault_t + self.idle_t)
        row = dict(t=round(m.t, 1), A=k['A'], B=k['B'], C=k['C'], R=k['reject'], total=k['total'], r10=k['rate10'], r60=k['rate60'],
                   acc=k['accuracy'], accp=st['acc_phys'], la=m.waiting_crates()['A'], lb=m.waiting_crates()['B'], lc=m.waiting_crates()['C'],
                   avail=round(100.0 * self.run_t / tot, 1), mass=round(sum(st['mass'].values()), 2), mA=st['mass']['A'], mB=st['mass']['B'], mC=st['mass']['C'])
        self.series.append(row)
        self.series = self.series[-self.keep:]
        w, b = m.plc.words, m.plc.bits
        ideal = 60.0 / max(0.1, w.get('D240', 20) / 10.0)
        run_min = max(1e-9, self.run_t / 60.0)
        actual = k['total'] / run_min if self.run_t > 5 else 0
        avail = self.run_t / tot
        perf = min(1.0, actual / ideal) if ideal else 0
        good = sum(st['conf'][i][i] for i in range(4))
        qual = good / st['n'] if st['n'] else 0
        self.snapshot = dict(
            t=round(m.t, 1), kpi=k, series=self.series, stats=st,
            live=dict(running=bool(b.get('M0')), fault=bool(b.get('M1')), estop=not b.get('X2', True), belt=bool(b.get('Y0')),
                      feeder=bool(b.get('Y1')), pushers=[bool(b.get('Y2')), bool(b.get('Y3')), bool(b.get('Y4'))],
                      pack_ready=[bool(b.get('Y5')), bool(b.get('Y6')), bool(b.get('Y7'))],
                      fill=[w.get('D50', 0), w.get('D51', 0), w.get('D52', 0)], cap=w.get('D230', 10),
                      lanes=[m.waiting_crates()[g] for g in 'ABC'], faults=[n for n in range(50, 58) if b.get(f'M{n}')],
                      robot=dict(busy=bool(m.arms['palletizer']['busy']), cycles=m.arms['palletizer']['cycles']),
                      plc=m.plc_status(),
                      onions_on_pallet=m.stats['onions_on_pallet'], pallet_crates=len(m.pallet_crates),
                      reject_bin=len(m.reject_bin) + max(0, m.stats['rejected'] - len(m.reject_bin)),
                      feed_s=w.get('D240', 20) / 10.0, p_sprout=getattr(m.source, 'p_sprout', 0)),
            oee=dict(avail=round(100 * avail, 1), perf=round(100 * perf, 1), qual=round(100 * qual, 1),
                     oee=round(100 * avail * perf * qual, 1), ideal=round(ideal, 1), actual=round(actual, 1),
                     run_s=round(self.run_t, 1), fault_s=round(self.fault_t, 1), idle_s=round(self.idle_t, 1)),
            faults_log=self.faults_log[-12:][::-1],
            sim_total=m.stats['spawned'] - self.spawn_base)


class Dashboard:
    def __init__(self, telemetry, port=8765, html=''):
        self.tel, self.port, self.html = telemetry, port, html
        self.cmds = queue.Queue()
        self.deferred = []          # (sim time, device): reset pulses wait 0.5 s so a just-cleared cause is seen first
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith('/api'):
                    self._send(200, json.dumps(outer.tel.snapshot).encode(), 'application/json')
                else:
                    self._send(200, outer.html.encode('utf-8'), 'text/html; charset=utf-8')

            def do_POST(self):
                try:
                    n = int(self.headers.get('Content-Length', 0))
                    outer.cmds.put(json.loads(self.rfile.read(n) or b'{}'))
                    self._send(200, b'{"ok":true}', 'application/json')
                except Exception:
                    self._send(400, b'{"ok":false}', 'application/json')

        self.httpd = ThreadingHTTPServer(('127.0.0.1', port), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def open_browser(self):
        try:
            webbrowser.open(f'http://127.0.0.1:{self.port}/')
        except Exception:
            pass

    def reset_stats(self):
        t = self.tel
        t.events.clear()
        t.series.clear()
        t.run_t = t.fault_t = t.idle_t = 0.0
        t.m.events.clear()
        t.ev_seen = 0
        t.spawn_base = t.m.stats['spawned']

    def apply_commands(self, m):
        """Run in the simulation thread: execute queued operator commands."""
        for item in list(self.deferred):
            if m.t >= item[0]:
                m.press(item[1], 0.3)
                self.deferred.remove(item)
        while True:
            try:
                c = self.cmds.get_nowait()
            except queue.Empty:
                return
            k = c.get('cmd')
            if k == 'start':
                m.press('X0', 0.3)
            elif k == 'stop':
                m.press('X1', 0.3)
            elif k == 'estop':
                m.set_estop(True)
            elif k == 'release':
                m.set_estop(False)
                self.deferred.append((m.t + 0.5, 'X3'))
            elif k == 'reset_faults':
                self.deferred.append((m.t + 0.5, 'X3'))
            elif k == 'reset_counters':
                m.press('M310', 0.3)
                self.reset_stats()
            elif k == 'feed':
                m.plc.words['D240'] = int(max(15, min(100, round(float(c['value']) * 10))))
            elif k == 'sprout' and hasattr(m.source, 'p_sprout'):
                m.source.p_sprout = max(0.0, min(0.6, float(c['value'])))
            elif k == 'clear_belt':          # operator removes every onion still on the belt, then the grade queue is re-initialised
                for o in m.onions:
                    if o.state in ('belt', 'fall'):
                        o.gone, o.state = True, 'removed'
                m.press('M310', 0.3)
                self.reset_stats()
            elif k == 'jam':
                m.beam_stuck = bool(c.get('value'))


# ====================================================================================================
#  4b. LOCAL HMI WINDOW (tkinter)
# ====================================================================================================

import time


class LocalHmi:
    def __init__(self, dash, model):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.dash, self.m = tk, ttk, dash, model
        self.root = tk.Tk()
        self.root.title('Onion sorting line - operator panel (HMI)')
        self.root.geometry('560x690+40+40')
        self.root.configure(bg='#1b1e23')
        self.root.protocol('WM_DELETE_WINDOW', self.root.iconify)
        self.alive = True
        self.last = 0.0
        BG, FG = '#1b1e23', '#e6e8eb'
        self.bg = BG
        f = lambda sz, b=False: ('Segoe UI', sz, 'bold' if b else 'normal')
        tk.Label(self.root, text='ONION SORTING LINE', bg=BG, fg=FG, font=f(16, True)).pack(pady=(10, 0))
        tk.Label(self.root, text='operator panel - local HMI', bg=BG, fg='#8b93a0', font=f(9)).pack()
        self.state = tk.Label(self.root, text='STOPPED', bg='#c9962b', fg='white', font=f(22, True), width=18, pady=6)
        self.state.pack(pady=8)
        # lamps
        lf = tk.Frame(self.root, bg=BG); lf.pack()
        self.lamps = {}
        for key, txt in (('belt', 'Belt'), ('feed', 'Feeder'), ('pa', 'Push A'), ('pb', 'Push B'), ('pc', 'Push C'), ('ra', 'Crate A'), ('rb', 'Crate B'), ('rc', 'Crate C')):
            cell = tk.Frame(lf, bg=BG); cell.pack(side='left', padx=5)
            c = tk.Canvas(cell, width=22, height=22, bg=BG, highlightthickness=0); c.pack()
            c.create_oval(3, 3, 19, 19, fill='#3a3f47', outline='#555', tags='l')
            tk.Label(cell, text=txt, bg=BG, fg='#9aa3ad', font=f(8)).pack()
            self.lamps[key] = c
        # counters
        cf = tk.LabelFrame(self.root, text=' Production ', bg=BG, fg='#8b93a0', font=f(9)); cf.pack(fill='x', padx=14, pady=8)
        self.vals = {}
        grid = (('A', 'Grade A', '#3fae5a'), ('B', 'Grade B', '#4a7fe0'), ('C', 'Grade C', '#e69a28'), ('reject', 'Reject', '#d9534f'),
                ('total', 'Total', '#e6e8eb'), ('rate', 'Rate /min', '#e6e8eb'), ('acc', 'Accuracy %', '#e6e8eb'), ('rej', 'Reject %', '#e6e8eb'))
        for i, (k, t, col) in enumerate(grid):
            cell = tk.Frame(cf, bg=BG); cell.grid(row=i // 4, column=i % 4, padx=14, pady=6)
            tk.Label(cell, text=t, bg=BG, fg='#8b93a0', font=f(9)).pack()
            self.vals[k] = tk.Label(cell, text='0', bg=BG, fg=col, font=f(20, True)); self.vals[k].pack()
        # packaging
        pf = tk.LabelFrame(self.root, text=' Packaging ', bg=BG, fg='#8b93a0', font=f(9)); pf.pack(fill='x', padx=14, pady=4)
        self.bars = {}
        for g, col in (('A', '#3fae5a'), ('B', '#4a7fe0'), ('C', '#e69a28')):
            row = tk.Frame(pf, bg=BG); row.pack(fill='x', padx=8, pady=2)
            tk.Label(row, text=f'Crate {g}', bg=BG, fg=FG, width=8, anchor='w', font=f(10)).pack(side='left')
            c = tk.Canvas(row, width=240, height=16, bg='#2a2e35', highlightthickness=0); c.pack(side='left')
            c.create_rectangle(0, 0, 0, 16, fill=col, width=0, tags='bar')
            t = tk.Label(row, text='', bg=BG, fg=FG, font=f(10), width=22, anchor='w'); t.pack(side='left', padx=6)
            self.bars[g] = (c, t, col)
        self.pal = tk.Label(pf, text='', bg=BG, fg=FG, font=f(10)); self.pal.pack(anchor='w', padx=8, pady=(2, 6))
        # parameters
        sf = tk.LabelFrame(self.root, text=' Process parameters ', bg=BG, fg='#8b93a0', font=f(9)); sf.pack(fill='x', padx=14, pady=4)
        self.info = tk.Label(sf, text='', bg=BG, fg=FG, font=f(10), justify='left'); self.info.pack(anchor='w', padx=8, pady=2)
        self.feed = tk.DoubleVar(value=self.m.plc.words.get('D240', 20) / 10.0)
        r1 = tk.Frame(sf, bg=BG); r1.pack(fill='x', padx=8)
        tk.Label(r1, text='Feed interval (s)', bg=BG, fg=FG, width=16, anchor='w').pack(side='left')
        s1 = tk.Scale(r1, from_=1.5, to=8.0, resolution=0.5, orient='horizontal', variable=self.feed, bg=BG, fg=FG, highlightthickness=0,
                      length=260, command=lambda v: self.dash.cmds.put({'cmd': 'feed', 'value': float(v)}))
        s1.pack(side='left')
        self.spr = tk.DoubleVar(value=round(getattr(self.m.source, 'p_sprout', 0.12) * 100))
        r2 = tk.Frame(sf, bg=BG); r2.pack(fill='x', padx=8, pady=(0, 4))
        tk.Label(r2, text='Sprouted onions (%)', bg=BG, fg=FG, width=16, anchor='w').pack(side='left')
        s2 = tk.Scale(r2, from_=0, to=40, resolution=1, orient='horizontal', variable=self.spr, bg=BG, fg=FG, highlightthickness=0,
                      length=260, command=lambda v: self.dash.cmds.put({'cmd': 'sprout', 'value': float(v) / 100.0}))
        s2.pack(side='left')
        # buttons
        bf = tk.Frame(self.root, bg=BG); bf.pack(pady=8)
        def btn(txt, col, cmd, w=11):
            b = tk.Button(bf, text=txt, bg=col, fg='white', activebackground=col, font=f(10, True), width=w, height=2, bd=0,
                          command=lambda: self.dash.cmds.put({'cmd': cmd}))
            b.pack(side='left', padx=4)
        btn('START', '#2f9e4f', 'start'); btn('STOP', '#a8742a', 'stop'); btn('E-STOP', '#c0392b', 'estop')
        bf2 = tk.Frame(self.root, bg=BG); bf2.pack()
        for txt, col, cmd in (('RELEASE E-STOP', '#3b6ea8', 'release'), ('RESET FAULTS', '#3b6ea8', 'reset_faults'), ('RESET COUNTERS', '#555b66', 'reset_counters')):
            tk.Button(bf2, text=txt, bg=col, fg='white', activebackground=col, font=f(9, True), width=16, height=1, bd=0,
                      command=lambda c=cmd: self.dash.cmds.put({'cmd': c})).pack(side='left', padx=4, pady=2)
        self.fault = tk.Label(self.root, text='', bg=BG, fg='#ff8a80', font=f(10, True), wraplength=520, justify='left'); self.fault.pack(pady=6)

    FAULT_TXT = {50: 'E-STOP', 51: 'belt JAM', 52: 'size/queue error', 53: 'pusher A', 54: 'pusher B', 55: 'queue overflow', 56: 'pusher C', 57: 'PLC link lost'}

    def set_lamp(self, key, on, col='#3fd35a'):
        self.lamps[key].itemconfig('l', fill=col if on else '#3a3f47')

    def update(self, m):
        if not self.alive or time.time() - self.last < 0.15:
            return
        self.last = time.time()
        b, w, k = m.plc.bits, m.plc.words, m.kpis()
        run, flt, es = bool(b.get('Y10')), bool(b.get('Y11')), not b.get('X2', True)
        txt, col = ('E-STOP', '#c0392b') if es else (('FAULT', '#c0392b') if flt else (('RUNNING', '#2f9e4f') if run else ('STOPPED', '#c9962b')))
        self.state.config(text=txt, bg=col)
        for key, dev in (('belt', 'Y0'), ('feed', 'Y1'), ('pa', 'Y2'), ('pb', 'Y3'), ('pc', 'Y4')):
            self.set_lamp(key, b.get(dev))
        for key, dev in (('ra', 'Y5'), ('rb', 'Y6'), ('rc', 'Y7')):
            self.set_lamp(key, b.get(dev), '#f0b429')
        tot = k['total']
        vals = dict(A=k['A'], B=k['B'], C=k['C'], reject=k['reject'], total=tot, rate=k['rate60'], acc=k['accuracy'],
                    rej=(round(100.0 * k['reject'] / tot, 1) if tot else 0))
        for key, v in vals.items():
            self.vals[key].config(text=str(v))
        cap = max(1, w.get('D230', 6))
        for g, d, i in (('A', 'D50', 0), ('B', 'D51', 1), ('C', 'D52', 2)):
            c, t, col = self.bars[g]
            fill = min(w.get(d, 0), cap)
            c.coords('bar', 0, 0, 240 * fill / cap, 16)
            t.config(text=f'{fill}/{cap}   packed crates: {k["crates"][i]}')
        self.pal.config(text=f'Pallet: {len(m.pallet_crates)} of 12 crates   |   pallets completed: {k["pallets"]}')
        ps = m.plc_status()
        self.info.config(text=f'Control: {ps["mode"]}' + ((' | link OK' if ps['link'] else ' | LINK LOST') if ps['ext'] else '') + f'\n'
                              f'Belt speed {w.get("D200", 100) / 1000.0:.2f} m/s    Crate size {cap} onions    Simulated time {m.t:,.0f} s\n'
                              f'Full crates waiting for the robot: {sum(m.waiting_crates().values())}    Robot cycles: {m.arms["palletizer"]["cycles"]}')
        flts = [self.FAULT_TXT.get(n, f'F{n}') for n in range(50, 58) if b.get(f'M{n}')]
        self.fault.config(text=('ACTIVE FAULTS: ' + ', '.join(flts)) if flts else '')
        try:
            self.root.update()
        except Exception:
            self.alive = False


def make_hmi(dash, model):
    try:
        return LocalHmi(dash, model)
    except Exception as e:
        print('  (local HMI window not available:', e, ')')
        return None


DASHBOARD_HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Onion Sorting Line - Live Dashboard</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--line:#2a313c;--tx:#e6edf3;--mu:#8b949e;--A:#3fb950;--B:#58a6ff;--C:#f0883e;--R:#f85149;--Y:#d29922;--P:#bc8cff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 "Segoe UI",system-ui,Arial,sans-serif}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;padding:12px 20px;border-bottom:1px solid var(--line);position:sticky;top:0;background:var(--bg);z-index:5}
h1{font-size:18px;margin:0 12px 0 0;font-weight:600}
.pill{padding:3px 12px;border-radius:99px;font-weight:600;font-size:12px;letter-spacing:.5px}
.ok{background:#12351d;color:var(--A)}.warn{background:#3a2d0c;color:var(--Y)}.bad{background:#40181a;color:var(--R)}.off{background:#222a35;color:var(--mu)}
.sp{flex:1}.btn{background:#21262d;color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:6px 12px;cursor:pointer;font-size:13px}
.btn:hover{background:#2d333b}.btn.go{border-color:#238636;color:var(--A)}.btn.no{border-color:#8e2a2a;color:var(--R)}
main{padding:16px 20px;max-width:1700px;margin:auto}
.grid{display:grid;gap:12px}.tiles{grid-template-columns:repeat(auto-fit,minmax(190px,1fr));margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-width:0}
.card h3{margin:0 0 8px;font-size:12px;color:var(--mu);font-weight:600;text-transform:uppercase;letter-spacing:.6px}
.big{font-size:30px;font-weight:700;line-height:1.1}.sub{color:var(--mu);font-size:12px;margin-top:3px}
.tile.must{border-color:#3a4658}.tag{float:right;font-size:10px;color:var(--mu);border:1px solid var(--line);border-radius:4px;padding:0 5px}
.c2{grid-template-columns:repeat(auto-fit,minmax(480px,1fr))}.c3{grid-template-columns:repeat(auto-fit,minmax(340px,1fr))}
canvas{width:100%;height:230px;display:block}canvas.s{height:190px}
.sec{margin:20px 0 8px;font-size:13px;color:var(--mu);text-transform:uppercase;letter-spacing:1px;font-weight:600}
table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:5px 8px;text-align:right;border-bottom:1px solid var(--line)}th{color:var(--mu);font-weight:600}td:first-child,th:first-child{text-align:left}
.bar{height:10px;background:#222a35;border-radius:5px;overflow:hidden;margin:4px 0 10px}.bar>i{display:block;height:100%}
.row{display:flex;justify-content:space-between;gap:8px}.lg{display:flex;gap:12px;flex-wrap:wrap;font-size:12px;color:var(--mu);margin-top:6px}
.lg b{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:-1px}
input[type=range]{width:100%}input[type=number]{width:62px;background:#0d1117;color:var(--tx);border:1px solid var(--line);border-radius:4px;padding:2px 4px}
.mono{font-family:Consolas,monospace}.empty{color:var(--mu);padding:10px 0}
</style></head><body>
<header>
 <h1>Onion Sorting Line &middot; Live Dashboard</h1>
 <span id="state" class="pill off">CONNECTING</span><span id="plcsrc" class="pill off" title="where the control logic runs">PLC</span><span class="mono" id="clock" style="color:var(--mu)"></span>
 <span class="sp"></span>
 <button class="btn go" onclick="cmd('start')">Start</button><button class="btn" onclick="cmd('stop')">Stop</button>
 <button class="btn no" onclick="cmd('estop')">E-STOP</button><button class="btn" onclick="cmd('release')">Release + reset faults</button>
 <button class="btn" onclick="if(confirm('Reset all counters and statistics?'))cmd('reset_counters')">Reset counters</button>
</header>
<main>
<div class="sec" style="margin-top:0">Required KPIs</div>
<div class="grid tiles">
 <div class="card tile must"><h3>Classification accuracy <span class="tag">KPI</span></h3><div class="big" id="k_acc">-</div><div class="sub" id="k_acc_s"></div></div>
 <div class="card tile must"><h3>Processing rate <span class="tag">KPI</span></h3><div class="big" id="k_rate">-</div><div class="sub" id="k_rate_s"></div></div>
 <div class="card tile must"><h3>Grade A <span class="tag">KPI</span></h3><div class="big" id="k_A" style="color:var(--A)">-</div><div class="sub" id="k_A_s">70 - &lt;90 mm</div></div>
 <div class="card tile must"><h3>Grade B <span class="tag">KPI</span></h3><div class="big" id="k_B" style="color:var(--B)">-</div><div class="sub" id="k_B_s">55 - &lt;70 mm</div></div>
 <div class="card tile must"><h3>Grade C <span class="tag">KPI</span></h3><div class="big" id="k_C" style="color:var(--C)">-</div><div class="sub" id="k_C_s">40 - &lt;55 mm</div></div>
 <div class="card tile must"><h3>Rejected <span class="tag">KPI</span></h3><div class="big" id="k_R" style="color:var(--R)">-</div><div class="sub" id="k_R_s"></div></div>
 <div class="card tile must"><h3>Total processed <span class="tag">KPI</span></h3><div class="big" id="k_T">-</div><div class="sub" id="k_T_s"></div></div>
</div>

<div class="sec">Performance and economics</div>
<div class="grid tiles">
 <div class="card"><h3>OEE</h3><div class="big" id="o_oee">-</div><div class="sub">availability &times; performance &times; quality</div></div>
 <div class="card"><h3>Availability</h3><div class="big" id="o_av">-</div><div class="sub" id="o_av_s"></div></div>
 <div class="card"><h3>Performance</h3><div class="big" id="o_pf">-</div><div class="sub" id="o_pf_s"></div></div>
 <div class="card"><h3>Quality</h3><div class="big" id="o_q">-</div><div class="sub">correctly sorted / processed</div></div>
 <div class="card"><h3>Sprout detection</h3><div class="big" id="s_rec">-</div><div class="sub" id="s_rec_s"></div></div>
 <div class="card"><h3>Estimated mass sorted</h3><div class="big" id="m_tot">-</div><div class="sub" id="m_tot_s"></div></div>
 <div class="card"><h3>Estimated value</h3><div class="big" id="v_tot">-</div><div class="sub">prices (Rs/kg): A <input type="number" id="pA" value="32"> B <input type="number" id="pB" value="24"> C <input type="number" id="pC" value="15"></div></div>
 <div class="card"><h3>Packing</h3><div class="big" id="p_cr">-</div><div class="sub" id="p_cr_s"></div></div>
</div>

<div class="sec">Production trends</div>
<div class="grid c2">
 <div class="card"><h3>Processing rate (onions/min)</h3><canvas id="ch_rate"></canvas><div class="lg"><span><b style="background:var(--P)"></b>last 10 s window</span><span><b style="background:var(--Y)"></b>last 60 s window</span></div></div>
 <div class="card"><h3>Cumulative output by grade</h3><canvas id="ch_cum"></canvas><div class="lg"><span><b style="background:var(--A)"></b>A</span><span><b style="background:var(--B)"></b>B</span><span><b style="background:var(--C)"></b>C</span><span><b style="background:var(--R)"></b>Reject</span></div></div>
 <div class="card"><h3>Output per minute by grade</h3><canvas id="ch_pm"></canvas><div class="lg"><span><b style="background:var(--A)"></b>A</span><span><b style="background:var(--B)"></b>B</span><span><b style="background:var(--C)"></b>C</span><span><b style="background:var(--R)"></b>Reject</span></div></div>
 <div class="card"><h3>Classification accuracy over time (%)</h3><canvas id="ch_acc"></canvas><div class="lg"><span><b style="background:var(--A)"></b>PLC accuracy KPI</span><span><b style="background:var(--B)"></b>physical truth</span><span><b style="background:var(--Y)"></b>availability</span></div></div>
 <div class="card"><h3>Reject rate and good-output share over time (%)</h3><canvas id="ch_rej"></canvas><div class="lg"><span><b style="background:var(--R)"></b>rejected share</span><span><b style="background:var(--P)"></b>sorted into A/B/C</span></div></div>
</div>

<div class="sec">Quality and classification</div>
<div class="grid c3">
 <div class="card"><h3>Grade share</h3><canvas class="s" id="ch_donut"></canvas></div>
 <div class="card" style="grid-column:span 2"><h3>Onion size distribution (mm) by true grade</h3><canvas class="s" id="ch_hist"></canvas><div class="lg"><span><b style="background:var(--A)"></b>A</span><span><b style="background:var(--B)"></b>B</span><span><b style="background:var(--C)"></b>C</span><span><b style="background:var(--R)"></b>reject class</span><span>dashed lines = grade limits 40 / 55 / 70 / 90 mm</span></div></div>
 <div class="card"><h3>Reject and error reasons</h3><canvas class="s" id="ch_reasons"></canvas></div>
 <div class="card"><h3>Confusion matrix (true grade &rarr; assigned)</h3><div id="conf"></div><div class="sub">Rows = what the onion really was, columns = where the line sent it. Diagonal = correct.</div></div>
 <div class="card"><h3>Size statistics per output (mm)</h3><div id="sizetab"></div></div>
</div>

<div class="sec">Line status</div>
<div class="grid c3">
 <div class="card"><h3>Crates filling</h3><div id="crates"></div></div>
 <div class="card"><h3>Full crates waiting for the robot</h3><canvas class="s" id="ch_lane"></canvas><div class="lg"><span><b style="background:var(--A)"></b>A</span><span><b style="background:var(--B)"></b>B</span><span><b style="background:var(--C)"></b>C</span></div></div>
 <div class="card"><h3>Time breakdown and estimates</h3><div id="time"></div></div>
 <div class="card"><h3>Estimated mass by grade (kg)</h3><canvas class="s" id="ch_mass"></canvas></div>
 <div class="card"><h3>Estimated value over time (Rs)</h3><canvas class="s" id="ch_val"></canvas></div>
 <div class="card"><h3>Faults</h3><div id="faultnow"></div><table id="flog"></table></div>
</div>

<div class="sec">Test controls (simulation only)</div>
<div class="grid c3">
 <div class="card"><h3>Feed interval: <span id="fv"></span> s per onion</h3><input type="range" id="feed" min="1.5" max="6" step="0.1" onchange="cmd('feed',this.value)"></div>
 <div class="card"><h3>Sprouted share of incoming onions: <span id="sv"></span> %</h3><input type="range" id="spr" min="0" max="40" step="1" onchange="cmd('sprout',this.value/100)"></div>
 <div class="card"><h3>Fault injection</h3><button class="btn no" onclick="cmd('jam',true)">Jam the size gauge</button> <button class="btn" onclick="cmd('jam',false)">Clear jam</button> <button class="btn" onclick="cmd('clear_belt')">Clear belt (operator)</button> <div class="sub">recovery: Stop, Clear belt, Release + reset faults, Start</div></div>
</div>
</main>
<script>
const COL={A:'#3fb950',B:'#58a6ff',C:'#f0883e',R:'#f85149',Y:'#d29922',P:'#bc8cff',mu:'#8b949e',grid:'#2a313c'};
const $=id=>document.getElementById(id);
function cmd(c,v){fetch('/cmd',{method:'POST',body:JSON.stringify({cmd:c,value:v})}).catch(()=>{})}
function prep(id){const c=$(id),r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;c.width=Math.max(10,r.width*d);c.height=Math.max(10,r.height*d);const x=c.getContext('2d');x.scale(d,d);x.clearRect(0,0,r.width,r.height);x.font='11px Segoe UI,Arial';return{x,w:r.width,h:r.height}}
function nice(max){if(max<=0)return 1;const p=Math.pow(10,Math.floor(Math.log10(max))),f=max/p;return(f<=1?1:f<=2?2:f<=5?5:10)*p}
function frame(c,xmin,xmax,ymin,ymax,xl,unit){const L=40,R=10,T=8,B=22,W=c.w-L-R,H=c.h-T-B,x=c.x;
 const X=v=>L+(v-xmin)/((xmax-xmin)||1)*W,Y=v=>T+H-(v-ymin)/((ymax-ymin)||1)*H;
 x.strokeStyle=COL.grid;x.fillStyle=COL.mu;x.lineWidth=1;x.textAlign='right';
 for(let i=0;i<=4;i++){const v=ymin+(ymax-ymin)*i/4,y=Y(v);x.beginPath();x.moveTo(L,y);x.lineTo(L+W,y);x.stroke();x.fillText((+v.toFixed(1))+(unit||''),L-4,y+3)}
 x.textAlign='center';if(xl!==false)for(let i=0;i<=5;i++){const v=xmin+(xmax-xmin)*i/5;x.fillText(xl?xl(v):Math.round(v),X(v),c.h-7)}
 return{X,Y,L,T,W,H}}
const tfmt=v=>{v=Math.round(v);return Math.floor(v/60)+':'+String(v%60).padStart(2,'0')};
function lines(id,xs,ss,o){o=o||{};const c=prep(id);if(xs.length<2){c.x.fillStyle=COL.mu;c.x.fillText('collecting data...',20,30);return}
 let ymax=o.ymax!=null?o.ymax:nice(Math.max(1,...ss.flatMap(s=>s.d))*1.05),ymin=o.ymin||0;
 const f=frame(c,xs[0],xs[xs.length-1],ymin,ymax,tfmt,o.unit);
 ss.forEach(s=>{c.x.strokeStyle=s.c;c.x.lineWidth=2;c.x.beginPath();s.d.forEach((v,i)=>{const px=f.X(xs[i]),py=f.Y(v);i?c.x.lineTo(px,py):c.x.moveTo(px,py)});c.x.stroke()})}
function stacked(id,xs,ss){const c=prep(id);if(xs.length<2){c.x.fillStyle=COL.mu;c.x.fillText('collecting data...',20,30);return}
 const tot=xs.map((_,i)=>ss.reduce((a,s)=>a+s.d[i],0)),f=frame(c,xs[0],xs[xs.length-1],0,nice(Math.max(1,...tot)*1.05),tfmt);
 let base=xs.map(()=>0);ss.forEach(s=>{const top=base.map((b,i)=>b+s.d[i]);c.x.fillStyle=s.c+'cc';c.x.beginPath();
  top.forEach((v,i)=>{const px=f.X(xs[i]),py=f.Y(v);i?c.x.lineTo(px,py):c.x.moveTo(px,py)});
  for(let i=xs.length-1;i>=0;i--)c.x.lineTo(f.X(xs[i]),f.Y(base[i]));c.x.closePath();c.x.fill();base=top})}
function bars(id,labels,groups,o){o=o||{};const c=prep(id),n=labels.length;if(!n){c.x.fillStyle=COL.mu;c.x.fillText('collecting data...',20,30);return}
 const stack=o.stack,vals=labels.map((_,i)=>stack?groups.reduce((a,g)=>a+g.d[i],0):Math.max(...groups.map(g=>g.d[i])));
 const f=frame(c,0,n,0,nice(Math.max(1,...vals)*1.1),false,o.unit);c.x.textAlign='center';
 const bw=f.W/n;labels.forEach((l,i)=>{const gx=f.L+i*bw;let acc=0;
  groups.forEach((g,j)=>{const v=g.d[i];c.x.fillStyle=g.c;
   if(stack){const y1=f.Y(acc+v),y0=f.Y(acc);c.x.fillRect(gx+bw*.15,y1,bw*.7,y0-y1);acc+=v}
   else{const w=bw*.7/groups.length,y=f.Y(v);c.x.fillRect(gx+bw*.15+j*w,y,w-1,f.T+f.H-y)}});
  c.x.fillStyle=COL.mu;if(n<=14||i%Math.ceil(n/12)==0)c.x.fillText(l,gx+bw/2,c.h-7)})}
function donut(id,vals,cols,labels){const c=prep(id),tot=vals.reduce((a,b)=>a+b,0),cx=c.w*.33,cy=c.h/2,r=Math.min(c.w*.3,c.h/2-6);
 if(!tot){c.x.fillStyle=COL.mu;c.x.fillText('no data yet',20,30);return}
 let a=-Math.PI/2;vals.forEach((v,i)=>{const b=a+v/tot*2*Math.PI;c.x.beginPath();c.x.moveTo(cx,cy);c.x.arc(cx,cy,r,a,b);c.x.closePath();c.x.fillStyle=cols[i];c.x.fill();a=b});
 c.x.beginPath();c.x.arc(cx,cy,r*.58,0,7);c.x.fillStyle='#161b22';c.x.fill();c.x.fillStyle='#e6edf3';c.x.textAlign='center';c.x.font='bold 18px Segoe UI';c.x.fillText(tot,cx,cy+6);c.x.font='11px Segoe UI';
 c.x.textAlign='left';vals.forEach((v,i)=>{const y=c.h/2-vals.length*11+i*22+8;c.x.fillStyle=cols[i];c.x.fillRect(c.w*.62,y-9,10,10);c.x.fillStyle='#e6edf3';c.x.fillText(labels[i]+'  '+v+' ('+(100*v/tot).toFixed(1)+'%)',c.w*.62+16,y)})}
function hist(id,h){const c=prep(id);if(!h.length){c.x.fillStyle=COL.mu;c.x.fillText('collecting data...',20,30);return}
 const lo=Math.floor(Math.min(25,h[0][0])/5)*5,hi=Math.max(110,h[h.length-1][0]+5),m={};h.forEach(r=>m[r[0]]=r.slice(1));
 const mx=nice(Math.max(1,...h.map(r=>r[1]+r[2]+r[3]+r[4]))*1.1),f=frame(c,lo,hi,0,mx);const bw=f.W/((hi-lo)/5);
 for(let k=lo;k<hi;k+=5){const v=m[k];if(!v)continue;let acc=0;[COL.A,COL.B,COL.C,COL.R].forEach((col,j)=>{if(!v[j])return;const y1=f.Y(acc+v[j]),y0=f.Y(acc);c.x.fillStyle=col;c.x.fillRect(f.X(k)+1,y1,bw-2,y0-y1);acc+=v[j]})}
 c.x.setLineDash([4,4]);c.x.strokeStyle=COL.mu;[40,55,70,90].forEach(t=>{c.x.beginPath();c.x.moveTo(f.X(t),f.T);c.x.lineTo(f.X(t),f.T+f.H);c.x.stroke()});c.x.setLineDash([])}
function hbar(id,items){const c=prep(id),mx=Math.max(1,...items.map(i=>i[1])),lw=104;c.x.textAlign='left';
 items.forEach((it,i)=>{const y=10+i*(c.h-14)/items.length,h=(c.h-14)/items.length-8;c.x.fillStyle=COL.mu;c.x.fillText(it[0],0,y+h/2+4);
  c.x.fillStyle=it[2];c.x.fillRect(lw,y,(c.w-lw-34)*it[1]/mx,h);c.x.fillStyle='#e6edf3';c.x.fillText(it[1],lw+(c.w-lw-34)*it[1]/mx+5,y+h/2+4)})}
const fmt=(v,d)=>v==null?'-':(+v).toFixed(d==null?0:d);
let last=null;
function render(s){last=s;const k=s.kpi,st=s.stats,L=s.live,o=s.oee,se=s.series,n=k.total,pr=['pA','pB','pC'].map(i=>+$(i).value||0);
 const sp=$('state'),stt=L.estop?['E-STOP','bad']:L.fault?['FAULT','bad']:L.running?['RUNNING','ok']:['STOPPED','off'];sp.textContent=stt[0];sp.className='pill '+stt[1];
 $('clock').textContent='sim time '+tfmt(s.t);
 const P=L.plc||{};$('plcsrc').textContent=(P.mode||'PLC')+(P.ext?(P.link?' | link OK':' | LINK LOST')+(P.ms?' | '+P.ms+' ms':''):'');$('plcsrc').className='pill '+(P.ext&&!P.link?'bad':'ok');
 $('k_acc').textContent=k.tested?k.accuracy+' %':'-';$('k_acc_s').textContent='PLC KPI over '+k.tested+' items | physical check '+st.acc_phys+' %';
 $('k_rate').textContent=k.rate10+' /min';$('k_rate_s').textContent='60 s window: '+k.rate60+' /min | avg since start: '+fmt(o.actual,1)+' /min';
 ['A','B','C'].forEach(g=>{$('k_'+g).textContent=k[g];$('k_'+g+'_s').textContent=(n?(100*k[g]/n).toFixed(1):0)+' % of total | '+fmt(st.mass[g],1)+' kg'});
 $('k_R').textContent=k.reject;$('k_R_s').textContent=(n?(100*k.reject/n).toFixed(1):0)+' % of total';$('k_T').textContent=n;$('k_T_s').textContent=s.sim_total+' fed | '+(s.sim_total-n)+' still on the line';
 $('o_oee').textContent=fmt(o.oee,1)+' %';$('o_av').textContent=fmt(o.avail,1)+' %';$('o_av_s').textContent='running '+tfmt(o.run_s)+', fault '+tfmt(o.fault_s)+', idle '+tfmt(o.idle_s);
 $('o_pf').textContent=fmt(o.perf,1)+' %';$('o_pf_s').textContent=fmt(o.actual,1)+' of ideal '+fmt(o.ideal,1)+' /min';$('o_q').textContent=fmt(o.qual,1)+' %';
 const sr=st.sprout;$('s_rec').textContent=sr.recall==null?'-':sr.recall+' %';$('s_rec_s').textContent='recall | precision '+fmt(sr.precision,1)+' % | false alarm '+fmt(sr.false_alarm,1)+' %';
 const mt=st.mass.A+st.mass.B+st.mass.C+st.mass.R;$('m_tot').textContent=fmt(mt,1)+' kg';$('m_tot_s').textContent='packed grades '+fmt(st.mass.A+st.mass.B+st.mass.C,1)+' kg, reject '+fmt(st.mass.R,1)+' kg';
 const val=st.mass.A*pr[0]+st.mass.B*pr[1]+st.mass.C*pr[2];$('v_tot').textContent='Rs '+fmt(val,0);
 const cr=k.crates;$('p_cr').textContent=(cr[0]+cr[1]+cr[2])+' crates';$('p_cr_s').textContent='A '+cr[0]+' | B '+cr[1]+' | C '+cr[2]+' | pallets '+k.pallets+' | on pallet '+L.pallet_crates+'/12';
 const xs=se.map(r=>r.t);
 lines('ch_rate',xs,[{c:COL.P,d:se.map(r=>r.r10)},{c:COL.Y,d:se.map(r=>r.r60)}]);
 stacked('ch_cum',xs,[{c:COL.A,d:se.map(r=>r.A)},{c:COL.B,d:se.map(r=>r.B)},{c:COL.C,d:se.map(r=>r.C)},{c:COL.R,d:se.map(r=>r.R)}]);
 const lab=[],gA=[],gB=[],gC=[],gR=[];for(let m=Math.max(0,Math.floor(s.t/60)-9);m<=Math.floor(s.t/60);m++){const at=t=>{let r=null;for(const q of se){if(q.t<=t)r=q;else break}return r||{A:0,B:0,C:0,R:0}},a=at(m*60),b=at(Math.min((m+1)*60,s.t));
  lab.push('m'+(m+1));gA.push(b.A-a.A);gB.push(b.B-a.B);gC.push(b.C-a.C);gR.push(b.R-a.R)}
 bars('ch_pm',lab,[{c:COL.A,d:gA},{c:COL.B,d:gB},{c:COL.C,d:gC},{c:COL.R,d:gR}],{stack:true});
 const sa=se.filter(r=>r.total>2),xa=sa.map(r=>r.t);
 lines('ch_acc',xa,[{c:COL.A,d:sa.map(r=>r.acc)},{c:COL.B,d:sa.map(r=>r.accp)},{c:COL.Y,d:sa.map(r=>r.avail)}],{ymax:100,ymin:50,unit:'%'});
 lines('ch_rej',xa,[{c:COL.R,d:sa.map(r=>100*r.R/Math.max(1,r.total))},{c:COL.P,d:sa.map(r=>100*(r.A+r.B+r.C)/Math.max(1,r.total))}],{ymax:100,unit:'%'});
 donut('ch_donut',[k.A,k.B,k.C,k.reject],[COL.A,COL.B,COL.C,COL.R],['A','B','C','Reject']);
 hist('ch_hist',st.hist);
 const r=st.reasons;hbar('ch_reasons',[['Sprouted (caught)',r.sprouted,COL.R],['Oversize >= 90',r.oversize,COL.R],['Undersize < 40',r.undersize,COL.R],['Good onion rejected',r.false_reject,COL.Y],['Sprout missed',r.sprout_missed,COL.C],['Wrong size grade',r.mis_sized,COL.C]]);
 let t='<table><tr><th>true \\ sent to</th><th>A</th><th>B</th><th>C</th><th>Rej</th></tr>';const mxc=Math.max(1,...st.conf.flat());
 ['A','B','C','Reject class'].forEach((nm,i)=>{t+='<tr><td>'+nm+'</td>'+st.conf[i].map((v,j)=>'<td style="background:rgba('+(i==j?'63,185,80':'248,81,73')+','+(v?0.12+0.55*v/mxc:0)+')">'+v+'</td>').join('')+'</tr>'});$('conf').innerHTML=t+'</table>';
 t='<table><tr><th></th><th>mean</th><th>std</th><th>min</th><th>max</th></tr>';[['A','A'],['B','B'],['C','C'],['R','Reject']].forEach(a=>{const v=st.size[a[0]];t+='<tr><td style="color:'+COL[a[0]]+'">'+a[1]+'</td><td>'+v[0]+'</td><td>'+v[1]+'</td><td>'+v[2]+'</td><td>'+v[3]+'</td></tr>'});$('sizetab').innerHTML=t+'</table>';
 const pcol=[COL.A,COL.B,COL.C];t='';['A','B','C'].forEach((g,i)=>{const f=L.fill[i],pc=Math.min(100,100*f/L.cap),rate=k[g]&&s.t>30?k[g]/(s.t/60):0,eta=rate>0?Math.max(0,(L.cap-f)/rate*60):null;
 t+='<div class="row"><b style="color:'+pcol[i]+'">Crate '+g+'</b><span>'+f+' / '+L.cap+(L.pack_ready[i]?' &middot; READY for robot':'')+'</span></div><div class="bar"><i style="width:'+pc+'%;background:'+pcol[i]+'"></i></div><div class="sub" style="margin-top:-6px;margin-bottom:8px">next crate in about '+(eta==null?'-':tfmt(eta))+' | packed '+cr[i]+'</div>'});
 const pp=L.pallet_crates/12*100;t+='<div class="row"><b>Pallet</b><span>'+L.pallet_crates+' / 12 crates, '+L.onions_on_pallet+' onions stacked in total</span></div><div class="bar"><i style="width:'+pp+'%;background:'+COL.P+'"></i></div>';$('crates').innerHTML=t;
 lines('ch_lane',xs,[{c:COL.A,d:se.map(r=>r.la)},{c:COL.B,d:se.map(r=>r.lb)},{c:COL.C,d:se.map(r=>r.lc)}],{ymax:9});
 const tt=Math.max(1,o.run_s+o.fault_s+o.idle_s);$('time').innerHTML='<div class="row"><span>Running</span><b>'+fmt(100*o.run_s/tt,1)+' %</b></div><div class="bar"><i style="width:'+100*o.run_s/tt+'%;background:'+COL.A+'"></i></div><div class="row"><span>Fault</span><b>'+fmt(100*o.fault_s/tt,1)+' %</b></div><div class="bar"><i style="width:'+100*o.fault_s/tt+'%;background:'+COL.R+'"></i></div><div class="row"><span>Idle / stopped</span><b>'+fmt(100*o.idle_s/tt,1)+' %</b></div><div class="bar"><i style="width:'+100*o.idle_s/tt+'%;background:#555"></i></div><table><tr><td>Robot palletizing cycles</td><td>'+L.robot.cycles+'</td></tr><tr><td>Reject bin content</td><td>'+L.reject_bin+'</td></tr><tr><td>Projected output / hour</td><td>'+fmt(o.actual*60,0)+' onions</td></tr><tr><td>Projected packed mass / hour</td><td>'+fmt(o.actual*60*(mt/Math.max(1,n)),1)+' kg</td></tr></table>';
 bars('ch_mass',['A','B','C','Reject'],[{c:COL.A,d:[st.mass.A,0,0,0]},{c:COL.B,d:[0,st.mass.B,0,0]},{c:COL.C,d:[0,0,st.mass.C,0]},{c:COL.R,d:[0,0,0,st.mass.R]}],{stack:true});
 lines('ch_val',xs,[{c:COL.Y,d:se.map(r=>r.mA*pr[0]+r.mB*pr[1]+r.mC*pr[2])}]);
 $('faultnow').innerHTML=L.faults.length?'<div class="pill bad" style="display:inline-block;margin-bottom:8px">'+L.faults.length+' ACTIVE</div>':'<div class="pill ok" style="display:inline-block;margin-bottom:8px">NO ACTIVE FAULT</div>';
 $('flog').innerHTML=s.faults_log.length?'<tr><th>t</th><th>event</th><th></th></tr>'+s.faults_log.map(f=>'<tr><td>'+tfmt(f.t)+'</td><td style="text-align:left">'+f.text+'</td><td style="color:'+(f.state=='RAISED'?COL.R:COL.A)+'">'+f.state+'</td></tr>').join(''):'<tr><td class="empty" colspan=3>no fault events</td></tr>';
 if(document.activeElement!==$('feed')){$('feed').value=L.feed_s;$('fv').textContent=L.feed_s}if(document.activeElement!==$('spr')){$('spr').value=Math.round(L.p_sprout*100);$('sv').textContent=Math.round(L.p_sprout*100)}}
async function tick(){try{const r=await fetch('/api');const s=await r.json();if(s&&s.kpi)render(s)}catch(e){$('state').textContent='NO CONNECTION';$('state').className='pill off'}}
setInterval(tick,1000);tick();window.addEventListener('resize',()=>last&&render(last));
</script></body></html>
'''



# =====================================================================================================
#  RUNNER
# =====================================================================================================
import argparse
import json
import os
import sys
import time


def get_sim(host, port):
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient(host, port)
    sim = client.require('sim')
    return client, sim


def kbd_poll():
    """Non-blocking single-key read (Windows/Linux/macOS terminals); returns '' when nothing was pressed."""
    try:
        import msvcrt
        return msvcrt.getwch() if msvcrt.kbhit() else ''
    except ImportError:
        pass
    try:
        import select
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            r, _, _ = select.select([sys.stdin], [], [], 0)
            return sys.stdin.read(1) if r else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception:
        return ''


def clear_scene(sim):
    """Remove everything left over from earlier runs, including loaded robot models (cameras and lights are kept).

    sim.removeObject deletes only the one object it is given (the children of a model base stay behind as a 'ghost' robot),
    so every object in the scene is collected first and removed one by one."""
    try:
        keep = {getattr(sim, n) for n in ('object_camera_type', 'object_light_type') if hasattr(sim, n)}
        objs = [h for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0) if sim.getObjectType(h) not in keep]
        n = 0
        for h in objs:
            try:
                sim.removeObject(h)
                n += 1
            except Exception:
                pass
        if objs:
            print(f'  removed {n} old objects from the scene')
    except Exception as e:
        print('  (could not clean the scene:', e, ')')


def split_st(st):
    """Split OnionSorting.st into the program code (after the last VAR block) and a CSV of every variable."""
    import re
    head, rest = st.split('\nPROGRAM OnionSorting\n', 1)
    decl, body = rest.rsplit('END_VAR\n', 1)
    body = body.split('END_PROGRAM')[0].strip() + '\n'
    rows = ['name,type,initial_value,opc_ua,owner,description']
    tagset = set(BY_NAME)
    for line in decl.splitlines():
        mm = re.match(r'\s+(\w+)\s*:\s*([^:;]+?)\s*(?::=\s*([^;]+))?;\s*(?:\(\*\s*(.*?)\s*\*\))?\s*$', line)
        if mm:
            n, typ, ini, note = mm.groups()
            owner = BY_NAME[n].owner if n in tagset else 'internal'
            rows.append(f'{n},"{typ}",{ini or ""},{"Y" if n in tagset else ""},{owner},"{(note or "").split(": ", 1)[-1] if n in tagset else ""}"')
    return body, '\n'.join(rows) + '\n'


def fmt_kpi(m):
    k = m.kpis()
    return (f"t={m.t:6.1f}s  A={k['A']:3d} B={k['B']:3d} C={k['C']:3d} Rej={k['reject']:3d} total={k['total']:3d} | "
            f"rate10={k['rate10']:3d}/min rate60={k['rate60']:3d}/min | accuracy={k['accuracy']}% (n={k['tested']}) | "
            f"crates A/B/C={k['crates']} pallets={k['pallets']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description='Onion sorting line: PLC logic + CoppeliaSim 3D scene')
    ap.add_argument('--offline', action='store_true', help='run the model without CoppeliaSim (prints KPIs only)')
    ap.add_argument('--duration', type=float, default=0, help='simulated seconds to run (0 = until Ctrl+C / q)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--feed-interval', type=float, default=2.0, help='seconds between onions leaving the hopper')
    ap.add_argument('--crate-size', type=int, default=6, help='onions per crate (2 to 6)')
    ap.add_argument('--p-sprout', type=float, default=0.12, help='fraction of sprouted onions')
    ap.add_argument('--sprout-detect', type=float, default=0.96, help='camera detection probability (1 = perfect)')
    ap.add_argument('--onions', type=int, default=None, help='stop feeding after N onions')
    ap.add_argument('--no-autostart', action='store_true', help='wait for the operator Start (press s)')
    ap.add_argument('--realtime', action='store_true', help='pace the loop to wall-clock (offline mode)')
    ap.add_argument('--save-scene', metavar='FILE.ttt', help='save the built scene to a .ttt file')
    ap.add_argument('--build-only', action='store_true', help='build the scene, save it and exit')
    ap.add_argument('--dashboard-port', type=int, default=8765, help='port of the local web dashboard')
    ap.add_argument('--no-dashboard', action='store_true')
    ap.add_argument('--no-hmi', action='store_true', help='do not open the local operator panel window')
    ap.add_argument('--no-browser', action='store_true', help='do not open the dashboard in the browser automatically')
    ap.add_argument('--speed', type=float, default=1.0, help='offline mode with dashboard: simulated seconds per real second')
    ap.add_argument('--ladder', choices=('simple', 'full'), default='simple', help='simple = about 50 rungs (default), full = 126-rung version')
    ap.add_argument('--no-models', action='store_true', help='do not load robots from the CoppeliaSim library (use simple primitive arms)')
    ap.add_argument('--coppelia-dir', metavar='DIR', help='CoppeliaSim install folder (only needed if the model library is not found automatically)')
    ap.add_argument('--palletizer-model', metavar='FILE.ttm', help='robot model file for the palletizing robot (default: UR10 from the library)')
    ap.add_argument('--check', action='store_true', help='print what the connected CoppeliaSim offers (version, model library, robots) and exit')
    ap.add_argument('--robot-test', action='store_true', help='build the scene, move both robots through every working point, report the accuracy and exit')
    ap.add_argument('--keep-scene', action='store_true', help='do not delete old objects from the open scene before building')
    ap.add_argument('--plc', choices=('builtin', 'demo', 'opcua'), default='builtin',
                    help='builtin = ladder inside this program (default); opcua = OpenPLC over OPC UA; demo = same, with a local stand-in PLC server')
    ap.add_argument('--opc-endpoint', default=None, help='OPC UA address of the OpenPLC server (default opc.tcp://127.0.0.1:4840/openplc/opcua)')
    ap.add_argument('--opc-user', default=os.environ.get('ONION_OPC_USER'), help='OPC UA user name (or env ONION_OPC_USER)')
    ap.add_argument('--opc-password', default=os.environ.get('ONION_OPC_PASSWORD'), help='OPC UA password (or env ONION_OPC_PASSWORD)')
    ap.add_argument('--opc-security', default=None, help='e.g. Basic256Sha256,SignAndEncrypt,client_cert.der,client_key.pem')
    ap.add_argument('--opc-map', metavar='FILE.json', help='optional {"TAG": "ns=2;s=NodeId"} map if the tags cannot be found by name')
    ap.add_argument('--opc-check', action='store_true', help='connect to the OPC UA server, list which tags are found/missing, show the PLC values and exit')
    ap.add_argument('--serve-demo', action='store_true', help='run only the stand-in PLC OPC UA server (to practise --opc-check)')
    ap.add_argument('--write-plc', nargs='?', const='.', metavar='DIR', help='write the PLC program (OnionSorting.st) and the tag list (io_map.csv) to DIR and exit')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=23000)
    a = ap.parse_args(argv)

    if a.write_plc is not None:
        os.makedirs(a.write_plc, exist_ok=True)
        open(os.path.join(a.write_plc, 'OnionSorting.st'), 'w').write(ST_SOURCE)
        open(os.path.join(a.write_plc, 'io_map.csv'), 'w').write(csv_text())
        body, variables = split_st(ST_SOURCE)
        open(os.path.join(a.write_plc, 'OnionSorting_body.st'), 'w').write(body)
        open(os.path.join(a.write_plc, 'variables.csv'), 'w').write(variables)
        print('written to', os.path.abspath(a.write_plc) + ':')
        print('  OnionSorting.st        complete program (also compiles on its own)')
        print('  OnionSorting_body.st   only the program code: paste it into the OpenPLC Editor code pane')
        print('  variables.csv          every variable to declare in the Editor (opc_ua = Y: expose it in the OPC UA server)')
        print('  io_map.csv             the 42 OPC UA tags with type, owner and meaning')
        return
    opc_map = json.load(open(a.opc_map)) if a.opc_map else None
    if a.serve_demo:
        ep = a.opc_endpoint or 'opc.tcp://127.0.0.1:4841/onion/'
        print('Stand-in PLC (Python copy of OnionSorting.st) serving', ep, ' - Ctrl+C to stop')
        try:
            serve_demo(ep)
        except KeyboardInterrupt:
            pass
        return
    if a.opc_check:
        ep = a.opc_endpoint or DEFAULT_OPC_ENDPOINT
        sys.exit(check_server(ep, a.opc_user, a.opc_password, a.opc_security, opc_map))
    ext = None
    if a.plc != 'builtin':
        ext = connect_external(a, opc_map)
    src = OnionSource(a.seed, p_sprout=a.p_sprout, limit=a.onions)
    m = LineModel(a.seed, src, p_detect=a.sprout_detect, p_false=0.01, feed_interval_s=a.feed_interval,
                  auto_start=not a.no_autostart, crate_capacity=a.crate_size, ladder=a.ladder, plc=ext)
    m.plc.bits['M302'] = True      # accuracy KPI on: the model supplies the true grade as the 'audit' reference (D302)
    if ext:
        print('Control: external PLC (' + m.plc_status()['mode'] + ') - the plant runs in real time so the PLC timers stay correct')
    else:
        print(f'Ladder program: {a.ladder} ({len(m.plc.rungs)} rungs)')
    me = sys.modules[__name__]
    tel = dash = None
    if not a.no_dashboard:
        tel = Telemetry(m)
        try:
            dash = Dashboard(tel, a.dashboard_port, DASHBOARD_HTML)
            print(f'Dashboard: http://127.0.0.1:{a.dashboard_port}/')
            if not a.no_browser:
                dash.open_browser()
        except OSError as e:
            print('  could not start the dashboard (port busy?):', e)
            tel = None

    hmi = None
    if dash and not a.no_hmi:
        hmi = make_hmi(dash, m)
        if hmi:
            print('Operator panel (HMI) window opened.')

    def tick_model():
        if dash:
            dash.apply_commands(m)
        m.step()
        if tel:
            tel.on_step()

    if a.offline:
        t_end = a.duration or (1e12 if tel else 120)
        nxt, w0 = 0, time.time()
        paced = a.realtime or (tel is not None) or m.external
        steps = 0
        try:
            while m.t < t_end:
                if m.external:                     # the PLC works in wall-clock time: the plant must follow the wall clock
                    n_due = max(1, min(100, int((time.time() - w0) / DT) - steps))
                else:
                    n_due = 10
                for _ in range(n_due):
                    tick_model()
                steps += n_due
                if hmi:
                    hmi.update(m)
                if m.t >= nxt:
                    print(fmt_kpi(m)); nxt += 5
                if paced:
                    ahead = m.t / (1.0 if m.external else a.speed) - (time.time() - w0)
                    if ahead > 0:
                        time.sleep(ahead)
        except KeyboardInterrupt:
            pass
        print('FINAL', fmt_kpi(m))
        return

    client, sim = get_sim(a.host, a.port)
    api = Api(sim)
    if a.check:
        diagnose(sim, a.coppelia_dir)
        return
    api.prepare()
    if not a.keep_scene:
        clear_scene(sim)
    view = SceneView(api, me, use_models=not a.no_models, models_dir=a.coppelia_dir,
                     palletizer_model=a.palletizer_model)
    view.attach(m)
    print('Building the 3D scene (about 10-30 s with the library robots) ...')
    view.build_static()
    print('  scene built:', len(api.cache) // 2, 'objects')
    if a.robot_test:
        view.robot_test()
        return
    if a.save_scene:
        try:
            sim.saveScene(a.save_scene)
            print('  scene saved to', a.save_scene)
        except Exception as e:
            print('  could not save scene:', e)
    if a.build_only:
        return

    try:
        client.setStepping(True)
    except Exception:
        sim.setStepping(True)
    sim.startSimulation()
    try:
        dt = float(sim.getSimulationTimeStep())
    except Exception:
        dt = 0.05
    n = max(1, int(round(dt / DT)))
    print(f'Simulation running: sim step {dt * 1000:.0f} ms = {n} model steps.  Keys: e=E-stop  r=release  '
          f'x=reset counters  s=start  t=stop  q=quit')
    last_print, tgt = 0.0, a.duration
    w0, steps = time.time(), 0
    try:
        while True:
            if m.external:                          # follow the wall clock (the PLC timers run in real time)
                n_due = int((time.time() - w0) / DT) - steps
                if n_due < 1:
                    time.sleep(max(0.0, (steps + 1) * DT - (time.time() - w0)))
                    n_due = 1
                n_due = min(n_due, 100)
            else:
                n_due = n
            for _ in range(n_due):
                tick_model()
            steps += n_due
            view.sync()
            if hmi:
                hmi.update(m)
            try:
                client.step()
            except Exception:
                sim.step()
            if m.t - last_print >= 2.0:
                print(fmt_kpi(m)); last_print = m.t
            ch = kbd_poll().lower()
            if ch == 'q':
                break
            elif ch == 'e':
                m.set_estop(True); print('E-STOP pressed')
            elif ch == 'r':
                m.set_estop(False); m.press('X3', 0.3); print('E-STOP released, fault reset')
            elif ch == 's':
                m.press('X0', 0.3); print('START')
            elif ch == 't':
                m.press('X1', 0.3); print('STOP')
            elif ch == 'x':
                m.press('M310', 0.3); print('counters reset')
            if tgt and m.t >= tgt:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print('FINAL', fmt_kpi(m))
        for line in view.robot_summary():
            print('  robot', line)
        try:
            sim.stopSimulation()
        except Exception:
            pass


if __name__ == '__main__':
    main()
