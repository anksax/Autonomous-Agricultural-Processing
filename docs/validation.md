# Verification record

Repository preparation check: 8 October 2026, Windows, Python 3.14. The supplied Python implementation was retained without logic changes.

## Reproduced offline run

```bash
python src/onion_sorting_line.py --offline --duration 120 --no-dashboard --no-hmi --seed 1
```

The process exited successfully using the simple 53-rung ladder. Final output:

```text
FINAL t= 120.0s  A= 23 B= 14 C= 15 Rej= 6 total= 58
rate10= 30/min rate60= 28/min | accuracy=98% (n=58)
crates A/B/C=(3, 2, 1) pallets=0
```

The result is a simulation sample with a fixed random seed. Classification accuracy uses simulated reference grades. It does not validate a physical vision system or demonstrate pallet completion within this 120-second run.

## Export check

The script's `--write-plc` option successfully generated complete ST, body-only ST, the variable table and the 42-tag OPC UA I/O map.

## Remaining verification

- CoppeliaSim scene loading, robot working points and 3D execution.
- OpenPLC compilation, task interval, actual OPC UA node mapping and end-to-end operation.
- GX Works3 and RT ToolBox native projects and their relationship to the supplied simulations.
- Stop, emergency-stop, fault-reset and communication-loss scenarios across external control modes.

The supplied report, screenshots and recordings are original project evidence; they were not independently reproduced during repository preparation.
