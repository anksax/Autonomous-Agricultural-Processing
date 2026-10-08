# Group 12 – Autonomous Agricultural Processing

Group 12's agricultural automation project: onion grading, sprout rejection, conveyor sorting, crate handling, and robotic palletizing. This repository contains the Python simulation, CoppeliaSim scene, OpenPLC Structured Text, project report, and screenshots of the Mitsubishi GX Works3 and RT ToolBox implementations.

## Problem statement

Design a post-harvest processing system that receives agricultural products, classifies them according to size/quality, sorts them and prepares them for packaging.

### Minimum features

- Minimum 3 grades/categories
- Automated feeding
- Classification decision
- Sorting
- Packaging
- Fault/rejection handling
- Quantity monitoring

### Expected flow

Product Input → Classification → Grading → Sorting → Packaging

### Suggested KPIs

- Classification accuracy
- Processing rate
- Grade-wise quantity

## Team — Group 12

Department of Robotics and Automation, Symbiosis Institute of Technology, Pune.

| Team member | PRN |
|---|---|
| Ankur Saxena | 24070127019 |
| Apara Shaligram | 24070127023 |
| Sakshi Goyal | 24070127101 |
| Yajat Alimchandani | 24070127137 |
| Om Sonawane | 25070127510 |
| Raghavendra Pathe | 25070127508 |

Team details are taken from the [project report](docs/report/Group12_Autonomous_Agricultural_Processing_Report_V7.pdf).

## Process

```text
Feeder → Size measurement → Sprout inspection → Grade A / B / C pushers
                                            → Reject bin
Sorted onions → Full crates → Robot pickup → Pallet
```

| Grade | Diameter |
|---|---|
| A | 70 mm to below 90 mm |
| B | 55 mm to below 70 mm |
| C | 40 mm to below 55 mm |
| Reject | Sprouted, below 40 mm, or 90 mm and above |

The Python program includes a built-in PLC interpreter, a local dashboard and operator panel, and optional external PLC control over OPC UA.

## Quick start

Use Python 3.10 or newer as a starting point; the offline check was performed with Python 3.14 on Windows. CoppeliaSim and OpenPLC are optional for the offline mode.

```bash
python src/onion_sorting_line.py --offline --duration 120 --no-dashboard --no-hmi --seed 1
```

For a simulation with the dashboard and operator panel:

```bash
python src/onion_sorting_line.py --offline --duration 120
```

### CoppeliaSim 3D simulation

Install dependencies, start CoppeliaSim, and open a new empty scene. The script constructs its scene and starts simulation. Its documented CoppeliaSim requirement is 4.6 or newer; record your installed version when reproducing a run.

```bash
python -m pip install -r requirements.txt
python src/onion_sorting_line.py --check
python src/onion_sorting_line.py
```

The default builder removes existing objects from the open scene. Use an empty scene, or the `--keep-scene` option if retaining existing objects is necessary. A supplied scene is also available at [simulation/onion_sim.ttt](simulation/onion_sim.ttt); its equivalence to the current script-generated scene has not been verified.

### OpenPLC / OPC UA

```bash
python -m pip install -r requirements-opcua.txt
python src/onion_sorting_line.py --opc-check
python src/onion_sorting_line.py --plc opcua
```

See [setup instructions](docs/setup.md) for variable declarations, tag mapping, credentials, and the stand-in PLC mode.

## Included material

| Location | Contents |
|---|---|
| `src/` | Python line model, PLC interpreter, scene builder, dashboard and HMI |
| `plc/openplc/` | Complete Structured Text, editor body, variable table and OPC UA I/O map |
| `simulation/` | Supplied CoppeliaSim scene |
| `docs/screenshots/` | OpenPLC, GX Works3 ladder rungs and RT ToolBox screenshots |
| `docs/report/` | Group 12 project report PDF |
| [docs/validation.md](docs/validation.md) | Reproduced offline run and verification limits |
| [docs/demos.md](docs/demos.md) | Recording inventory |

## Implementation evidence

![OpenPLC program](docs/screenshots/OpenPLC%20Screenshot.jpeg)

![GX Works3 ladder rungs 1–7](docs/screenshots/GxWorks3Rung1-7.png)

![RT ToolBox robot program](docs/screenshots/RTToolBox%20Photo.png)

The GX Works3 screenshots and RT ToolBox photo document separate implementations. They do not establish that those programs were connected to this Python simulation. The built-in interpreter uses different device addresses from the GX Works3 screenshots. The native GX Works3 project, OpenPLC editor project, and RT ToolBox program/position exports were not supplied.

## Simulation assumptions

Onions, crates, and tooling are moved kinematically, with physics switched off. Sprout detection is probabilistic simulated sensor data, rather than a trained vision model. Accuracy is checked against the simulation's reference grades and is not a measured real-world camera accuracy. The default crate capacity is **6 onions**. Use `--help` for the current options; some introductory comments in the original source describe older defaults.

This is an academic simulation; the repository does not establish a validated industrial control or safety system.

## Credits and reuse

Project credit: Group 12. See the report for the original project context. CoppeliaSim, OpenPLC, Mitsubishi GX Works3 and RT ToolBox are external software tools and are not bundled here.

No open-source license has been selected yet. See [LICENSE-NOTICE.md](LICENSE-NOTICE.md). Third-party software and model assets remain subject to their respective terms.
