# Setup and operating modes

Run commands from the repository root.

## 1. Offline model

```bash
python src/onion_sorting_line.py --offline --duration 120 --no-dashboard --no-hmi --seed 1
```

This mode uses the Python standard library and the embedded PLC interpreter. Omitting `--no-dashboard` opens a local dashboard (default port 8765); the operator window requires tkinter in the Python installation. `--no-hmi` disables that window.

The default ladder is `simple` (53 rungs in the verified run). `--ladder full` selects the older expanded program. These modes should be evaluated separately.

Terminal controls: `s` start, `t` stop, `e` emergency stop, `r` release/reset, `x` reset counters, `q` quit.

## 2. CoppeliaSim

1. Install CoppeliaSim; the source documents 4.6 or newer.
2. Install `requirements.txt`.
3. Start CoppeliaSim and open a new empty scene.
4. Run `python src/onion_sorting_line.py --check` to inspect the model library.
5. Run `python src/onion_sorting_line.py` to build and simulate the cell.

If the robot library cannot be found, pass `--coppelia-dir PATH`. `--no-models` uses a primitive arm. `--robot-test` exercises the working points. These 3D checks were not executed during repository preparation.

To generate a saved scene:

```bash
python src/onion_sorting_line.py --build-only --save-scene simulation/generated-line.ttt
```

## 3. OpenPLC editor

The supplied editor screenshot names its program `main`, whereas the complete Structured Text export names it `OnionSorting`. Keep the actual editor instance name and OPC UA node paths consistent with your server.

1. Create a Structured Text program in OpenPLC Editor.
2. Declare every variable in `plc/openplc/variables.csv`, including local state and timer/function-block instances.
3. Paste `plc/openplc/OnionSorting_body.st` into the program code pane. The full `OnionSorting.st` also contains declarations and the program wrapper; do not paste that wrapper into a body-only pane.
4. Assign the program to a cyclic task. The tested OpenPLC scan interval was not provided; document the interval used for your run.
5. Build and deploy the project to your OpenPLC runtime.
6. Expose the 42 public interface tags listed in `io_map.csv` through OPC UA, with the listed types. The `opc_ua` column in `variables.csv` identifies those variables.
7. Allow the plant to write sensor/session tags and the operator interface to write operator tags. The plant reads PLC outputs and KPIs.
8. Configure your endpoint and verify tag discovery before starting the line.

```bash
python -m pip install -r requirements-opcua.txt
python src/onion_sorting_line.py --opc-check --opc-endpoint opc.tcp://127.0.0.1:4840/openplc/opcua
python src/onion_sorting_line.py --plc opcua --opc-endpoint opc.tcp://127.0.0.1:4840/openplc/opcua
```

The endpoint is the script default, not a guarantee of your runtime configuration. For authenticated servers use environment variables `ONION_OPC_USER` and `ONION_OPC_PASSWORD`. Do not commit credentials. `--opc-security` accepts the security configuration documented in `--help`. If names cannot be discovered, provide `--opc-map FILE.json`, mapping tag names to the actual OPC UA NodeIds. No installation-specific NodeIds are supplied.

## 4. Stand-in OPC UA demo

```bash
python src/onion_sorting_line.py --offline --plc demo --duration 120 --no-hmi
```

This runs the Python stand-in PLC, not OpenPLC. It exercises the OPC UA path when the optional dependency is installed. It has not been verified during preparation of this repository.

## Regenerating PLC exports

```bash
python src/onion_sorting_line.py --write-plc plc/openplc
```

This overwrites the complete ST, body, variable table and I/O map with the exports embedded in the Python source. The supplied ST files were retained; the CSV tables were generated from that source. The CSV files do not replace a native OpenPLC editor project.
