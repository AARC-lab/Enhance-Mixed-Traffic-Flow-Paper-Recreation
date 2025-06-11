# Enhanced Mixed Traffic (EMT) Simulation using SUMO and Python

This repository simulates enhanced mixed traffic behavior using SUMO (Simulation of Urban MObility) and Python. It provides flexible control over traffic configurations, supports Minimum Performance Requirements (MPR) based flow regulation, and allows runtime customization for simulation duration and data collection.

---

## Project Structure
```
EMT_project/
├── deployment/
│ └── environment/
│ ├── main_config.sumocfg
│ ├── main_config_without_ramp.sumocfg
│ ├── road.net.xml
│ ├── road_without_ramp.net.xml
│ ├── route.rou.xml
│ ├── route_without_ramp.rou.xml
├── sumo_control/
│ ├── control_handler/
│ │ └── main.py
│ └── control_helper/
│ ├── _config.py
│ ├── _generation_files.py
│ ├── control.py
│ ├── data_coll.py
│ ├── run_for_main.py
├── requirements.txt
├── readme.md
```
---

## Requirements

Install the following dependencies before running the simulation.

### Python Packages

Install all required packages via:

```bash
pip install -r requirements.txt
```
## How to Run

### 1. Clone the repository and navigate to the project folder

```bash
cd EMT_project
```
### 2. Choose the configuration
You can simulate with or without an on-ramp:
- With ramp: Uses main_config.sumocfg
- Without ramp: Uses main_config_without_ramp.sumocfg

Set this option by modifying the base_setting_para argument in main.py.

### 3. Run the simulation
```
cd sumo_control/control_handler
python main.py
```
## How It Works

### `main.py`

- The entry point for the simulation.
- Loads configuration and sets parameters like flow, MPR rate, and whether to use a ramp.
- Calls `run_for_main.py` to launch the simulation.

### `control.py`

- Contains core logic for Minimum Performance Requirement (MPR)-based traffic control.
- Dynamically manages platoon formation, speed, and flow behavior.

### `run_for_main.py`

- Contains the simulation loop.
- To change simulation duration, i.e., set interrupt_timestamp = 500

## Configurable Parameters

You can modify simulation behavior via:

| File             | Parameter                   | Description                              |
|------------------|-----------------------------|------------------------------------------|
| `main.py`        | `base_setting_para = [...]` | Simulation scenario selector             |
| `run_for_main.py`| `range(...)` in loop        | Sets simulation duration                 |
| `_config.py`     | Paths and flags             | Central config file                      |
| `control.py`     | Control logic               | Controls how vehicles behave dynamically |

## Attribution

This work is a modified version of the original project available at:  
[https://github.com/CASUUU/SUMO-BeginnerControl.git](https://github.com/CASUUU/SUMO-BeginnerControl.git)

**Original Paper Title:**  
*Enhancing Mixed Traffic Flow with Platoon Control and Lane Management for Connected and Autonomous Vehicles*






