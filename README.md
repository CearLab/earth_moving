# BeaverBot 🦫

A multi-agent simulation framework for modeling beaver behavior and ecological engineering in realistic terrains. BeaverBot simulates beaver colonies interacting with Digital Elevation Model (DEM) data, including vegetation dynamics, resource harvesting, and environmental modifications.

## Overview

BeaverBot provides a modular simulation environment where autonomous beaver agents:
- Navigate realistic terrain based on real-world elevation data
- Harvest vegetation and manage resources
- Reinforce canals and and modify their environment
- Exhibit exploration and foraging behaviors

The framework is built on a flexible backend architecture supporting visualization, multi-agent coordination, and environmental dynamics.

## Installation

### Prerequisites

- Python 3.8+
- pip package manager
- (Recommended) Virtual environment manager (venv, conda, or virtualenv)

### Setup

1. Clone the repository:
```bash
git clone https://github.com/beaverbotassistance/beaverbot.git
cd beaverbot
```

2. Create and activate a Python virtual environment (recommended):

**Using venv:**
```bash
python -m venv beaverbot_env
source beaverbot_env/bin/activate  # On Windows: beaverbot_env\Scripts\activate
```

**Using conda:**
```bash
conda create -n beaverbot python=3.10
conda activate beaverbot
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Test the installation:
```bash
python -c "import beaverbot; print('BeaverBot successfully installed!')"
```

## Quick Start

### 1. Interactive DEM Processing

Start by exploring how to process Digital Elevation Model data:

```bash
jupyter notebook beaverbot/utils/dem_conversion_interactive.ipynb
```

This interactive notebook teaches you how to:
- Load and process raw elevation data
- Configure coordinate systems
- Remove water bodies using stream thresholding
- Export data ready for simulation

### 2. Pre-configured Demo Simulations

Run the example simulations with pre-processed data:

#### Acadia National Park Demo (Recommended First)
```bash
jupyter notebook notebooks/demo_csv_acadia.ipynb
```

**Features:**
- Smaller area (~150m × 150m)
- Faster computation
- High resolution (0.5m per pixel)
- Ideal for learning and testing

#### Boston Rumney Marsh Demo (Advanced)
```bash
jupyter notebook notebooks/demo_csv_boston.ipynb
```

**Features:**
- Larger area (~400m × 400m)
- More computationally intensive
- 1m resolution
- Realistic large-scale simulation

⚠️ **Note**: The Boston demo requires more memory and processing time. Start with Acadia to familiarize yourself with the framework.

Both demos include:
- Pre-loaded elevation data
- Configured beaver agents
- Optimized simulation parameters
- Visualization examples

## DEM Data Processing

### Processing Your Own Elevation Data

After trying the demo notebooks, you can process your own DEM data using the interactive converter:

```bash
jupyter notebook beaverbot/utils/dem_conversion_interactive.ipynb
```

The notebook provides a step-by-step workflow:
1. **Loading CSV data** with elevation coordinates
2. **Setting resolution** and resampling
3. **Normalizing elevation** to simulation ranges
4. **Stream threshold removal** to identify water bodies
5. **Coordinate transformation** between CRS systems
6. **Polygon masking** to select regions of interest
7. **Exporting processed data** ready for simulation

### Supported Data Formats

- **Geographic coordinates**: CSV with `x` (longitude), `y` (latitude), `elevation`
- **Projected coordinates**: CSV with `X_m`, `Y_m`, `elevation`
- **Output format**: NumPy arrays (.npy) with metadata JSON

### EPSG Coordinate Systems

The tool supports automatic conversion between coordinate reference systems:
- **WGS84** (EPSG:4326): Standard GPS coordinates
- **NAD83 State Plane**: Regional projected systems (feet or meters)
- Custom EPSG codes for any region

Example regions:
- **Massachusetts**: EPSG:6483 (meters) or EPSG:2249 (feet)
- **Maine**: EPSG:6484
- Find yours at [epsg.io](https://epsg.io/)

## Project Structure

```
beaverbot/
├── beaverbot/
│   ├── ral/                      # Robot Abstraction Layer
│   │   ├── backend/              # Simulation backends
│   │   │   ├── base_backend.py
│   │   │   ├── beavers_visualizer_backend.py
│   │   │   └── modules/          # Color maps and utilities
│   │   ├── environment/          # Environment simulation
│   │   │   ├── environment_backend.py
│   │   │   └── environment_beavers_backend.py
│   │   ├── robot/                # Agent behaviors
│   │   │   ├── robot_backend.py
│   │   │   ├── robot_beavers_backend.py
│   │   │   └── modules/          # Beaver-specific modules
│   │   └── algorithms/           # Utility algorithms
│   ├── scenarios/                # Pre-defined simulation scenarios
│   │   └── standard_beavers_scenario.py
│   ├── utils/                    # DEM processing utilities
│   │   ├── dem_to_matrix.py
│   │   └── dem_conversion_interactive.ipynb
│   └── constants.py              # Global constants
├── notebooks/                    # Example notebooks
│   ├── demo_csv_boston.ipynb
│   └── demo_csv_acadia.ipynb
├── requirements.txt
└── README.md
```

## Visualization

The framework provides comprehensive real-time visualization of the simulation through two main plotting functions:

### Environment Heatmaps

Three-panel visualization showing:

1. **Initial Vegetation Quality**
   - Starting state of the environment
   - Elevation with blue-brown-green colormap
   - Water bodies (streams/rivers) shown in blue/negative values

2. **Current Vegetation Quality**
   - Real-time vegetation state during simulation
   - Shows vegetation depletion from harvesting
   - Regrowth dynamics over time
   - Agent positions marked (red = explorers, black = expanders)
   - Home base locations (gray rectangles)

3. **Agent Visit Frequency**
   - Heatmap showing exploration patterns
   - Color-coded by visit intensity
   - Reveals agent movement strategies and territorial behavior
   - Overlay shows water bodies for reference

### Simulation Analytics

Per-agent statistics tracking:

1. **Distance from Initial Position**
   - Movement range over simulation
   - River crossing events (blue markers)
   - Territory exploration patterns

2. **Load Variation**
   - Current load vs maximum capacity
   - Harvesting efficiency
   - Resource transport patterns

3. **Control Error Norm**
   - PID controller performance
   - Navigation accuracy
   - Behavioral stability metrics

4. **Exploration Parameters**
   - Exploration eta (resource quality threshold)
   - Harvest threshold boundaries (min/max)
   - Adaptive behavior indicators

## Dependencies

Core dependencies:
- `numpy`: Numerical computing and array operations
- `scipy`: Scientific computing and optimization
- `matplotlib`: Visualization and plotting
- `mesa` (v1.2.0): Agent-based modeling framework
- `jupyter`: Interactive notebook environment (includes ipykernel, ipython, ipywidgets)

See `requirements.txt` for complete list.

## Contributing

Contributions are welcome! Areas for improvement:
- Additional agent behaviors
- Enhanced environmental dynamics
- Performance optimizations
- New visualization tools
- Documentation and examples

## Citation

If you use BeaverBot in your research, please cite:

```bibtex
@software{beaverbot2025,
  title={BeaverBot: Multi-Agent Simulation Framework for Ecological Engineering},
  year={2025},
  url={https://github.com/beaverbotassistance/beaverbot}
}
```

## License

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at:

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

## Contact

For questions, issues, or collaboration:
- Open an issue on GitHub

## Acknowledgments

This is a research tool developed for ecological engineering simulation.

---

**Note**: This is a research tool. Simulation results should be validated against real-world observations before drawing ecological conclusions.