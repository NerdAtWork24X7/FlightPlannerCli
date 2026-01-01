# FlightPlannerCli

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Purpose ✅

**FlightPlannerCli** builds IFR flight plans from a local Little Navmap / navigation SQLite database. It constructs an airway graph, extracts SID/STAR and transitions, computes an airway-based route between two airports, and writes a little-navmap-compatible PLN flight plan.

## Features 🚀

- Generates airway-based routes with SID/STAR support
- Works with Little Navmap SQLite databases
- Supports long-distance flights with expanded bounding boxes
- Handles waypoint deduplication and fallback routing
- Outputs Microsoft Flight Simulator compatible PLN files

## Requirements 🔧

- Python 3.8+
- pandas
- networkx
- sqlalchemy
- lxml

Install dependencies:

```bash
pip install pandas networkx sqlalchemy lxml
```

## Database Setup 📁

You need a Little Navmap compatible SQLite database file. Place it in the `Database/` directory or update the `DB_PATH` environment variable.

## Quick usage — CLI & environment variables ⚙️

- Run with defaults (script uses built-in example parameters when executed directly):

```powershell
python FltPlanGenerator.py
```

- Override environment variables (PowerShell example):

```powershell
$env:DB_PATH = "Database\little_navmap.sqlite"
$env:SID_NAME = "RAXE2A"
python FltPlanGenerator.py
```

- Override environment variables (CMD example):

```cmd
set DB_PATH=Database\little_navmap.sqlite
set SID_NAME=RAXE2A
python FltPlanGenerator.py
```

Important environment variables and defaults:

- `DB_PATH` — path to SQLite DB (default: `Database\little_navmap.sqlite`)
- `OUTPUT_FILE` — default output file name (default defined in script)
- `SID_NAME`, `STAR_NAME`, `SID_RWY`, `ATC_ROUTE` — preferred SID/STAR/runway/ATC route overrides

## Programmatic usage (importable functions) 💡

You can call the helpers from Python:

```python
from FltPlanGenerator import generate_route, Create_plan_with_close_waypoint

# Generate route objects
db, nav, writer, route = generate_route(dep="KSFO", dest="PHNL", include_sid=True, include_star=True)

# Create and write a PLN file
Create_plan_with_close_waypoint(db_path="Database\little_navmap.sqlite", dep="KSFO", dest="PHNL", output_file="out.pln", include_sid=True, include_star=True)
```

## Notes & Contributing ✨

- The script currently runs a sample plan when executed (see the bottom of `FltPlanGenerator.py`). If you prefer to import the module without running the example, remove or comment out the bottom call.
- Generated PLN files are MSFS compatible flight plans.
- Contributions and issues welcome via pull requests.

---

**Files:** `FltPlanGenerator.py`, `Database/little_navmap.sqlite` (SQLite DB), `LICENSE`

**License:** MIT
