<!-- OPENSPEC:START -->
# OpenSpec Instructions

These instructions are for AI assistants working in this project.

Always open `@/openspec/AGENTS.md` when the request:
- Mentions planning or proposals (words like proposal, spec, change, plan)
- Introduces new capabilities, breaking changes, architecture shifts, or big performance/security work
- Sounds ambiguous and you need the authoritative spec before coding

Use `@/openspec/AGENTS.md` to learn:
- How to create and apply change proposals
- Spec format and conventions
- Project structure and guidelines

Keep this managed block so 'openspec update' can refresh the instructions.

<!-- OPENSPEC:END -->

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DJI drone mission planning tool for Mavic 3T facade photography. Converts GPS metadata from 4 facade corner photos into DJI-compatible KMZ mission files (WPML 1.0.6 standard).

**Language**: Python 3.12
**Domain**: Geospatial analysis, drone flight planning, KMZ/KML generation

## Commands

### Setup
```bash
source drone_env/bin/activate
pip install -r requirements.txt
```

### Run GUI Application
```bash
python3 gui.py
```

### Run CLI Mission Generator
```bash
# With 4 facade corner photos
python3 mavic3T_pp_kmz.py photo1.jpg photo2.jpg photo3.jpg photo4.jpg

# With hardcoded PHOTO_PATHS in script
python3 mavic3T_pp_kmz.py
```

### Height Conversion Analysis
```bash
python3 height_converter.py <image.jpg>
```

### Validate Generated KMZ
- Unzip and inspect `wpmz/template.kml` and `wpmz/waylines.wpml`
- View `*_preview.kml` in Google Earth
- Upload to DJI FlySafe website

## Architecture

### Coordinate Transformation Pipeline
```
GPS (WGS84 from EXIF)
  → ENU Coordinates (East-North-Up)
  → Facade Local Coordinates (X'/Y'/Z' via plane fitting)
  → Waypoint Grid (snake pattern with overlap)
  → DJI WPML Format
```

### Key Files
- **gui.py**: PySide6 desktop GUI application
- **mavic3T_pp_kmz.py**: Core KMZ generator with `FacadeTransformer` class
- **height_converter.py**: WGS84 ↔ EGM96 height conversion utility

### Output Structure
```
mission.kmz/
├── wpmz/template.kml     # DJI editor (EGM96 heights)
└── wpmz/waylines.wpml    # DJI execution (WGS84 heights)

mission_preview.kml       # Google Earth preview (absolute altitude)
```

### Critical Height Standards
- **WGS84**: Ellipsoidal height (GPS raw) - used in waylines.wpml
- **EGM96**: Orthometric/MSL height - used in template.kml for display
- **Hong Kong geoid separation**: ~6.3m (hardcoded)

### Key Configuration (in mavic3T_pp_kmz.py)
```python
PHOTO_DISTANCE = 5.0        # Distance from camera to facade when photos taken (RTK prior)
FLIGHT_DISTANCE = 5.0       # Desired flight distance from facade
CAMERA_HFOV = 84.0          # Horizontal FOV degrees
CAMERA_VFOV = 62.0          # Vertical FOV degrees
OVERLAP_RATE = 0.65         # Photo overlap
```

## Important Patterns

### RTK Workflow & Four-Point Facade Detection
The system supports RTK-enhanced GPS for accurate positioning. The workflow:

1. Take 4 photos at facade corners with RTK GPS enabled
2. Camera positions define a plane **parallel** to the actual facade
3. `PHOTO_DISTANCE` (user prior knowledge) specifies camera-to-facade distance when photos were taken
4. Flight plane offset: `Y' = camera_plane - PHOTO_DISTANCE + FLIGHT_DISTANCE`

```
Camera Plane (RTK GPS)     Facade Plane           Flight Plane
        |                      |                      |
        |<-- PHOTO_DISTANCE -->|<-- FLIGHT_DISTANCE ->|
```

The `FacadeTransformer` class:
1. Extracts GPS from EXIF metadata
2. Converts to ENU coordinates
3. Fits a plane to detect facade orientation (parallel to actual facade)
4. Transforms to facade-local coordinate system (X'/Y'/Z' axes)

### Dual File Generation
DJI missions require separate height references:
- **Execution file**: WGS84 for GPS accuracy
- **Editor file**: EGM96 for human-readable display
- **Preview file**: Absolute altitude for Google Earth compatibility

### WPML 1.0.6 Compliance
- XML must include proper namespace declarations
- Files must be in `wpmz/` subdirectory within KMZ
- Drone type 67 = Mavic 3T
