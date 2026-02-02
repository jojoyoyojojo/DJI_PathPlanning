# Project Context

## Purpose
DJI drone mission planning tool for Mavic 3T facade photography. Converts RTK-enhanced GPS metadata from 4 facade corner photos into DJI-compatible KMZ mission files (WPML 1.0.6 standard). Automates flight path generation for building facade inspection and photography.

## Tech Stack
- **Language**: Python 3.12
- **Virtual Environment**: venv (`drone_env/`)
- **Core Libraries**:
  - `exifread` - Extract GPS/metadata from drone photos
  - `numpy` - Numerical computing, matrix operations for coordinate transforms
  - `pyproj` - Geographic coordinate transformations, geoid calculations
  - `simplekml` - KML/KMZ file generation
- **Standard Library**: xml.etree.ElementTree, zipfile, math, pathlib

## Project Conventions

### Code Style
- Chinese comments for domain logic explanations
- English for function/variable names and public documentation
- Configuration constants at file top in SCREAMING_SNAKE_CASE
- Classes use PascalCase, functions use snake_case
- Type hints not used (legacy codebase)

### Architecture Patterns
- **Coordinate Pipeline**: GPS (WGS84) → ENU → Facade Local (X'/Y'/Z') → Waypoints → WPML
- **Transformer Pattern**: `FacadeTransformer` / `FacadeCoordinateTransformer` classes encapsulate coordinate system conversions
- **Dual Output**: Separate files for DJI execution (WGS84) and editor display (EGM96)
- **Single-file scripts**: No package structure; each `.py` file is standalone

### Testing Strategy
- No automated tests (manual validation)
- Validate output by:
  - Unzipping KMZ and inspecting XML structure
  - Viewing `*_preview.kml` in Google Earth
  - Uploading to DJI FlySafe website

### Git Workflow
- No formal branching strategy documented
- Single-developer workflow

## Domain Context

### Height Reference Systems
- **WGS84**: Ellipsoidal height (raw GPS) - used in `waylines.wpml` for execution accuracy
- **EGM96**: Orthometric/MSL height - used in `template.kml` for human-readable display
- **Hong Kong geoid separation**: ~6.3m (hardcoded for local operations)

### RTK Workflow
1. Fly drone to 4 facade corners with RTK GPS enabled
2. Take photos - camera positions define plane parallel to facade
3. User provides `PHOTO_DISTANCE` (known distance from camera to facade)
4. System calculates actual facade plane by offsetting camera plane
5. Flight waypoints generated at `FLIGHT_DISTANCE` from true facade

### Facade Coordinate System (X'/Y'/Z')
- **X'**: Width direction (horizontal along facade)
- **Y'**: Depth direction (perpendicular to facade, positive = away from building)
- **Z'**: Height direction (vertical, positive = up)

### DJI WPML 1.0.6 Format
- KMZ must contain `wpmz/` subdirectory with `template.kml` and `waylines.wpml`
- Drone type 67 = Mavic 3T
- Supports snake-pattern waypoint generation with configurable overlap

## Important Constraints
- Requires exactly 4 input photos for facade corner detection
- Photos must contain valid GPS EXIF data (RTK-enhanced preferred)
- Assumes facade is approximately planar (vertical plane)
- Geoid separation hardcoded for Hong Kong region (~6.3m)
- No GUI - command-line only

## External Dependencies
- **DJI FlySafe**: For mission validation and upload
- **Google Earth**: For preview KML visualization
- **RTK Base Station**: Required for accurate GPS positioning in photos
- **DJI Pilot 2 App**: For mission execution on Mavic 3T
