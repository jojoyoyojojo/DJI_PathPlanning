# Design: PySide6 Desktop GUI

## Context
The drone mission planning tool currently requires users to edit Python source code to configure parameters. A GUI will provide a more accessible interface while maintaining the existing core algorithm.

## Goals / Non-Goals
**Goals:**
- Expose all algorithm parameters via intuitive GUI widgets
- Organize interface into pipeline stages for clarity
- Display real-time feedback (image metadata, waypoint stats)
- Allow saving/loading configuration presets

**Non-Goals:**
- 3D visualization of flight path (out of scope)
- Real-time drone connection (out of scope)
- Map-based waypoint editing (out of scope)

## Decisions

### Framework: PySide6
- **Why:** Qt-based, cross-platform, already has PyQt5 in requirements (similar API)
- **Alternatives:** Tkinter (dated UI), PyQt6 (GPL licensing concerns), web-based (overcomplicated)

### Architecture: Single-file GUI with Core Import
- **Why:** Keep GUI separate from algorithm; import functions from `mavic3T_pp_kmz.py`
- **Pattern:** MVC-lite with QMainWindow containing zone widgets

### Window Zones Layout
```
┌─────────────────────────────────────────────────────────────┐
│                        Menu Bar                              │
├─────────────────────────────────────────────────────────────┤
│  Zone 1: Input Settings                                      │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ ┌─────────────────────────────────────────────────────┐ ││
│  │ │     Drag & Drop Zone (images or folder)             │ ││
│  │ │  ┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐           │ ││
│  │ │  │ IMG 1 │ │ IMG 2 │ │ IMG 3 │ │ IMG 4 │           │ ││
│  │ │  │ thumb │ │ thumb │ │ thumb │ │ thumb │           │ ││
│  │ │  └───────┘ └───────┘ └───────┘ └───────┘           │ ││
│  │ │  Drop images here or use buttons below              │ ││
│  │ └─────────────────────────────────────────────────────┘ ││
│  │ [Select Images...] [Select Folder...] [Clear All]       ││
│  │ Mission Name: [___________]                              ││
│  │ Photo Distance (m): [5.0]    Flight Distance (m): [5.0] ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  Zone 2: Camera & Planning Settings                          │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ HFOV (°): [84.0]   VFOV (°): [62.0]  Overlap: [0.65]   ││
│  │ [✓] Enable Smart Planning                                ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  Zone 3: Flight Settings                                     │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ Speed (m/s): [4.0]  Gimbal Pitch (°): [0.0]            ││
│  │ Drone Type: [M3T ▼]  Height Mode: [WGS84 ▼] [EGM96 ▼]  ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  Zone 4: Image Info (Read-only)                              │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ Photo 1: 22.3456°N, 114.1234°E, 85.2m (WGS84)          ││
│  │ Photo 2: 22.3457°N, 114.1235°E, 85.1m (WGS84)          ││
│  │ Photo 3: ...                                             ││
│  │ Photo 4: ...                                             ││
│  │ Facade: 12.5m × 8.2m | Plane quality: Good              ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  Zone 5: Path Generation                                     │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ [ Generate Mission ]                                     ││
│  │ Status: 42 waypoints generated (vertical snake pattern) ││
│  │ Flight Y' offset: camera 0.00m → flight 0.00m           ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  Zone 6: Output                                              │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ Output Directory: [/path/to/output] [Browse]            ││
│  │ [ Save KMZ ]  [ Save Preview KML ]  [ Save Both ]       ││
│  │ Status: ✓ Saved Facade Mission.kmz                      ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

### Image Input Modes

**Mode 1: Drag & Drop**
- User drags image files (JPG/JPEG) directly onto drop zone
- User drags a folder containing images onto drop zone
- Drop zone accepts up to 4 images; extras are ignored with warning
- Visual feedback: highlight border on drag-over, thumbnail preview on drop

**Mode 2: File Picker Buttons**
- "Select Images..." → Multi-file dialog filtered for JPG/JPEG
- "Select Folder..." → Folder dialog; auto-detects JPG/JPEG files inside
- "Clear All" → Removes all loaded images

**Folder Processing Logic**
```python
# When folder is dropped or selected:
1. Scan folder for *.jpg, *.jpeg files (case-insensitive)
2. Sort by filename (alphabetical)
3. Take first 4 images
4. Warn if <4 images found or >4 images (extras ignored)
```

**Thumbnail Display**
- Show small thumbnail (64×64) for each loaded image
- Display filename below thumbnail
- Click thumbnail to replace with different image

### Parameter Mapping

| GUI Widget | Algorithm Parameter | Type |
|------------|---------------------|------|
| Photo file pickers (×4) | `PHOTO_PATHS` | List[str] |
| Mission name input | `MISSION_NAME` | str |
| Photo distance spin | `PHOTO_DISTANCE` | float |
| Flight distance spin | `FLIGHT_DISTANCE` | float |
| HFOV spin | `CAMERA_HFOV` | float |
| VFOV spin | `CAMERA_VFOV` | float |
| Overlap slider/spin | `OVERLAP_RATE` | float (0-1) |
| Smart planning checkbox | `ENABLE_SMART_PLANNING` | bool |
| Speed spin | `AUTO_FLIGHT_SPEED` | float |
| Gimbal pitch spin | `GIMBAL_PITCH_DEG` | float |
| Drone type combo | `DRONE_TYPE` | str |
| Execute height mode combo | `EXECUTE_HEIGHT_MODE` | str |
| Template height mode combo | `TEMPLATE_HEIGHT_MODE` | str |

## Risks / Trade-offs
- **Risk:** PySide6 package size (~100MB) → Accept for full Qt functionality
- **Risk:** Cross-platform font rendering differences → Use system fonts
- **Mitigation:** Test on macOS (primary), Windows, Linux

## Migration Plan
1. Add PySide6 to requirements.txt
2. Create gui.py with zone-based layout
3. Refactor core algorithm to accept parameters dict (optional, can use module-level assignment)
4. Add entry point script or `if __name__ == "__main__"` in gui.py

## Open Questions
- Should we bundle with PyInstaller for single-executable distribution?
- Add preset save/load functionality in v1 or defer?
