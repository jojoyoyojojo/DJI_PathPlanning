# Change: Add PySide6 Desktop GUI

## Why
The current CLI-only workflow requires users to edit Python source code to configure parameters and photo paths. A desktop GUI will make the tool accessible to non-technical users and provide visual feedback throughout the mission planning pipeline.

## What Changes
- Add new `gui.py` module with PySide6-based desktop application
- Create zoned interface reflecting the algorithm pipeline stages
- Expose all configurable parameters via GUI widgets
- Support two image input modes:
  - **Drag & Drop**: Drag images or image folder directly onto input zone
  - **File Picker**: Browse for individual images or select a folder
- Display extracted image metadata and generated waypoint information
- Maintain CLI compatibility (core algorithm unchanged)

## Impact
- Affected specs: New `gui` capability (no existing specs affected)
- Affected code:
  - New `gui.py` (main GUI module)
  - `mavic3T_pp_kmz.py` (refactor config constants into importable module or pass as parameters)
- Dependencies: Add `PySide6` to requirements.txt
