# Change: Force Vertical Plane for Flight Path

## Why
The current plane-fitting algorithm preserves any tilt in the camera positions when generating the flight path. If the operator's 4 reference photos were taken at slightly different altitudes (creating a tilted camera plane), the resulting flight path will also be tilted/sheared relative to the ground. This is problematic because:

1. Building facades are typically vertical, so a tilted flight path is incorrect
2. Operator positioning errors should not propagate to mission geometry
3. RTK accuracy guarantees horizontal position, but altitude variations from manual flying can introduce tilt

## What Changes
- Add `FORCE_VERTICAL_PLANE` configuration option (default: `True`)
- Modify `FacadeTransformer._build()` to project the fitted plane normal onto the horizontal plane when enabled
- Force Z' axis to true vertical [0, 0, 1] when enabled
- Add "Force Vertical Plane" checkbox to GUI (Camera & Planning Settings zone)
- Preserve original behavior when option is disabled

## Impact
- Affected specs: `gui` (add checkbox), new `path-planning` spec
- Affected code:
  - `mavic3T_pp_kmz.py`: Modify `FacadeTransformer._build()`, add config constant
  - `gui.py`: Add checkbox to Camera & Planning Settings zone
- No breaking changes: Default behavior produces vertical planes (more correct for typical facades)
