# Change: Add Mission Safety and Payload Settings

## Why
Several safety-relevant DJI WPML fields are currently hard-coded, and the GUI does not show whether the 4 reference photos were captured with RTK FIX quality. Mavic 3T missions also need an explicit way to request visible and/or infrared image output instead of always writing `wide`.

## What Changes
- Add GUI controls for advanced mission safety fields: finish action, RC-loss behavior, takeoff security height, and global transitional speed.
- Detect RTK FIX metadata from each input photo using `GPSStatus="RTK"` and `drone-dji:RtkFlag="50"`; show a warning when any photo is not confirmed RTK FIX.
- Write `wpml:positioningType` from the detected positioning source, using RTK only when all four input photos are confirmed RTK FIX.
- Add M3T payload image-format selection so users can choose visible, infrared, or visible + infrared output.
- Keep facade waypoint turn mode conservative for coverage: stop at points with discontinuity curvature by default, regardless of continuous capture mode.

## Impact
- Affected specs: `gui`, `wpml-output`
- Affected code:
  - `mavic3T_pp_kmz.py`: EXIF metadata extraction, WPML config constants, `template.kml` / `waylines.wpml` tag generation, turn-mode policy
  - `gui.py`: RTK quality display, advanced safety controls, payload image-format controls
- No breaking file-format changes: generated KMZ remains DJI WPML 1.0.6 compatible.

## Guidance Linkage
- This proposal is created under the OpenSpec workflow referenced by both `AGENTS.md` and `CLAUDE.md`.
- `AGENTS.md` and the OpenSpec-managed block at the top of `CLAUDE.md` direct agents to read `openspec/AGENTS.md` before planning or implementing capability changes.
- This change record is the project-level source of truth for the proposed mission-safety work until it is approved, implemented, and archived into the active specs.
- No edits are required in `AGENTS.md` or `CLAUDE.md` for this proposal; those files already contain the managed OpenSpec linkage.
