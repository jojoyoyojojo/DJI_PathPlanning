# Implementation Record: Mission Safety and Payload Settings

## Summary
This implementation adds mission-safety controls, RTK quality detection, payload image-format selection, revised capture modes, video recording support, and the new user-facing application name `AeroFacade Studio`.

## User-Facing Changes
- Renamed the GUI window and startup logs from `Mavic 3T Facade Mission Planner` to `AeroFacade Studio`.
- Default drone type is now `M3E`.
- Capture modes are now:
  - `By time`
  - `Fixed-point photos`
  - `Video`
  - `No capture`
- Removed distance-triggered photo capture from the GUI.
- Added a `Video` mode that starts recording at the first waypoint and stops at the final waypoint.
- Added a prominent orange warning for `By time` mode when the current flight speed does not match the recommended speed for the selected interval and overlap.
- Added a confirmation dialog before generating a by-time mission with mismatched speed.
- Standardized GUI warnings to orange; red is reserved for error states.

## Safety Defaults
- `wpml:finishAction`: `noAction`
- `wpml:exitOnRCLost`: `executeLostAction`
- `wpml:executeRCLostAction`: `hover`
- `wpml:takeOffSecurityHeight`: `80`
- `wpml:globalTransitionalSpeed`: `5`
- Takeoff security height range: `1.2` to `1500` meters
- Global transitional speed range: `1` to `15` m/s

## RTK Detection
- Added metadata extraction for DJI XMP RTK fields:
  - `drone-dji:GpsStatus="RTK"`
  - `drone-dji:RtkFlag="50"`
- DJI XMP RTK status is preferred over standard EXIF `GPS GPSStatus`, because the standard field may contain only `A` or `V`.
- If all four facade photos are RTK FIX, the GUI shows RTK FIX status and WPML writes `wpml:positioningType` as `RTKBaseStation`.
- If any photo is not confirmed RTK FIX, the GUI shows an orange warning and WPML writes `wpml:positioningType` as `GPS`.

## WPML Output Changes
- Mission safety fields are now generated from configurable values instead of hard-coded constants.
- `template.kml` now writes `wpml:positioningType` under `wpml:waylineCoordinateSysParam`.
- `wpml:imageFormat` is now configurable:
  - `wide`
  - `ir`
  - `wide,ir`
- M3T exposes visible, infrared, and visible + infrared output options.
- M3E exposes visible wide output.
- Timed photo capture keeps stop-at-point turn behavior for facade coverage.
- Video capture uses continuous curvature turns for smoother constant-speed footage.

## Files Changed
- `gui.py`
  - Added RTK status display and warnings.
  - Added advanced safety controls.
  - Added image-format selector.
  - Reworked capture modes.
  - Added by-time speed mismatch warning and confirmation.
  - Updated window title to `AeroFacade Studio`.
- `mavic3T_pp_kmz.py`
  - Added RTK metadata parsing.
  - Added configurable mission safety fields.
  - Added configurable positioning type and image format.
  - Added video start/stop actions.
  - Updated capture-mode validation and turn-mode policy.
  - Updated default drone and mission safety settings.
- `GUI_DESIGN_DESCRIPTION.md`
  - Updated the user-facing product name.
- `openspec/changes/add-mission-safety-settings/`
  - Added proposal, task list, GUI spec delta, WPML output spec delta, and this implementation record.

## Validation
- `python3 -m py_compile gui.py mavic3T_pp_kmz.py` passed.
- IDE lint check reported no errors for `gui.py` and `mavic3T_pp_kmz.py`.
- `openspec` CLI was not available in the local shell, so strict OpenSpec validation was not run.

## Remaining Manual Checks
- Generate a KMZ from RTK FIX sample photos and confirm `wpml:positioningType` is `RTKBaseStation`.
- Generate a KMZ from non-RTK or missing-RTK sample photos and confirm the GUI warning plus `GPS` positioning output.
- Inspect generated `template.kml` and `waylines.wpml` for selected safety fields, image format, by-time photo action, and video start/stop actions.
