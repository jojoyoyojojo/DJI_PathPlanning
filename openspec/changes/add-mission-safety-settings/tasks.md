## 1. Implementation
- [x] 1.1 Add EXIF helper to read `GPSStatus`, `drone-dji:RtkFlag`, and derive per-photo RTK FIX status.
- [x] 1.2 Display RTK status for all 4 photos in the GUI and show a warning when any photo is not confirmed RTK FIX.
- [x] 1.3 Add advanced safety controls for finish action, RC-loss behavior, takeoff security height, and global transitional speed.
- [x] 1.4 Add M3T image-format controls for visible, infrared, and visible + infrared output.
- [x] 1.5 Update WPML generation to write `wpml:positioningType`, selected safety fields, and selected `wpml:imageFormat`.
- [x] 1.6 Update waypoint/global turn-mode policy so continuous capture does not enable curvature turns by default.
- [x] 1.7 Remove distance-triggered capture mode and add video recording mode.
- [x] 1.8 Update defaults for drone type and advanced safety settings.
- [x] 1.9 Add prominent by-time speed mismatch warning and generation confirmation.

## 2. Validation
- [ ] 2.1 Generate a KMZ with RTK FIX sample photos and confirm `wpml:positioningType` is RTK.
- [ ] 2.2 Generate a KMZ with missing/non-FIX RTK metadata and confirm GUI warning plus GPS positioning output.
- [ ] 2.3 Inspect `template.kml` and `waylines.wpml` for selected safety fields and image format.
- [ ] 2.4 Validate that continuous capture still uses point-and-stop turn mode and zero damping unless a future explicit smooth-turn option is added.
