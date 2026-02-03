# Tasks: Add PySide6 Desktop GUI

## 1. Setup
- [x] 1.1 Add PySide6 to requirements.txt
- [x] 1.2 Create gui.py skeleton with QMainWindow

## 2. Zone 1: Input Settings
- [x] 2.1 Create drag-and-drop zone widget with visual feedback
- [x] 2.2 Implement drop handler for image files (JPG/JPEG)
- [x] 2.3 Implement drop handler for folders (scan for images)
- [x] 2.4 Create thumbnail display grid (4 slots, 64×64 thumbnails)
- [x] 2.5 Add "Select Images..." button (multi-file dialog)
- [x] 2.6 Add "Select Folder..." button (folder dialog with auto-scan)
- [x] 2.7 Add "Clear All" button
- [x] 2.8 Add mission name text input
- [x] 2.9 Add photo distance spin box (float, 0.1-100m range)
- [x] 2.10 Add flight distance spin box (float, 0.1-100m range)
- [x] 2.11 Wire image loading to trigger GPS extraction
- [x] 2.12 Handle <4 or >4 images with appropriate warnings

## 3. Zone 2: Camera & Planning Settings
- [x] 3.1 Add HFOV spin box (float, 1-180° range)
- [x] 3.2 Add VFOV spin box (float, 1-180° range)
- [x] 3.3 Add overlap slider/spin box (float, 0-0.99 range)
- [x] 3.4 Add smart planning checkbox

## 4. Zone 3: Flight Settings
- [x] 4.1 Add flight speed spin box (float, 0.1-15 m/s range)
- [x] 4.2 Add gimbal pitch spin box (float, -90 to +30° range)
- [x] 4.3 Add drone type combo box (M3T, etc.)
- [x] 4.4 Add execute height mode combo box (WGS84, relativeToStartPoint)
- [x] 4.5 Add template height mode combo box (EGM96, relativeToStartPoint)

## 5. Zone 4: Image Info Display
- [x] 5.1 Create read-only text area for GPS metadata display
- [x] 5.2 Implement `read_gps()` call on photo selection
- [x] 5.3 Display facade dimensions after all 4 photos loaded
- [x] 5.4 Show plane fitting quality indicator

## 6. Zone 5: Path Generation
- [x] 6.1 Add "Generate Mission" button
- [x] 6.2 Implement generation logic (call `build_waypoints_from_images`)
- [x] 6.3 Display waypoint count and flight direction
- [x] 6.4 Show RTK offset calculation details
- [x] 6.5 Add progress indicator for generation

## 7. Zone 6: Output
- [x] 7.1 Add output directory picker
- [x] 7.2 Add "Save KMZ" button
- [x] 7.3 Add "Save Preview KML" button
- [x] 7.4 Add "Save Both" button
- [x] 7.5 Display save status and file paths

## 8. Integration & Polish
- [x] 8.1 Add input validation (all 4 photos required before generate)
- [x] 8.2 Add error handling with message boxes
- [x] 8.3 Disable/enable buttons based on state
- [x] 8.4 Add menu bar (File → Open Photos, Save Config, Exit)
- [x] 8.5 Test on macOS

## 9. Documentation
- [x] 9.1 Update CLAUDE.md with GUI launch command
- [x] 9.2 Update openspec/project.md
