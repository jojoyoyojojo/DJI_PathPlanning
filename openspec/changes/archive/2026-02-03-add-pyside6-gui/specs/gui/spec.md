# GUI Capability Specification

## ADDED Requirements

### Requirement: Desktop Application Window
The system SHALL provide a PySide6-based desktop application with a main window containing distinct zones for each pipeline stage.

#### Scenario: Application launch
- **WHEN** user runs gui.py
- **THEN** a main window opens with all six zones visible

#### Scenario: Window layout
- **WHEN** application is displayed
- **THEN** zones are arranged vertically: Input Settings, Camera Settings, Flight Settings, Image Info, Path Generation, Output

---

### Requirement: Input Settings Zone
The system SHALL provide a zone for selecting input photos via drag-and-drop or file picker, and configuring RTK distance parameters.

#### Scenario: Drag and drop images
- **WHEN** user drags image files (JPG/JPEG) onto the drop zone
- **THEN** images are loaded and thumbnails displayed
- **AND** GPS extraction is triggered for each image
- **AND** drop zone shows visual highlight during drag-over

#### Scenario: Drag and drop folder
- **WHEN** user drags a folder onto the drop zone
- **THEN** folder is scanned for JPG/JPEG files
- **AND** first 4 images (sorted alphabetically) are loaded
- **AND** warning is shown if folder contains fewer than 4 or more than 4 images

#### Scenario: Select images button
- **WHEN** user clicks "Select Images..." button
- **THEN** multi-file dialog opens filtered for JPG/JPEG
- **AND** selected images (up to 4) are loaded with thumbnails

#### Scenario: Select folder button
- **WHEN** user clicks "Select Folder..." button
- **THEN** folder picker dialog opens
- **AND** selected folder is scanned for images
- **AND** first 4 images are loaded

#### Scenario: Clear all button
- **WHEN** user clicks "Clear All" button
- **THEN** all loaded images are removed
- **AND** thumbnail slots are cleared
- **AND** image info zone is cleared

#### Scenario: Thumbnail display
- **WHEN** images are loaded
- **THEN** 64×64 thumbnail previews are shown in grid
- **AND** filename is displayed below each thumbnail

#### Scenario: Distance parameters
- **WHEN** user modifies photo distance or flight distance spin boxes
- **THEN** values are validated (positive numbers, reasonable range 0.1-100m)
- **AND** values are stored for path generation

#### Scenario: Mission naming
- **WHEN** user enters a mission name
- **THEN** invalid characters are automatically sanitized
- **AND** name is used for output file naming

---

### Requirement: Camera Settings Zone
The system SHALL provide a zone for configuring camera FOV and overlap parameters.

#### Scenario: FOV configuration
- **WHEN** user modifies HFOV or VFOV spin boxes
- **THEN** values are validated (1-180 degrees range)
- **AND** values affect waypoint spacing calculation

#### Scenario: Overlap configuration
- **WHEN** user adjusts overlap slider/spin
- **THEN** value is constrained to 0.00-0.99 range
- **AND** value affects waypoint density

#### Scenario: Smart planning toggle
- **WHEN** user toggles smart planning checkbox
- **THEN** path generation uses automatic direction selection (checked) or horizontal-only (unchecked)

---

### Requirement: Flight Settings Zone
The system SHALL provide a zone for configuring drone flight parameters.

#### Scenario: Speed configuration
- **WHEN** user modifies flight speed spin box
- **THEN** value is validated (0.1-15 m/s range)

#### Scenario: Gimbal pitch configuration
- **WHEN** user modifies gimbal pitch spin box
- **THEN** value is validated (-90 to +30 degrees range)

---

### Requirement: Image Info Zone
The system SHALL provide a read-only zone displaying extracted GPS metadata from selected photos.

#### Scenario: GPS extraction on photo load
- **WHEN** a photo is selected in Input Settings zone
- **THEN** GPS coordinates (lat, lon, alt) are extracted and displayed
- **AND** coordinate format shows degrees with direction (e.g., "22.3456°N, 114.1234°E, 85.2m")

#### Scenario: Facade info display
- **WHEN** all 4 photos are loaded
- **THEN** facade dimensions (width × height) are calculated and displayed
- **AND** plane fitting quality is indicated

#### Scenario: Extraction error handling
- **WHEN** photo lacks GPS EXIF data
- **THEN** error message is displayed in the info zone
- **AND** generation is blocked until valid photo provided

---

### Requirement: Path Generation Zone
The system SHALL provide a zone for triggering mission generation and displaying results.

#### Scenario: Generate button
- **WHEN** user clicks "Generate Mission" with all 4 valid photos loaded
- **THEN** waypoints are calculated using core algorithm
- **AND** waypoint count and flight direction are displayed

#### Scenario: Generate validation
- **WHEN** user clicks "Generate Mission" without all 4 photos
- **THEN** error message indicates missing photos
- **AND** generation does not proceed

#### Scenario: Progress feedback
- **WHEN** generation is in progress
- **THEN** visual indicator shows processing state

---

### Requirement: Output Zone
The system SHALL provide a zone for saving generated mission files.

#### Scenario: Output directory selection
- **WHEN** user clicks browse button
- **THEN** directory picker dialog opens
- **AND** selected path is displayed

#### Scenario: Save KMZ
- **WHEN** user clicks "Save KMZ" after successful generation
- **THEN** KMZ file is written to output directory with mission name
- **AND** success status is displayed with file path

#### Scenario: Save preview KML
- **WHEN** user clicks "Save Preview KML" after successful generation
- **THEN** preview KML file is written to output directory
- **AND** success status is displayed

#### Scenario: Save both
- **WHEN** user clicks "Save Both"
- **THEN** both KMZ and preview KML are saved
- **AND** success status shows both file paths

#### Scenario: Save before generate
- **WHEN** user clicks any save button without generating first
- **THEN** error message indicates mission must be generated first

---

### Requirement: State Management
The system SHALL maintain proper widget enable/disable states based on workflow progression.

#### Scenario: Initial state
- **WHEN** application starts
- **THEN** Generate button is disabled
- **AND** Save buttons are disabled

#### Scenario: Photos loaded state
- **WHEN** all 4 valid photos are loaded
- **THEN** Generate button becomes enabled

#### Scenario: Mission generated state
- **WHEN** mission is successfully generated
- **THEN** Save buttons become enabled

#### Scenario: Parameter change after generation
- **WHEN** user modifies any input parameter after generation
- **THEN** Save buttons are disabled
- **AND** user must regenerate before saving
