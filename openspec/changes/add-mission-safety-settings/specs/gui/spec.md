# GUI Capability Specification

## ADDED Requirements

### Requirement: RTK Quality Status
The system SHALL inspect RTK-related EXIF metadata for each selected facade-corner photo and display whether the complete input set is confirmed RTK FIX.

#### Scenario: All photos are RTK FIX
- **GIVEN** all 4 selected photos contain `drone-dji:GpsStatus="RTK"`
- **AND** all 4 selected photos contain `drone-dji:RtkFlag="50"`
- **WHEN** GPS metadata is extracted
- **THEN** the GUI displays an RTK FIX status for the input set
- **AND** no positioning-accuracy warning is shown

#### Scenario: One or more photos are not RTK FIX
- **GIVEN** at least one selected photo is missing RTK metadata or has RTK metadata other than `drone-dji:GpsStatus="RTK"` and `drone-dji:RtkFlag="50"`
- **WHEN** GPS metadata is extracted
- **THEN** the GUI displays a positioning-accuracy warning
- **AND** the warning states that facade geometry may be inaccurate and close facade flight is not recommended
- **AND** mission generation remains available when the photos still contain valid GPS coordinates

#### Scenario: Per-photo RTK details
- **WHEN** selected photo metadata is displayed
- **THEN** each photo row includes RTK status details when available
- **AND** missing RTK fields are shown as unknown instead of being treated as RTK FIX

---

### Requirement: Advanced Mission Safety Settings
The system SHALL provide advanced GUI controls for safety-relevant WPML mission configuration values that materially affect flight behavior.

#### Scenario: Default safety settings
- **WHEN** the GUI starts
- **THEN** the selected drone type defaults to `M3E`
- **AND** finish action defaults to `noAction`
- **AND** RC-loss continuation defaults to `executeLostAction`
- **AND** RC-loss execution action defaults to `hover`
- **AND** takeoff security height defaults to `80` meters
- **AND** global transitional speed defaults to `5` m/s
- **AND** takeoff security height accepts values from `1.2` to `1500` meters
- **AND** global transitional speed accepts values from `1` to `15` m/s

#### Scenario: User changes safety settings
- **WHEN** the user changes any advanced safety setting
- **THEN** the generated KMZ uses the selected value in the relevant WPML mission configuration fields
- **AND** existing generated output is invalidated until the mission is regenerated

#### Scenario: RC-loss behavior clarity
- **WHEN** RC-loss settings are displayed
- **THEN** the GUI explains the relationship between continuing the route and executing a loss-of-control action
- **AND** the user can see which behavior will be written before saving the KMZ

---

### Requirement: Payload Image Format Selection
The system SHALL let the user choose the requested image output format for supported aircraft payloads instead of always writing wide-angle output.

#### Scenario: M3T payload options
- **GIVEN** the selected aircraft is M3T
- **WHEN** the flight settings are displayed
- **THEN** the user can choose visible, infrared, or visible + infrared image output

#### Scenario: Default payload option
- **WHEN** the GUI starts with the default M3E aircraft profile
- **THEN** the image-format selection defaults to visible wide output

#### Scenario: User changes image format
- **WHEN** the user changes the image-format selection
- **THEN** existing generated output is invalidated until the mission is regenerated
- **AND** the generated KMZ writes the selected image format in the payload parameters

---

### Requirement: Capture Mode Selection
The system SHALL provide capture modes for timed photo capture, fixed-point photo capture, video recording, and no capture.

#### Scenario: Capture mode options
- **WHEN** the flight settings are displayed
- **THEN** the capture mode options are by time, fixed-point photos, video, and no capture
- **AND** distance-triggered photo capture is not offered

#### Scenario: By time speed mismatch warning
- **GIVEN** the user selects by time capture mode
- **AND** the current flight speed does not match the recommended speed for the selected interval and overlap
- **WHEN** the flight settings are displayed or mission generation is requested
- **THEN** the GUI displays a prominent warning with the recommended speed and current speed
- **AND** mission generation asks for confirmation before proceeding

#### Scenario: Video mode behavior
- **GIVEN** the user selects video mode
- **WHEN** the mission is generated
- **THEN** the KMZ starts recording at the first waypoint
- **AND** stops recording at the final waypoint
- **AND** uses continuous turns for smoother constant-speed footage
