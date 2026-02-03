# GUI Capability Specification

## MODIFIED Requirements

### Requirement: Camera Settings Zone
The system SHALL provide a zone for configuring camera FOV, overlap parameters, and plane fitting options.

#### Scenario: Force vertical plane toggle
- **WHEN** user toggles "Force Vertical Plane" checkbox
- **THEN** path generation uses vertical-constrained plane fitting (checked) or original tilted fitting (unchecked)
- **AND** checkbox is checked by default

#### Scenario: Force vertical plane tooltip
- **WHEN** user hovers over "Force Vertical Plane" checkbox
- **THEN** tooltip displays: "Ensure flight path is on a true vertical plane regardless of camera position tilt"
