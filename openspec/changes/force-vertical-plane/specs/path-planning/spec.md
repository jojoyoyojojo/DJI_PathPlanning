# Path Planning Capability Specification

## ADDED Requirements

### Requirement: Force Vertical Plane Option
The system SHALL provide an option to force the flight plane to be vertical regardless of camera position tilt.

#### Scenario: Default behavior with force vertical enabled
- **GIVEN** `FORCE_VERTICAL_PLANE` is True (default)
- **WHEN** 4 camera positions form a tilted plane
- **THEN** the flight path Z' axis is aligned with true vertical [0,0,1]
- **AND** the facade normal Y' is projected onto the horizontal plane
- **AND** the width axis X' is perpendicular to both Y' and Z'

#### Scenario: Original behavior with force vertical disabled
- **GIVEN** `FORCE_VERTICAL_PLANE` is False
- **WHEN** 4 camera positions form a tilted plane
- **THEN** the flight path preserves the tilt of the fitted plane
- **AND** Z' may not be aligned with true vertical

#### Scenario: Horizontal facade normal error
- **GIVEN** `FORCE_VERTICAL_PLANE` is True
- **WHEN** the fitted plane normal is nearly vertical (facade nearly horizontal)
- **THEN** an error is raised indicating invalid facade orientation
- **AND** the user is informed that facade must be approximately vertical

---

### Requirement: Coordinate System Integrity
The system SHALL maintain a valid right-handed coordinate system after forcing vertical.

#### Scenario: Right-handed coordinate system
- **GIVEN** `FORCE_VERTICAL_PLANE` is True
- **WHEN** the coordinate system is constructed
- **THEN** X' × Y' = Z' (right-hand rule is satisfied)
- **AND** all axes are unit vectors (length 1)

#### Scenario: Facade normal orientation
- **GIVEN** `FORCE_VERTICAL_PLANE` is True
- **WHEN** the horizontal normal Y' is computed
- **THEN** Y' points away from the facade (toward camera positions)
- **AND** the orientation is verified by checking signed distance from centroid
