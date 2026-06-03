# WPML Output Capability Specification

## ADDED Requirements

### Requirement: Positioning Type Output
The system SHALL write `wpml:positioningType` in `template.kml` according to the RTK quality detected from the 4 input photos.

#### Scenario: RTK FIX positioning source
- **GIVEN** all 4 selected photos are confirmed RTK FIX using `drone-dji:GpsStatus="RTK"` and `drone-dji:RtkFlag="50"`
- **WHEN** `template.kml` is generated
- **THEN** `wpml:positioningType` is written as an RTK positioning source

#### Scenario: GPS fallback positioning source
- **GIVEN** any selected photo is not confirmed RTK FIX
- **WHEN** `template.kml` is generated
- **THEN** `wpml:positioningType` is written as `GPS`
- **AND** waypoint generation still uses the available GPS coordinates

#### Scenario: Positioning type location
- **WHEN** `template.kml` is generated
- **THEN** `wpml:positioningType` is written under `wpml:waylineCoordinateSysParam`
- **AND** the value is one of DJI WPML supported values: `GPS`, `RTKBaseStation`, `QianXun`, or `Custom`

---

### Requirement: Safety Mission Config Output
The system SHALL write user-selected safety settings consistently into WPML mission configuration.

#### Scenario: Mission config fields
- **WHEN** `template.kml` and `waylines.wpml` are generated
- **THEN** `wpml:finishAction` uses the selected finish action
- **AND** `wpml:exitOnRCLost` uses the selected RC-loss continuation behavior
- **AND** `wpml:executeRCLostAction` uses the selected RC-loss execution action
- **AND** `wpml:takeOffSecurityHeight` uses the selected takeoff security height
- **AND** `wpml:globalTransitionalSpeed` uses the selected global transitional speed

#### Scenario: Default mission config compatibility
- **WHEN** the user does not change advanced safety settings
- **THEN** generated WPML preserves the current defaults: `noAction`, `executeLostAction`, `hover`, `80`, and `5`

---

### Requirement: Payload Image Format Output
The system SHALL write the selected payload image format instead of always writing `wide`.

#### Scenario: Visible image output
- **GIVEN** the user selects visible image output
- **WHEN** `template.kml` is generated
- **THEN** `wpml:imageFormat` represents visible image capture for the selected aircraft profile

#### Scenario: Infrared image output
- **GIVEN** the user selects infrared image output
- **WHEN** `template.kml` is generated
- **THEN** `wpml:imageFormat` is written as `ir`

#### Scenario: Visible and infrared image output
- **GIVEN** the user selects visible + infrared image output
- **WHEN** `template.kml` is generated
- **THEN** `wpml:imageFormat` requests both visible and infrared output using DJI WPML comma-list syntax

---

### Requirement: Facade Coverage Turn Mode
The system SHALL use stop-at-point turn behavior by default for facade coverage, independent of capture trigger mode.

#### Scenario: Waypoint capture turn mode
- **GIVEN** capture mode is stop-at-waypoint
- **WHEN** `template.kml` and `waylines.wpml` are generated
- **THEN** `wpml:globalWaypointTurnMode` and per-waypoint `wpml:waypointTurnMode` are `toPointAndStopWithDiscontinuityCurvature`
- **AND** turn damping distance is `0`

#### Scenario: Timed photo capture turn mode
- **GIVEN** capture mode is continuous by time
- **WHEN** `template.kml` and `waylines.wpml` are generated
- **THEN** `wpml:globalWaypointTurnMode` and per-waypoint `wpml:waypointTurnMode` remain `toPointAndStopWithDiscontinuityCurvature`
- **AND** turn damping distance remains `0`
- **AND** timed photo capture only changes the action trigger strategy, not the route cornering policy

#### Scenario: Video recording turn mode
- **GIVEN** capture mode is video
- **WHEN** `template.kml` and `waylines.wpml` are generated
- **THEN** `wpml:globalWaypointTurnMode` and per-waypoint `wpml:waypointTurnMode` are `toPointAndPassWithContinuityCurvature`
- **AND** turn damping distance is non-zero for smoother constant-speed footage
- **AND** video recording starts at the first waypoint and stops at the final waypoint
