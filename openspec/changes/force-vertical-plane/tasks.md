# Tasks

- [x] Task 1: Add FORCE_VERTICAL_PLANE configuration constant
- Add `FORCE_VERTICAL_PLANE = True` to configuration section of `mavic3T_pp_kmz.py`
- **Verification**: Constant exists and defaults to True

- [x] Task 2: Implement vertical plane projection in FacadeTransformer
- Modify `FacadeTransformer._build()` method:
  - When `FORCE_VERTICAL_PLANE=True`:
    - Project fitted normal onto horizontal plane (set Z=0, renormalize)
    - Set Z' = [0, 0, 1] (true vertical)
    - Calculate X' = Z' × Y' (horizontal width direction)
    - Add error handling for nearly-horizontal facade normal
  - When `FORCE_VERTICAL_PLANE=False`:
    - Preserve existing behavior
- **Verification**:
  - Test with tilted input → verify Z' = [0,0,1] when enabled
  - Test with same input, disabled → verify original tilted Z'

- [x] Task 3: Add GUI checkbox for Force Vertical Plane
- Add checkbox to Camera & Planning Settings zone in `gui.py`
- Label: "Force Vertical Plane"
- Default: Checked
- Add tooltip text
- Wire to `core.FORCE_VERTICAL_PLANE` in `_generate_mission()`
- **Verification**: Checkbox appears, toggles affect generation

- [ ] Task 4: Manual validation
- Generate mission with tilted camera positions, force vertical ON → verify vertical flight plane
- Generate mission with same positions, force vertical OFF → verify tilted flight plane
- View both in Google Earth to confirm visual difference
- **Verification**: Visual inspection confirms expected behavior (user task)
