# Design: Force Vertical Plane Algorithm

## Problem Statement

The `FacadeTransformer` class fits a plane to 4 camera GPS positions. The coordinate system built from this plane has:
- **Y'**: Normal to fitted plane (points away from facade)
- **X'**: Width direction (derived from lower edge vector)
- **Z'**: Cross product of X' and Y'

When camera positions form a tilted plane (due to altitude variations during manual flight), Z' is not vertical. This causes the generated waypoint grid to be on a sheared plane rather than a true vertical plane.

## Solution: Project Normal to Horizontal

When `FORCE_VERTICAL_PLANE=True`:

1. Fit plane to 4 points as before → get raw normal `n = [a, b, c]`
2. Project normal onto XY plane: `n_horiz = [a, b, 0]` then normalize
3. Set Y' = `n_horiz` (facade normal is now purely horizontal)
4. Set Z' = `[0, 0, 1]` (true vertical up)
5. Set X' = `Z' × Y'` (horizontal, perpendicular to both)

```
Before (tilted):           After (forced vertical):
    Z' ↗                       Z' ↑
       \                          |
        \  Y'                     |  Y'
         ↘→                       |→
```

## Coordinate System Handedness

The facade coordinate system must remain right-handed:
- X' × Y' = Z' (right-hand rule)
- With Z' = [0,0,1] and Y' horizontal, X' = Z' × Y' ensures correct handedness

## Edge Cases

1. **Nearly vertical facade normal**: If the fitted plane is nearly horizontal (facade nearly parallel to ground), the horizontal projection will fail. This is an invalid input scenario - facades should be vertical.

2. **Orientation check**: After projection, verify Y' still points toward camera positions (away from building), flip if needed.

## Configuration

```python
# In mavic3T_pp_kmz.py
FORCE_VERTICAL_PLANE = True  # Default: force flight plane to be vertical

# In FacadeTransformer._build()
if FORCE_VERTICAL_PLANE:
    # Project normal to horizontal, force Z' vertical
else:
    # Original behavior: use fitted plane as-is
```

## GUI Integration

Add checkbox in Camera & Planning Settings zone:
- Label: "Force Vertical Plane"
- Default: Checked (True)
- Tooltip: "Ensure flight path is on a true vertical plane regardless of camera position tilt"
