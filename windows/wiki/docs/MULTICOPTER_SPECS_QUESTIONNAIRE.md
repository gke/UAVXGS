# Multicopter Physical Specifications Questionnaire

For each airframe in the `_Tuned.af` files, please provide the following physical specifications so we can accurately model the dynamics in simulation.

## Required Parameters

| Parameter | Unit | Description | Example |
|-----------|------|-------------|---------|
| `PHYS_MASS_KG` | kg | Total AUW (All-Up Weight) including battery | 0.91 |
| `PHYS_FRAME_MM` | mm | Diagonal motor-to-motor distance | 220 |
| `PHYS_MOTOR_KV` | RPM/V | Motor KV rating | 1000 |
| `PHYS_PROP_DIAMETER_INCH` | inch | Propeller diameter | 11 |
| `PHYS_PROP_PITCH_INCH` | inch | Propeller pitch | 3.7 |
| `PHYS_PROP_BLADES` | count | Number of blades per prop (2, 3, 4) | 2 |
| `PHYS_CELL_COUNT` | S | Battery cell count (1S, 2S, 3S, 4S, 6S) | 4 |
| `PHYS_BATTERY_MAH` | mAh | Battery capacity | 1500 |
| `PHYS_ARM_LEN_M` | m | Center-to-motor distance | 0.11 |
| `PHYS_INERTIA_ROLL_PITCH` | kg·m² | Roll/Pitch moment of inertia | 0.002 |
| `PHYS_INERTIA_YAW` | kg·m² | Yaw moment of inertia | 0.005 |
| `PHYS_MAX_THRUST_N` | N | Max thrust per motor (measured or estimated) | 3.5 |

## Optional (Fixed Wing Only)

| Parameter | Unit | Description |
|-----------|------|-------------|
| `PHYS_CRUISE_SPEED_MS` | m/s | Cruise airspeed |
| `PHYS_MIN_SPEED_MS` | m/s | Minimum airspeed (stall) |
| `PHYS_MAX_SPEED_MS` | m/s | Maximum airspeed |
| `PHYS_LIFT_COEFF` | - | Wing lift coefficient |
| `PHYS_DRAG_COEFF` | - | Parasite drag coefficient |

---

## Airframes Needing Specs

### Ecks Series (all 2208 1000kv, 11x3.7" 2-blade, 4S)
- [ ] **EcksTuned.af** — 910g AUW, 220mm frame
- [ ] **Ecks_220mm_Tuned.af** — same as above
- [ ] **Ecks_800g_Moderate.af** — 800g AUW, 220mm frame  
- [ ] **Ecks_800g_Sport.af** — 800g AUW, 220mm frame
- [ ] **Ecks_1kg_Moderate.af** — 1kg AUW, BR2205 2300kv + Cyclone 5x3x3
- [ ] **Ecks_1kg_Sport.af** — 1kg AUW, BR2205 2300kv + Cyclone 5x3x3
- [ ] **EcksQuatQ.af** — quaternion retune

### Ken's Models
- [ ] **Ken_s_Alpha_Test_Tuned.af** — LOW_VOLT_THRES=13.8V (4S?), 450mm frame?
- [ ] **Ken_s_450_1165_Tuned.af** — 450-1165 frame, what motors/props?
- [ ] **Ken_s_LadyBug_Tuned.af** — WLToys brushed 1S, what frame/motors?

### Other
- [ ] **DevEBox_Tuned.af** — what hardware?
- [ ] **S500_1137_Tuned.af** — S500 frame, what motors/props?
- [ ] **150mm_Brushed_Tuned.af** — 150mm brushed, 1S/2S?
- [ ] **Rok_s_Quad_Tuned.af** — AF_TYPE=QUAD (not QUAD_X), LOW_VOLT_THRES=10V (3S?), BATTERY_CAPACITY=50 (5000mAh?), ESC=FAST_PWM, what frame/motors/props?

---

## How to Fill

1. **Mass**: Weigh with battery installed (AUW)
2. **Frame**: Measure motor-to-motor diagonal
3. **Motor**: Check label or invoice for KV
4. **Prop**: Check prop markings (e.g., "11x3.7" = 11" dia, 3.7" pitch)
5. **Battery**: Check label for S-count and mAh
6. **Arm length**: Half of frame diagonal (or measure center to motor)
7. **Inertia**: If unknown, use approximation:
   - `I_roll_pitch ≈ mass × (arm_len)² / 12`
   - `I_yaw ≈ 2.5 × I_roll_pitch`
8. **Max thrust**: If unknown, estimate from motor+prop data or use `(mass × 9.81) / (hover_throttle × num_motors)`

---

## Output Format

Please provide as:

```
EcksTuned.af:
  PHYS_MASS_KG = 0.91
  PHYS_FRAME_MM = 220
  PHYS_MOTOR_KV = 1000
  PHYS_PROP_DIAMETER_INCH = 11
  PHYS_PROP_PITCH_INCH = 3.7
  PHYS_PROP_BLADES = 2
  PHYS_CELL_COUNT = 4
  PHYS_BATTERY_MAH = 1500
  PHYS_ARM_LEN_M = 0.11
  PHYS_INERTIA_ROLL_PITCH = 0.002
  PHYS_INERTIA_YAW = 0.005
  PHYS_MAX_THRUST_N = 3.5
```

Save responses to: `uavx-python/src/airframes/SPECS_RESPONSES.md`