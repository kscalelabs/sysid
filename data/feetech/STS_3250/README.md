## Actuator Error Gain Measurement

To accurately simulate actuator behavior, an `error_gain` parameter was measured. This parameter helps calculate the motor duty cycle during simulation using the following control law:

```duty cycle = K_P * error_gain * position_error ```

### Measurement Setup
- Motor leads decoupled
- Control gains: \( K_P = 32 \), \( K_I = 0 \), \( K_D = 0 \)
- Step inputs of varying magnitude applied to measure the control response


### **Measured Data**

| Error (rad) | Error Gain |
|-------------|------------|
| 0.1381      | 0.16382926 |
| 0.1457      | 0.16390694 |
| 0.1534      | 0.16382926 |
| 0.1611      | 0.03915222 |
| 0.1703      | 0.03704475 |

**Selected Error Gain**: **0.16382926** (chosen based on stable region before duty cycle drop-off due to torque limit protection)

---

### **Visual Representation**

```
Error (rad) | Error Gain
------------|-----------------------------------------
0.1381      | ████████████████████████   0.1638
0.1457      | ████████████████████████   0.1639
0.1534      | ████████████████████████   0.1638
0.1611      | ██                         0.0392
0.1703      | ██                         0.0370
```
```

Error Gain as a function of Position Error


# Dataset: STS_3250_V12_20250317

## Testbench Setup: Pendulum
- **Actuator**: STS3215 (12V)
- **Test Date**: 2025-03-17
- **Sampling Rate**: 100 Hz (interpolated to 200 Hz)
- **Total Experimental Runs**: 960
- **Arm Lengths**: 100 mm, 150 mm
- **Point Masses**: 0.535 kg, 0.741 kg, 0.969 kg, 1.168 kg
- **KP Gains**: 4, 8, 12, 16, 24, 32
- **Trajectory Types**:
  - Brutal (Large Step Input)
  - SinSin
  - SinTimeSquared
  - Up and Down
  - Lift and Drop
- **Repetitions**: 4

## Data Processing Notes
- Data was linearly interpolated from 100 Hz to 200 Hz.
- Specific segments with artifacts, oscillations, or dead zones were trimmed (see details per fitting).

---

## Model Fitting Results

### LuGre Model (Full Friction Dynamics, Model M6)

| Fit ID | Best Score | Notes | Plot Book |
|--------|------------|-------|-----------|
| id001  | 0.0383    | Initial fit, all data included | sts3250-12v_m6_id001.pdf |
| id002  | 0.0321    | Removed Brutal trajectory (0–1s, 4–6s) | sts3250-12v_m6_id002.pdf |
| id003  | 0.0311    | Removed Brutal (0–1s, 4–6s), Lift and Drop (4–6s), spurious spikes | sts3250-12v_m6_id003.pdf |
| id004  | **0.0299** | Final cleaned dataset, removed backlash/oscillation and regions with dead dynamics, consolidated repetitions | sts3250-12v_m6_id004.pdf |

---

### **Final Parameters (M6, id004)**
```yaml
kt: 2.6418
R: 3.4002
armature: 0.0080
q_offset: 0.0059
friction_base: ~0.0000002
friction_stribeck: 0.1210
load_friction_motor: ~0.0000004
load_friction_external: 0.3835
load_friction_motor_stribeck: 0.5504
load_friction_external_stribeck: ~0.0000033
load_friction_motor_quad: 0.0067
load_friction_external_quad: 0.0027
dtheta_stribeck: 0.0226
alpha: 2.9592
friction_viscous: 0.0122

### Alternative Models

| Model | Best Score | Parameters | Notes |
|-------|------------|------------|-------|
| M1    | 0.0329    | [m1.json](friction_params/m1.json) | Consolidated dataset (final) |
| M2    | 0.0328    | [m2.json](friction_params/m2.json) | Consolidated dataset (final) |
| M3    | 0.0307    | [m3.json](friction_params/m3.json) | Consolidated dataset (final) |
| M4    | 0.0304    | [m4.json](friction_params/m4.json) | Consolidated dataset (final) |
| M5    | 0.0303    | [m5.json](friction_params/m5.json) | Consolidated dataset (final) |



### Important Notes on Data Trimming
- Post Process, 100hz data to 200hz (with linear interpolation)
- Cut Brutal Trajectory (0-1sec,  4-6sec)
- Cut Lift and Drop (4-6 sec)
- Removed data with spurious spikes
- Cut Lift and Drop w/ 100mm arm length (3.5-4sec)
- Removed a few high KP sin time squared tests that failed (motor stalled)
- Cut Brutal (0-0.4sec)
- Cut Lift and Drop (3.5-4sec)
- Opened up kT search window (max=10 was 2.5) ** noticed it was hitting up against ceiling during previous fit attempts
- Modified Actautor Motor resistance init guess to 1.2ohm
- Removed Lift and Drop run for length=0.15 mass=0.74117694 (This was performed during a mechanical binding issue with servo horn, i think this is not good representation of dynamics)
- Consolidate to 2 reps per conditions
---

### Exported Parameters to be used in simulation
###### M1
Armature: 0.014376833235471145
Force range: 7.317806472535506
Kp: 38.36403950065574
Friction loss: 1.422802604279253e-13
Damping: 1.660372517014714

###### M2
Armature: 0.01427243315073713
Force range: 7.520925708250064
Kp: 39.42890428104346
Friction loss: 4.5279444031997677e-13
Damping: 1.7071590603359401

###### M3
Armature: 0.008511870872501204
Force range: 14.375862248967877
Kp: 75.36632039194905
Friction loss: 1.3892830868095233e-09
Damping: 3.135666479095258

###### M4
Armature: 0.008188877355288296
Force range: 14.54456567883787
Kp: 76.25075824524826
Friction loss: 2.747347248830658e-10
Damping: 3.165434726043113

###### M5
Armature: 0.008458935728514421
Force range: 9.209377124006823
Kp: 48.2807121352332
Friction loss: 1.4223176767344103e-06
Damping: 2.0340511826552032

###### M6
Armature: 0.008049433273002293
Force range: 9.400899271795291
Kp: 49.28477848634313
Friction loss: 1.687509618663902e-07
Damping: 2.0646938310069753

## File Structure

```
sysid-results/
├── data/
│   └── feetech/
│       └── STS_3215_12V/
│           ├── experiment_configs/
│           ├── raw/
│           ├── processed/
│           ├── plots/
│           ├── friction_params/
│           │   ├── m1.json
│           │   ├── m2.json
│           │   ├── m3.json
│           │   ├── m4.json
│           │   ├── m5.json
│           │   └── m6.json
│           ├── README.md
│           └── NOTES.txt
├── scripts/
├── simulation_examples/
└── docs/
```

---

**See individual JSON parameter files in `friction_params/` for fitted friction and dynamic parameters per model.**

