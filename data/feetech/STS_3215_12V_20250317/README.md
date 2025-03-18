# Dataset: STS_3215_V12_20250317

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
| id001 | 0.0415 | All data included | sts3215-12v_m6_id001.pdf |
| id002 | 0.0342 | Removed Brutal trajectory (0–1 s, 4–6 s) | sts3215-12v_m6_id002.pdf |
| id003 | 0.0331 | Removed Brutal (0–1 s, 4–6 s), Lift and Drop (4–6 s), spurious spikes | sts3215-12v_m6_id003.pdf |
| id004 | **0.0309** | Final cleaned dataset, removed backlash/oscillation and regions with dead dynamics, consolidated repetitions | sts3215-12v_m6_id004.pdf |

**Final Parameters (M6, id004)**:
```yaml
kt: 3.5555
R: 7.1214
armature: 0.0283
q_offset: -0.0140
friction_base: ~0.00006
friction_stribeck: 0.0840
load_friction_motor: 0.2892
load_friction_external: ~0.00002
load_friction_motor_stribeck: 0.2985
load_friction_external_stribeck: ~0.00004
load_friction_motor_quad: 0.0089
load_friction_external_quad: 0.0021
dtheta_stribeck: 0.0748
alpha: 1.2952
friction_viscous: 0.0762
```

### Alternative Models

| Model | Best Score | Parameters | Notes |
|-------|------------|------------|-------|
| M1 | 0.0323 | [m1.json](params/m1.json) | Consolidated dataset (final) |
| M2 | 0.0316 | [m2.json](params/m2.json) | Consolidated dataset (final) |
| M3 | 0.0320 | [m3.json](params/m3.json) | Consolidated dataset (final) |
| M4 | 0.0313 | [m4.json](params/m4.json) | Consolidated dataset (final) |
| M5 | 0.0309 | [m5.json](params/m5.json) | Consolidated dataset (final) |

### Important Notes on Data Trimming
- **Brutal trajectory segments** removed due to backlash and oscillations at high KP values.
- **Lift and Drop segments** (especially at 100 mm arm length) removed due to non-representative dynamics.
- **Spurious spikes** were identified and excluded from fitting.
- Final consolidation reduced repetitions to 2 per condition, with the remaining data reserved for validation.

---

### Exported Parameters to be used in simulation
**M1**
- Armature: 0.027772754874253427
- Force range: 4.445701494260475
- Kp: 23.45765502043551
- Friction loss: 0.05283175681661876
- Damping: 1.38430752168554

**M2**
- Armature: 0.027688893594746772
- Force range: 4.494839880624263
- Kp: 23.71693273331631
- Friction loss: 0.006867898305963405
- Damping: 1.4244308409107151

**M3**
- Armature: 0.026955301983553615
- Force range: 4.825533578397589
- Kp: 25.46183141574331
- Friction loss: 0.04362715825639339
- Damping: 1.4894734131361451

**M4**
- Armature: 0.026562450412688003
- Force range: 5.017467716550088
- Kp: 26.474568057022207
- Friction loss: 0.02343128488450586
- Damping: 1.5728336013078086

**M5**
- Armature: 0.02825455347700246
- Force range: 5.693006405710866
- Kp: 30.039034439605274
- Friction loss: 0.00032609339427433384
- Damping: 1.7523864548500465

**M6**
- Armature: 0.02826032533017913
- Force range: 6.04123795927937
- Kp: 31.876471267378417
- Friction loss: 6.042148883943358e-05
- Damping: 1.8513880353731311

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

