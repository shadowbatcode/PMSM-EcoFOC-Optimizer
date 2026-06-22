# Technical Audit Report

Generated for GitHub packaging on 2026-06-15 and refreshed for MATLAB/Simulink
R2025b on 2026-06-22.

## Project identity

Recommended repository name:

```text
pmsm-ecofoc-optimizer
```

Display name:

```text
PMSM EcoFOC Optimizer
```

The repository presents a MATLAB/Simulink implementation of PMSM field-oriented
control with a model-free d-axis current optimizer for efficiency-oriented
operation.

## Canonical implementation

The reproducible implementation is organized around these files:

- `scripts/init_parameters.m`: centralized plant, controller, optimizer,
  scenario, metric, and path parameters.
- `scripts/build_models.m`: regenerates executable Simulink wrapper models.
- `scripts/run_all.m`: runs all study cases, calculates metrics, exports
  figures, and writes the final run status.
- `scripts/run_case_set.m`: runs baseline and optimization models through
  Simulink and converts `timeseries` outputs into structured run logs.
- `scripts/calculate_metrics.m`: computes overshoot, rise time, settling time,
  steady-state error, input power, current magnitude, efficiency, convergence
  time, projection samples, freeze samples, and relative Pin reduction.
- `src/pmsm_foc_step.m`: shared fixed-step PMSM FOC engine.
- `src/pmsm_foc_sfunc.m`: Level-2 MATLAB S-Function bridge used by Simulink.
- `src/pmsm_foc_output_names.m`: ordered signal contract between S-Function,
  Simulink demux, and result extraction scripts.

## Executable Simulink assets

The canonical models are:

- `models/PMSM_FOC_Baseline.slx`
- `models/PMSM_FOC_Optimization.slx`

Both models are executable wrappers around `src/pmsm_foc_sfunc.m`. The wrapper
models expose named output signals through `To Workspace` blocks and use a
fixed-step discrete solver with `Ts = 1.0e-4`.

The archived `_legacy_from_copied_project/` tree is preserved for traceability
only. It should not be treated as the canonical implementation.

## Implemented control details

The numerical core implements:

- mechanical speed-loop PID producing `iq_ref`;
- d/q-axis current-loop PI control with decoupling feedforward terms;
- PMSM dq electrical dynamics and mechanical rotor dynamics;
- input power, mechanical output power, and efficiency estimation;
- current magnitude limiting and voltage vector saturation handling;
- anti-windup behavior for speed and current integrators;
- optimizer enable gating using speed, current, power-slope, and saturation
  conditions;
- sinusoidal d-axis current perturbation;
- power high-pass behavior through input-power low-pass subtraction;
- demodulated gradient estimation and low-pass filtering;
- projected d-axis reference update with box, current, and voltage safeguards;
- slew-rate limiting, optimizer freeze, and restart ramp logic.

## Reproduced cases

The packaged project includes these study cases:

- `speed_step`
- `efficiency_optimization`
- `load_step`
- `parameter_perturbation`
- `constraint_test`

Each case compares the baseline `i_d^* = 0` controller and the model-free
optimization controller.

## Reproduced metrics

The main generated table is `results/comparison_table.csv`. After R2025b
retuning, the steady-state and constrained cases reproduce the paper-scale
energy improvement while the speed transient remains essentially unchanged:

| Case | Baseline Pin W | Optimized Pin W | Pin reduction |
|---|---:|---:|---:|
| speed_step | 362.2681 | 362.0360 | 0.0641% |
| efficiency_optimization | 272.0886 | 271.8278 | 0.0959% |
| load_step | 713.7116 | 708.2181 | 0.7696% |
| parameter_perturbed | 278.2831 | 278.4980 | -0.0772% |
| constraint_test | 1295.6536 | 1294.3343 | 0.1018% |

The project should therefore be presented as an engineering reproduction and
algorithm pipeline implementation, not as hardware-validated evidence. The
parameter-perturbed case documents a robustness boundary where the optimizer can
lose the small nominal advantage under artificial plant mismatch.

## GitHub packaging notes

Recommended files for public presentation are now present:

- polished `README.md`;
- architecture notes in `docs/ARCHITECTURE.md`;
- reproducibility notes in `docs/REPRODUCIBILITY.md`;
- project package verification in `tests/Verify_Project_Package.ps1`;
- `.gitignore` for MATLAB/Simulink generated artifacts;
- `.gitattributes` for binary MATLAB/Simulink files.

The repository is ready to initialize with:

```powershell
git init
git add .
git commit -m "Package PMSM FOC optimizer project for GitHub"
```
