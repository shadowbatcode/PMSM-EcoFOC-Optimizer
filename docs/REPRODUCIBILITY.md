# Reproducibility

This project is packaged so the Simulink models, numerical results, comparison
tables, and figures can be regenerated from source scripts.

## Environment

Validated target environment:

- MATLAB R2021a
- Simulink R2021a
- Windows PowerShell

The project uses Level-2 MATLAB S-Functions, Simulink fixed-step discrete
simulation, MAT files, CSV exports, and MATLAB figure export.

## Full reproduction

From MATLAB:

```matlab
cd PMSM_FOC_Optimization_Project
run('scripts/init_parameters.m')
run('scripts/build_models.m')
run('scripts/run_all.m')
```

Expected output files:

```text
models/PMSM_FOC_Baseline.slx
models/PMSM_FOC_Optimization.slx
results/comparison_table.csv
results/comparison_table.mat
results/data/all_results.mat
results/logs/last_run_status.txt
figures/fig_speed_step.png
figures/fig_efficiency_optimization.png
figures/fig_load_step.png
figures/fig_parameter_perturbation.png
figures/fig_constraint_test.png
```

## Package verification

From the project root:

```powershell
powershell -ExecutionPolicy Bypass -File tests/Verify_Project_Package.ps1
```

This check verifies:

- required source, script, model, result, and figure files exist;
- required figures and tables are non-empty;
- Simulink cache directories/files are not mixed into the committed tree;
- README documents the legacy archive boundary;
- core algorithm keywords are present in `src/pmsm_foc_step.m`;
- `run_all.m` calls all required scenario scripts.

## Current result reference

The current checked-in `results/comparison_table.csv` contains these reproduced
steady-state power comparisons:

| Case | Method | Mean Pin W | Efficiency % | Overshoot % |
|---|---|---:|---:|---:|
| speed_step | baseline | 362.2681 | 96.3761 | 3.9658 |
| speed_step | optimization | 362.2604 | 96.3782 | 3.9654 |
| efficiency_optimization | baseline | 272.0886 | 95.2716 | 3.3601 |
| efficiency_optimization | optimization | 272.0851 | 95.2728 | 3.3601 |
| load_step | baseline | 713.7116 | 89.1416 | 3.3601 |
| load_step | optimization | 713.6817 | 89.1454 | 3.3601 |
| parameter_perturbed | baseline | 278.2831 | 93.1508 | 3.9481 |
| parameter_perturbed | optimization | 278.2813 | 93.1514 | 3.9481 |
| constraint_test | baseline | 1295.6536 | 90.9545 | 0.5129 |
| constraint_test | optimization | 1295.6397 | 90.9555 | 0.5129 |

Floating-point results may vary slightly across MATLAB/Simulink versions.

## Artifact policy

Commit:

- source code in `src/` and `scripts/`;
- canonical Simulink models in `models/`;
- representative figures and result tables;
- documentation and verification scripts.

Do not commit generated cache/build artifacts such as:

- `slprj/`
- `*.slxc`
- `*_ert_rtw/`
- `*_grt_rtw/`
- `*.asv`

These are covered by `.gitignore`.
