# Simulink Notes

This folder contains the inspected paper architecture model:

- `PMSM_FOC_Model.slx`
- `PMSM_FOC_Model_Chapter3.slx`

The inspected model shows the paper block layout, but the internal subsystems are mostly port shells. The executable control logic used by this repository lives in `src/pmsm_foc_step.m` and `src/pmsm_foc_simulate.m`.

`scripts/build_models.m` copies the inspected architecture model into `models/` as reproducibility artifacts.
