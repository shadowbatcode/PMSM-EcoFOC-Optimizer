# Architecture

This document explains how the project is split between MATLAB scripts,
Simulink wrapper models, and the shared PMSM FOC simulation engine.

## System decomposition

```text
scripts/init_parameters.m
        |
        v
scripts/build_models.m -----> models/*.slx
        |                         |
        |                         v
        |                  src/pmsm_foc_sfunc.m
        |                         |
        v                         v
scripts/run_all.m --------> src/pmsm_foc_step.m
        |
        v
results/*.csv, results/*.mat, figures_chapter3/*.png
```

The key design choice is to keep the controller and plant math in one shared
function, `src/pmsm_foc_step.m`. MATLAB scripts and Simulink runs therefore use
the same numerical implementation instead of maintaining two drifting versions
of the algorithm.

## Fixed-step simulation core

`src/pmsm_foc_step.m` advances one sample of the complete drive system:

1. Speed error is converted to `iq_ref` by the speed-loop PID.
2. The selected method sets `id_ref`:
   - baseline: `id_ref = 0`;
   - optimization: model-free extremum-seeking update plus perturbation.
3. Current and voltage constraints project the reference into a feasible range.
4. d/q current PI controllers compute `u_d` and `u_q`.
5. PMSM dq current dynamics and rotor mechanical dynamics are integrated.
6. Power, efficiency, optimizer state, saturation flags, and diagnostic signals
   are emitted for logging.

The fixed step is configured in `scripts/init_parameters.m`:

```matlab
p.Ts = 1.0e-4;
p.Tstop = 4.0;
```

## Model-free optimizer

The optimizer searches for a lower-input-power d-axis current reference without
requiring a loss-map model.

Main signals:

- `id_bar`: slowly updated d-axis current operating point;
- `perturbation`: sinusoidal excitation added around `id_bar`;
- `Pin`: instantaneous electrical input power estimate;
- `Pin_lpf`: low-pass input-power estimate;
- `P_ac`: high-frequency power component used for demodulation;
- `demod_signal`: power component multiplied by the perturbation carrier;
- `g_hat`: low-pass gradient estimate;
- `optimizer_enable`: true only when steady-state conditions are satisfied;
- `projection_active`: true when current/voltage/box constraints intervene;
- `freeze_state`: true while the optimizer is paused during transient behavior.

Update path:

```text
Pin -> low-pass subtraction -> demodulation -> gradient low-pass
    -> constrained negative-gradient update -> slew-limited id_bar
```

The implementation intentionally gates optimizer updates during speed
transients, load changes, and saturation. This prevents the extremum-seeking
loop from interpreting normal drive dynamics as an efficiency gradient.

## Simulink wrapper models

`scripts/build_models.m` generates:

- `models/PMSM_FOC_Baseline.slx`
- `models/PMSM_FOC_Optimization.slx`

Each model contains:

- one Level-2 MATLAB S-Function block named `PMSM_Engine`;
- one demux block that splits the vector output;
- named `To Workspace` blocks for every exported signal;
- annotations documenting solver settings, plant parameters, and method tag.

The wrapper models are executable and intentionally compact. They are not meant
to duplicate every equation as individual Simulink primitive blocks; instead,
they provide a reproducible Simulink execution boundary around the tested
MATLAB control engine.

`scripts/build_component_models.m` additionally generates R2025b visible
component models:

- `models/PMSM_FOC_Component_Baseline.slx`
- `models/PMSM_FOC_Component_Optimization.slx`

These models implement the drive as separate Simulink MATLAB Function
components with explicit Unit Delay state feedback:

```text
omega_ref -> Speed_Controller_PID -> Safety_Projection
                                  -> Current_Controller_PI
                                  -> PMSM_dq_Plant
                                  -> Power_Efficiency_Monitor
             ModelFree_Optimizer --^
```

The component models are intended for inspection, teaching, and paper figures.
They contain no top-level S-Function block and can be run with
`scripts/run_component_efficiency_case.m`.

`scripts/beautify_component_models.m` applies the paper-facing layout: separated
fast vector-control and outer optimization layers, colored component groups,
named signals, explicit Unit Delay state banks, and paper-aligned logged signal
sinks. `scripts/generate_paper_aligned_outputs.m` exports the Chapter 3 figure
set and result-comparison tables from the beautified component model.

## Signal contract

The S-Function emits the ordered signal vector defined in
`src/pmsm_foc_output_names.m`:

```text
omega_m, omega_rpm, id, iq, id_ref, iq_ref, Pin, Pout, eta,
g_hat, id_bar, steady_flag, optimizer_enable, freeze_state,
projection_active, current_saturated, voltage_saturated, Te, us, is
```

`scripts/run_case_set.m` expects Simulink output variables named
`<signal>_sim`, converts them from `timeseries`, and adds compatibility fields
used by the metric and plotting scripts.

## Scenario and metric pipeline

Scenario scripts define time-varying speed references and load torque profiles.
`scripts/run_all.m` executes the scenario set, calls
`scripts/calculate_metrics.m`, and then uses `scripts/plot_required_figures.m`
to export figures.

Generated artifacts:

- `results/comparison_table.csv`
- `results/comparison_table.mat`
- `results/data/all_results.mat`
- `figures_chapter3/*.png`

## Design boundaries

The project focuses on simulation, reproducibility, and algorithm structure.
It does not include:

- hardware-in-the-loop verification;
- inverter switching detail;
- thermal modeling;
- high-speed field-weakening validation;
- production embedded-code generation.
