# Tests

This directory contains lightweight verification scripts for the packaged
repository.

Run from the project root:

```powershell
powershell -ExecutionPolicy Bypass -File tests/Verify_Project_Package.ps1
```

The check is intentionally license-light: it validates the project package and
source structure without launching MATLAB. Full numerical reproduction still
requires MATLAB/Simulink:

```matlab
run('scripts/init_parameters.m')
run('scripts/build_models.m')
run('scripts/run_all.m')
```
