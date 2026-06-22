param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"

function Assert-Exists {
    param([string]$Path, [string]$Message)
    if (-not (Test-Path -LiteralPath $Path)) {
        throw $Message
    }
}

function Assert-NotExists {
    param([string]$Path, [string]$Message)
    if (Test-Path -LiteralPath $Path) {
        throw $Message
    }
}

function Assert-NonEmptyFile {
    param([string]$Path, [string]$Message, [int]$MinBytes = 1024)
    Assert-Exists $Path $Message
    $item = Get-Item -LiteralPath $Path
    if ($item.Length -lt $MinBytes) {
        throw "$Message File is too small: $Path"
    }
}

Assert-Exists $ProjectRoot "Project root not found: $ProjectRoot"

$requiredFiles = @(
    "README.md",
    "audit_report.md",
    "docs/ARCHITECTURE.md",
    "docs/REPRODUCIBILITY.md",
    "models/PMSM_FOC_Baseline.slx",
    "models/PMSM_FOC_Optimization.slx",
    "models/PMSM_FOC_Component_Baseline.slx",
    "models/PMSM_FOC_Component_Optimization.slx",
    "scripts/init_parameters.m",
    "scripts/build_models.m",
    "scripts/build_component_models.m",
    "scripts/beautify_component_models.m",
    "scripts/generate_paper_aligned_outputs.m",
    "scripts/run_all.m",
    "scripts/run_case_set.m",
    "scripts/run_component_efficiency_case.m",
    "scripts/run_speed_step.m",
    "scripts/run_efficiency_optimization.m",
    "scripts/run_load_step.m",
    "scripts/run_parameter_perturbation.m",
    "scripts/run_constraint_test.m",
    "scripts/calculate_metrics.m",
    "scripts/plot_required_figures.m",
    "scripts/cleanup_simulink_artifacts.m",
    "src/pmsm_foc_step.m",
    "src/pmsm_foc_sfunc.m",
    "src/pmsm_foc_simulate.m",
    "src/pmsm_foc_output_names.m",
    "results/comparison_table.csv",
    "results/comparison_table.mat",
    "tables_chapter3/table3_1_simulation_parameters.csv",
    "tables_chapter3/table3_2_simulation_cases.csv",
    "tables_chapter3/table3_3_performance_comparison.csv",
    "tables_chapter3/document_result_comparison.md",
    "chapter3_insert_order.md"
)

foreach ($rel in $requiredFiles) {
    Assert-Exists (Join-Path $ProjectRoot $rel) "Required file missing: $rel"
}

$requiredFigures = @(
    "figures_chapter3/fig3_1_simulink_overall_model.png",
    "figures_chapter3/fig3_2_model_free_optimizer_subsystem.png",
    "figures_chapter3/fig3_3_speed_step_response.png",
    "figures_chapter3/fig3_4_power_id_convergence.png",
    "figures_chapter3/fig3_5_disturbance_response.png"
)

foreach ($rel in $requiredFigures) {
    Assert-NonEmptyFile (Join-Path $ProjectRoot $rel) "Required figure missing or empty: $rel"
}

Assert-NonEmptyFile (Join-Path $ProjectRoot "results/comparison_table.csv") "Comparison table CSV missing or empty." 256
Assert-NonEmptyFile (Join-Path $ProjectRoot "results/comparison_table.mat") "Comparison table MAT missing or empty." 1024

$forbiddenPaths = @(
    "slprj",
    "scripts/slprj",
    "figures",
    "results/figures",
    "experiments",
    "simulink"
)

foreach ($rel in $forbiddenPaths) {
    Assert-NotExists (Join-Path $ProjectRoot $rel) "Generated or legacy path should not be in canonical root: $rel"
}

$cacheFiles = @()
$cacheFiles += Get-ChildItem -LiteralPath $ProjectRoot -Filter "*.slxc" -File -ErrorAction SilentlyContinue
$cacheFiles += Get-ChildItem -LiteralPath (Join-Path $ProjectRoot "scripts") -Filter "*.slxc" -File -ErrorAction SilentlyContinue
if ($cacheFiles.Count -gt 0) {
    throw "Generated Simulink cache files found: $($cacheFiles.FullName -join ', ')"
}

$readme = Get-Content -LiteralPath (Join-Path $ProjectRoot "README.md") -Raw -Encoding UTF8
foreach ($needle in @("PMSM EcoFOC Optimizer", "figures_chapter3/fig3_1_simulink_overall_model.png", "R2025b", "generate_paper_aligned_outputs.m", "table3_3_performance_comparison.csv")) {
    if ($readme -notmatch [regex]::Escape($needle)) {
        throw "README missing expected text: $needle"
    }
}

$engine = Get-Content -LiteralPath (Join-Path $ProjectRoot "src/pmsm_foc_step.m") -Raw -Encoding UTF8
foreach ($needle in @("perturbation", "demod_signal", "g_hat", "projection_active", "freeze_state", "voltage_saturated", "current_saturated")) {
    if ($engine -notmatch [regex]::Escape($needle)) {
        throw "pmsm_foc_step.m missing expected algorithm text: $needle"
    }
}

$runAll = Get-Content -LiteralPath (Join-Path $ProjectRoot "scripts/run_all.m") -Raw -Encoding UTF8
foreach ($needle in @("run_speed_step", "run_efficiency_optimization", "run_load_step", "run_parameter_perturbation", "run_constraint_test", "calculate_metrics")) {
    if ($runAll -notmatch [regex]::Escape($needle)) {
        throw "run_all.m missing required call: $needle"
    }
}

Write-Host "Project package verification passed: $ProjectRoot"
