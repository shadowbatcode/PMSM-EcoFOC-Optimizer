if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end

baselineModel = fullfile(p.paths.models, 'PMSM_FOC_Baseline.slx');
optimizationModel = fullfile(p.paths.models, 'PMSM_FOC_Optimization.slx');
if exist(baselineModel, 'file') ~= 2 || exist(optimizationModel, 'file') ~= 2
    run(fullfile(p.paths.scripts, 'build_models.m'));
end

run(fullfile(p.paths.scripts, 'run_speed_step.m'));
allResults.speed_step = result;

run(fullfile(p.paths.scripts, 'run_efficiency_optimization.m'));
allResults.efficiency_optimization = result;

run(fullfile(p.paths.scripts, 'run_load_step.m'));
allResults.load_step = result;

run(fullfile(p.paths.scripts, 'run_parameter_perturbation.m'));
allResults.parameter_perturbation = result;

run(fullfile(p.paths.scripts, 'run_constraint_test.m'));
allResults.constraint_test = result;

run(fullfile(p.paths.scripts, 'calculate_metrics.m'));

plot_required_figures(allResults, metrics, p);
save(fullfile(p.paths.data, 'all_results.mat'), 'allResults', 'metrics', 'p');
cleanup_simulink_artifacts(p);
fid = fopen(fullfile(p.paths.logs, 'last_run_status.txt'), 'w');
if fid >= 0
    fprintf(fid, 'run_all completed at %s\n', datestr(now));
    fclose(fid);
end
