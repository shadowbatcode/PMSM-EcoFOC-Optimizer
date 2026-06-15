function allResults = Run_All_Experiments()
%RUN_ALL_EXPERIMENTS One-click runner for all PMSM FOC optimization cases.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'experiments'));
addpath(fullfile(projectRoot, 'simulink'));

p = Parameter_Set();
ensureOutputDirs(p);

fprintf('Running PMSM FOC optimization simulations...\n');
allResults.constant_load = Exp01_ConstantLoad(p);
allResults.load_step = Exp02_LoadStep(p);
allResults.speed_step = Exp03_SpeedStep(p);
allResults.parameter_perturbation = Exp04_ParameterPerturbation(p);
allResults.noise_delay = Exp05_NoiseDelay(p);
allResults.constraints = Exp06_Constraints(p);

metrics = Compute_Metrics(allResults, p);
Plot_Results(allResults, metrics, p);
Generate_Report_Text(allResults, metrics, p);

save(fullfile(p.paths.data, 'all_results.mat'), 'allResults', 'metrics', 'p');
try
    Generate_Simulink_Model(p);
catch ME
    warning('PMSM:SimulinkGenerationFailed', ...
        'Simulink model generation was skipped: %s', ME.message);
end
fprintf('Done. Results saved under: %s\n', p.paths.results);
end

function ensureOutputDirs(p)
dirs = {p.paths.results, p.paths.data, p.paths.figures, p.paths.tables, p.paths.simulink};
for i = 1:numel(dirs)
    if ~exist(dirs{i}, 'dir')
        mkdir(dirs{i});
    end
end
end
