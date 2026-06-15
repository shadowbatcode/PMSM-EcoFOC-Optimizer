if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end
scenario.name = 'efficiency_optimization';
scenario.Tstop = p.Tstop;
scenario.omega_ref = @(t) p.exp.constant.omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.exp.constant.TL;
result = run_case_set(scenario, p, 'efficiency_optimization');
save(fullfile(p.paths.data, 'efficiency_optimization.mat'), 'result', 'scenario', 'p');
