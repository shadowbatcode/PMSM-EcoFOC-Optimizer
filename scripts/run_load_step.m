if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end
scenario.name = 'load_step';
scenario.Tstop = p.Tstop;
scenario.omega_ref = @(t) p.exp.load_step.omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.exp.load_step.TL1 + (t >= p.exp.load_step.t_step) * (p.exp.load_step.TL2 - p.exp.load_step.TL1);
result = run_case_set(scenario, p, 'load_step');
save(fullfile(p.paths.data, 'load_step.mat'), 'result', 'scenario', 'p');
