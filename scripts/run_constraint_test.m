if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end
scenario.name = 'constraint_test';
scenario.Tstop = p.Tstop;
scenario.omega_ref = @(t) p.exp.constraint.omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.exp.constraint.TL;
result = run_case_set(scenario, p, 'constraint_test');
save(fullfile(p.paths.data, 'constraint_test.mat'), 'result', 'scenario', 'p');
