if ~exist('p', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_parameters.m'));
end
scenario.name = 'parameter_perturbation';
scenario.Tstop = p.Tstop;
scenario.omega_ref = @(t) p.exp.constant.omega_rpm * 2*pi/60;
scenario.load_torque = @(t) p.exp.constant.TL;
result.nominal = run_case_set(scenario, p, 'parameter_nominal');
p2 = p;
p2.Rs = p.Rs * p.exp.perturb.Rs_scale;
p2.psi_f = p.psi_f * p.exp.perturb.psi_f_scale;
p2.Ld = p.Ld * p.exp.perturb.Ld_scale;
p2.Lq = p.Lq * p.exp.perturb.Lq_scale;
result.perturbed = run_case_set(scenario, p2, 'parameter_perturbed');
save(fullfile(p.paths.data, 'parameter_perturbation.mat'), 'result', 'scenario', 'p');
