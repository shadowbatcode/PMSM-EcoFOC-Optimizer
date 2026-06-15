function result = Exp05_NoiseDelay(p)
%EXP05_NOISEDELAY Measurement noise, delay, and DC-link ripple case.

clean = localBaseScenario('noise_delay_clean', 'Experiment 5 clean reference case', p);
clean.omega_ref = @(t) p.exp.constant.omega_ref_rpm * p.rpm_to_rad;
clean.load_torque = @(t) p.exp.constant.T_L;
clean.seed = 6;

nonideal = clean;
nonideal.name = 'noise_delay_nonideal';
nonideal.description = 'Experiment 5 nonideal measurement and inverter case';
nonideal.nonideal = true;
nonideal.seed = 7;

result.clean = localRunMethods(clean, p);
result.nonideal = localRunMethods(nonideal, p);
result.scenario = nonideal;
save(fullfile(p.paths.data, 'exp05_noise_delay.mat'), 'result');
end

function result = localRunMethods(scenario, p)
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    result.(method) = Simulate_PMSM_FOC(method, scenario, p);
end
result.scenario = scenario;
end

function scenario = localBaseScenario(name, description, p)
scenario.name = name;
scenario.description = description;
scenario.Tsim = p.Tsim;
scenario.Ts = p.Ts;
scenario.perturbed = false;
scenario.nonideal = false;
end
