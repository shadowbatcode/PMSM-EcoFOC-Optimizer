function result = Exp04_ParameterPerturbation(p)
%EXP04_PARAMETERPERTURBATION Nominal and perturbed plant comparison.

nominal = localBaseScenario('parameter_perturbation_nominal', 'Experiment 4 nominal plant', p);
nominal.omega_ref = @(t) p.exp.constant.omega_ref_rpm * p.rpm_to_rad;
nominal.load_torque = @(t) p.exp.constant.T_L;
nominal.seed = 4;

perturbed = nominal;
perturbed.name = 'parameter_perturbation_perturbed';
perturbed.description = 'Experiment 4 perturbed plant';
perturbed.perturbed = true;
perturbed.seed = 5;

result.nominal = localRunMethods(nominal, p);
result.perturbed = localRunMethods(perturbed, p);
result.scenario = perturbed;
save(fullfile(p.paths.data, 'exp04_parameter_perturbation.mat'), 'result');
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
