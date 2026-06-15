if ~exist('allResults', 'var') || ~exist('p', 'var')
    error('calculate_metrics.m requires allResults and p in the workspace.');
end

metrics = struct();
metrics.speed_step = calc_case_metrics(allResults.speed_step, p, 'speed_step');
metrics.efficiency_optimization = calc_case_metrics(allResults.efficiency_optimization, p, 'efficiency_optimization');
metrics.load_step = calc_case_metrics(allResults.load_step, p, 'load_step');
metrics.parameter_perturbation.nominal = calc_case_metrics(allResults.parameter_perturbation.nominal, p, 'parameter_nominal');
metrics.parameter_perturbation.perturbed = calc_case_metrics(allResults.parameter_perturbation.perturbed, p, 'parameter_perturbed');
metrics.constraint_test = calc_case_metrics(allResults.constraint_test, p, 'constraint_test');

comparison_table = [metrics.speed_step; metrics.efficiency_optimization; metrics.load_step; ...
    metrics.parameter_perturbation.nominal; metrics.parameter_perturbation.perturbed; metrics.constraint_test];

baseline_idx = strcmp(comparison_table.method, 'baseline');
for caseIdx = 1:height(comparison_table)
    same_case = strcmp(comparison_table.case_name, comparison_table.case_name(caseIdx)) & baseline_idx;
    if any(same_case)
        basePin = comparison_table.mean_Pin_W(find(same_case, 1));
        comparison_table.pin_reduction_vs_id0_pct(caseIdx) = 100 * (basePin - comparison_table.mean_Pin_W(caseIdx)) / max(abs(basePin), eps);
    end
end

writetable(comparison_table, fullfile(p.paths.results, 'comparison_table.csv'));
writetable(comparison_table, fullfile(p.paths.tables, 'comparison_table.csv'));
save(fullfile(p.paths.results, 'comparison_table.mat'), 'metrics', 'comparison_table', 'p');

function rows = calc_case_metrics(group, p, case_name)
rows = table();
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    run = group.(method);
    rows = [rows; calc_run_metrics(run, p, case_name, method)]; %#ok<AGROW>
end
end

function row = calc_run_metrics(run, p, case_name, method)
tail = run.t >= run.t(end) * (1 - p.metric_tail_fraction);
omega_final = mean(run.omega_ref(tail)) * 60/(2*pi);
omega_rpm_tail = run.omega_rpm(tail);
overshoot_pct = max(0, (max(run.omega_rpm) - omega_final) / max(abs(omega_final), eps) * 100);
rise_time_s = estimate_rise_time(run, omega_final);
settling_time_s = estimate_settling_time(run, omega_final, 0.02);
steady_error_rpm = mean(omega_final - omega_rpm_tail);
recovery_time_s = estimate_load_recovery(run, p);
convergence_time_s = estimate_optimizer_convergence(run, method, p);
mechanical_output_W = mean(run.Pout(tail));
efficiency_pct = mean(run.eta(tail)) * 100;

row = table(string(case_name), string(method), overshoot_pct, rise_time_s, settling_time_s, ...
    steady_error_rpm, mean(run.Pin(tail)), mean(run.is(tail)), mechanical_output_W, ...
    efficiency_pct, max(abs(run.iq)), max(abs(run.Te)), recovery_time_s, convergence_time_s, ...
    sqrt(mean(run.g_hat.^2)), sum(run.projection_active > 0), sum(run.freeze_state > 0), NaN, ...
    'VariableNames', {'case_name','method','overshoot_pct','rise_time_s','settling_time_s', ...
    'steady_error_rpm','mean_Pin_W','mean_is_A','mean_Pout_W','efficiency_pct', ...
    'peak_iq_A','peak_Te_Nm','load_recovery_time_s','optimizer_convergence_time_s', ...
    'gradient_rms','projection_samples','freeze_samples','pin_reduction_vs_id0_pct'});
end

function tr = estimate_rise_time(run, omega_final)
if omega_final <= 0
    tr = NaN;
    return;
end
y = run.omega_rpm;
t = run.t;
i10 = find(y >= 0.1 * omega_final, 1, 'first');
i90 = find(y >= 0.9 * omega_final, 1, 'first');
if isempty(i10) || isempty(i90)
    tr = NaN;
else
    tr = t(i90) - t(i10);
end
end

function ts = estimate_settling_time(run, omega_final, tol)
band = max(abs(omega_final) * tol, 1e-6);
inside = abs(run.omega_rpm - omega_final) <= band;
suffix = flipud(cumprod(flipud(double(inside))) > 0);
idx = find(suffix, 1, 'first');
if isempty(idx)
    ts = NaN;
else
    ts = run.t(idx);
end
end

function tr = estimate_load_recovery(run, p)
idx = find(abs(diff(run.T_L)) > 1e-12, 1, 'first');
if isempty(idx)
    tr = NaN;
    return;
end
idx = idx + 1;
target = run.omega_ref(end) * 60/(2*pi);
band = 0.02 * max(abs(target), 1);
post = find(abs(run.omega_rpm(idx:end) - target) <= band, 1, 'first');
if isempty(post)
    tr = NaN;
else
    tr = run.t(idx + post - 1) - run.t(idx);
end
end

function tc = estimate_optimizer_convergence(run, method, p)
if ~strcmp(method, 'optimization')
    tc = NaN;
    return;
end
tail = run.t >= run.t(end) * (1 - p.metric_tail_fraction);
target = mean(run.id_bar(tail));
band = max(0.05, 0.05 * abs(target));
inside = abs(run.id_bar - target) <= band;
suffix = flipud(cumprod(flipud(double(inside))) > 0);
idx = find(suffix & run.optimizer_enable > 0, 1, 'first');
if isempty(idx)
    tc = NaN;
else
    tc = run.t(idx);
end
end
