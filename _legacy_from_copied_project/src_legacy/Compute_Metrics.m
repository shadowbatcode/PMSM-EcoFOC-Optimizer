function metrics = Compute_Metrics(allResults, p)
%COMPUTE_METRICS Compute and save CSV metrics for all experiments.

metrics.constant_load = computeGroupMetrics(allResults.constant_load, p, 'constant_load', 'base');
writeRowsCsv(metrics.constant_load, fullfile(p.paths.tables, 'metrics_constant_load.csv'));

metrics.load_step = computeGroupMetrics(allResults.load_step, p, 'load_step', 'base');
writeRowsCsv(metrics.load_step, fullfile(p.paths.tables, 'metrics_load_step.csv'));

metrics.speed_step = computeGroupMetrics(allResults.speed_step, p, 'speed_step', 'base');
writeRowsCsv(metrics.speed_step, fullfile(p.paths.tables, 'metrics_speed_step.csv'));

rowsNominal = computeGroupMetrics(allResults.parameter_perturbation.nominal, p, 'parameter_perturbation', 'nominal');
rowsPerturbed = computeGroupMetrics(allResults.parameter_perturbation.perturbed, p, 'parameter_perturbation', 'perturbed');
metrics.parameter_perturbation = [rowsNominal; rowsPerturbed];
writeRowsCsv(metrics.parameter_perturbation, fullfile(p.paths.tables, 'metrics_parameter_perturbation.csv'));

rowsClean = computeGroupMetrics(allResults.noise_delay.clean, p, 'noise_delay', 'clean');
rowsNonideal = computeGroupMetrics(allResults.noise_delay.nonideal, p, 'noise_delay', 'nonideal');
metrics.noise_delay = [rowsClean; rowsNonideal];
writeRowsCsv(metrics.noise_delay, fullfile(p.paths.tables, 'metrics_noise_delay.csv'));

metrics.constraints = computeGroupMetrics(allResults.constraints, p, 'constraints', 'base');
writeRowsCsv(metrics.constraints, fullfile(p.paths.tables, 'metrics_constraints.csv'));
end

function rows = computeGroupMetrics(group, p, caseName, condition)
rows = repmat(emptyMetricRow(), 0, 1);
for i = 1:numel(p.method_order)
    method = p.method_order{i};
    rows(end+1, 1) = computeRunMetrics(group.(method), p, caseName, condition, method); %#ok<AGROW>
end

id0Pin = getMethodValue(rows, 'id0', 'mean_Pin_W');
mtpaPin = getMethodValue(rows, 'mtpa', 'mean_Pin_W');
for i = 1:numel(rows)
    if isfinite(id0Pin) && abs(id0Pin) > eps
        rows(i).pin_reduction_vs_id0_pct = 100 * (id0Pin - rows(i).mean_Pin_W) / id0Pin;
    end
    if isfinite(mtpaPin) && abs(mtpaPin) > eps
        rows(i).pin_diff_vs_mtpa_pct = 100 * (rows(i).mean_Pin_W - mtpaPin) / mtpaPin;
    end
end
end

function row = computeRunMetrics(run, p, caseName, condition, method)
row = emptyMetricRow();
row.case_name = caseName;
row.condition = condition;
row.method = method;

N = numel(run.t);
tailStart = max(1, floor((1 - p.metric_tail_fraction) * N));
tail = tailStart:N;

eventIdx = findLastReferenceChange(run.omega_ref);
if isempty(eventIdx)
    eventIdx = 1;
end

refFinal = mean(run.omega_ref(tail));
omegaFinal = mean(run.omega_m(tail));
row.overshoot_pct = max(0, max(run.omega_m(eventIdx:end) - refFinal)) / max(abs(refFinal), eps) * 100;
row.settling_time_2pct_s = settlingTime(run.t, run.omega_m, refFinal, p.settling_tol_2pct, eventIdx);
row.settling_time_5pct_s = settlingTime(run.t, run.omega_m, refFinal, p.settling_tol_5pct, eventIdx);
row.steady_error_rpm = (refFinal - omegaFinal) * p.rad_to_rpm;

loadEventIdx = findLastReferenceChange(run.T_L);
if ~isempty(loadEventIdx)
    post = loadEventIdx:N;
    row.max_speed_drop_rpm = max(0, (refFinal - min(run.omega_m(post))) * p.rad_to_rpm);
    rec = settlingTime(run.t, run.omega_m, refFinal, p.settling_tol_2pct, loadEventIdx);
    if isfinite(rec)
        row.recovery_time_s = rec;
    end
end

row.steady_id_A = finiteMean(run.id(tail));
row.steady_iq_A = finiteMean(run.iq(tail));
row.steady_is_A = finiteMean(run.i_s(tail));
row.iq_peak_A = max(abs(run.iq));
row.current_limit_time_s = sum(run.i_s >= 0.99 * p.i_s_max) * p.Ts;

row.mean_Pin_W = finiteMean(run.P_in(tail));
row.mean_Pout_W = finiteMean(run.P_out(tail));
row.mean_eta_pct = finiteMean(run.eta(tail)) * 100;
row.mean_Pcu_W = finiteMean(run.P_cu(tail));

row.id_convergence_time_s = idConvergenceTime(run, tail, method);
steadyIdx = find(run.steady_flag > 0.5, 1, 'first');
if ~isempty(steadyIdx)
    row.steady_detector_start_s = run.t(steadyIdx);
end
active = run.optimizer_active > 0.5;
row.freeze_count = sum(diff(active) < 0);
constraint = run.constraint_flag > 0.5;
row.constraint_trigger_count = sum(diff([false; constraint]) > 0);
row.gradient_rms = sqrt(finiteMean(run.g_hat.^2));
end

function idx = findLastReferenceChange(signal)
d = abs(diff(signal));
idx = find(d > max(1e-9, 1e-6 * max(abs(signal))), 1, 'last');
if ~isempty(idx)
    idx = idx + 1;
end
end

function ts = settlingTime(t, y, ref, tol, startIdx)
ts = NaN;
band = max(abs(ref) * tol, 1e-6);
inside = abs(y - ref) <= band;
suffixAll = flipud(cumprod(flipud(double(inside))) > 0);
candidate = find(suffixAll(startIdx:end), 1, 'first');
if ~isempty(candidate)
    idx = startIdx + candidate - 1;
    ts = t(idx) - t(startIdx);
end
end

function tc = idConvergenceTime(run, tail, method)
if strcmp(method, 'mfo')
    signal = run.id_bar;
else
    signal = run.id_ref;
end
target = finiteMean(signal(tail));
band = max(0.05, 0.05 * abs(target));
inside = abs(signal - target) <= band;
suffixAll = flipud(cumprod(flipud(double(inside))) > 0);
idx = find(suffixAll, 1, 'first');
if isempty(idx)
    tc = NaN;
else
    tc = run.t(idx);
end
end

function v = finiteMean(x)
x = x(isfinite(x));
if isempty(x)
    v = NaN;
else
    v = mean(x);
end
end

function v = getMethodValue(rows, method, fieldName)
v = NaN;
for i = 1:numel(rows)
    if strcmp(rows(i).method, method)
        v = rows(i).(fieldName);
        return;
    end
end
end

function row = emptyMetricRow()
row.case_name = '';
row.condition = '';
row.method = '';
row.overshoot_pct = NaN;
row.settling_time_2pct_s = NaN;
row.settling_time_5pct_s = NaN;
row.steady_error_rpm = NaN;
row.max_speed_drop_rpm = NaN;
row.recovery_time_s = NaN;
row.steady_id_A = NaN;
row.steady_iq_A = NaN;
row.steady_is_A = NaN;
row.iq_peak_A = NaN;
row.current_limit_time_s = NaN;
row.mean_Pin_W = NaN;
row.mean_Pout_W = NaN;
row.mean_eta_pct = NaN;
row.mean_Pcu_W = NaN;
row.pin_reduction_vs_id0_pct = NaN;
row.pin_diff_vs_mtpa_pct = NaN;
row.id_convergence_time_s = NaN;
row.steady_detector_start_s = NaN;
row.freeze_count = NaN;
row.constraint_trigger_count = NaN;
row.gradient_rms = NaN;
end

function writeRowsCsv(rows, filename)
if isempty(rows)
    return;
end
fid = fopen(filename, 'w');
if fid < 0
    error('Unable to write CSV file: %s', filename);
end
cleanup = onCleanup(@() fclose(fid));

fields = fieldnames(rows);
for i = 1:numel(fields)
    if i > 1
        fprintf(fid, ',');
    end
    fprintf(fid, '%s', fields{i});
end
fprintf(fid, '\n');

for r = 1:numel(rows)
    for c = 1:numel(fields)
        if c > 1
            fprintf(fid, ',');
        end
        value = rows(r).(fields{c});
        if ischar(value)
            fprintf(fid, '"%s"', strrep(value, '"', '""'));
        elseif isnumeric(value)
            if isnan(value)
                fprintf(fid, '');
            else
                fprintf(fid, '%.10g', value);
            end
        else
            fprintf(fid, '"%s"', char(value));
        end
    end
    fprintf(fid, '\n');
end
end
