function Generate_Report_Text(allResults, metrics, p)
%GENERATE_REPORT_TEXT Write concise thesis-ready Markdown result snippets.

filename = fullfile(p.paths.results, 'report_snippets.md');
fid = fopen(filename, 'w', 'n', 'UTF-8');
if fid < 0
    error('Unable to write report snippets: %s', filename);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# 第3章仿真结果文字片段\n\n');
fprintf(fid, '> 以下内容均由 `Run_All_Experiments.m` 调用脚本仿真生成，属于仿真结果，不是硬件实验数据。\n\n');

writeConstantSection(fid, metrics.constant_load, p);
writeLoadStepSection(fid, metrics.load_step, p);
writeSpeedStepSection(fid, metrics.speed_step, p);
writeParameterSection(fid, metrics.parameter_perturbation, p);
writeNoiseSection(fid, metrics.noise_delay, p);
writeConstraintSection(fid, allResults.constraints.mfo, metrics.constraints, p);
end

function writeConstantSection(fid, rows, p)
mfo = findRow(rows, 'mfo', 'base');
id0 = findRow(rows, 'id0', 'base');
mtpa = findRow(rows, 'mtpa', 'base');
fprintf(fid, '## 实验1：恒速恒载工况\n\n');
fprintf(fid, '本工况设置转速指令为 %.0f r/min，负载转矩为 %.1f N·m。', p.exp.constant.omega_ref_rpm, p.exp.constant.T_L);
fprintf(fid, '脚本同时计算 i_d^*=0、MTPA 和无模型反馈优化三种控制策略的稳态输入功率、效率与电流幅值。\n\n');
fprintf(fid, '仿真稳态阶段，i_d^*=0 方法平均输入功率为 %.2f W，MTPA 为 %.2f W，无模型反馈优化为 %.2f W。', id0.mean_Pin_W, mtpa.mean_Pin_W, mfo.mean_Pin_W);
fprintf(fid, '相对 i_d^*=0，无模型反馈优化的输入功率变化为 %.2f%%，平均效率为 %.2f%%。', mfo.pin_reduction_vs_id0_pct, mfo.mean_eta_pct);
fprintf(fid, '该结果可用于说明外层功率反馈寻优在稳态条件下对 d 轴电流参考值进行了在线修正。\n\n');
end

function writeLoadStepSection(fid, rows, p)
mfo = findRow(rows, 'mfo', 'base');
fprintf(fid, '## 实验2：负载阶跃工况\n\n');
fprintf(fid, '本工况中转速指令保持 %.0f r/min，负载转矩在 %.2f s 由 %.1f N·m 阶跃到 %.1f N·m。', p.exp.load_step.omega_ref_rpm, p.exp.load_step.step_time, p.exp.load_step.T_L_initial, p.exp.load_step.T_L_final);
fprintf(fid, '无模型反馈优化方法的最大转速跌落为 %.2f r/min，恢复时间为 %.4f s，q 轴电流峰值为 %.2f A。', mfo.max_speed_drop_rpm, mfo.recovery_time_s, mfo.iq_peak_A);
fprintf(fid, '稳态判别标志在动态扰动阶段使外层优化器冻结，负载重新稳定后再恢复更新，可避免优化环节干扰速度环动态响应。\n\n');
end

function writeSpeedStepSection(fid, rows, p)
mfo = findRow(rows, 'mfo', 'base');
fprintf(fid, '## 实验3：转速阶跃工况\n\n');
fprintf(fid, '本工况中负载转矩保持 %.1f N·m，转速指令在 %.2f s 由 %.0f r/min 阶跃到 %.0f r/min。', p.exp.speed_step.T_L, p.exp.speed_step.step_time, p.exp.speed_step.omega_ref_initial_rpm, p.exp.speed_step.omega_ref_final_rpm);
fprintf(fid, '无模型反馈优化方法的超调量为 %.2f%%，2%% 调节时间为 %.4f s，稳态误差为 %.2f r/min。', mfo.overshoot_pct, mfo.settling_time_2pct_s, mfo.steady_error_rpm);
fprintf(fid, '从 S(k) 曲线可检查动态阶段外层优化是否冻结；若冻结持续时间过短，可适当增大 N_s 或收紧 epsilon_w、epsilon_i。\n\n');
end

function writeParameterSection(fid, rows, p)
nomMfo = findRow(rows, 'mfo', 'nominal');
perMfo = findRow(rows, 'mfo', 'perturbed');
perMtpa = findRow(rows, 'mtpa', 'perturbed');
fprintf(fid, '## 实验4：参数摄动工况\n\n');
fprintf(fid, '参数摄动设置为 R_s 增加 %.0f%%，psi_f 降低 %.0f%%，L_d 和 L_q 分别按 %.0f%%、%.0f%% 缩放。', (p.perturb.R_s_scale-1)*100, (1-p.perturb.psi_f_scale)*100, p.perturb.L_d_scale*100, p.perturb.L_q_scale*100);
fprintf(fid, '摄动后 MTPA 控制仍使用标称参数，无模型反馈优化仅依赖输入功率反馈在线调整。');
fprintf(fid, '摄动后 MTPA 平均输入功率为 %.2f W，无模型反馈优化为 %.2f W；无模型方法相对 MTPA 的功率差异为 %.2f%%。', perMtpa.mean_Pin_W, perMfo.mean_Pin_W, perMfo.pin_diff_vs_mtpa_pct);
fprintf(fid, '名义对象下无模型方法平均效率为 %.2f%%，摄动对象下为 %.2f%%，可据此评价参数失配下的能效保持能力。\n\n', nomMfo.mean_eta_pct, perMfo.mean_eta_pct);
end

function writeNoiseSection(fid, rows, p)
clean = findRow(rows, 'mfo', 'clean');
nonideal = findRow(rows, 'mfo', 'nonideal');
fprintf(fid, '## 实验5：测量噪声与采样延迟工况\n\n');
fprintf(fid, '本工况加入电流测量噪声、速度测量噪声、一阶测量延迟和直流母线小幅波动。');
fprintf(fid, '无噪声基准下无模型方法平均输入功率为 %.2f W，非理想因素下为 %.2f W；梯度估计 RMS 分别为 %.4f 和 %.4f。', clean.mean_Pin_W, nonideal.mean_Pin_W, clean.gradient_rms, nonideal.gradient_rms);
fprintf(fid, '若 g_hat 曲线出现持续高频大幅振荡，应增大 LPF 时间常数、减小 alpha 或降低扰动幅值 a。\n\n');
end

function writeConstraintSection(fid, run, rows, p)
mfo = findRow(rows, 'mfo', 'base');
fprintf(fid, '## 实验6：电流/电压约束工况\n\n');
fprintf(fid, '本工况设置转速指令 %.0f r/min、负载转矩 %.1f N·m，使系统接近电流或电压约束。', p.exp.constraint.omega_ref_rpm, p.exp.constraint.T_L);
fprintf(fid, '仿真中 i_s 最大值为 %.2f A，u_s 最大值为 %.2f V，投影/限幅触发次数为 %.0f。', max(run.i_s), max(run.u_s), mfo.constraint_trigger_count);
fprintf(fid, '投影机制将 bar_i_d^* 限制在电流、电压与 d 轴边界共同确定的可行域内，避免不可行参考值直接进入电流环。\n\n');
end

function row = findRow(rows, method, condition)
row = rows(1);
for i = 1:numel(rows)
    if strcmp(rows(i).method, method) && strcmp(rows(i).condition, condition)
        row = rows(i);
        return;
    end
end
end
