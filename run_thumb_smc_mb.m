function run_thumb_smc_mb()
% Runs the three-link robotic thumb simulation using Model-Based SMC.
%
% Third-Thumb-inspired wearable manipulator (planar, 3-DOF).
% Integration: fixed-step RK4, dt = 0.5 ms over T = 15 s.
% Prints a performance table, saves thumb_results.mat, exports PNG figures,
% and launches the 2-D animation.

clc;
fprintf('Robotic Thumb — Model-Based SMC  (3-DOF, wearable scale)\n\n');

% ---- Simulation parameters --------------------------------------------------
dt = 0.0005;   T = 15;
t  = (0:dt:T)';   N = length(t);

% ---- Controller tuning (K > |d|_max per joint; Λ=12 → τ_reach ≈ 83 ms; δ=0.01 rad) -------
Lambda = diag([12, 12, 12]);          % sliding-surface slope  [s^-1]
K      = diag([0.010, 0.008, 0.0010]);% switching gain         [N·m]
delta  = 0.01;                        % boundary-layer width   [rad]

% ---- Initial conditions & storage ------------------------------------------
q0  = [0.10; 0.10; 0.05];   % open / extended [rad]
dq0 = [0;    0;    0   ];
x0  = [q0; dq0];

X    = zeros(N, 6);   % state  [q dq]
Tau  = zeros(N, 3);   % control torque [N·m]
S    = zeros(N, 3);   % sliding surface
Qref = zeros(N, 3);   % reference

X(1,:) = x0';

% Pre-compute reference (for metrics / plotting)
for k = 1:N
    [qr, ~, ~] = ref_trajectory(t(k));
    Qref(k,:)  = qr';
end

% ---- Integration loop -------------------------------------------------------
fprintf('Integrating %d steps ...  ', N);
tic_int = tic;

for k = 1:N-1
    tk = t(k);

    [tau_k, sk] = smcmb_eval(X(k,:)', tk, Lambda, K, delta);
    Tau(k,:) = tau_k';
    S(k,:)   = sk';

    X(k+1,:) = rk4(@(x,tt) thumb_xdot(x,tt,Lambda,K,delta), X(k,:)', tk, dt)';
end

% Fill last step
[tau_N, sN] = smcmb_eval(X(N,:)', t(N), Lambda, K, delta);
Tau(N,:) = tau_N';
S(N,:)   = sN';

fprintf('Done in %.1f s\n\n', toc(tic_int));

% ---- Performance metrics  (P1 CLOSE 0-5s | P2 HOLD 5-10s | P3 SUSTAIN 10-15s) ----------------
idx_p1 = t >= 0  & t < 5;
idx_p2 = t >= 5  & t < 10;
idx_p3 = t >= 10 & t <= 15;
Q = X(:,1:3);
E = Q - Qref;

rms_p1 = rms(E(idx_p1,:), 1);
rms_p2 = rms(E(idx_p2,:), 1);
rms_p3 = rms(E(idx_p3,:), 1);
ss_p2  = mean(abs(E(idx_p2,:)), 1);
pk_p3  = max(abs(E(idx_p3,:)), [], 1);
tau_pk = max(abs(Tau), [], 1);

fprintf('Per-phase tracking error (rad):\n');
fprintf('  %-22s  %10s %10s %10s\n', 'Phase', 'q1', 'q2', 'q3');
fprintf('  %-22s  %10.5f %10.5f %10.5f\n', 'P1 CLOSE   rms',     rms_p1);
fprintf('  %-22s  %10.5f %10.5f %10.5f\n', 'P2 HOLD    rms',     rms_p2);
fprintf('  %-22s  %10.5f %10.5f %10.5f\n', 'P2 HOLD   |e|_avg',  ss_p2);
fprintf('  %-22s  %10.5f %10.5f %10.5f\n', 'P3 SUSTAIN rms',     rms_p3);
fprintf('  %-22s  %10.5f %10.5f %10.5f\n', 'P3 SUSTAIN |e|_max', pk_p3);
fprintf('  %-22s  %10.5f %10.5f %10.5f\n', '|tau|_max (N·m)',    tau_pk);
fprintf('\n(Torques in N·m — multiply by 1000 for mN·m)\n\n');

e_rms = rms(E, 1);
e_ss  = ss_p2;

% ---- Save results -----------------------------------------------------------
save('thumb_results.mat', 't','X','Tau','S','Qref', ...
     'e_rms','e_ss','tau_pk', ...
     'rms_p1','rms_p2','rms_p3','ss_p2','pk_p3', ...
     'Lambda','K','delta');
fprintf('Results saved → thumb_results.mat\n\n');

% ---- Figures ----------------------------------------------------------------
set(0, 'DefaultFigureVisible',               'on');
set(0, 'DefaultFigureColor',                 'white');
set(0, 'DefaultAxesColor',                   'white');
set(0, 'DefaultAxesFontSize',                13);
set(0, 'DefaultAxesFontWeight',              'bold');
set(0, 'DefaultAxesXColor',                  'k');
set(0, 'DefaultAxesYColor',                  'k');
set(0, 'DefaultAxesGridColor',               [0.75 0.75 0.75]);
set(0, 'DefaultAxesGridAlpha',               1);
set(0, 'DefaultAxesLineWidth',               1.2);
set(0, 'DefaultAxesLabelFontSizeMultiplier', 1.1);
set(0, 'DefaultAxesTitleFontSizeMultiplier', 1.15);
set(0, 'DefaultAxesTitleFontWeight',         'bold');
set(0, 'DefaultTextColor',                   'k');
set(0, 'DefaultLegendColor',                 'white');
set(0, 'DefaultLegendTextColor',             'k');
set(0, 'DefaultLegendEdgeColor',             'k');

colors  = {[0.00 0.30 0.85], [0.90 0.10 0.10], [0.00 0.55 0.00]};
ref_clr = [0.90 0.00 0.90];
lw      = 2.2;
ref_lw  = 3.0;
fsz     = 14;
jnames  = {'Proximal  q_1', 'Middle  q_2', 'Distal  q_3'};
Qdot    = X(:, 4:6);

% ---- Figure 1 : Position tracking + tracking error (2 × 3) -----------------
% Helper: draw three phase-shading patches under a subplot
ph_col = {[1.00 0.85 0.20], [0.25 0.85 0.45], [0.95 0.30 0.25]};
ph_t   = [0 5; 5 10; 10 15];

fig1 = figure('Position', [60 60 1420 880], 'Color', 'white');
for ji = 1:3
    ax1 = subplot(2, 3, ji);
    % phase shading
    for ph = 1:3
        yl_tmp = [min(Qref(:,ji))*0.95-0.05, max(Q(:,ji))*1.05+0.05];
        patch(ax1, [ph_t(ph,1) ph_t(ph,2) ph_t(ph,2) ph_t(ph,1)], ...
              [yl_tmp(1) yl_tmp(1) yl_tmp(2) yl_tmp(2)], ...
              ph_col{ph}, 'FaceAlpha', 0.07, 'EdgeColor', 'none', 'HandleVisibility','off');
        hold on;
    end
    plot(t, Qref(:,ji), '-',  'Color', ref_clr,    'LineWidth', ref_lw, ...
         'DisplayName', 'Reference  q_{ref}');
    plot(t, Q(:,ji),    '-',  'Color', colors{ji}, 'LineWidth', lw, ...
         'DisplayName', 'SMC-MB');
    xline(5,  '--', 'Color',[0.70 0.65 0.20],'LineWidth',1.0,'HandleVisibility','off');
    xline(10, '--', 'Color',[0.80 0.22 0.18],'LineWidth',1.0,'HandleVisibility','off');
    text(ax1, 2.5,  max(Q(:,ji))*1.00, 'CLOSE', 'FontSize',7,'Color',[0.60 0.50 0.10], ...
         'HorizontalAlignment','center','FontWeight','bold','HandleVisibility','off');
    text(ax1, 7.5,  max(Q(:,ji))*1.00, 'HOLD',  'FontSize',7,'Color',[0.10 0.55 0.28], ...
         'HorizontalAlignment','center','FontWeight','bold','HandleVisibility','off');
    text(ax1, 12.5, max(Q(:,ji))*1.00, 'SUSTAIN','FontSize',7,'Color',[0.65 0.15 0.12], ...
         'HorizontalAlignment','center','FontWeight','bold','HandleVisibility','off');
    xlabel('Time (s)');
    ylabel(sprintf('q_%d  (rad)', ji));
    title(sprintf('Joint %d — %s', ji, jnames{ji}), ...
          'FontWeight', 'bold', 'FontSize', fsz, 'Color', 'k');
    legend('Location', 'southeast', 'Box', 'on', 'FontSize', 10);
    grid on;  box on;  xlim([0 15]);

    ax2 = subplot(2, 3, 3 + ji);
    e = Q(:,ji) - Qref(:,ji);
    emax = max(abs(e))*1.1 + 1e-5;
    for ph = 1:3
        patch(ax2, [ph_t(ph,1) ph_t(ph,2) ph_t(ph,2) ph_t(ph,1)], ...
              [-emax -emax emax emax], ...
              ph_col{ph}, 'FaceAlpha', 0.07, 'EdgeColor', 'none', 'HandleVisibility','off');
        hold on;
    end
    plot(t, e, '-', 'Color', colors{ji}, 'LineWidth', lw, ...
         'DisplayName', sprintf('e_%d', ji));
    yline(0, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'LineStyle', ':', ...
          'HandleVisibility', 'off');
    xline(5,  '--', 'Color',[0.70 0.65 0.20],'LineWidth',1.0,'HandleVisibility','off');
    xline(10, '--', 'Color',[0.80 0.22 0.18],'LineWidth',1.0,'HandleVisibility','off');
    xlabel('Time (s)');
    ylabel(sprintf('e_%d  (rad)', ji));
    title(sprintf('Tracking Error  e_%d', ji), ...
          'FontWeight', 'bold', 'FontSize', fsz, 'Color', 'k');
    legend('Location', 'northeast', 'Box', 'on', 'FontSize', 10);
    grid on;  box on;  xlim([0 15]);
end
sgtitle('Robotic Thumb SMC-MB — Joint Position Tracking   (P1 CLOSE | P2 HOLD | P3 SUSTAIN)', ...
        'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
print(fig1, 'thumb_tracking', '-dpng', '-r150');
fprintf('Saved: thumb_tracking.png\n');

% ---- Figure 2 : Joint velocities -------------------------------------------
vel_ylbl = {'dq_1  (rad/s)', 'dq_2  (rad/s)', 'dq_3  (rad/s)'};
fig2 = figure('Position', [60 60 1420 560], 'Color', 'white');
for ji = 1:3
    subplot(1, 3, ji);
    plot(t, Qdot(:,ji), '-', 'Color', colors{ji}, 'LineWidth', lw);
    yline(0, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'LineStyle', ':');
    xlabel('Time (s)');
    ylabel(vel_ylbl{ji});
    title(sprintf('Joint %d — Velocity', ji), ...
          'FontWeight', 'bold', 'FontSize', fsz, 'Color', 'k');
    grid on;  box on;  xlim([0 15]);
end
sgtitle('Robotic Thumb SMC-MB — Joint Velocities', ...
        'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
print(fig2, 'thumb_velocity', '-dpng', '-r150');
fprintf('Saved: thumb_velocity.png\n');

% ---- Figure 3 : Sliding surfaces -------------------------------------------
fig3 = figure('Position', [60 60 1420 600], 'Color', 'white');
for ji = 1:3
    subplot(1, 3, ji);
    s_vec = S(:,ji);
    reach_idx = find(abs(s_vec) < delta, 1, 'first');
    if isempty(reach_idx), reach_idx = N; end
    yl = [min(s_vec)*1.20 - 0.001,  max(s_vec)*1.20 + 0.001];
    patch([0, t(reach_idx), t(reach_idx), 0], ...
          [yl(1), yl(1), yl(2), yl(2)], ...
          [1.00 0.87 0.87], 'EdgeColor', 'none', 'FaceAlpha', 0.60);
    hold on;
    patch([t(reach_idx), 15, 15, t(reach_idx)], ...
          [yl(1), yl(1), yl(2), yl(2)], ...
          [0.87 1.00 0.87], 'EdgeColor', 'none', 'FaceAlpha', 0.60);
    plot(t, s_vec, '-', 'Color', colors{ji}, 'LineWidth', lw);
    yline(0, 'Color', [0.30 0.30 0.30], 'LineWidth', 1.2, 'LineStyle', '--');
    xline(t(reach_idx), 'Color', [0.80 0 0], 'LineWidth', 1.8, ...
          'Label', sprintf('  t = %.2f s', t(reach_idx)), ...
          'LabelVerticalAlignment', 'bottom', 'FontSize', 9);
    xlabel('Time (s)');
    ylabel(sprintf('s_%d', ji));
    title(sprintf('Sliding Surface  s_%d(t)', ji), ...
          'FontWeight', 'bold', 'FontSize', fsz, 'Color', 'k');
    legend({'Reaching', 'Sliding', 's(t)'}, ...
           'Location', 'northeast', 'Box', 'on', 'FontSize', 10);
    grid on;  box on;  xlim([0 15]);  ylim(yl);
end
sgtitle('SMC Sliding Surfaces — Reaching Phase  \rightarrow  Sliding Mode', ...
        'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
print(fig3, 'thumb_sliding', '-dpng', '-r150');
fprintf('Saved: thumb_sliding.png\n');

% ---- Figure 4 : Control torques --------------------------------------------
fig4 = figure('Position', [60 60 1420 560], 'Color', 'white');
for ji = 1:3
    subplot(1, 3, ji);
    plot(t, Tau(:,ji)*1000, '-', 'Color', colors{ji}, 'LineWidth', lw);
    yline(0, 'Color', [0.5 0.5 0.5], 'LineWidth', 0.9, 'LineStyle', ':');
    xlabel('Time (s)');
    ylabel('\tau  (mN·m)');
    th = title(sprintf('Joint %d — Control Torque', ji), ...
               'FontWeight', 'bold', 'FontSize', fsz);
    th.Color = colors{ji};
    [~, pk_idx] = max(abs(Tau(:,ji)));
    pk_val = Tau(pk_idx, ji) * 1000;
    text(t(pk_idx) + 0.3, pk_val, sprintf('  %.1f mN·m', pk_val), ...
         'FontSize', 9, 'Color', colors{ji}, 'FontWeight', 'bold', ...
         'BackgroundColor', 'white', 'Margin', 1);
    grid on;  box on;  xlim([0 15]);
end
sgtitle('Control Torques — Robotic Thumb SMC-MB', ...
        'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
print(fig4, 'thumb_torques', '-dpng', '-r150');
fprintf('Saved: thumb_torques.png\n');

% ---- Figure 5 : Phase-3 disturbance-rejection zoom [9.5, 15] s ---------------
fig5 = figure('Position', [60 60 1420 880], 'Color', 'white');
t_win = t >= 9.5;
for ji = 1:3
    subplot(2, 3, ji);
    plot(t(t_win), Qref(t_win,ji), '-',  'Color', ref_clr,    'LineWidth', ref_lw, ...
         'DisplayName', 'Reference  q_{ref}');  hold on;
    plot(t(t_win), Q(t_win,ji),    '-',  'Color', colors{ji}, 'LineWidth', lw, ...
         'DisplayName', 'SMC-MB');
    xline(10, '--', 'Color',[0.85 0.10 0.10], 'LineWidth',1.6, ...
         'Label','  SUDDEN LOAD','LabelVerticalAlignment','top','FontSize',9, ...
         'HandleVisibility','off');
    xlabel('Time (s)');
    ylabel(sprintf('q_%d  (rad)', ji));
    title(sprintf('Joint %d  Hold-through-Load', ji), ...
          'FontWeight', 'bold', 'FontSize', fsz, 'Color', 'k');
    legend('Location', 'best', 'Box', 'on', 'FontSize', 10);
    grid on;  box on;

    subplot(2, 3, 3 + ji);
    e = Q(t_win,ji) - Qref(t_win,ji);
    plot(t(t_win), e, '-', 'Color', colors{ji}, 'LineWidth', lw);  hold on;
    yline(0, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'LineStyle', ':');
    xline(10, '--', 'Color',[0.85 0.10 0.10], 'LineWidth',1.6, ...
         'HandleVisibility','off');
    xlabel('Time (s)');
    ylabel(sprintf('e_%d  (rad)', ji));
    title(sprintf('Residual + Rejection  e_%d', ji), ...
          'FontWeight', 'bold', 'FontSize', fsz, 'Color', 'k');
    grid on;  box on;
end
sgtitle('Phase 3 — Sudden-Load Disturbance Rejection  (load applied at t = 10 s)', ...
        'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
print(fig5, 'thumb_phase3_reject', '-dpng', '-r150');
fprintf('Saved: thumb_phase3_reject.png\n\n');

% ---- 2-D Animation ----------------------------------------------------------
fprintf('Launching 2-D animation...\n');
TAU_D_main = zeros(N, 3);
for k_ = 1:N, TAU_D_main(k_,:) = disturbance(t(k_))'; end
animate_thumb_2d(t, Q, Qref, TAU_D_main, ...
    [0 5 10 15], ...
    {'PHASE 1  CLOSE GRIP', ...
     'PHASE 2  HOLD POSTURE', ...
     'PHASE 3  SUDDEN LOAD — REJECT'}, ...
    {[1.00 0.85 0.30], [0.35 0.92 0.55], [1.00 0.48 0.42]}, ...
    'thumb_animation', ...
    'Robotic Thumb — Model-Based SMC  |  3-DOF Wearable Exoskeleton');

fprintf('Robotic thumb simulation complete.\n');
end

% =============================================================================
%  LOCAL HELPER FUNCTIONS
% =============================================================================

function [tau, s] = smcmb_eval(x, t, Lambda, K, delta)
    q    = x(1:3);  qdot = x(4:6);
    [qr, dqr, ddqr] = ref_traj(t);

    e_t  = q    - qr;
    de_t = qdot - dqr;
    s    = de_t + Lambda*e_t;

    qdot_r  = dqr  - Lambda*e_t;
    qddot_r = ddqr - Lambda*de_t;

    M_mat          = thumb_M(q(1), q(2), q(3));
    [C_r, G_vec]   = thumb_CG(q(1), q(2), q(3), qdot_r(1), qdot_r(2), qdot_r(3));
    tau_eq  = M_mat*qddot_r + C_r + G_vec;
    tau_sw  = -K * sat_s(s, delta);
    tau     = tau_eq + tau_sw;
end

function xdot = thumb_xdot(x, t, Lambda, K, delta)
    q    = x(1:3);  qdot = x(4:6);
    td   = dist_torque(t);
    [tau, ~] = smcmb_eval(x, t, Lambda, K, delta);
    M_mat          = thumb_M(q(1), q(2), q(3));
    [C_vec, G_vec] = thumb_CG(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
    qddot = M_mat \ (tau + td - C_vec - G_vec);
    xdot  = [qdot; qddot];
end

function xnext = rk4(f, x, t, dt)
    k1 = f(x,           t       );
    k2 = f(x + dt/2*k1, t + dt/2);
    k3 = f(x + dt/2*k2, t + dt/2);
    k4 = f(x + dt*k3,   t + dt  );
    xnext = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function [qr, dqr, ddqr] = ref_traj(t)
    [qr, dqr, ddqr] = ref_trajectory(t);   % single source of truth
end

function td = dist_torque(t)
    td = disturbance(t);                   % single source of truth
end

function y = sat_s(s, delta)
    y = zeros(size(s));
    for i = 1:numel(s)
        if abs(s(i)) < delta
            y(i) = s(i)/delta;
        else
            y(i) = sign(s(i));
        end
    end
end

function M = thumb_M(q1, q2, q3)
    r1=0.050; r2=0.035; r3=0.025;
    m1=0.015; m2=0.010; m3=0.006;
    J1=(1/3)*m1*r1^2; J2=(1/3)*m2*r2^2; J3=(1/3)*m3*r3^2;
    c1=r1/2; c2=r2/2; c3=r3/2;

    M11 = J1+J2+J3 + m1*c1^2 ...
        + m2*(r1^2+c2^2+2*r1*c2*cos(q2)) ...
        + m3*(r1^2+r2^2+c3^2 +2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
    M22 = J2+J3 + m2*c2^2 + m3*(r2^2+c3^2+2*r2*c3*cos(q3));
    M33 = J3 + m3*c3^2;
    M12 = J2+J3 + m2*(c2^2+r1*c2*cos(q2)) ...
        + m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
    M13 = J3 + m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));
    M23 = J3 + m3*(c3^2+r2*c3*cos(q3));

    M = [M11 M12 M13; M12 M22 M23; M13 M23 M33];
end

function [Cv, Gv] = thumb_CG(q1,q2,q3,dq1,dq2,dq3)
    r1=0.050; r2=0.035; r3=0.025;
    m2=0.010; m3=0.006;
    c2=r2/2; c3=r3/2; g=9.81;
    m1=0.015; c1=r1/2;

    a2 = m2*r1*c2*sin(q2) + m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));
    b3 = m3*r2*c3*sin(q3);
    a3 = m3*r1*c3*sin(q2+q3) + b3;
    p3 = m3*r1*c3*sin(q2+q3);

    C1 = -2*a2*dq1*dq2 - a2*dq2^2 - 2*a3*(dq1+dq2)*dq3 - a3*dq3^2;
    C2 =  a2*dq1^2 - 2*b3*(dq1+dq2)*dq3 - b3*dq3^2;
    C3 =  p3*dq1^2 + b3*(dq1+dq2)^2;   % corrected: Gamma_312 = b3 only
    Cv = [C1; C2; C3];

    s1=sin(q1); s12=sin(q1+q2); s123=sin(q1+q2+q3);
    G1 = g*(m1*c1*s1 + m2*(r1*s1+c2*s12) + m3*(r1*s1+r2*s12+c3*s123));
    G2 = g*(m2*c2*s12 + m3*(r2*s12+c3*s123));
    G3 = g*m3*c3*s123;
    Gv = [G1; G2; G3];
end
