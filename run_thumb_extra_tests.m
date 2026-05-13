function run_thumb_extra_tests()
% 5-phase SMC-MB full-cycle validation: CLOSE | HOLD | LOAD (2.40x) | RETRACE | HOLD-OPEN.
% Produces: thumb_fullcycle.mat, thumb_fullcycle_tracking.png, thumb_fullcycle_torques.png,
%           thumb_fullcycle_phase.png, thumb_fullcycle_anim.mp4.

clc;
fprintf('Robotic Thumb — Full Validation Scenario\n');
fprintf('==========================================\n\n');

set_plot_defaults();

% ---- Parameters ----------------------------------------------------------
dt     = 0.0005;
Lambda = diag([12, 12, 12]);
K      = diag([0.010, 0.008, 0.0010]);
delta  = 0.01;

q0   = [0;    0;    0   ];   % fully extended / straight
qf   = [0.70; 1.20; 0.85];  % curled grip
segs = [0, 5, 10, 15, 22, 25];  % phase boundary times
T    = segs(end);

ref_fn  = @(t) ref_full_cycle(t, q0, qf, segs);
dist_fn = @(t) dist_with_load(t, 10.0);   % sudden load at t = 10 s

% ---- Simulate ------------------------------------------------------------
[t, X, Tau, S, Qref] = simulate_thumb(T, dt, q0, Lambda, K, delta, ref_fn, dist_fn);

% ---- Metrics -------------------------------------------------------------
ph_names = {'P1 CLOSE   rms', 'P2 HOLD    rms', 'P3 LOAD    rms', ...
            'P4 RETRACE rms', 'P5 HOLDOPN rms'};
print_metrics(t, X(:,1:3), Qref, Tau, segs, ph_names);

% ---- Static figures ------------------------------------------------------
plot_scenario(t, X, Qref, Tau, S, segs);

% ---- Save ----------------------------------------------------------------
save('thumb_fullcycle.mat', 't','X','Tau','S','Qref', ...
     'Lambda','K','delta','q0','qf','segs');
fprintf('Saved -> thumb_fullcycle.mat\n\n');

% ---- Animation -----------------------------------------------------------
fprintf('Building animation...\n');
TAU_D = zeros(length(t), 3);
for k = 1:length(t), TAU_D(k,:) = dist_fn(t(k))'; end

animate_thumb_2d(t, X(:,1:3), Qref, TAU_D, segs, ...
    {'P1  CLOSE GRIP', ...
     'P2  HOLD POSTURE', ...
     'P3  LOAD REJECT', ...
     'P4  RETRACE', ...
     'P5  HOLD OPEN'}, ...
    {[1.00 0.85 0.30], [0.40 0.95 0.60], [1.00 0.48 0.42], ...
     [0.50 0.68 1.00], [0.78 0.82 0.90]}, ...
    'thumb_fullcycle_anim', ...
    'SMC-MB  —  Aggressive Disturbance  |  Load Scale x2.40  |  3-DOF Robotic Thumb');
fprintf('\nScenario complete.\n');
end


function [t, X, Tau, S, Qref] = simulate_thumb(T, dt, q0, Lambda, K, delta, ref_fn, dist_fn)
    t = (0:dt:T)';   N = length(t);
    X    = zeros(N, 6);
    Tau  = zeros(N, 3);
    S    = zeros(N, 3);
    Qref = zeros(N, 3);
    X(1,:) = [q0; 0; 0; 0]';

    for k = 1:N
        [qr,~,~] = ref_fn(t(k));
        Qref(k,:) = qr';
    end

    fprintf('  Integrating %d steps ... ', N); tic;
    for k = 1:N-1
        [tau_k, sk] = smc_eval(X(k,:)', t(k), Lambda, K, delta, ref_fn);
        Tau(k,:) = tau_k';
        S(k,:)   = sk';
        f = @(x,tt) thumb_xdot(x, tt, Lambda, K, delta, ref_fn, dist_fn);
        X(k+1,:) = rk4(f, X(k,:)', t(k), dt)';
    end
    [tau_N, sN] = smc_eval(X(N,:)', t(N), Lambda, K, delta, ref_fn);
    Tau(N,:) = tau_N';   S(N,:) = sN';
    fprintf('done in %.1f s\n', toc);
end


function [tau, s] = smc_eval(x, t, Lambda, K, delta, ref_fn)
    q = x(1:3);   dq = x(4:6);
    [qr, dqr, ddqr] = ref_fn(t);
    e  = q  - qr;
    de = dq - dqr;
    s  = de + Lambda*e;
    dqr_mod  = dqr  - Lambda*e;
    ddqr_mod = ddqr - Lambda*de;
    M = thumb_M(q(1), q(2), q(3));
    [C_r, G_v] = thumb_CG(q(1), q(2), q(3), dqr_mod(1), dqr_mod(2), dqr_mod(3));
    tau_eq = M*ddqr_mod + C_r + G_v;
    tau_sw = -K * sat_s(s, delta);
    tau    = tau_eq + tau_sw;
end


function xdot = thumb_xdot(x, t, Lambda, K, delta, ref_fn, dist_fn)
    q = x(1:3);   dq = x(4:6);
    td = dist_fn(t);
    [tau, ~] = smc_eval(x, t, Lambda, K, delta, ref_fn);
    M = thumb_M(q(1), q(2), q(3));
    [C_v, G_v] = thumb_CG(q(1), q(2), q(3), dq(1), dq(2), dq(3));
    qddot = M \ (tau + td - C_v - G_v);
    xdot = [dq; qddot];
end


function xn = rk4(f, x, t, dt)
    k1 = f(x,           t       );
    k2 = f(x + dt/2*k1, t + dt/2);
    k3 = f(x + dt/2*k2, t + dt/2);
    k4 = f(x + dt*k3,   t + dt  );
    xn = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end


function y = sat_s(s, delta)
    y = zeros(size(s));
    for i = 1:numel(s)
        if abs(s(i)) < delta, y(i) = s(i)/delta;
        else,                 y(i) = sign(s(i));
        end
    end
end


function [qr, dqr, ddqr] = ref_full_cycle(t, q0, qf, segs)
    t1 = segs(2);   t3 = segs(4);   t4 = segs(5);

    if t < t1
        T = t1;  u = t / T;
        [s, ds, dds] = quintic_s(u, T);
        dq = qf - q0;
        qr = q0 + dq*s;  dqr = dq*ds;  ddqr = dq*dds;
    elseif t < t3
        qr = qf;  dqr = zeros(3,1);  ddqr = zeros(3,1);
    elseif t < t4
        T = t4 - t3;  u = (t - t3) / T;
        [s, ds, dds] = quintic_s(u, T);
        dq = q0 - qf;
        qr = qf + dq*s;  dqr = dq*ds;  ddqr = dq*dds;
    else
        qr = q0;  dqr = zeros(3,1);  ddqr = zeros(3,1);
    end
end


function [s, ds, dds] = quintic_s(u, T)
    s   =  10*u^3 - 15*u^4 +  6*u^5;
    ds  = (30*u^2 - 60*u^3 + 30*u^4) / T;
    dds = (60*u   -180*u^2 +120*u^3) / T^2;
end


function td = dist_with_load(t, t_load)
    % LOAD_SCALE = 1.0 → nominal;  2.40 → breaking-point (q2 reaching condition violated)
    LOAD_SCALE = 2.40;
    td_bg = [ 0.0020*sin(t)       + 0.0005*sin(200*pi*t);
              0.0010*cos(2*t)     + 0.0005*sin(200*pi*t);
              0.00015*sin(0.5*t)  + 0.00005*sin(200*pi*t) ];
    sigma    = 0.5*(1 + tanh((t - t_load)/0.2));
    td_load  = LOAD_SCALE * sigma * [-0.0030; -0.0030; -0.00020];
    td = td_bg + td_load;
end


function print_metrics(t, Q, Qref, Tau, segs, names)
    E  = Q - Qref;
    P  = length(segs) - 1;
    fprintf('  %-22s  %10s %10s %10s\n', 'Phase', 'q1', 'q2', 'q3');
    for ph = 1:P
        idx = t >= segs(ph) & t < segs(ph+1);
        v   = rms(E(idx,:), 1);
        fprintf('  %-22s  %10.5f %10.5f %10.5f\n', names{ph}, v(1), v(2), v(3));
    end
    pk = max(abs(Tau), [], 1);
    fprintf('  %-22s  %10.5f %10.5f %10.5f\n', '|tau|_max (N·m)', pk(1), pk(2), pk(3));
end


function plot_scenario(t, X, Qref, Tau, S, segs)
    set_plot_defaults();
    Q  = X(:,1:3);
    P  = length(segs) - 1;
    colors  = {[0.00 0.30 0.85], [0.90 0.10 0.10], [0.00 0.55 0.00]};
    ref_clr = [0.90 0.00 0.90];
    ph_col  = {[1.00 0.85 0.20], [0.25 0.85 0.45], [0.95 0.30 0.25], ...
               [0.40 0.55 0.95], [0.82 0.82 0.82]};
    ph_lbl  = {'CLOSE','HOLD','LOAD','RETRACE','HOLD'};
    jnames  = {'Proximal  q_1', 'Middle  q_2', 'Distal  q_3'};
    T       = segs(end);

    % --- Tracking + error -------------------------------------------------
    fig = figure('Position',[60 60 1420 880],'Color','white');
    for ji = 1:3
        ax = subplot(2,3,ji);   hold on;
        yl_pos = [min([Qref(:,ji);Q(:,ji)])-0.05, max([Qref(:,ji);Q(:,ji)])+0.05];
        for ph = 1:P
            patch(ax, [segs(ph) segs(ph+1) segs(ph+1) segs(ph)], ...
                  [yl_pos(1) yl_pos(1) yl_pos(2) yl_pos(2)], ph_col{ph}, ...
                  'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
        end
        plot(t, Qref(:,ji), '-', 'Color', ref_clr, 'LineWidth', 3.0, 'DisplayName','q_{ref}');
        plot(t, Q(:,ji),    '-', 'Color', colors{ji}, 'LineWidth', 2.2, 'DisplayName','SMC-MB');
        for ph = 2:P
            xline(segs(ph),'--','Color',[0.5 0.5 0.5],'HandleVisibility','off');
        end
        xlabel('Time (s)');  ylabel(sprintf('q_%d  (rad)',ji));
        title(sprintf('Joint %d — %s', ji, jnames{ji}));
        legend('Location','best','Box','on');
        grid on;  box on;  xlim([0 T]);  ylim(yl_pos);

        ax2 = subplot(2,3,3+ji);   hold on;
        e = Q(:,ji) - Qref(:,ji);  emax = max(abs(e))*1.1 + 1e-5;
        for ph = 1:P
            patch(ax2, [segs(ph) segs(ph+1) segs(ph+1) segs(ph)], ...
                  [-emax -emax emax emax], ph_col{ph}, ...
                  'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
        end
        plot(t, e, '-', 'Color', colors{ji}, 'LineWidth', 2.2);
        yline(0, ':', 'Color', [0.5 0.5 0.5]);
        for ph = 2:P
            xline(segs(ph),'--','Color',[0.5 0.5 0.5],'HandleVisibility','off');
        end
        xlabel('Time (s)');  ylabel(sprintf('e_%d  (rad)',ji));
        title(sprintf('Tracking Error  e_%d', ji));
        grid on;  box on;  xlim([0 T]);
    end
    sgtitle('Full Validation — CLOSE | HOLD | LOAD | RETRACE | HOLD OPEN', ...
            'FontSize',16,'FontWeight','bold');
    print(fig,'thumb_fullcycle_tracking','-dpng','-r150');
    fprintf('  Saved: thumb_fullcycle_tracking.png\n');

    % --- Torques ----------------------------------------------------------
    fig = figure('Position',[60 60 1420 560],'Color','white');
    for ji = 1:3
        subplot(1,3,ji);  hold on;
        plot(t, Tau(:,ji)*1000, '-', 'Color', colors{ji}, 'LineWidth', 2.0);
        yline(0, ':', 'Color', [0.5 0.5 0.5]);
        for ph = 2:P
            xline(segs(ph),'--','Color',[0.5 0.5 0.5]);
        end
        xlabel('Time (s)');  ylabel('\tau  (mN·m)');
        title(sprintf('Joint %d — Control Torque', ji));
        grid on;  box on;  xlim([0 T]);
    end
    sgtitle('Full Validation — Control Torques','FontSize',16,'FontWeight','bold');
    print(fig,'thumb_fullcycle_torques','-dpng','-r150');
    fprintf('  Saved: thumb_fullcycle_torques.png\n');

    % --- Sliding surfaces + phase-plane -----------------------------------
    fig = figure('Position',[60 60 1420 900],'Color','white');
    Qdot      = X(:,4:6);
    Qdot_disp = zeros(size(Qdot));
    Q_disp    = zeros(size(Q));
    for ji = 1:3
        Qdot_disp(:,ji) = movmean(Qdot(:,ji), 401);
        Q_disp(:,ji)    = movmean(Q(:,ji),    401);
        subplot(2,3,ji);   hold on;
        plot(t, S(:,ji), '-', 'Color', colors{ji}, 'LineWidth', 1.4);
        yline(0,'--','Color',[0.4 0.4 0.4]);
        for ph = 2:P
            xline(segs(ph),'--','Color',[0.5 0.5 0.5]);
        end
        xlabel('Time (s)');  ylabel(sprintf('s_%d', ji));
        title(sprintf('Sliding Surface s_%d', ji));
        grid on;  box on;  xlim([0 T]);

        subplot(2,3,3+ji);  hold on;
        plot(Q_disp(:,ji), Qdot_disp(:,ji), '-', 'Color', colors{ji}, ...
             'LineWidth', 1.6, 'DisplayName', 'Trajectory');
        plot(Q_disp(1,ji),   Qdot_disp(1,ji),   'o', 'MarkerSize', 11, ...
             'MarkerFaceColor',[0.10 0.65 0.20],'MarkerEdgeColor','k','DisplayName','Start');
        plot(Q_disp(end,ji), Qdot_disp(end,ji), 's', 'MarkerSize', 11, ...
             'MarkerFaceColor',[0.85 0.15 0.15],'MarkerEdgeColor','k','DisplayName','End');
        xlabel(sprintf('q_%d  (rad)',ji));  ylabel(sprintf('dq_%d/dt  (rad/s)',ji));
        title(sprintf('Phase Plane — Joint %d', ji));
        legend('Location','best','Box','on','FontSize',9);
        grid on;  box on;
    end
    sgtitle('Sliding Surfaces  &  Phase Planes  (start \approx end \Rightarrow full retrace)', ...
            'FontSize',15,'FontWeight','bold');
    print(fig,'thumb_fullcycle_phase','-dpng','-r150');
    fprintf('  Saved: thumb_fullcycle_phase.png\n');
end


function set_plot_defaults()
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
end

function M = thumb_M(q1, q2, q3) %#ok<INUSL>
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


function [Cv, Gv] = thumb_CG(q1, q2, q3, dq1, dq2, dq3)
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
    C3 =  p3*dq1^2 + b3*(dq1+dq2)^2;
    Cv = [C1; C2; C3];
    s1=sin(q1); s12=sin(q1+q2); s123=sin(q1+q2+q3);
    G1 = g*(m1*c1*s1 + m2*(r1*s1+c2*s12) + m3*(r1*s1+r2*s12+c3*s123));
    G2 = g*(m2*c2*s12 + m3*(r2*s12+c3*s123));
    G3 = g*m3*c3*s123;
    Gv = [G1; G2; G3];
end
