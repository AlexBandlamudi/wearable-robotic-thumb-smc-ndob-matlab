function out = run_thumb_ndob(load_scale, L_diag, Knew_diag, do_animate)
% NDOB+SMC on the 5-phase scenario.  Chen-2000 observer cancels disturbance,
% reduced switching gain K_new replaces the original K.
% Defaults: load_scale=2.40, L=[50 50 50], K_new=[2 2 0.3] mNm, do_animate=true.
% Produces: thumb_ndob_results.mat, thumb_ndob_tracking.png,
%           thumb_ndob_disturbance_est.png, thumb_ndob_animation.mp4

if nargin < 1 || isempty(load_scale),  load_scale  = 2.40;            end
if nargin < 2 || isempty(L_diag),      L_diag      = [50 50 50];     end
if nargin < 3 || isempty(Knew_diag),   Knew_diag   = [0.002 0.002 0.0003]; end
if nargin < 4 || isempty(do_animate),  do_animate  = true;           end

% Auto-convert Knew from mN·m to N·m if user passes large numbers
Knew_diag = Knew_diag(:)';
if max(Knew_diag) > 0.05, Knew_diag = Knew_diag * 1e-3; end

fprintf('\nRobotic Thumb — NDOB + SMC  (Chen 2000)\n');
fprintf('=======================================\n');
fprintf('  load_scale = %.3f\n', load_scale);
fprintf('  L          = diag(%.0f, %.0f, %.0f)  rad/s\n', L_diag);
fprintf('  K_new      = diag(%.3f, %.3f, %.4f)  N·m\n\n', Knew_diag);

set_plot_defaults();

% ---- Parameters ---------------------------------------------------------
dt     = 0.0005;
Lambda = diag([12, 12, 12]);
K      = diag(Knew_diag);
delta  = 0.01;
Lobs   = diag(L_diag);

q0   = [0;    0;    0   ];
qf   = [0.70; 1.20; 0.85];
segs = [0, 5, 10, 15, 22, 25];
T    = segs(end);

ref_fn  = @(t) ref_full_cycle(t, q0, qf, segs);
dist_fn = @(t) dist_with_load_scaled(t, 10.0, load_scale);

[t, X9, Tau, S, Qref, TAUD, TDH] = ...
    simulate_thumb_ndob(T, dt, q0, Lambda, K, delta, Lobs, ref_fn, dist_fn);

ph_names = {'P1 CLOSE   rms', 'P2 HOLD    rms', 'P3 LOAD    rms', ...
            'P4 RETRACE rms', 'P5 HOLDOPN rms'};
[rms_per_phase, peak_per_phase] = ...
    print_metrics(t, X9(:,1:3), Qref, Tau, segs, ph_names);

idx_p3 = t >= segs(3) & t < segs(4);
sat_frac_p3 = mean(abs(S(idx_p3,:)) > delta, 1);
ed = TAUD - TDH;
ed_rms_p3 = rms(ed(idx_p3,:), 1);
tau_peak  = max(abs(Tau), [], 1);
fprintf('\n  P3 sliding |s|>delta fraction:    %.3f  %.3f  %.3f\n', sat_frac_p3);
fprintf('  P3 NDOB residual e_d RMS (mN·m):  %.4f  %.4f  %.4f\n', ed_rms_p3*1000);
fprintf('  Peak |tau_i| (mN·m):              %.3f  %.3f  %.3f\n\n', tau_peak*1000);

ph_col  = {[1.00 0.85 0.20], [0.25 0.85 0.45], [0.95 0.30 0.25], ...
           [0.40 0.55 0.95], [0.82 0.82 0.82]};
ph_lbl  = {'CLOSE','HOLD','LOAD','RETRACE','HOLD-OPEN'};
plot_scenario_ndob(t, X9, Qref, Tau, S, TAUD, TDH, segs, ph_col, ...
                   load_scale, L_diag, Knew_diag);

out = struct('t',t,'X',X9,'Tau',Tau,'S',S,'Qref',Qref, ...
             'TAUD',TAUD,'TDH',TDH,'segs',segs,'Lambda',Lambda,'K',K, ...
             'delta',delta,'L_obs',Lobs,'q0',q0,'qf',qf,'load_scale',load_scale, ...
             'rms_per_phase',rms_per_phase,'peak_per_phase',peak_per_phase, ...
             'sat_frac_p3',sat_frac_p3,'ed_rms_p3',ed_rms_p3,'tau_peak',tau_peak);
save('thumb_ndob_results.mat','-struct','out');
fprintf('Saved -> thumb_ndob_results.mat\n');

% ---- Animation (5-phase, with tau_d_hat overlay) ----------------------
if do_animate
    fprintf('Building 5-phase NDOB animation...\n');
    title_str = sprintf('NDOB + SMC  —  Load Scale %.2f  |  L = %.0fI rad/s  |  K_new = diag(%.1f, %.1f, %.2f) mNm', ...
                         load_scale, L_diag(1), Knew_diag*1000);
    animate_thumb_2d_ndob(t, X9(:,1:3), Qref, TAUD, TDH, segs, ...
        {'P1  CLOSE GRIP','P2  HOLD POSTURE','P3  SUDDEN LOAD — REJECT', ...
         'P4  RETRACE','P5  HOLD OPEN'}, ...
        {[1.00 0.85 0.30],[0.40 0.95 0.60],[1.00 0.48 0.42], ...
         [0.50 0.68 1.00],[0.78 0.82 0.90]}, ...
        'thumb_ndob_animation', title_str);
end
fprintf('\nNDOB run complete.\n');
end


function [t, X9, Tau, S, Qref, TAUD, TDH] = simulate_thumb_ndob( ...
        T, dt, q0, Lambda, K, delta, Lobs, ref_fn, dist_fn)
    t = (0:dt:T)';   N = length(t);
    X9    = zeros(N, 9);
    Tau   = zeros(N, 3);
    S     = zeros(N, 3);
    Qref  = zeros(N, 3);
    TAUD  = zeros(N, 3);
    TDH   = zeros(N, 3);
    % p(0) = -L*M(q0)*dq0  =>  tau_d_hat(0) = 0
    M0 = thumb_M(q0(1), q0(2), q0(3));
    p0 = -Lobs * M0 * [0;0;0];
    X9(1,:) = [q0; 0; 0; 0; p0]';

    for k = 1:N
        [qr,~,~] = ref_fn(t(k));
        Qref(k,:) = qr';
    end

    fprintf('  Integrating %d steps (NDOB) ... ', N); tic;
    for k = 1:N-1
        [tau_k, sk, tdh_k] = ndob_smc_eval(X9(k,:)', t(k), Lambda, K, delta, Lobs, ref_fn);
        Tau(k,:) = tau_k';   S(k,:) = sk';   TDH(k,:) = tdh_k';
        TAUD(k,:) = dist_fn(t(k))';
        f = @(z,tt) thumb_xdot_ndob(z, tt, Lambda, K, delta, Lobs, ref_fn, dist_fn);
        X9(k+1,:) = rk4(f, X9(k,:)', t(k), dt)';
    end
    [tauN, sN, tdhN] = ndob_smc_eval(X9(N,:)', t(N), Lambda, K, delta, Lobs, ref_fn);
    Tau(N,:) = tauN';   S(N,:) = sN';   TDH(N,:) = tdhN';
    TAUD(N,:) = dist_fn(t(N))';
    fprintf('done in %.1f s\n', toc);
end

function [tau, s, tau_d_hat] = ndob_smc_eval(z, t, Lambda, K, delta, Lobs, ref_fn)
    q = z(1:3);   dq = z(4:6);   p = z(7:9);
    [qr, dqr, ddqr] = ref_fn(t);
    e  = q  - qr;
    de = dq - dqr;
    s  = de + Lambda*e;
    dqr_mod  = dqr  - Lambda*e;
    ddqr_mod = ddqr - Lambda*de;
    M = thumb_M(q(1), q(2), q(3));
    [C_r, G_v] = thumb_CG(q(1), q(2), q(3), dqr_mod(1), dqr_mod(2), dqr_mod(3));
    tau_d_hat = p + Lobs * M * dq;
    tau_eq = M*ddqr_mod + C_r + G_v;
    tau_sw = -K * sat_s(s, delta);
    tau    = tau_eq - tau_d_hat + tau_sw;
end

function zdot = thumb_xdot_ndob(z, t, Lambda, K, delta, Lobs, ref_fn, dist_fn)
    q = z(1:3);   dq = z(4:6);   p = z(7:9);
    td = dist_fn(t);
    [tau, ~, ~] = ndob_smc_eval(z, t, Lambda, K, delta, Lobs, ref_fn);
    M = thumb_M(q(1), q(2), q(3));
    [C_v, G_v] = thumb_CG(q(1), q(2), q(3), dq(1), dq(2), dq(3));
    qddot = M \ (tau + td - C_v - G_v);
    p_dot = -Lobs*p - Lobs*(Lobs*M*dq - C_v - G_v + tau);
    zdot = [dq; qddot; p_dot];
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
    t1 = segs(2);  t3 = segs(4);  t4 = segs(5);
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

function td = dist_with_load_scaled(t, t_load, load_scale)
    td_bg = [ 0.0020*sin(t)       + 0.0005*sin(200*pi*t);
              0.0010*cos(2*t)     + 0.0005*sin(200*pi*t);
              0.00015*sin(0.5*t)  + 0.00005*sin(200*pi*t) ];
    sigma   = 0.5*(1 + tanh((t - t_load)/0.2));
    td_load = load_scale * sigma * [-0.0030; -0.0030; -0.00020];
    td = td_bg + td_load;
end


function [Rms, Pk] = print_metrics(t, Q, Qref, Tau, segs, names)
    E = Q - Qref;
    P = length(segs) - 1;
    Rms = zeros(P,3);  Pk = zeros(P,3);
    fprintf('  %-22s  %10s %10s %10s\n', 'Phase', 'q1', 'q2', 'q3');
    for ph = 1:P
        idx = t >= segs(ph) & t < segs(ph+1);
        Rms(ph,:) = rms(E(idx,:), 1);
        Pk(ph,:)  = max(abs(E(idx,:)), [], 1);
        fprintf('  %-22s  %10.5f %10.5f %10.5f\n', names{ph}, ...
                Rms(ph,1), Rms(ph,2), Rms(ph,3));
    end
    pk = max(abs(Tau), [], 1);
    fprintf('  %-22s  %10.5f %10.5f %10.5f\n', '|tau|_max (N·m)', ...
            pk(1), pk(2), pk(3));
end

function plot_scenario_ndob(t, X, Qref, Tau, S, TAUD, TDH, segs, ph_col, ...
                            load_scale, L_diag, Knew_diag)
    set_plot_defaults();
    Q = X(:,1:3);
    P = length(segs) - 1;
    colors  = {[0.00 0.30 0.85], [0.90 0.10 0.10], [0.00 0.55 0.00]};
    ref_clr = [0.90 0.00 0.90];
    jnames  = {'Proximal q_1', 'Middle q_2', 'Distal q_3'};
    T = segs(end);

    % --- Tracking + error -------------------------------------------------
    fig = figure('Position',[60 60 1420 880],'Color','white');
    for ji = 1:3
        ax = subplot(2,3,ji); hold on;
        yl_pos = [min([Qref(:,ji);Q(:,ji)])-0.05, max([Qref(:,ji);Q(:,ji)])+0.05];
        for ph = 1:P
            patch(ax,[segs(ph) segs(ph+1) segs(ph+1) segs(ph)], ...
                  [yl_pos(1) yl_pos(1) yl_pos(2) yl_pos(2)], ph_col{ph}, ...
                  'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
        end
        plot(t, Qref(:,ji), '-', 'Color', ref_clr, 'LineWidth', 3.0, 'DisplayName','q_{ref}');
        plot(t, Q(:,ji),    '-', 'Color', colors{ji}, 'LineWidth', 2.2, 'DisplayName','NDOB+SMC');
        for ph = 2:P, xline(segs(ph),'--','Color',[0.5 0.5 0.5],'HandleVisibility','off'); end
        xlabel('Time (s)'); ylabel(sprintf('q_%d  (rad)',ji));
        title(sprintf('Joint %d — %s', ji, jnames{ji}));
        legend('Location','best','Box','on'); grid on; box on;
        xlim([0 T]); ylim(yl_pos);

        ax2 = subplot(2,3,3+ji); hold on;
        e = Q(:,ji) - Qref(:,ji);
        emax = max(abs(e))*1.1 + 1e-5;
        for ph = 1:P
            patch(ax2,[segs(ph) segs(ph+1) segs(ph+1) segs(ph)], ...
                  [-emax -emax emax emax], ph_col{ph}, ...
                  'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
        end
        plot(t, e, '-', 'Color', colors{ji}, 'LineWidth', 2.2);
        yline(0, ':', 'Color', [0.5 0.5 0.5]);
        for ph = 2:P, xline(segs(ph),'--','Color',[0.5 0.5 0.5],'HandleVisibility','off'); end
        xlabel('Time (s)'); ylabel(sprintf('e_%d  (rad)',ji));
        title(sprintf('Tracking Error  e_%d', ji));
        grid on; box on; xlim([0 T]);
    end
    sgtitle(sprintf('NDOB + SMC  —  Load Scale %.2f  |  L = %.0fI rad/s  |  K_new = diag(%.1f, %.1f, %.2f) mNm', ...
            load_scale, L_diag(1), Knew_diag*1000), ...
            'FontSize',16,'FontWeight','bold','Color','k');
    print(fig,'thumb_ndob_tracking','-dpng','-r150');
    fprintf('  Saved: thumb_ndob_tracking.png\n');

    % --- Disturbance estimation -------------------------------------------
    fig = figure('Position',[60 60 1420 560],'Color','white');
    for ji = 1:3
        subplot(1,3,ji); hold on;
        plot(t, TAUD(:,ji)*1000, '-', 'Color', [0.55 0.55 0.55], ...
             'LineWidth', 2.0, 'DisplayName','\tau_d (true)');
        plot(t, TDH(:,ji)*1000,  '-', 'Color', colors{ji}, ...
             'LineWidth', 1.8, 'DisplayName','\tau_d^{\^} (NDOB)');
        for ph = 2:P, xline(segs(ph),'--','Color',[0.5 0.5 0.5],'HandleVisibility','off'); end
        xlabel('Time (s)'); ylabel('Disturbance  (mN·m)');
        title(sprintf('Joint %d — NDOB Estimation', ji));
        legend('Location','best','Box','on'); grid on; box on; xlim([0 T]);
    end
    sgtitle('NDOB — True vs Estimated Disturbance', ...
            'FontSize',16,'FontWeight','bold','Color','k');
    print(fig,'thumb_ndob_disturbance_est','-dpng','-r150');
    fprintf('  Saved: thumb_ndob_disturbance_est.png\n');
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


% =========================================================================
%  DYNAMICS  (verbatim from run_thumb_smc_mb.m / run_thumb_extra_tests.m)
% =========================================================================
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
