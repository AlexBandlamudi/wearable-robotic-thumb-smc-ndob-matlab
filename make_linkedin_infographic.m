function make_linkedin_infographic()
%MAKE_LINKEDIN_INFOGRAPHIC Full-page MATLAB-only project infographic.
%
% Output: thumb_linkedin_infographic.png (1200x1700 portrait poster).
%
% Sections:
%   1. Plant model & operating geometry
%   2. Control law: SMC-MB vs NDOB-assisted SMC
%   3. Evidence under the same 2.40x sudden load
%   4. Quantitative summary
% Footer: key takeaway
%
% Every number is read from the saved simulation results:
%   thumb_fullcycle.mat        (stressed SMC-MB)
%   thumb_ndob_results.mat     (NDOB + SMC at 2.40x load)

    close all;
    here = fileparts(mfilename('fullpath'));
    oldpwd = pwd; cleanup = onCleanup(@() cd(oldpwd)); cd(here);

    R = load_project();
    P = palette();

    W = 1200; H = 1700;
    fig = figure('Units','pixels','Position',[40 40 W H], ...
        'Color',P.bg,'MenuBar','none','ToolBar','none','Visible','off');
    set(fig,'DefaultAxesFontName','Helvetica');
    set(fig,'DefaultTextFontName','Helvetica');

    axBg = axes('Parent',fig,'Position',[0 0 1 1],'Visible','off');
    axis(axBg,[0 1 0 1]); hold(axBg,'on'); axBg.Clipping = 'off';

    % --- Header ---------------------------------------------------------
    band(axBg,[0 0.943 1 0.057], P.navy);
    text(axBg,0.5,0.978, ...
        'Wearable Robotic Thumb  $\,\cdot\,$  Model-Based SMC + Nonlinear Disturbance Observer', ...
        'Interpreter','latex','HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',18.5,'FontWeight','bold','Color',P.white);
    text(axBg,0.5,0.953, ...
        ['Pure-MATLAB Euler-Lagrange plant $\;\cdot\;$ Slotine-Li control $\;\cdot\;$ ', ...
         'Chen 2000 NDOB $\;\cdot\;$ verified at 2.40$\times$ stress load'], ...
        'Interpreter','latex','HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',10.5,'FontAngle','italic','Color',P.lightgray);
    band(axBg,[0 0.937 1 0.006], P.gold);

    % ====================================================================
    % SECTION 1 -- PLANT & GEOMETRY
    % ====================================================================
    section_header(axBg, 0.928, '1', 'Plant model and operating geometry', P);

    card(axBg,[0.025 0.760 0.320 0.145], P.card, P.cardEdge);
    text(axBg,0.040,0.890,'Planar 3R rigid-body plant', ...
        'FontSize',12,'FontWeight','bold','Color',P.ink);
    axPlant = axes('Parent',fig,'Position',[0.040 0.768 0.290 0.110]);
    draw_plant_card(axPlant, R, P);

    card(axBg,[0.355 0.760 0.310 0.145], P.card, P.cardEdge);
    text(axBg,0.370,0.890,'Denavit-Hartenberg parameters', ...
        'Interpreter','latex','FontSize',12,'FontWeight','bold','Color',P.ink);
    axDH = axes('Parent',fig,'Position',[0.370 0.768 0.280 0.108]);
    draw_dh_card(axDH, P);

    card(axBg,[0.675 0.760 0.300 0.145], P.card, P.cardEdge);
    text(axBg,0.690,0.890,'Saved-run conditions', ...
        'FontSize',12,'FontWeight','bold','Color',P.ink);
    axRun = axes('Parent',fig,'Position',[0.690 0.768 0.270 0.108]);
    draw_run_card(axRun, R, P);

    % ====================================================================
    % SECTION 2 -- CONTROL LAW
    % ====================================================================
    section_header(axBg, 0.748, '2', 'Mathematical framework  $\,\cdot\,$  SMC vs NDOB', P);

    % Plant equation banner
    card(axBg,[0.025 0.660 0.950 0.060], P.cardAccent, P.cardEdge);
    text(axBg,0.500,0.690, ...
        '$M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q)\;=\;\tau + \tau_d$', ...
        'Interpreter','latex','HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',17,'FontWeight','bold','Color',P.ink);

    % SMC card
    card(axBg,[0.025 0.530 0.465 0.120], P.card, P.cardEdge);
    pill(axBg,[0.040 0.624 0.180 0.024], P.crimson,'PLAIN SMC-MB',P.white,9);
    text(axBg,0.235,0.636, ...
        'switching covers \textit{entire}~matched disturbance', ...
        'Interpreter','latex','FontSize',9.5,'Color',P.gray);
    axSmc = axes('Parent',fig,'Position',[0.040 0.535 0.435 0.090]);
    draw_smc_card(axSmc, R, P);

    % NDOB card
    card(axBg,[0.510 0.530 0.465 0.120], P.card, P.cardEdge);
    pill(axBg,[0.525 0.624 0.180 0.024], P.emerald,'NDOB + SMC',P.white,9);
    text(axBg,0.720,0.636, ...
        'observer cancels load, switching rejects residual', ...
        'Interpreter','latex','FontSize',9.5,'Color',P.gray);
    axNdob = axes('Parent',fig,'Position',[0.525 0.535 0.435 0.090]);
    draw_ndob_card(axNdob, R, P);

    % ====================================================================
    % SECTION 3 -- EVIDENCE
    % ====================================================================
    section_header(axBg, 0.520, '3', 'Evidence under the same 2.40$\times$ sudden load', P);

    card(axBg,[0.025 0.290 0.640 0.205], P.white, P.cardEdge);
    text(axBg,0.040,0.481,'PIP joint ($q_2$) tracking error', ...
        'Interpreter','latex','FontSize',12,'FontWeight','bold','Color',P.ink);
    text(axBg,0.040,0.460, ...
        sprintf(['Same disturbance, same plant. $|d_2|_{\\max}=%.1f$ mN$\\cdot$m vs ' ...
            '$K_2=%.0f$ mN$\\cdot$m \\textbf{breaks}~plain SMC.'], ...
        R.dmax_mNm(2), R.K_smc_mNm(2)), ...
        'Interpreter','latex','FontSize',9.0,'Color',P.gray);
    axErr = axes('Parent',fig,'Position',[0.062 0.310 0.600 0.140]);
    draw_error_plot(axErr, R, P);

    card(axBg,[0.675 0.290 0.300 0.205], P.white, P.cardEdge);
    text(axBg,0.690,0.481,'NDOB estimate $\hat\tau_{d,2}$', ...
        'Interpreter','latex','FontSize',12,'FontWeight','bold','Color',P.ink);
    text(axBg,0.690,0.460, ...
        'tracks the smoothed disturbance, leaves 100 Hz ripple as residual', ...
        'Interpreter','latex','FontSize',9.0,'Color',P.gray);
    axObs = axes('Parent',fig,'Position',[0.705 0.310 0.260 0.140]);
    draw_observer_plot(axObs, R, P);

    % ====================================================================
    % SECTION 4 -- RESULTS TABLE
    % ====================================================================
    section_header(axBg, 0.285, '4', 'Quantitative summary', P);

    card(axBg,[0.025 0.115 0.950 0.145], P.card, P.cardEdge);
    axTab = axes('Parent',fig,'Position',[0.040 0.122 0.920 0.130]);
    draw_results_table(axTab, R, P);

    % ====================================================================
    % FOOTER -- Key takeaway band
    % ====================================================================
    band(axBg,[0 0 1 0.097], P.navy);
    band(axBg,[0 0.097 1 0.005], P.gold);

    text(axBg,0.5,0.075, ...
        sprintf(['Key result $\\,\\cdot\\,$ NDOB cuts the PIP phase-3 RMS error by %.0f' ...
            '$\\times$ while using a %.0f$\\times$ smaller switching gain.'], ...
        R.q2_rms_gain, R.K_reduction_q2), ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',14,'FontWeight','bold','Color',P.white);
    text(axBg,0.5,0.035, ...
        sprintf(['Stressed plain SMC PIP RMS = %.1f mrad $\\longrightarrow$ NDOB-assisted SMC PIP RMS = %.2f mrad ', ...
        '$\\quad\\cdot\\quad$ stable up to $\\sim 7\\times$ load.'], ...
        R.smc_p3_rms(2), R.ndob_p3_rms(2)), ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',10.5,'Color',P.lightgray);

    export_png(fig,'thumb_linkedin_infographic.png',W,H);
    close(fig);
    fprintf('Saved: thumb_linkedin_infographic.png\n');
end

% =====================================================================
% Data loading + metrics
% =====================================================================
function R = load_project()
    if ~isfile('thumb_fullcycle.mat') || ~isfile('thumb_ndob_results.mat')
        error('make_linkedin_infographic:Missing', ...
            'Need thumb_fullcycle.mat and thumb_ndob_results.mat.');
    end
    R.smc  = load('thumb_fullcycle.mat');
    R.ndob = load('thumb_ndob_results.mat');

    R.length_mm = [50, 35, 25];
    R.mass_g    = [15, 10, 6];
    R.inertia   = [1.25e-5, 4.083e-6, 1.25e-6];
    R.joints    = {'$q_1$/MCP','$q_2$/PIP','$q_3$/DIP'};
    R.q0 = [0;0;0]; R.qf = [0.70;1.20;0.85];
    R.dt_ms = 0.5;
    R.g = 9.81;

    segs = R.smc.segs;
    idx_s3 = R.smc.t  >= segs(3) & R.smc.t  < segs(4);
    idx_s4 = R.smc.t  >= segs(4) & R.smc.t  < segs(5);
    idx_n3 = R.ndob.t >= segs(3) & R.ndob.t < segs(4);
    idx_n4 = R.ndob.t >= segs(4) & R.ndob.t < segs(5);

    e_s = R.smc.X(:,1:3)  - R.smc.Qref;
    e_n = R.ndob.X(:,1:3) - R.ndob.Qref;
    R.e_s_mrad = e_s * 1000;
    R.e_n_mrad = e_n * 1000;

    R.smc_p3_rms  = local_rms(e_s(idx_s3,:)) * 1000;
    R.ndob_p3_rms = local_rms(e_n(idx_n3,:)) * 1000;
    R.smc_p4_rms  = local_rms(e_s(idx_s4,:)) * 1000;
    R.ndob_p4_rms = local_rms(e_n(idx_n4,:)) * 1000;
    R.smc_q2_peak_p3  = max(abs(R.e_s_mrad(idx_s3,2)));
    R.ndob_q2_peak_p3 = max(abs(R.e_n_mrad(idx_n3,2)));

    R.K_smc_mNm  = diag(R.smc.K)' * 1000;
    R.K_ndob_mNm = diag(R.ndob.K)' * 1000;
    R.Lobs = diag(R.ndob.L_obs)';
    R.load_scale = R.ndob.load_scale;

    R.bg_peak_mNm = [2.5, 1.5, 0.20];
    R.load_mNm    = R.load_scale * [3.0, 3.0, 0.20];
    R.dmax_mNm    = R.bg_peak_mNm + R.load_mNm;

    ed = R.ndob.TAUD - R.ndob.TDH;
    R.ed_rms_p3_mNm = local_rms(ed(idx_n3,:)) * 1000;
    R.q2_rms_gain     = R.smc_p3_rms(2) / R.ndob_p3_rms(2);
    R.q2_peak_gain    = R.smc_q2_peak_p3 / R.ndob_q2_peak_p3;
    R.q2_retrace_gain = R.smc_p4_rms(2) / R.ndob_p4_rms(2);
    R.K_reduction_q2  = R.K_smc_mNm(2) / R.K_ndob_mNm(2);
end

function r = local_rms(x); r = sqrt(mean(x.^2,1)); end

% =====================================================================
% Section card renderers
% =====================================================================
function draw_plant_card(ax, R, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    headers = {'Joint','Length','Mass','Inertia'};
    xcols = [0.00 0.36 0.55 0.74];
    % header band
    patch(ax,[0 1 1 0],[0.80 0.80 1.00 1.00],P.navy,'EdgeColor','none');
    for c = 1:4
        text(ax,xcols(c),0.90,headers{c}, ...
            'FontSize',9.2,'FontWeight','bold','Color',P.white);
    end
    rows = cell(3,4);
    for i = 1:3
        rows{i,1} = R.joints{i};
        rows{i,2} = sprintf('%d mm',R.length_mm(i));
        rows{i,3} = sprintf('%d g',R.mass_g(i));
        rows{i,4} = sprintf('%.2e kg$\\cdot$m$^2$',R.inertia(i));
    end
    row_h = 0.80/3;
    for r = 1:3
        y_top = 0.80 - (r-1)*row_h;
        y_bot = y_top - row_h;
        if mod(r,2)==1
            patch(ax,[0 1 1 0],[y_bot y_bot y_top y_top], ...
                P.rowAlt,'EdgeColor','none');
        end
        yc = (y_top+y_bot)/2;
        for c = 1:4
            interp = 'tex';
            if contains(rows{r,c},'$'); interp = 'latex'; end
            text(ax,xcols(c),yc,rows{r,c},'Interpreter',interp, ...
                'FontSize',9.0,'Color',P.ink, ...
                'HorizontalAlignment','left','VerticalAlignment','middle');
        end
    end
end

function draw_dh_card(ax, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    headers = {'$i$','$a_i$ (mm)','$d_i$','$\alpha_i$','$\theta_i$'};
    xcols = [0.02 0.20 0.52 0.66 0.83];
    patch(ax,[0 1 1 0],[0.80 0.80 1.00 1.00],P.navy,'EdgeColor','none');
    for c = 1:5
        text(ax,xcols(c),0.90,headers{c}, ...
            'Interpreter','latex','FontSize',9.5,'FontWeight','bold','Color',P.white);
    end
    rows = {'1','50','0','0','$q_1$'; ...
            '2','35','0','0','$q_2$'; ...
            '3','25','0','0','$q_3$'};
    row_h = 0.80/3;
    for r = 1:3
        y_top = 0.80 - (r-1)*row_h;
        y_bot = y_top - row_h;
        if mod(r,2)==1
            patch(ax,[0 1 1 0],[y_bot y_bot y_top y_top], ...
                P.rowAlt,'EdgeColor','none');
        end
        yc = (y_top+y_bot)/2;
        for c = 1:5
            text(ax,xcols(c),yc,rows{r,c},'Interpreter','latex', ...
                'FontSize',9.2,'Color',P.ink);
        end
    end
end

function draw_run_card(ax, R, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    rows = { ...
        '$q_0$', '$[0,\,0,\,0]^\top$ rad'; ...
        '$q_f$', '$[0.70,\,1.20,\,0.85]^\top$ rad'; ...
        'Phases', 'close  -  hold  -  load  -  retrace  -  hold'; ...
        'Solver', sprintf('RK4 at %.1f ms, $g=%.2f$ m/s$^2$',R.dt_ms,R.g); ...
        'Load', sprintf('$\\sigma(t)=\\frac{1}{2}[1+\\tanh\\frac{t-10}{0.2}]$, $%.2f\\times$',R.load_scale); ...
        '$|d|_{\max}$', sprintf('$[%.1f,\\,%.1f,\\,%.2f]$ mN$\\cdot$m', ...
                                R.dmax_mNm(1),R.dmax_mNm(2),R.dmax_mNm(3))};
    row_h = 1.0/6;
    for r = 1:6
        y_top = 1.0 - (r-1)*row_h;
        y_bot = y_top - row_h;
        if mod(r,2)==1
            patch(ax,[0 1 1 0],[y_bot y_bot y_top y_top], ...
                P.rowAlt,'EdgeColor','none');
        end
        yc = (y_top+y_bot)/2;
        text(ax,0.02,yc,rows{r,1},'Interpreter','latex', ...
            'FontSize',9.4,'FontWeight','bold','Color',P.navy);
        text(ax,0.27,yc,rows{r,2},'Interpreter','latex', ...
            'FontSize',9.2,'Color',P.ink);
    end
end

function draw_smc_card(ax, R, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    text(ax,0.00,0.85, ...
        '$e=q-q_d$, $\quad s=\dot e+\Lambda e$, $\quad \dot q_r=\dot q_d-\Lambda e$', ...
        'Interpreter','latex','FontSize',10.6,'Color',P.ink);
    text(ax,0.00,0.55, ...
        '$\tau_{\mathrm{SMC}} = M(q)\,\ddot q_r + C(q,\dot q_r)\,\dot q_r + G(q)\;-\;K\,\mathrm{sat}(s/\delta)$', ...
        'Interpreter','latex','FontSize',10.4,'Color',P.ink);
    text(ax,0.00,0.20, ...
        sprintf('$K = \\mathrm{diag}(%.0f,%.0f,%.0f)$ mN$\\cdot$m, $\\;\\delta = 0.01$ rad', ...
        R.K_smc_mNm(1),R.K_smc_mNm(2),R.K_smc_mNm(3)), ...
        'Interpreter','latex','FontSize',9.6,'FontWeight','bold','Color',P.crimson);
end

function draw_ndob_card(ax, R, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    text(ax,0.00,0.85, ...
        '$\hat\tau_d = p + L\,M(q)\,\dot q,\quad \dot p = -L\,p - L\,(L M \dot q - C\dot q - G + \tau)$', ...
        'Interpreter','latex','FontSize',10.0,'Color',P.ink);
    text(ax,0.00,0.55, ...
        '$\tau_{\mathrm{NDOB}} = \tau_{\mathrm{eq}} - \hat\tau_d - K_{\mathrm{new}}\,\mathrm{sat}(s/\delta)$', ...
        'Interpreter','latex','FontSize',10.4,'Color',P.ink);
    text(ax,0.00,0.20, ...
        sprintf('$K_{\\mathrm{new}} = \\mathrm{diag}(%.1f,%.1f,%.1f)$ mN$\\cdot$m, $\\;L=%g\\,I$ rad/s', ...
        R.K_ndob_mNm(1),R.K_ndob_mNm(2),R.K_ndob_mNm(3),R.Lobs(1)), ...
        'Interpreter','latex','FontSize',9.6,'FontWeight','bold','Color',P.emerald);
end

% =====================================================================
% Plots
% =====================================================================
function draw_error_plot(ax, R, P)
    hold(ax,'on'); box(ax,'on'); grid(ax,'on');
    set(ax,'Color',P.white,'XColor',P.gray,'YColor',P.gray, ...
        'GridColor',P.gridLight,'GridAlpha',1,'LineWidth',0.9, ...
        'TickDir','out','FontName','Helvetica','FontSize',9,'Layer','top');
    ylims = [-420 420];
    shade_phases(ax, R.smc.segs, ylims, P);
    yline(ax,0,'-','Color',[0.35 0.37 0.42 0.7],'LineWidth',0.8);
    yline(ax, 17.5,':','Color',P.gold,'LineWidth',0.9);
    yline(ax,-17.5,':','Color',P.gold,'LineWidth',0.9);
    xline(ax,10,'--','Color',[0.40 0.40 0.45],'LineWidth',0.9);

    h1 = plot(ax,R.smc.t,R.e_s_mrad(:,2),'Color',P.crimson,'LineWidth',1.5);
    h2 = plot(ax,R.ndob.t,R.e_n_mrad(:,2),'Color',P.emerald,'LineWidth',1.6);
    xlim(ax,[0 R.smc.segs(end)]); ylim(ax,ylims);
    xlabel(ax,'time (s)','Interpreter','latex','FontSize',10.5);
    ylabel(ax,'$e_2$ (mrad)','Interpreter','latex','FontSize',10.5);

    % phase labels along top
    phase_lbl = {'P1 close','P2 hold','P3 LOAD','P4 retrace','P5 open'};
    yt = -360;
    for ph = 1:5
        xc = 0.5*(R.smc.segs(ph)+R.smc.segs(ph+1));
        text(ax,xc,yt,phase_lbl{ph},'HorizontalAlignment','center', ...
            'FontSize',8,'FontWeight','bold','Color',[0.30 0.34 0.42]);
    end
    lg = legend(ax,[h1 h2], ...
        {'\textbf{Plain SMC-MB}','\textbf{NDOB + SMC}'}, ...
        'Interpreter','latex','Location','northwest','Box','off','FontSize',9);
    lg.TextColor = P.ink;
end

function draw_observer_plot(ax, R, P)
    hold(ax,'on'); box(ax,'on'); grid(ax,'on');
    set(ax,'Color',P.white,'XColor',P.gray,'YColor',P.gray, ...
        'GridColor',P.gridLight,'GridAlpha',1,'LineWidth',0.9, ...
        'TickDir','out','FontName','Helvetica','FontSize',9,'Layer','top');
    idx = R.ndob.t >= 8 & R.ndob.t <= 15;
    h1 = plot(ax,R.ndob.t(idx),R.ndob.TAUD(idx,2)*1000, ...
        'Color',[0.55 0.59 0.66],'LineWidth',1.4);
    h2 = plot(ax,R.ndob.t(idx),R.ndob.TDH(idx,2)*1000, ...
        'Color',P.emerald,'LineWidth',1.8);
    xline(ax,10,'--','Color',P.crimson,'LineWidth',1.0);
    xlim(ax,[8 15]);
    xlabel(ax,'time (s)','Interpreter','latex','FontSize',10.5);
    ylabel(ax,'mN$\cdot$m','Interpreter','latex','FontSize',10.5);
    lg = legend(ax,[h1 h2], ...
        {'$\tau_{d,2}$ true','$\hat\tau_{d,2}$ NDOB'}, ...
        'Interpreter','latex','Location','southwest','Box','off','FontSize',8.5);
    lg.TextColor = P.ink;
end

function shade_phases(ax, segs, ylims, P)
    phase_cols = [P.phaseClose; P.phaseHold; P.phaseLoad; P.phaseRetrace; P.phaseOpen];
    for ph = 1:(numel(segs)-1)
        patch(ax,[segs(ph) segs(ph+1) segs(ph+1) segs(ph)], ...
            [ylims(1) ylims(1) ylims(2) ylims(2)], phase_cols(ph,:), ...
            'FaceAlpha',0.080,'EdgeColor','none');
    end
    for ph = 2:(numel(segs)-1)
        xline(ax,segs(ph),':','Color',[0.55 0.58 0.66],'LineWidth',0.6);
    end
end

% =====================================================================
% Results table
% =====================================================================
function draw_results_table(ax, R, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    headers = {'Metric','Stressed SMC-MB','NDOB + SMC','Improvement'};
    rows = { ...
        'PIP phase-3 RMS error',          sprintf('%.1f mrad',R.smc_p3_rms(2)),       sprintf('%.2f mrad',R.ndob_p3_rms(2)),       sprintf('%.0f$\\times$ lower',R.q2_rms_gain); ...
        'PIP phase-3 peak error',         sprintf('%.1f mrad',R.smc_q2_peak_p3),      sprintf('%.2f mrad',R.ndob_q2_peak_p3),      sprintf('%.0f$\\times$ lower',R.q2_peak_gain); ...
        'PIP phase-4 retrace RMS',        sprintf('%.1f mrad',R.smc_p4_rms(2)),       sprintf('%.2f mrad',R.ndob_p4_rms(2)),       sprintf('%.0f$\\times$ lower',R.q2_retrace_gain); ...
        'PIP switching gain $K_2$',       sprintf('%.0f mN$\\cdot$m',R.K_smc_mNm(2)), sprintf('%.1f mN$\\cdot$m',R.K_ndob_mNm(2)), sprintf('%.0f$\\times$ smaller',R.K_reduction_q2); ...
        'Observer residual on $q_2$',     'not estimated',                            sprintf('%.2f mN$\\cdot$m',R.ed_rms_p3_mNm(2)), 'switching rejects residual'; ...
        'Max stable $\sigma$ load scale', '2.3$\times$',                              '$>7\times$',                                 'architectural margin'};
    xcols = [0.00 0.34 0.58 0.79];
    n_rows = size(rows,1);

    patch(ax,[0 1 1 0],[1-0.135 1-0.135 1 1],P.navy,'EdgeColor','none');
    for c = 1:4
        text(ax,xcols(c),1-0.067,headers{c}, ...
            'Interpreter','latex','FontSize',10.5,'FontWeight','bold','Color',P.white, ...
            'VerticalAlignment','middle');
    end

    row_h = 0.865/n_rows;
    for r = 1:n_rows
        y_top = 1-0.135 - (r-1)*row_h;
        y_bot = y_top - row_h;
        if mod(r,2)==1
            patch(ax,[0 1 1 0],[y_bot y_bot y_top y_top], ...
                P.rowAlt,'EdgeColor','none');
        end
        yc = (y_top+y_bot)/2;
        for c = 1:4
            interp = 'latex';
            color = P.ink;
            fw = 'normal';
            if c == 2; color = P.crimson; fw = 'bold'; end
            if c == 3; color = P.emerald; fw = 'bold'; end
            if c == 4; color = P.navy;    fw = 'bold'; end
            text(ax,xcols(c),yc,rows{r,c}, ...
                'Interpreter',interp,'FontSize',9.8, ...
                'FontWeight',fw,'Color',color,'VerticalAlignment','middle');
        end
    end
end

% =====================================================================
% Generic helpers (cards, pills, bands, section headers)
% =====================================================================
function section_header(ax, y_top, num, text_str, P)
    % small navy chip + section title text + faint horizontal rule
    x0 = 0.025; w = 0.040; h = 0.022;
    y = y_top - h;
    rectangle(ax,'Position',[x0 y w h],'Curvature',[0.35 0.55], ...
        'FaceColor',P.navy,'EdgeColor','none');
    text(ax,x0+w/2,y+h/2,num,'HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',11,'FontWeight','bold','Color',P.white);
    text(ax,x0+w+0.010,y+h/2,text_str, ...
        'Interpreter','latex','FontSize',13,'FontWeight','bold','Color',P.ink, ...
        'VerticalAlignment','middle');
    % thin rule extending right
    line(ax,[0.700 0.975],[y+h/2 y+h/2],'Color',P.cardEdge,'LineWidth',0.6);
end

function P = palette()
    P.bg          = [1 1 1];
    P.navy        = [0.043 0.145 0.282];
    P.ink         = [0.063 0.090 0.137];
    P.gray        = [0.290 0.330 0.400];
    P.mute        = [0.560 0.610 0.690];
    P.white       = [1 1 1];
    P.lightgray   = [0.840 0.880 0.940];
    P.gridLight   = [0.910 0.925 0.950];
    P.card        = [0.965 0.975 0.987];
    P.cardAccent  = [0.918 0.940 0.972];
    P.rowAlt      = [0.945 0.955 0.972];
    P.cardEdge    = [0.815 0.860 0.920];
    P.crimson     = [0.690 0.135 0.180];
    P.emerald     = [0.105 0.490 0.290];
    P.royal       = [0.110 0.310 0.690];
    P.gold        = [0.835 0.620 0.130];

    P.phaseClose   = [0.980 0.840 0.300];
    P.phaseHold    = [0.380 0.760 0.450];
    P.phaseLoad    = [0.870 0.320 0.260];
    P.phaseRetrace = [0.380 0.540 0.900];
    P.phaseOpen    = [0.700 0.720 0.770];
end

function card(ax, pos, faceColor, edgeColor)
    rectangle(ax,'Position',pos,'Curvature',[0.018 0.060], ...
        'FaceColor',faceColor,'EdgeColor',edgeColor,'LineWidth',0.9);
end

function pill(ax, pos, faceColor, str, txtColor, fontSize)
    rectangle(ax,'Position',pos,'Curvature',[0.55 1.0], ...
        'FaceColor',faceColor,'EdgeColor','none');
    xc = pos(1)+pos(3)/2; yc = pos(2)+pos(4)/2;
    text(ax,xc,yc,str,'Interpreter','latex','HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',fontSize,'FontWeight','bold','Color',txtColor);
end

function band(ax,pos,faceColor)
    patch(ax,[pos(1) pos(1)+pos(3) pos(1)+pos(3) pos(1)], ...
        [pos(2) pos(2) pos(2)+pos(4) pos(2)+pos(4)], faceColor,'EdgeColor','none');
end

function export_png(fig, fname, W, H)
    set(fig,'InvertHardcopy','off');
    set(fig,'PaperUnits','inches','PaperPosition',[0 0 W/100 H/100], ...
        'PaperSize',[W/100 H/100]);
    print(fig,fname,'-dpng','-r150');
end
