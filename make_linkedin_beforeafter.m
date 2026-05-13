function make_linkedin_beforeafter()
%MAKE_LINKEDIN_BEFOREAFTER Before/after slide -- stressed SMC vs NDOB recovery.
% Output: thumb_linkedin_beforeafter.png (1200x627, LinkedIn-friendly).
%
% All data comes from the saved .mat files written by the simulation runs:
%   thumb_fullcycle.mat        (SMC-MB at LOAD_SCALE = 2.40)
%   thumb_ndob_results.mat     (NDOB + SMC at LOAD_SCALE = 2.40, K_new diag(2,2,0.3) mN.m)

    close all;
    here = fileparts(mfilename('fullpath'));
    oldpwd = pwd; cleanup = onCleanup(@() cd(oldpwd)); cd(here);

    smc  = load_required('thumb_fullcycle.mat');
    ndob = load_required('thumb_ndob_results.mat');
    M = comparison_metrics(smc, ndob);
    P = palette();

    W = 1200; H = 627;
    fig = figure('Units','pixels','Position',[60 60 W H], ...
        'Color',P.bg,'MenuBar','none','ToolBar','none','Visible','off');
    set(fig,'DefaultAxesFontName','Helvetica');
    set(fig,'DefaultTextFontName','Helvetica');

    axBg = axes('Parent',fig,'Position',[0 0 1 1],'Visible','off');
    axis(axBg,[0 1 0 1]); hold(axBg,'on'); axBg.Clipping = 'off';

    % --- Header band -----------------------------------------------------
    band(axBg,[0 0.880 1 0.120], P.navy);
    text(axBg,0.5,0.948, ...
        'PIP/$q_2$ tracking under a 2.40$\times$ sudden grip-load', ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',20,'FontWeight','bold','Color',P.white);
    text(axBg,0.5,0.907, ...
        'Plain model-based SMC loses the reaching margin; an NDOB cancels the load before switching.', ...
        'HorizontalAlignment','center','FontSize',11,'FontAngle','italic','Color',P.lightgray);
    band(axBg,[0 0.872 1 0.008], P.gold);

    % --- Diagnosis strip (between header and plots) ---------------------
    text(axBg,0.5,0.835, ...
        sprintf(['Channel-2 reaching condition: $|d_2|_{\\max}=%.1f$ mN$\\cdot$m ' ...
            '\\textbf{vs}~$K_2=%.0f$ mN$\\cdot$m   $\\Rightarrow$   $\\dot V<0$ violated'], ...
        M.dmax_mNm(2), M.K_smc_mNm(2)), ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',12,'FontWeight','bold','Color',P.ink);

    % --- Left plot card: stressed SMC -----------------------------------
    card(axBg,[0.030 0.300 0.435 0.490], P.card, P.crimsonEdge);
    pill(axBg,[0.040 0.755 0.180 0.034],P.crimson,'BEFORE',P.white,10);
    text(axBg,0.225,0.772, 'Plain SMC-MB', ...
        'Interpreter','latex','FontSize',13.5,'FontWeight','bold','Color',P.ink);

    axL = axes('Parent',fig,'Position',[0.066 0.345 0.380 0.380]);
    plot_error_panel(axL, smc.t, M.smc_e2_mrad, smc.segs, [-420 420], P.crimson, P);

    % --- Right plot card: NDOB recovery ---------------------------------
    card(axBg,[0.535 0.300 0.435 0.490], P.card, P.emeraldEdge);
    pill(axBg,[0.545 0.755 0.180 0.034],P.emerald,'AFTER',P.white,10);
    text(axBg,0.730,0.772, 'NDOB + SMC', ...
        'Interpreter','latex','FontSize',13.5,'FontWeight','bold','Color',P.ink);

    axR = axes('Parent',fig,'Position',[0.571 0.345 0.380 0.380]);
    plot_error_panel(axR, ndob.t, M.ndob_e2_mrad, smc.segs, [-25 25], P.emerald, P);

    % --- VS callout chip in the centre ----------------------------------
    chip_circle(axBg,0.500,0.535,0.043,P.navy,P.white,'\boldmath$\rightarrow$',P);

    % --- Bottom band: 3 KPI tiles ---------------------------------------
    tile(axBg,[0.030 0.045 0.300 0.220], P.card, P.cardEdge);
    tile(axBg,[0.355 0.045 0.290 0.220], P.card, P.cardEdge);
    tile(axBg,[0.670 0.045 0.300 0.220], P.card, P.cardEdge);

    % Tile 1: q2 RMS
    text(axBg,0.180,0.244,'PIP phase-3 RMS error', ...
        'HorizontalAlignment','center','FontSize',10.5,'FontWeight','bold','Color',P.gray);
    text(axBg,0.110,0.158, sprintf('%.1f',M.smc_p3_rms(2)), ...
        'HorizontalAlignment','center','FontSize',24,'FontWeight','bold','Color',P.crimson);
    text(axBg,0.110,0.093,'mrad','HorizontalAlignment','center','FontSize',9,'Color',P.gray);
    text(axBg,0.180,0.130,'$\rightarrow$','Interpreter','latex', ...
        'HorizontalAlignment','center','FontSize',14,'Color',P.ink);
    text(axBg,0.265,0.158, sprintf('%.2f',M.ndob_p3_rms(2)), ...
        'HorizontalAlignment','center','FontSize',24,'FontWeight','bold','Color',P.emerald);
    text(axBg,0.265,0.093,'mrad','HorizontalAlignment','center','FontSize',9,'Color',P.gray);
    text(axBg,0.180,0.060, sprintf('%.0f$\\times$ lower',M.q2_rms_gain), ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',10.5,'FontWeight','bold','Color',P.navy);

    % Tile 2: q2 peak
    text(axBg,0.500,0.244,'PIP phase-3 peak error', ...
        'HorizontalAlignment','center','FontSize',10.5,'FontWeight','bold','Color',P.gray);
    text(axBg,0.430,0.158, sprintf('%.0f',M.smc_q2_peak_p3), ...
        'HorizontalAlignment','center','FontSize',24,'FontWeight','bold','Color',P.crimson);
    text(axBg,0.430,0.093,'mrad','HorizontalAlignment','center','FontSize',9,'Color',P.gray);
    text(axBg,0.500,0.130,'$\rightarrow$','Interpreter','latex', ...
        'HorizontalAlignment','center','FontSize',14,'Color',P.ink);
    text(axBg,0.580,0.158, sprintf('%.1f',M.ndob_q2_peak_p3), ...
        'HorizontalAlignment','center','FontSize',24,'FontWeight','bold','Color',P.emerald);
    text(axBg,0.580,0.093,'mrad','HorizontalAlignment','center','FontSize',9,'Color',P.gray);
    text(axBg,0.500,0.060, sprintf('%.0f$\\times$ lower',M.q2_peak_gain), ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',10.5,'FontWeight','bold','Color',P.navy);

    % Tile 3: switching gain reduction
    text(axBg,0.820,0.244,'PIP switching gain $K_2$', ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',10.5,'FontWeight','bold','Color',P.gray);
    text(axBg,0.750,0.158, sprintf('%.0f',M.K_smc_mNm(2)), ...
        'HorizontalAlignment','center','FontSize',24,'FontWeight','bold','Color',P.crimson);
    text(axBg,0.750,0.093,'mN$\cdot$m','Interpreter','latex', ...
        'HorizontalAlignment','center','FontSize',9,'Color',P.gray);
    text(axBg,0.820,0.130,'$\rightarrow$','Interpreter','latex', ...
        'HorizontalAlignment','center','FontSize',14,'Color',P.ink);
    text(axBg,0.905,0.158, sprintf('%.1f',M.K_ndob_mNm(2)), ...
        'HorizontalAlignment','center','FontSize',24,'FontWeight','bold','Color',P.emerald);
    text(axBg,0.905,0.093,'mN$\cdot$m','Interpreter','latex', ...
        'HorizontalAlignment','center','FontSize',9,'Color',P.gray);
    text(axBg,0.820,0.060, sprintf('%.0f$\\times$ smaller',M.K_reduction_q2), ...
        'Interpreter','latex','HorizontalAlignment','center', ...
        'FontSize',10.5,'FontWeight','bold','Color',P.navy);

    export_png(fig,'thumb_linkedin_beforeafter.png',W,H);
    close(fig);
    fprintf('Saved: thumb_linkedin_beforeafter.png\n');
end

% =====================================================================
function plot_error_panel(ax, t, e_mrad, segs, ylims, line_color, P)
    hold(ax,'on'); box(ax,'on'); grid(ax,'on');
    set(ax,'Color',P.white,'XColor',P.gray,'YColor',P.gray, ...
        'GridColor',P.gridLight,'GridAlpha',1,'LineWidth',0.9, ...
        'TickDir','out','FontName','Helvetica','FontSize',9.0,'Layer','top');

    shade_phases(ax, segs, ylims, P);
    yline(ax,0,'-','Color',[0.35 0.37 0.42 0.7],'LineWidth',0.8);
    yline(ax, 17.5,':','Color',P.gold,'LineWidth',1.0);
    yline(ax,-17.5,':','Color',P.gold,'LineWidth',1.0);
    xline(ax,10,'--','Color',[0.40 0.40 0.45],'LineWidth',0.9);

    plot(ax,t,e_mrad,'Color',line_color,'LineWidth',1.6);
    xlim(ax,[segs(1) segs(end)]); ylim(ax,ylims);

    xlabel(ax,'Time (s)','Interpreter','latex','FontSize',11,'Color',P.ink);
    ylabel(ax,'$e_2=q_2-q_{d,2}$ (mrad)','Interpreter','latex','FontSize',11,'Color',P.ink);

    % phase labels at top
    yt = ylims(1) + 0.93*(ylims(2)-ylims(1));
    phase_lbl = {'P1','P2','P3','P4','P5'};
    for ph = 1:5
        xc = 0.5*(segs(ph)+segs(ph+1));
        text(ax,xc,yt,phase_lbl{ph},'HorizontalAlignment','center', ...
            'FontSize',8.5,'FontWeight','bold','Color',[0.40 0.43 0.50]);
    end
    % tactile threshold label -- only for low-range panel
    if ylims(2) <= 30
        text(ax,segs(1)+0.3, 14.5,'human tactile $\sim$17.5 mrad', ...
            'Interpreter','latex','HorizontalAlignment','left', ...
            'FontSize',7.5,'Color',P.gold);
    end
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
function S = load_required(fname)
    if ~isfile(fname)
        error('make_linkedin_beforeafter:Missing','Missing %s -- run the simulation first.',fname);
    end
    S = load(fname);
end

function M = comparison_metrics(smc, ndob)
    segs = smc.segs;
    idx_s = smc.t >= segs(3) & smc.t < segs(4);
    idx_n = ndob.t >= segs(3) & ndob.t < segs(4);
    e_s = smc.X(:,1:3) - smc.Qref;
    e_n = ndob.X(:,1:3) - ndob.Qref;
    M.smc_e2_mrad = e_s(:,2) * 1000;
    M.ndob_e2_mrad = e_n(:,2) * 1000;
    M.smc_p3_rms = local_rms(e_s(idx_s,:)) * 1000;
    M.ndob_p3_rms = local_rms(e_n(idx_n,:)) * 1000;
    M.smc_q2_peak_p3 = max(abs(M.smc_e2_mrad(idx_s)));
    M.ndob_q2_peak_p3 = max(abs(M.ndob_e2_mrad(idx_n)));
    M.q2_rms_gain = M.smc_p3_rms(2) / M.ndob_p3_rms(2);
    M.q2_peak_gain = M.smc_q2_peak_p3 / M.ndob_q2_peak_p3;
    M.K_smc_mNm = diag(smc.K)' * 1000;
    M.K_ndob_mNm = diag(ndob.K)' * 1000;
    M.K_reduction_q2 = M.K_smc_mNm(2) / M.K_ndob_mNm(2);
    M.load_scale = ndob.load_scale;
    M.bg_peak_mNm = [2.5, 1.5, 0.20];
    M.load_mNm = M.load_scale * [3.0, 3.0, 0.20];
    M.dmax_mNm = M.bg_peak_mNm + M.load_mNm;
end

function r = local_rms(x); r = sqrt(mean(x.^2,1)); end

% =====================================================================
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
    P.cardEdge    = [0.815 0.860 0.920];
    P.crimson     = [0.690 0.135 0.180];
    P.crimsonEdge = [0.820 0.620 0.600];
    P.emerald     = [0.105 0.490 0.290];
    P.emeraldEdge = [0.620 0.800 0.700];
    P.royal       = [0.110 0.310 0.690];
    P.gold        = [0.835 0.620 0.130];

    P.phaseClose   = [0.980 0.840 0.300];
    P.phaseHold    = [0.380 0.760 0.450];
    P.phaseLoad    = [0.870 0.320 0.260];
    P.phaseRetrace = [0.380 0.540 0.900];
    P.phaseOpen    = [0.700 0.720 0.770];
end

function card(ax, pos, faceColor, edgeColor)
    rectangle(ax,'Position',pos,'Curvature',[0.025 0.060], ...
        'FaceColor',faceColor,'EdgeColor',edgeColor,'LineWidth',1.4);
end

function tile(ax, pos, faceColor, edgeColor)
    rectangle(ax,'Position',pos,'Curvature',[0.040 0.110], ...
        'FaceColor',faceColor,'EdgeColor',edgeColor,'LineWidth',0.9);
end

function pill(ax, pos, faceColor, str, txtColor, fontSize)
    rectangle(ax,'Position',pos,'Curvature',[0.50 1.0], ...
        'FaceColor',faceColor,'EdgeColor','none');
    xc = pos(1)+pos(3)/2; yc = pos(2)+pos(4)/2;
    text(ax,xc,yc,str,'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',fontSize,'FontWeight','bold','Color',txtColor);
end

function chip_circle(ax, cx, cy, r, faceColor, txtColor, str, P) %#ok<INUSD>
    th = linspace(0,2*pi,80);
    patch(ax,cx+r*cos(th),cy+r*sin(th)*1.8,faceColor,'EdgeColor','none');
    text(ax,cx,cy,str,'Interpreter','latex','HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',18,'FontWeight','bold','Color',txtColor);
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
