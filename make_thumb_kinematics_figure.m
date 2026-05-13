function make_thumb_kinematics_figure()
%MAKE_THUMB_KINEMATICS_FIGURE Premium DH/forward-kinematics slide (1200x627 PNG).
%
% Visual narrative:
%   1. Geometry of the 3R thumb in open and closed posture.
%   2. Denavit-Hartenberg table.
%   3. Closed-form homogeneous transform A_i and T_0^3.
%   4. Numeric fingertip pose at the saved closed-grip target q_f.
%
% All numbers come from thumb_kinematics.m -- no hand-typed constants.

    close all;
    here = fileparts(mfilename('fullpath'));
    oldpwd = pwd; cleanup = onCleanup(@() cd(oldpwd)); cd(here);

    q0 = [0; 0; 0];
    qf = [0.70; 1.20; 0.85];
    K0 = thumb_kinematics(q0);
    Kf = thumb_kinematics(qf);
    P  = palette();

    W = 1200; H = 627;
    fig = figure('Units','pixels','Position',[60 60 W H], ...
        'Color',P.bg,'MenuBar','none','ToolBar','none','Visible','off');
    set(fig,'DefaultAxesFontName','Helvetica');
    set(fig,'DefaultTextFontName','Helvetica');

    % Background canvas (cards + section dividers live here)
    axBg = axes('Parent',fig,'Position',[0 0 1 1],'Visible','off');
    axis(axBg,[0 1 0 1]); hold(axBg,'on'); axBg.Clipping = 'off';

    % --- Header band -----------------------------------------------------
    band(axBg, [0 0.880 1 0.120], P.navy, P.navy);
    text(axBg,0.5,0.945,'Denavit-Hartenberg Kinematics of the Planar 3R Robotic Thumb', ...
        'Interpreter','latex','HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',20,'FontWeight','bold','Color',P.white);
    text(axBg,0.5,0.905, ...
        'Standard DH parameters \rightarrow individual transforms A_i \rightarrow full fingertip transform T_0^3', ...
        'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize',11,'FontAngle','italic','Color',P.lightgray);

    % accent stripe under header
    band(axBg,[0 0.872 1 0.008],P.gold,P.gold);

    % --- Left panel: kinematic chain ------------------------------------
    card(axBg,[0.025 0.060 0.460 0.790],P.card,P.cardEdge);
    text(axBg,0.040,0.825,'Kinematic chain', ...
        'FontSize',12.5,'FontWeight','bold','Color',P.ink);
    text(axBg,0.040,0.795, ...
        'Open posture $q_0=[0,0,0]^\top$ vs. closed grip $q_f=[0.70,\,1.20,\,0.85]^\top$ rad', ...
        'Interpreter','latex','FontSize',9.5,'Color',P.gray);

    axChain = axes('Parent',fig,'Position',[0.060 0.105 0.405 0.610]);
    draw_chain(axChain, K0, Kf, P);

    % --- Right column: three cards --------------------------------------
    % Card A: DH table
    card(axBg,[0.505 0.530 0.470 0.320],P.card,P.cardEdge);
    text(axBg,0.520,0.825,'DH parameter table', ...
        'FontSize',12.5,'FontWeight','bold','Color',P.ink);
    text(axBg,0.520,0.795, ...
        'All links are planar, so $d_i=0$ and $\alpha_i=0$ for every joint.', ...
        'Interpreter','latex','FontSize',9.5,'Color',P.gray);
    axDH = axes('Parent',fig,'Position',[0.520 0.545 0.440 0.225]);
    draw_dh_table(axDH, Kf, P);

    % Card B: Forward kinematics equations
    card(axBg,[0.505 0.290 0.470 0.225],P.card,P.cardEdge);
    text(axBg,0.520,0.490,'Forward kinematics', ...
        'FontSize',12.5,'FontWeight','bold','Color',P.ink);
    axFK = axes('Parent',fig,'Position',[0.520 0.305 0.440 0.170]);
    draw_fk_equations(axFK, P);

    % Card C: Numeric fingertip pose
    card(axBg,[0.505 0.060 0.470 0.215],P.card,P.cardEdge);
    text(axBg,0.520,0.250,'Closed-grip pose at $q_f$', ...
        'Interpreter','latex','FontSize',12.5,'FontWeight','bold','Color',P.ink);
    axPose = axes('Parent',fig,'Position',[0.520 0.075 0.440 0.160]);
    draw_pose_block(axPose, Kf, P);

    export_png(fig, 'thumb_dh_forward_kinematics.png', W, H);
    close(fig);
    fprintf('Saved: thumb_dh_forward_kinematics.png\n');
end

% =====================================================================
function draw_chain(ax, K0, Kf, P)
    p0 = K0.joint_positions_mm;
    pf = Kf.joint_positions_mm;
    hold(ax,'on'); box(ax,'on'); axis(ax,'equal');
    set(ax,'Color','none','XColor',P.gray,'YColor',P.gray, ...
        'GridColor',P.gridLight,'GridAlpha',1.0,'LineWidth',0.9, ...
        'TickDir','out','FontName','Helvetica','FontSize',9, ...
        'XMinorGrid','off','YMinorGrid','off','Layer','top');
    grid(ax,'on');
    xlim(ax,[-12 110]); ylim(ax,[-8 92]);
    xlabel(ax,'$x_0$ (mm)','Interpreter','latex','FontSize',11,'Color',P.ink);
    ylabel(ax,'$y_0$ (mm)','Interpreter','latex','FontSize',11,'Color',P.ink);

    % Open posture ghost
    plot(ax,p0(:,1),p0(:,2),'-','Color',[P.mute 0.6],'LineWidth',2.0);
    scatter(ax,p0(:,1),p0(:,2),28,P.mute,'filled','MarkerEdgeColor',P.mute, ...
        'MarkerFaceAlpha',0.4);

    % Closed-grip pose -- coloured segments by link
    link_colors = [P.royal; P.emerald; P.crimson];
    for i = 1:3
        plot(ax,[pf(i,1) pf(i+1,1)],[pf(i,2) pf(i+1,2)], ...
            '-','Color',link_colors(i,:),'LineWidth',5.0);
        % midpoint length label
        mx = 0.5*(pf(i,1)+pf(i+1,1));
        my = 0.5*(pf(i,2)+pf(i+1,2));
        % perpendicular offset
        dx = pf(i+1,1)-pf(i,1); dy = pf(i+1,2)-pf(i,2);
        L = hypot(dx,dy);
        nx = -dy/L; ny = dx/L;
        text(ax,mx+9*nx,my+9*ny, ...
            sprintf('$a_{%d}\\!=\\!%d$ mm',i,round(K0.link_lengths_m(i)*1000)), ...
            'Interpreter','latex','FontSize',9.4,'Color',link_colors(i,:), ...
            'HorizontalAlignment','center','FontWeight','bold');
    end

    % Joints
    joint_names = {'Base','MCP','PIP','Tip'};
    joint_offsets = [-4 -7; 4 -6; 8 -2; -4 7];
    for i = 1:4
        if i == 1
            scatter(ax,pf(i,1),pf(i,2),140,P.navy,'filled','MarkerEdgeColor',P.navy);
            scatter(ax,pf(i,1),pf(i,2), 60,P.gold,'filled');
        elseif i == 4
            scatter(ax,pf(i,1),pf(i,2),110,P.gold,'filled','MarkerEdgeColor',P.ink,'LineWidth',1.2);
        else
            scatter(ax,pf(i,1),pf(i,2),90,P.white,'filled','MarkerEdgeColor',P.ink,'LineWidth',1.6);
        end
        text(ax,pf(i,1)+joint_offsets(i,1),pf(i,2)+joint_offsets(i,2), ...
            joint_names{i},'FontWeight','bold','FontSize',9.5,'Color',P.ink);
    end

    % Joint angle arcs
    draw_angle_arc(ax,[0 0],         14, 0,    0.70, '$q_1$', P.royal);
    draw_angle_arc(ax,pf(2,:),       10, 0.70, 1.90, '$q_2$', P.emerald);
    draw_angle_arc(ax,pf(3,:),       8,  1.90, 2.75, '$q_3$', P.crimson);

    % Base frame triad
    quiver(ax,0,0,18,0,0,'Color',P.ink,'LineWidth',1.0,'MaxHeadSize',0.5);
    quiver(ax,0,0,0,18,0,'Color',P.ink,'LineWidth',1.0,'MaxHeadSize',0.5);
    text(ax,19,-1.5,'$x_0$','Interpreter','latex','FontSize',10,'Color',P.ink);
    text(ax,1.5,19,'$y_0$','Interpreter','latex','FontSize',10,'Color',P.ink);

    % Tip pose readout shown in pose card; omitted here to avoid label cluster.

    % Legend block (manual, top-right corner)
    text(ax,72,86,'$q_0$ open','Interpreter','latex','FontSize',9.0,'Color',P.mute);
    plot(ax,[63 70],[86 86],'-','Color',P.mute,'LineWidth',2.0);
    text(ax,72,79,'$q_f$ closed','Interpreter','latex','FontSize',9.0,'Color',P.ink);
    plot(ax,[63 70],[79 79],'-','Color',P.royal,'LineWidth',3.0);
end

function draw_angle_arc(ax, origin, radius, a0, a1, label, color)
    th = linspace(a0,a1,60);
    plot(ax,origin(1)+radius*cos(th),origin(2)+radius*sin(th), ...
        'Color',color,'LineWidth',1.6);
    am = (a0+a1)/2;
    text(ax,origin(1)+(radius+5)*cos(am),origin(2)+(radius+5)*sin(am), ...
        label,'Interpreter','latex','FontSize',11.5,'FontWeight','bold','Color',color, ...
        'HorizontalAlignment','center');
end

% =====================================================================
function draw_dh_table(ax, Kf, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    headers = {'Joint','a_i (mm)','d_i (m)','\alpha_i (rad)','\theta_i'};
    n_rows = 3;
    a_mm = round(Kf.link_lengths_m * 1000);
    row_strs = { ...
        sprintf('q_1 / MCP'), sprintf('%d',a_mm(1)), '0', '0', '$q_1$'; ...
        sprintf('q_2 / PIP'), sprintf('%d',a_mm(2)), '0', '0', '$q_2$'; ...
        sprintf('q_3 / DIP'), sprintf('%d',a_mm(3)), '0', '0', '$q_3$'};
    xcols = [0.02 0.30 0.48 0.66 0.86];

    % Header band
    patch(ax,[0 1 1 0],[0.78 0.78 1 1],P.navy,'EdgeColor','none');
    for c = 1:numel(headers)
        text(ax,xcols(c),0.89,headers{c}, ...
            'FontSize',10,'FontWeight','bold','Color',P.white, ...
            'HorizontalAlignment','left','VerticalAlignment','middle');
    end

    % Rows
    row_h = 0.78/n_rows;
    for r = 1:n_rows
        y_top = 0.78 - (r-1)*row_h;
        y_bot = y_top - row_h;
        if mod(r,2)==1
            patch(ax,[0 1 1 0],[y_bot y_bot y_top y_top], ...
                P.rowAlt,'EdgeColor','none');
        end
        yc = (y_top+y_bot)/2;
        for c = 1:size(row_strs,2)
            s = row_strs{r,c};
            interp = 'tex';
            if contains(s,'$'); interp = 'latex'; end
            col = P.ink;
            fw = 'normal';
            if c == 1; fw = 'bold'; end
            text(ax,xcols(c),yc,s,'Interpreter',interp, ...
                'FontSize',10,'FontWeight',fw,'Color',col, ...
                'HorizontalAlignment','left','VerticalAlignment','middle');
        end
    end
    % bottom rule
    line(ax,[0 1],[0 0],'Color',P.cardEdge,'LineWidth',0.8);
end

% =====================================================================
function draw_fk_equations(ax, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    text(ax,0.00,0.92, ...
        '$A_i=\mathrm{Rot}_z(q_i)\,\mathrm{Trans}_x(a_i),\qquad T_0^{\,3}=A_1\,A_2\,A_3$', ...
        'Interpreter','latex','FontSize',11.5,'Color',P.ink);

    text(ax,0.00,0.62, ...
        '$p_x = a_1\cos q_1 + a_2\cos(q_1\!+\!q_2) + a_3\cos\phi$', ...
        'Interpreter','latex','FontSize',10.8,'Color',P.ink);
    text(ax,0.00,0.34, ...
        '$p_y = a_1\sin q_1 + a_2\sin(q_1\!+\!q_2) + a_3\sin\phi$', ...
        'Interpreter','latex','FontSize',10.8,'Color',P.ink);
    text(ax,0.00,0.06, ...
        '$\phi = q_1+q_2+q_3 \;\Rightarrow\; T_0^{\,3}=[\,R(\phi)\;\,p\,]$ in $SE(2)$', ...
        'Interpreter','latex','FontSize',10.5,'Color',P.gray);
end

% =====================================================================
function draw_pose_block(ax, Kf, P)
    axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');
    % big numeric callouts in three columns
    cols_x = [0.025 0.36 0.69];
    labels = {'$p_x$','$p_y$','$\phi$'};
    vals = {sprintf('%.1f',Kf.tip_position_mm(1)), ...
            sprintf('%.1f',Kf.tip_position_mm(2)), ...
            sprintf('$%.1f^{\\circ}$',Kf.tip_angle_deg)};
    units = {'mm','mm','distal-link angle'};
    accent = [P.royal; P.emerald; P.crimson];
    for k = 1:3
        text(ax,cols_x(k)+0.06,0.85,labels{k}, ...
            'Interpreter','latex','FontSize',13,'FontWeight','bold','Color',P.ink, ...
            'HorizontalAlignment','left');
        text(ax,cols_x(k)+0.06,0.45,vals{k}, ...
            'Interpreter','latex','FontSize',24,'FontWeight','bold','Color',accent(k,:), ...
            'HorizontalAlignment','left');
        text(ax,cols_x(k)+0.06,0.10,units{k}, ...
            'FontSize',9,'Color',P.gray, ...
            'HorizontalAlignment','left');
    end
    % light vertical dividers
    line(ax,[0.345 0.345],[0.10 0.90],'Color',P.cardEdge,'LineWidth',0.6);
    line(ax,[0.675 0.675],[0.10 0.90],'Color',P.cardEdge,'LineWidth',0.6);
end

% =====================================================================
% Shared helpers
% =====================================================================
function P = palette()
    P.bg        = [1 1 1];
    P.navy      = [0.043 0.145 0.282];
    P.ink       = [0.063 0.090 0.137];
    P.gray      = [0.290 0.330 0.400];
    P.mute      = [0.560 0.610 0.690];
    P.lightgray = [0.840 0.880 0.940];
    P.gridLight = [0.910 0.925 0.950];
    P.white     = [1 1 1];
    P.card      = [0.965 0.975 0.987];
    P.rowAlt    = [0.945 0.955 0.972];
    P.cardEdge  = [0.815 0.860 0.920];
    P.royal     = [0.110 0.310 0.690];
    P.crimson   = [0.690 0.135 0.180];
    P.emerald   = [0.105 0.490 0.290];
    P.gold      = [0.835 0.620 0.130];
end

function card(ax, pos, faceColor, edgeColor)
    x = pos(1); y = pos(2); w = pos(3); h = pos(4);
    rectangle(ax,'Position',[x y w h],'Curvature',[0.045 0.085], ...
        'FaceColor',faceColor,'EdgeColor',edgeColor,'LineWidth',0.9);
end

function band(ax, pos, faceColor, edgeColor) %#ok<INUSD>
    x = pos(1); y = pos(2); w = pos(3); h = pos(4);
    patch(ax,[x x+w x+w x],[y y y+h y+h],faceColor,'EdgeColor','none');
end

function export_png(fig, fname, W, H)
    set(fig,'InvertHardcopy','off');
    set(fig,'PaperUnits','inches','PaperPosition',[0 0 W/100 H/100], ...
        'PaperSize',[W/100 H/100]);
    print(fig,fname,'-dpng','-r150');
end
