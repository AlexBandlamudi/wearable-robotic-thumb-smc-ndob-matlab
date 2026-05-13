function animate_thumb_2d(t, Q, Qref, TAU_D_Nm, segs, labels, label_clrs, out_base, title_str)
% 2-D animation of the 3-DOF robotic thumb: robot arm (left) + live disturbance panel (right).
% Pass no args to auto-load thumb_results.mat; any arg can be [] for defaults.

    % ------------------------------------------------------------------
    % 0.  Defaults (backward-compatible)
    % ------------------------------------------------------------------
    if nargin == 0
        here_ = fileparts(mfilename('fullpath'));
        mat_  = fullfile(here_, 'thumb_results.mat');
        if ~isfile(mat_)
            error('animate_thumb_2d: thumb_results.mat not found.');
        end
        D_    = load(mat_, 't', 'X', 'Qref');
        t     = D_.t;
        Q     = D_.X(:, 1:3);
        Qref  = D_.Qref;
    end

    if nargin < 4 || isempty(TAU_D_Nm)
        N_ = length(t);
        TAU_D_Nm = zeros(N_, 3);
        for k_ = 1:N_
            TAU_D_Nm(k_,:) = disturbance(t(k_))';
        end
    end

    if nargin < 5 || isempty(segs)
        segs = [0 5 10 15];
    end
    if nargin < 6 || isempty(labels)
        labels = {'PHASE 1  CLOSE GRIP', ...
                  'PHASE 2  HOLD POSTURE', ...
                  'PHASE 3  SUDDEN LOAD — REJECT'};
    end
    if nargin < 7 || isempty(label_clrs)
        label_clrs = {[1.00 0.85 0.30], [0.35 0.92 0.55], [1.00 0.48 0.42]};
    end
    if nargin < 8 || isempty(out_base)
        out_base = 'thumb_animation';
    end
    if nargin < 9 || isempty(title_str)
        title_str = 'Robotic Thumb — Model-Based SMC  |  3-DOF Wearable Exoskeleton';
    end

    here = fileparts(mfilename('fullpath'));
    P    = numel(labels);   % number of phases

    % ------------------------------------------------------------------
    % 1.  Geometry & data preparation
    % ------------------------------------------------------------------
    r1 = 50;  r2 = 35;  r3 = 25;
    fk = @(q) fk3(q, r1, r2, r3);

    N  = length(t);
    dt = t(2) - t(1);

    % Display-only smoothing (does NOT affect saved metrics)
    Q_disp = Q;
    for ji = 1:3
        Q_disp(:,ji) = movmean(Q(:,ji), 101);
    end

    % Precompute tip loci
    tip_ref = zeros(N, 2);
    tip_act = zeros(N, 2);
    for k = 1:N
        tmp = fk(Qref(k,:));    tip_ref(k,:) = tmp(4,:);
        tmp = fk(Q_disp(k,:));  tip_act(k,:) = tmp(4,:);
    end

    % Disturbance in mN·m, smoothed for display
    TD_mNm = TAU_D_Nm * 1000;
    TD_disp = TD_mNm;
    for ji = 1:3
        TD_disp(:,ji) = movmean(TD_mNm(:,ji), 101);
    end
    D_max = max(abs(TD_disp(:))) * 1.3 + 0.01;

    t_lo = segs(1);
    t_hi = segs(end);

    % ------------------------------------------------------------------
    % 2.  Figure setup
    % ------------------------------------------------------------------
    % Detect batch / headless mode
    batch_mode = ~usejava('desktop');

    fig = figure('Position',    [30 30 1380 820], ...
                 'Color',       [0.07 0.08 0.11], ...
                 'Name',        sprintf('Robotic Thumb — %s', title_str), ...
                 'NumberTitle', 'off', ...
                 'Visible',     'off');

    % --- LEFT: robot animation axis -----------------------------------
    ax = axes('Parent',   fig, ...
              'Position', [0.04 0.09 0.59 0.85], ...
              'Color',         [0.09 0.11 0.15], ...
              'XColor',        [0.70 0.72 0.78], ...
              'YColor',        [0.70 0.72 0.78], ...
              'GridColor',     [0.22 0.25 0.30], ...
              'GridAlpha',     1.0, ...
              'FontSize',      11, ...
              'FontWeight',    'bold', ...
              'TickDir',       'out', ...
              'LineWidth',     1.0);
    axis(ax, 'equal');  grid(ax, 'on');  box(ax, 'on');
    xl = [-30 145];   yl = [-30 115];
    xlim(ax, xl);     ylim(ax, yl);
    xlabel(ax, 'x  (mm)', 'FontSize', 12, 'FontWeight', 'bold', ...
               'Color', [0.82 0.84 0.90]);
    ylabel(ax, 'y  (mm)', 'FontSize', 12, 'FontWeight', 'bold', ...
               'Color', [0.82 0.84 0.90]);
    ax.XAxis.TickLabelColor = [0.70 0.72 0.78];
    ax.YAxis.TickLabelColor = [0.70 0.72 0.78];
    title(ax, title_str, 'FontSize', 12, 'FontWeight', 'bold', ...
          'Color', [0.94 0.95 1.00], 'Interpreter', 'none');
    hold(ax, 'on');

    % --- RIGHT: disturbance panel axis --------------------------------
    axD = axes('Parent',   fig, ...
               'Position', [0.67 0.09 0.30 0.85], ...
               'Color',         [0.09 0.11 0.15], ...
               'XColor',        [0.70 0.72 0.78], ...
               'YColor',        [0.70 0.72 0.78], ...
               'GridColor',     [0.22 0.25 0.30], ...
               'GridAlpha',     1.0, ...
               'FontSize',      10, ...
               'FontWeight',    'bold', ...
               'TickDir',       'out', ...
               'LineWidth',     1.0);
    hold(axD, 'on');  grid(axD, 'on');  box(axD, 'on');
    xlim(axD, [t_lo t_hi]);
    ylim(axD, [-D_max D_max]);
    xlabel(axD, 'Time  (s)', 'FontSize', 10, 'Color', [0.82 0.84 0.90]);
    ylabel(axD, '\tau_d  (mN·m)', 'FontSize', 10, 'Color', [0.82 0.84 0.90]);
    axD.XAxis.TickLabelColor = [0.70 0.72 0.78];
    axD.YAxis.TickLabelColor = [0.70 0.72 0.78];
    title(axD, 'Live External Disturbance', ...
          'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.94 0.95 1.00]);

    % Phase shading + boundary lines + short labels on right panel
    for ph = 1:P
        patch(axD, [segs(ph) segs(ph+1) segs(ph+1) segs(ph)], ...
              [-D_max -D_max D_max D_max], label_clrs{ph}, ...
              'FaceAlpha', 0.10, 'EdgeColor', 'none');
        text(axD, mean(segs(ph:ph+1)), D_max*0.88, sprintf('P%d', ph), ...
             'FontSize', 8, 'FontWeight', 'bold', ...
             'Color', label_clrs{ph}, 'HorizontalAlignment', 'center');
    end
    yline(axD, 0, 'Color', [0.55 0.58 0.65], 'LineWidth', 0.8);
    for ph = 2:P
        xline(axD, segs(ph), '--', 'Color', label_clrs{ph}, ...
              'LineWidth', 1.0, 'Alpha', 0.7);
    end

    % Disturbance history lines
    dist_clr = {[0.45 0.72 1.00],   % q1 — steel blue
                [0.30 0.88 0.70],   % q2 — teal
                [1.00 0.72 0.30]};  % q3 — amber
    h_dline = gobjects(3,1);
    for ji = 1:3
        h_dline(ji) = plot(axD, NaN, NaN, '-', ...
                           'Color', dist_clr{ji}, 'LineWidth', 1.8);
    end
    h_tcur = xline(axD, t_lo, '-', 'Color', [0.90 0.92 1.00], ...
                   'LineWidth', 1.5, 'Alpha', 0.7);

    legend(axD, h_dline, {'\tau_{d1} (MCP)', '\tau_{d2} (PIP)', '\tau_{d3} (DIP)'}, ...
           'Location', 'south', 'FontSize', 9, 'Box', 'on', ...
           'TextColor', [0.88 0.88 0.94], ...
           'Color', [0.09 0.11 0.15], ...
           'EdgeColor', [0.35 0.38 0.45]);

    % ------------------------------------------------------------------
    % 3.  Static background on LEFT axis
    % ------------------------------------------------------------------
    % Wrist backing plate
    patch(ax, [-26 26 26 -26], [-30 -30 -3 -3], ...
          [0.14 0.17 0.22], 'EdgeColor', [0.35 0.40 0.48], ...
          'LineWidth', 1.0, 'HandleVisibility', 'off');
    % Palm body
    patch(ax, [-18, 18, 20, 20, -20, -20], ...
              [  0,  0, -3, -22, -22,  -3], ...
          [0.22 0.27 0.34], 'EdgeColor', [0.50 0.56 0.65], ...
          'LineWidth', 1.8, 'HandleVisibility', 'off');
    % Bolts
    bolt_xy = [-12 -7; -1 -7; 11 -7; -12 -17; -1 -17; 11 -17];
    th_b    = linspace(0, 2*pi, 22);
    for bi = 1:size(bolt_xy,1)
        patch(ax, bolt_xy(bi,1) + 2.0*cos(th_b), ...
                  bolt_xy(bi,2) + 2.0*sin(th_b), ...
              [0.12 0.15 0.19], 'EdgeColor', [0.40 0.45 0.55], ...
              'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
    text(ax,  0, -11, 'PALM  MOUNT', 'FontSize', 7.5, 'FontWeight', 'bold', ...
         'Color', [0.45 0.50 0.60], 'HorizontalAlignment', 'center', ...
         'HandleVisibility', 'off');
    text(ax,  0, -26, 'W R I S T   /   M C P   B A S E', 'FontSize', 6.5, ...
         'Color', [0.32 0.37 0.45], 'HorizontalAlignment', 'center', ...
         'HandleVisibility', 'off');

    % Scale bar
    sb_x0 = xl(2) - 30;  sb_y0 = yl(1) + 7;
    plot(ax, [sb_x0 sb_x0+20], [sb_y0 sb_y0], '-', ...
         'Color', [0.70 0.72 0.78], 'LineWidth', 2.5, 'HandleVisibility', 'off');
    for bx = [sb_x0, sb_x0+20]
        plot(ax, [bx bx], [sb_y0-1.5 sb_y0+1.5], '-', ...
             'Color', [0.70 0.72 0.78], 'LineWidth', 2.0, 'HandleVisibility', 'off');
    end
    text(ax, sb_x0+10, sb_y0+4, '20 mm', 'FontSize', 8, ...
         'Color', [0.70 0.72 0.78], 'HorizontalAlignment', 'center', ...
         'HandleVisibility', 'off');

    % Reference tip arc (full 25s reference path — not the actual, so showing full route is correct)
    plot(ax, tip_ref(:,1), tip_ref(:,2), '--', ...
         'Color', [0.92 0.20 0.88], 'LineWidth', 1.8, ...
         'DisplayName', 'Reference arc');
    % NOTE: No pre-drawn grey actual trace — the cyan 'Tip path' below builds
    %       incrementally so you only ever see where the robot HAS been, not
    %       where it will go. Pre-drawing the full actual path was misleading
    %       (showed Phase-3 failure loops at t=1s before they happened).
    % Start / End markers
    tmp1 = fk(Qref(1,:));   tmp2 = fk(Qref(end,:));
    plot(ax, tmp1(4,1), tmp1(4,2), 'o', 'MarkerSize', 8, ...
         'MarkerFaceColor', [0.92 0.20 0.88], 'MarkerEdgeColor', [1 1 1], ...
         'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, tmp2(4,1), tmp2(4,2), 's', 'MarkerSize', 8, ...
         'MarkerFaceColor', [0.92 0.20 0.88], 'MarkerEdgeColor', [1 1 1], ...
         'LineWidth', 1.5, 'HandleVisibility', 'off');
    text(ax, tmp1(4,1)+4, tmp1(4,2)-4, 'Start', 'FontSize', 8.5, ...
         'Color', [0.92 0.20 0.88], 'HandleVisibility', 'off');
    text(ax, tmp2(4,1)+4, tmp2(4,2),   'End',   'FontSize', 8.5, ...
         'Color', [0.92 0.20 0.88], 'HandleVisibility', 'off');

    % ------------------------------------------------------------------
    % 4.  Animated elements — links, joints, tip trace, force arrows
    % ------------------------------------------------------------------
    link_clrs  = {[0.30 0.58 0.90], [0.55 0.77 0.97], [0.80 0.92 1.00]};
    link_names = {'Proximal (50 mm)', 'Middle (35 mm)', 'Distal (25 mm)'};
    link_lw    = [14, 11, 8];

    h_links = gobjects(3,1);
    h_shine = gobjects(3,1);
    for li = 1:3
        h_links(li) = plot(ax, [0 0], [0 0], '-', ...
                           'Color', link_clrs{li}, 'LineWidth', link_lw(li), ...
                           'DisplayName', link_names{li});
        hi_c = min(link_clrs{li}*1.30 + 0.25, [1 1 1]);
        h_shine(li) = plot(ax, [0 0], [0 0], '-', ...
                           'Color', hi_c, 'LineWidth', 2.0, ...
                           'HandleVisibility', 'off');
    end

    jnt_clrs  = {[0.95 0.97 1.00], [0.80 0.88 0.98], ...
                 [0.58 0.73 0.92], [0.38 0.56 0.88]};
    jnt_sz    = [18, 15, 13];
    jnt_names = {'MCP', 'PIP', 'DIP'};
    h_jnt  = gobjects(4,1);
    h_jlbl = gobjects(4,1);
    for ji = 1:3
        h_jnt(ji)  = plot(ax, 0, 0, 'o', ...
                          'MarkerSize', jnt_sz(ji), ...
                          'MarkerFaceColor', jnt_clrs{ji}, ...
                          'MarkerEdgeColor', [0.95 0.95 0.95], ...
                          'LineWidth', 1.8, 'HandleVisibility', 'off');
        h_jlbl(ji) = text(ax, 0, 0, jnt_names{ji}, ...
                          'FontSize', 9, 'FontWeight', 'bold', ...
                          'Color', [0.88 0.90 0.96], ...
                          'HorizontalAlignment', 'center', ...
                          'VerticalAlignment', 'bottom', ...
                          'HandleVisibility', 'off');
    end
    h_jnt(4)  = plot(ax, 0, 0, 'd', ...
                     'MarkerSize', 13, ...
                     'MarkerFaceColor', jnt_clrs{4}, ...
                     'MarkerEdgeColor', [0.95 0.95 0.95], ...
                     'LineWidth', 1.8, 'DisplayName', 'Fingertip');
    h_jlbl(4) = text(ax, 0, 0, 'Tip', ...
                     'FontSize', 9, 'FontWeight', 'bold', ...
                     'Color', [0.88 0.90 0.96], ...
                     'HorizontalAlignment', 'center', ...
                     'VerticalAlignment', 'bottom', ...
                     'HandleVisibility', 'off');

    % Growing tip trace
    h_trace = plot(ax, NaN, NaN, '-', ...
                   'Color', [0.30 0.75 1.00], 'LineWidth', 1.5, ...
                   'DisplayName', 'Tip path');

    % Force arrows at joints (visible from phase 2 onwards)
    arrow_scale = 8;   % mm per mN·m
    arr_clr = {[0.45 0.72 1.00], [0.30 0.88 0.70], [1.00 0.72 0.30]};
    h_arr = gobjects(3,1);
    for ji = 1:3
        h_arr(ji) = quiver(ax, 0, 0, 0, 0, 0, ...
                           'Color', arr_clr{ji}, ...
                           'LineWidth', 2.5, ...
                           'MaxHeadSize', 0.8, ...
                           'HandleVisibility', 'off');
    end

    % ------------------------------------------------------------------
    % 5.  Overlay text (left axis)
    % ------------------------------------------------------------------
    h_time = text(ax, xl(2)-3, yl(2)-3, sprintf('t = %.2f s', t_lo), ...
                  'FontSize', 13, 'FontWeight', 'bold', ...
                  'Color', [1.00 0.92 0.28], ...
                  'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
                  'BackgroundColor', [0.09 0.11 0.15], ...
                  'EdgeColor', [0.38 0.42 0.50], 'Margin', 4, ...
                  'HandleVisibility', 'off');

    h_phase = text(ax, xl(2)-3, yl(2)-20, labels{1}, ...
                   'FontSize', 10, 'FontWeight', 'bold', ...
                   'Color', label_clrs{1}, ...
                   'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
                   'BackgroundColor', [0.09 0.11 0.15], ...
                   'EdgeColor', [0.38 0.42 0.50], 'Margin', 3, ...
                   'HandleVisibility', 'off');

    text(ax, xl(1)+2, yl(1)+4, ...
         'SMC-MB  | \Lambda=12I  K=diag(10,8,1) mNm  \delta=0.01 rad', ...
         'FontSize', 7.5, 'Color', [0.40 0.43 0.52], ...
         'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
         'HandleVisibility', 'off');

    leg = legend(ax, 'Location', 'southeast', 'FontSize', 9, 'Box', 'on', ...
                 'NumColumns', 1);
    leg.TextColor = [0.88 0.88 0.94];
    leg.Color     = [0.09 0.11 0.15];
    leg.EdgeColor = [0.35 0.38 0.45];
    drawnow;
    leg.Position(2) = 0.10;
    drawnow;

    % ------------------------------------------------------------------
    % 6.  Video writer
    % ------------------------------------------------------------------
    vid_file = fullfile(here, [out_base '.mp4']);
    fps_out  = 10;   % 10 fps is adequate; keeps frame count low for batch rendering
    save_vid = false;
    try
        vw           = VideoWriter(vid_file, 'MPEG-4');
        vw.FrameRate = fps_out;
        vw.Quality   = 92;
        open(vw);
        save_vid = true;
    catch ME
        warning('animate_thumb_2d: VideoWriter failed (%s).', ME.message);
    end

    % ------------------------------------------------------------------
    % 7.  Animation loop
    % ------------------------------------------------------------------
    stride = max(1, round(1 / (fps_out * dt)));
    frames = 1:stride:N;
    nf     = length(frames);

    tip_x  = nan(1, nf);
    tip_y  = nan(1, nf);
    fi     = 0;
    dist_t = nan(1, nf);
    dist_d = {nan(1,nf), nan(1,nf), nan(1,nf)};

    fprintf('  Rendering %d frames ...', nf);
    for k = frames
        fi  = fi + 1;
        pts = fk(Q_disp(k,:));

        % Links + sheen
        for li = 1:3
            x1 = pts(li,1);   y1 = pts(li,2);
            x2 = pts(li+1,1); y2 = pts(li+1,2);
            set(h_links(li), 'XData', [x1 x2], 'YData', [y1 y2]);
            dx = x2-x1; dy = y2-y1;
            len = hypot(dx,dy) + 1e-9;
            ox = -dy/len*2.0;  oy = dx/len*2.0;
            set(h_shine(li), 'XData', [x1+ox x2+ox], 'YData', [y1+oy y2+oy]);
        end

        % Joints + labels
        lbl_off = [0 6; 0 5; 0 4.5; 0 4];
        for ji = 1:4
            set(h_jnt(ji),  'XData', pts(ji,1), 'YData', pts(ji,2));
            set(h_jlbl(ji), 'Position', ...
                [pts(ji,1)+lbl_off(ji,1), pts(ji,2)+lbl_off(ji,2), 0]);
        end

        % Growing tip trace
        tip_x(fi) = pts(4,1);
        tip_y(fi) = pts(4,2);
        set(h_trace, 'XData', tip_x(1:fi), 'YData', tip_y(1:fi));

        % Force arrows (show from phase 2 onwards)
        tk     = t(k);
        td_k   = TD_disp(k,:);
        in_arr = (tk >= segs(2));
        for ji = 1:3
            if in_arr
                set(h_arr(ji), ...
                    'XData', pts(ji,1), 'YData', pts(ji,2), ...
                    'UData', 0,         'VData', -td_k(ji)*arrow_scale);
            else
                set(h_arr(ji), 'UData', 0, 'VData', 0);
            end
        end

        % Time counter
        set(h_time, 'String', sprintf('t = %.2f s', tk));

        % Active phase label
        ph_idx = find(tk >= segs(1:end-1), 1, 'last');
        if isempty(ph_idx), ph_idx = 1; end
        if ph_idx > P,      ph_idx = P; end
        set(h_phase, 'String', labels{ph_idx}, 'Color', label_clrs{ph_idx});

        % Right panel: growing disturbance history + cursor
        dist_t(fi) = tk;
        for ji = 1:3
            dist_d{ji}(fi) = td_k(ji);
            set(h_dline(ji), 'XData', dist_t(1:fi), 'YData', dist_d{ji}(1:fi));
        end
        h_tcur.Value = tk;

        drawnow;
        if save_vid
            writeVideo(vw, getframe(fig));
        end
    end
    fprintf('  done.\n');

    if save_vid
        close(vw);
        fprintf('  Video -> %s\n', vid_file);
    end

    snap = fullfile(here, [out_base '_final.png']);
    print(fig, snap, '-dpng', '-r150');
    fprintf('  Snap  -> %s\n', snap);
    close(fig);
end

% ---- Forward kinematics: 3-link planar arm (mm) --------------------------
function pts = fk3(q, r1, r2, r3)
    a1 = q(1);
    a2 = q(1) + q(2);
    a3 = q(1) + q(2) + q(3);
    p0 = [0, 0];
    p1 = p0 + r1 * [cos(a1), sin(a1)];
    p2 = p1 + r2 * [cos(a2), sin(a2)];
    p3 = p2 + r3 * [cos(a3), sin(a3)];
    pts = [p0; p1; p2; p3];
end
