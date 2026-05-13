function build_thumb_smc_model()
% Builds model_thumb_smc_mb.slx for the 3-DOF robotic thumb FROM SCRATCH.
%
% Uses new_system / add_block / add_line — no template copy, so there is
% zero stale compiled-port-size data.  Mirror of assignment/model_SMC_mb
% structure, extended to 3-DOF with thumb-scale parameters.
%
% Run from the "Robotic THUMB" folder or via:
%   matlab -batch "cd('Robotic THUMB'); addpath(pwd); build_thumb_smc_model()"

    thumb_dir = fileparts(mfilename('fullpath'));
    out_file  = fullfile(thumb_dir, 'model_thumb_smc_mb.slx');
    mdl       = 'model_thumb_smc_mb';

    % ---- Clean up any previous version ------------------------------------
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if isfile(out_file), delete(out_file); end

    % =====================================================================
    % 1. CREATE MODEL AND SET SOLVER
    % =====================================================================
    new_system(mdl);
    set_param(mdl, 'StopTime',   '15', ...
                   'Solver',     'ode4', ...
                   'FixedStep',  '0.0005', ...
                   'SolverType', 'Fixed-step');
    fprintf('Created model: %s\n', mdl);

    % =====================================================================
    % 2. ADD BLOCKS  (positions are [left top right bottom] in pixels)
    % =====================================================================

    % Clock
    ab(mdl, 'built-in/Clock', 'Clock', [40 255 70 275]);

    % Ref_Gen — MATLAB Function (1 input t, 1 output [qref;dqref;ddqref] 9x1)
    ab(mdl, 'simulink/User-Defined Functions/MATLAB Function', ...
        'Ref_Gen', [140 235 220 285]);

    % Demux_ref: splits [qref(3);dqref(3);ddqref(3)] into 3 outputs
    ab(mdl, 'built-in/Demux', 'Demux_ref', [270 210 290 330]);
    set_param([mdl '/Demux_ref'], 'Outputs', '[3 3 3]');

    % Err_Sum: qref(+) - q(-) = e
    ab(mdl, 'built-in/Sum', 'Err_Sum', [380 248 410 278]);
    set_param([mdl '/Err_Sum'], 'Inputs', '+-', 'IconShape', 'round');

    % Int_xi: integrates e (unused integral channel)
    ab(mdl, 'built-in/Integrator', 'Int_xi', [440 410 480 440]);
    set_param([mdl '/Int_xi'], 'InitialCondition', '[0; 0; 0]');

    % Controller — MATLAB Function (6 inputs, 2 outputs [tau; s])
    ab(mdl, 'simulink/User-Defined Functions/MATLAB Function', ...
        'Controller', [530 160 620 510]);

    % Disturbance — MATLAB Function (1 input t, 1 output tau_d 3x1)
    ab(mdl, 'simulink/User-Defined Functions/MATLAB Function', ...
        'Disturbance', [140 415 220 455]);

    % Mux_plant: concatenates [q qdot tau tau_d]
    ab(mdl, 'built-in/Mux', 'Mux_plant', [680 320 700 490]);
    set_param([mdl '/Mux_plant'], 'Inputs', '4');

    % Plant — SubSystem
    ab(mdl, 'built-in/SubSystem', 'Plant', [760 365 840 445]);
    try
        delete_line([mdl '/Plant'], 'In1/1', 'Out1/1');
        delete_block([mdl '/Plant/In1']);
        delete_block([mdl '/Plant/Out1']);
    catch
    end
    ab([mdl '/Plant'], 'built-in/Inport',  'In1',       [30  240 60  260]);
    ab([mdl '/Plant'], 'built-in/Demux',   'Demux_in',  [110 225 130 280]);
    set_param([mdl '/Plant/Demux_in'], 'Outputs', '[3 3 3 3]');
    ab([mdl '/Plant'], 'simulink/User-Defined Functions/MATLAB Function', ...
        'Plant_Fcn', [200 225 290 285]);
    ab([mdl '/Plant'], 'built-in/Outport', 'Out1',      [370 240 400 260]);

    % Int_qdot
    ab(mdl, 'built-in/Integrator', 'Int_qdot', [660 335 700 365]);
    set_param([mdl '/Int_qdot'], 'InitialCondition', '[0; 0; 0]');

    % Int_q
    ab(mdl, 'built-in/Integrator', 'Int_q', [740 238 780 268]);
    set_param([mdl '/Int_q'], 'InitialCondition', '[0.45; 0.60; 0.30]');

    % ---- ToWorkspace blocks --------------------------------------------------
    tws(mdl, 'ToWS_t',    [140 170 210 190], 't_thumb');
    tws(mdl, 'ToWS_qref', [320 175 390 195], 'qref_thumb');
    tws(mdl, 'ToWS_tau',  [660 175 730 195], 'tau_thumb');
    tws(mdl, 'ToWS_s',    [660 200 730 220], 's_thumb');
    tws(mdl, 'ToWS_q',    [840 155 910 175], 'q_thumb');
    tws(mdl, 'ToWS_qdot', [840 180 910 200], 'qdot_thumb');

    % ---- Demux blocks for per-joint viewing ----------------------------------
    % q → Demux_q → 3 individual signals
    ab(mdl, 'built-in/Demux', 'Demux_q',    [920 238 940 268]);
    set_param([mdl '/Demux_q'],    'Outputs', '3');
    ab(mdl, 'built-in/Demux', 'Demux_qdot', [920 335 940 365]);
    set_param([mdl '/Demux_qdot'], 'Outputs', '3');
    ab(mdl, 'built-in/Demux', 'Demux_tau',  [660 155 680 195]);
    set_param([mdl '/Demux_tau'],  'Outputs', '3');
    ab(mdl, 'built-in/Demux', 'Demux_s',    [660 200 680 240]);
    set_param([mdl '/Demux_s'],    'Outputs', '3');
    ab(mdl, 'built-in/Demux', 'Demux_qref', [320 215 340 255]);
    set_param([mdl '/Demux_qref'], 'Outputs', '3');

    % ---- Scope: Joint Positions (q1,q2,q3 vs qref1,qref2,qref3) using Mux --
    % We use 6-channel scopes fed by a Mux [q1 qref1; q2 qref2; q3 qref3]
    % Per-joint pair: Mux_pos_j, Scope_pos_j
    for ji = 1:3
        mname = sprintf('Mux_pos_%d', ji);
        sname = sprintf('Scope_pos_%d', ji);
        yoff  = 50 + (ji-1)*120;
        ab(mdl, 'built-in/Mux',   mname, [1000 yoff+5  1020 yoff+55]);
        set_param([mdl '/' mname], 'Inputs', '2');
        ab(mdl, 'built-in/Scope', sname, [1060 yoff    1100 yoff+60]);
        set_scope_title(mdl, sname, sprintf('Position  q_%d  vs  q_{ref,%d}', ji, ji));

        mname = sprintf('Mux_vel_%d', ji);
        sname = sprintf('Scope_vel_%d', ji);
        yoff2 = 410 + (ji-1)*120;
        ab(mdl, 'built-in/Mux',   mname, [1000 yoff2+5  1020 yoff2+35]);
        set_param([mdl '/' mname], 'Inputs', '1');
        ab(mdl, 'built-in/Scope', sname, [1060 yoff2    1100 yoff2+60]);
        set_scope_title(mdl, sname, sprintf('Velocity  \\dot{q}_%d  (rad/s)', ji));

        mname = sprintf('Mux_tau_%d', ji);
        sname = sprintf('Scope_tau_%d', ji);
        yoff3 = 770 + (ji-1)*120;
        ab(mdl, 'built-in/Mux',   mname, [820 yoff3+5   840 yoff3+35]);
        set_param([mdl '/' mname], 'Inputs', '1');
        ab(mdl, 'built-in/Scope', sname, [880 yoff3     920 yoff3+60]);
        set_scope_title(mdl, sname, sprintf('Control Torque  \\tau_%d  (N\\cdotm)', ji));

        sname = sprintf('Scope_s_%d', ji);
        yoff4 = 1130 + (ji-1)*120;
        ab(mdl, 'built-in/Scope', sname, [880 yoff4     920 yoff4+60]);
        set_scope_title(mdl, sname, sprintf('Sliding Surface  s_%d', ji));
    end

    fprintf('All blocks added\n');

    % =====================================================================
    % 3. SET MATLAB FUNCTION SCRIPTS
    % =====================================================================
    rt = sfroot;
    charts = rt.find('-isa', 'Stateflow.EMChart');

    set_chart(charts, [mdl '/Ref_Gen'],         script_refgen());
    set_chart(charts, [mdl '/Disturbance'],     script_disturbance());
    set_chart(charts, [mdl '/Controller'],      script_controller());
    set_chart(charts, [mdl '/Plant/Plant_Fcn'], script_plant());

    fprintf('MATLAB Function scripts set\n');

    % =====================================================================
    % 4. CONNECT LINES
    % =====================================================================
    % Plant internal connections
    al([mdl '/Plant'], 'In1/1',       'Demux_in/1');
    al([mdl '/Plant'], 'Demux_in/1',  'Plant_Fcn/1');
    al([mdl '/Plant'], 'Demux_in/2',  'Plant_Fcn/2');
    al([mdl '/Plant'], 'Demux_in/3',  'Plant_Fcn/3');
    al([mdl '/Plant'], 'Demux_in/4',  'Plant_Fcn/4');
    al([mdl '/Plant'], 'Plant_Fcn/1', 'Out1/1');

    % Clock → Ref_Gen, Disturbance, ToWS_t
    al(mdl, 'Clock/1',  'Ref_Gen/1');
    al(mdl, 'Clock/1',  'Disturbance/1');
    al(mdl, 'Clock/1',  'ToWS_t/1');

    % Ref_Gen → Demux_ref
    al(mdl, 'Ref_Gen/1', 'Demux_ref/1');

    % Demux_ref/1=qref → Err_Sum(+), ToWS_qref, Demux_qref
    al(mdl, 'Demux_ref/1', 'Err_Sum/1');
    al(mdl, 'Demux_ref/1', 'ToWS_qref/1');
    al(mdl, 'Demux_ref/1', 'Demux_qref/1');
    % Demux_ref/2=dqref → Controller(3)
    al(mdl, 'Demux_ref/2', 'Controller/3');
    % Demux_ref/3=ddqref → Controller(4)
    al(mdl, 'Demux_ref/3', 'Controller/4');

    % Err_Sum=e → Controller(1), Int_xi
    al(mdl, 'Err_Sum/1', 'Controller/1');
    al(mdl, 'Err_Sum/1', 'Int_xi/1');
    al(mdl, 'Int_xi/1',  'Controller/6');

    % Plant → Int_qdot
    al(mdl, 'Plant/1',    'Int_qdot/1');

    % Int_qdot=qdot → Int_q, Controller(2), Mux_plant(2), ToWS_qdot, Demux_qdot
    al(mdl, 'Int_qdot/1', 'Int_q/1');
    al(mdl, 'Int_qdot/1', 'Controller/2');
    al(mdl, 'Int_qdot/1', 'Mux_plant/2');
    al(mdl, 'Int_qdot/1', 'ToWS_qdot/1');
    al(mdl, 'Int_qdot/1', 'Demux_qdot/1');

    % Int_q=q → Err_Sum(-), Controller(5), Mux_plant(1), ToWS_q, Demux_q
    al(mdl, 'Int_q/1', 'Err_Sum/2');
    al(mdl, 'Int_q/1', 'Controller/5');
    al(mdl, 'Int_q/1', 'Mux_plant/1');
    al(mdl, 'Int_q/1', 'ToWS_q/1');
    al(mdl, 'Int_q/1', 'Demux_q/1');

    % Controller/1=tau → Mux_plant(3), ToWS_tau, Demux_tau
    al(mdl, 'Controller/1', 'Mux_plant/3');
    al(mdl, 'Controller/1', 'ToWS_tau/1');
    al(mdl, 'Controller/1', 'Demux_tau/1');

    % Controller/2=s → ToWS_s, Demux_s
    al(mdl, 'Controller/2', 'ToWS_s/1');
    al(mdl, 'Controller/2', 'Demux_s/1');

    % Disturbance/1=tau_d → Mux_plant(4)
    al(mdl, 'Disturbance/1', 'Mux_plant/4');

    % Mux_plant → Plant
    al(mdl, 'Mux_plant/1', 'Plant/1');

    % Per-joint scope connections
    for ji = 1:3
        % Position scope: [q_j, qref_j]
        al(mdl, sprintf('Demux_q/%d',    ji), sprintf('Mux_pos_%d/1', ji));
        al(mdl, sprintf('Demux_qref/%d', ji), sprintf('Mux_pos_%d/2', ji));
        al(mdl, sprintf('Mux_pos_%d/1',  ji), sprintf('Scope_pos_%d/1', ji));
        % Velocity scope
        al(mdl, sprintf('Demux_qdot/%d', ji), sprintf('Mux_vel_%d/1', ji));
        al(mdl, sprintf('Mux_vel_%d/1',  ji), sprintf('Scope_vel_%d/1', ji));
        % Torque scope
        al(mdl, sprintf('Demux_tau/%d',  ji), sprintf('Mux_tau_%d/1', ji));
        al(mdl, sprintf('Mux_tau_%d/1',  ji), sprintf('Scope_tau_%d/1', ji));
        % Sliding surface scope
        al(mdl, sprintf('Demux_s/%d',    ji), sprintf('Scope_s_%d/1', ji));
    end

    fprintf('All lines connected\n');

    % =====================================================================
    % 5. VERIFY
    % =====================================================================
    fprintf('\nRunning 0.01 s test simulation...\n');
    try
        simOut = sim(mdl, 'StopTime', '0.01', 'SaveOutput', 'off');
        fprintf('Test sim: OK\n');
    catch ME
        fprintf('Test sim ERROR: %s\n', ME.message);
        for i = 1:numel(ME.cause)
            fprintf('  Cause %d: %s\n', i, ME.cause{i}.message);
        end
    end

    % =====================================================================
    % 6. SAVE
    % =====================================================================
    save_system(mdl, out_file);
    fprintf('Model saved -> %s\n', out_file);
    close_system(mdl, 0);
    fprintf('\nbuild_thumb_smc_model: DONE\n');
end

% =============================================================================
%  Shorthand helpers
% =============================================================================
function ab(sys, lib, name, pos)
    add_block(lib, [sys '/' name], 'Position', pos);
end

function al(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'on');
end

function tws(mdl, name, pos, varname)
    add_block('built-in/ToWorkspace', [mdl '/' name], ...
        'Position', pos, ...
        'VariableName', varname, ...
        'SaveFormat', 'Array', ...
        'MaxDataPoints', 'inf');
end

function set_chart(charts, path, new_script)
    found = false;
    for i = 1:numel(charts)
        if strcmp(charts(i).Path, path)
            charts(i).Script = new_script;
            found = true;
            break;
        end
    end
    if ~found
        warning('Chart not found: %s', path);
    end
end

function set_scope_title(mdl, sname, title_str)
    % Set scope title via ScopeSpecification if available, else ignore
    try
        sp = get_param([mdl '/' sname], 'ScopeSpecificationObject');
        sp.Title = title_str;
    catch
    end
end

% =============================================================================
%  MATLAB Function scripts
% =============================================================================

function s = script_refgen()
nl = newline;
s = ['function y = fcn(t)' nl ...
     'q0 = [0.45; 0.60; 0.30];' nl ...
     'qf = [1.00; 0.80; 0.40];' nl ...
     'dq = q0 - qf;' nl ...
     'e1 = exp(-t);  e4 = exp(-4*t);' nl ...
     'qref   = qf + (4/3).*dq.*e1 - (1/3).*dq.*e4;' nl ...
     'dqref  =    - (4/3).*dq.*e1 + (4/3).*dq.*e4;' nl ...
     'ddqref =      (4/3).*dq.*e1 - (16/3).*dq.*e4;' nl ...
     'y = [qref; dqref; ddqref];' nl ...
     'end' nl];
end

% -----------------------------------------------------------------------------
function s = script_disturbance()
nl = newline;
s = ['function tau_d = fcn(t)' nl ...
     'tau_d = [0.008.*sin(t)     + 0.002.*sin(200.*pi.*t);' nl ...
     '         0.005.*cos(2.*t)   + 0.002.*sin(200.*pi.*t);' nl ...
     '         0.003.*sin(0.5.*t) + 0.001.*sin(200.*pi.*t)];' nl ...
     'end' nl];
end

% -----------------------------------------------------------------------------
function s = script_controller()
nl = newline;
s = ['function [tau, s] = fcn(e, qdot, dqref, ddqref, q, xi)' nl ...
     'Lambda = diag([8, 7, 8]);' nl ...
     'K      = diag([0.012, 0.008, 0.005]);' nl ...
     'delta  = 0.03;' nl ...
     'eq_t  = -e;' nl ...
     'deq_t = qdot - dqref;' nl ...
     's     = deq_t + Lambda*eq_t + 0*xi;' nl ...
     'qdot_r  = dqref  - Lambda*eq_t;' nl ...
     'qddot_r = ddqref - Lambda*deq_t;' nl ...
     'r1=0.050; r2=0.035; r3=0.025;' nl ...
     'm1=0.015; m2=0.010; m3=0.006;' nl ...
     'J1=(1/3).*m1.*r1.^2; J2=(1/3).*m2.*r2.^2; J3=(1/3).*m3.*r3.^2;' nl ...
     'c1=r1./2; c2=r2./2; c3=r3./2; g=9.81;' nl ...
     'q1=q(1); q2=q(2); q3=q(3);' nl ...
     'M11=J1+J2+J3+m1.*c1.^2+m2.*(r1.^2+c2.^2+2.*r1.*c2.*cos(q2))+m3.*(r1.^2+r2.^2+c3.^2+2.*r1.*r2.*cos(q2)+2.*r1.*c3.*cos(q2+q3)+2.*r2.*c3.*cos(q3));' nl ...
     'M22=J2+J3+m2.*c2.^2+m3.*(r2.^2+c3.^2+2.*r2.*c3.*cos(q3));' nl ...
     'M33=J3+m3.*c3.^2;' nl ...
     'M12=J2+J3+m2.*(c2.^2+r1.*c2.*cos(q2))+m3.*(r2.^2+c3.^2+r1.*r2.*cos(q2)+r1.*c3.*cos(q2+q3)+2.*r2.*c3.*cos(q3));' nl ...
     'M13=J3+m3.*(c3.^2+r1.*c3.*cos(q2+q3)+r2.*c3.*cos(q3));' nl ...
     'M23=J3+m3.*(c3.^2+r2.*c3.*cos(q3));' nl ...
     'M=[M11 M12 M13; M12 M22 M23; M13 M23 M33];' nl ...
     'a2=m2.*r1.*c2.*sin(q2)+m3.*(r1.*r2.*sin(q2)+r1.*c3.*sin(q2+q3));' nl ...
     'b3=m3.*r2.*c3.*sin(q3);' nl ...
     'p3=m3.*r1.*c3.*sin(q2+q3);' nl ...
     'qr1=qdot_r(1); qr2=qdot_r(2); qr3=qdot_r(3);' nl ...
     'C1=-2.*a2.*qr1.*qr2 - a2.*qr2.^2 - 2.*(p3+b3).*(qr1+qr2).*qr3 - (p3+b3).*qr3.^2;' nl ...
     'C2= a2.*qr1.^2 - 2.*b3.*(qr1+qr2).*qr3 - b3.*qr3.^2;' nl ...
     'C3= p3.*qr1.^2 + b3.*(qr1+qr2).^2;' nl ...
     'Cv_r=[C1; C2; C3];' nl ...
     's1=sin(q1); s12=sin(q1+q2); s123=sin(q1+q2+q3);' nl ...
     'Gv=[g.*(m1.*c1.*s1+m2.*(r1.*s1+c2.*s12)+m3.*(r1.*s1+r2.*s12+c3.*s123));' nl ...
     '    g.*(m2.*c2.*s12+m3.*(r2.*s12+c3.*s123));' nl ...
     '    g.*m3.*c3.*s123];' nl ...
     'tau_eq = M*qddot_r + Cv_r + Gv;' nl ...
     'tau_sw = -K*sat_layer(s, delta);' nl ...
     'tau    = tau_eq + tau_sw;' nl ...
     'end' nl nl ...
     'function y = sat_layer(s, delta)' nl ...
     'y = zeros(3,1);' nl ...
     'for i = 1:3' nl ...
     '    if abs(s(i)) < delta, y(i) = s(i)./delta; else, y(i) = sign(s(i)); end' nl ...
     'end' nl ...
     'end' nl];
end

% -----------------------------------------------------------------------------
function s = script_plant()
nl = newline;
s = ['function qddot = fcn(q, qdot, tau, tau_d)' nl ...
     'q1=q(1); q2=q(2); q3=q(3);' nl ...
     'qd1=qdot(1); qd2=qdot(2); qd3=qdot(3);' nl ...
     'r1=0.050; r2=0.035; r3=0.025;' nl ...
     'm1=0.015; m2=0.010; m3=0.006;' nl ...
     'J1=(1/3).*m1.*r1.^2; J2=(1/3).*m2.*r2.^2; J3=(1/3).*m3.*r3.^2;' nl ...
     'c1=r1./2; c2=r2./2; c3=r3./2; g=9.81;' nl ...
     'M11=J1+J2+J3+m1.*c1.^2+m2.*(r1.^2+c2.^2+2.*r1.*c2.*cos(q2))+m3.*(r1.^2+r2.^2+c3.^2+2.*r1.*r2.*cos(q2)+2.*r1.*c3.*cos(q2+q3)+2.*r2.*c3.*cos(q3));' nl ...
     'M22=J2+J3+m2.*c2.^2+m3.*(r2.^2+c3.^2+2.*r2.*c3.*cos(q3));' nl ...
     'M33=J3+m3.*c3.^2;' nl ...
     'M12=J2+J3+m2.*(c2.^2+r1.*c2.*cos(q2))+m3.*(r2.^2+c3.^2+r1.*r2.*cos(q2)+r1.*c3.*cos(q2+q3)+2.*r2.*c3.*cos(q3));' nl ...
     'M13=J3+m3.*(c3.^2+r1.*c3.*cos(q2+q3)+r2.*c3.*cos(q3));' nl ...
     'M23=J3+m3.*(c3.^2+r2.*c3.*cos(q3));' nl ...
     'M=[M11 M12 M13; M12 M22 M23; M13 M23 M33];' nl ...
     'a2=m2.*r1.*c2.*sin(q2)+m3.*(r1.*r2.*sin(q2)+r1.*c3.*sin(q2+q3));' nl ...
     'b3=m3.*r2.*c3.*sin(q3);' nl ...
     'p3=m3.*r1.*c3.*sin(q2+q3);' nl ...
     'C1=-2.*a2.*qd1.*qd2 - a2.*qd2.^2 - 2.*(p3+b3).*(qd1+qd2).*qd3 - (p3+b3).*qd3.^2;' nl ...
     'C2= a2.*qd1.^2 - 2.*b3.*(qd1+qd2).*qd3 - b3.*qd3.^2;' nl ...
     'C3= p3.*qd1.^2 + b3.*(qd1+qd2).^2;' nl ...
     'Cv=[C1; C2; C3];' nl ...
     's1=sin(q1); s12=sin(q1+q2); s123=sin(q1+q2+q3);' nl ...
     'Gv=[g.*(m1.*c1.*s1+m2.*(r1.*s1+c2.*s12)+m3.*(r1.*s1+r2.*s12+c3.*s123));' nl ...
     '    g.*(m2.*c2.*s12+m3.*(r2.*s12+c3.*s123));' nl ...
     '    g.*m3.*c3.*s123];' nl ...
     'qddot = M \ (tau + tau_d - Cv - Gv);' nl ...
     'end' nl];
end
