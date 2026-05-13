function K = thumb_kinematics(q)
% Standard DH forward kinematics for the planar 3R robotic thumb.
% Returns transforms A_i, T_0i, T_03, joint positions, and fingertip pose.
% Default q = [0.70; 1.20; 0.85] (closed grip) if omitted.

    if nargin < 1 || isempty(q)
        q = [0.70; 1.20; 0.85];
    end
    q = q(:);
    if numel(q) ~= 3
        error('thumb_kinematics:BadInput', 'q must be a 3x1 joint vector [q1;q2;q3].');
    end

    a = [0.050; 0.035; 0.025];
    d = zeros(3,1);
    alpha = zeros(3,1);
    theta = q;

    A = zeros(4,4,3);
    T_0i = zeros(4,4,3);
    T = eye(4);
    joint_positions_m = zeros(4,3);

    for i = 1:3
        A(:,:,i) = dh_standard(a(i), alpha(i), d(i), theta(i));
        T = T * A(:,:,i);
        T_0i(:,:,i) = T;
        joint_positions_m(i+1,:) = T(1:3,4).';
    end

    phi = sum(q);
    px = a(1)*cos(q(1)) + a(2)*cos(q(1)+q(2)) + a(3)*cos(phi);
    py = a(1)*sin(q(1)) + a(2)*sin(q(1)+q(2)) + a(3)*sin(phi);

    T_closed_form = [cos(phi), -sin(phi), 0, px;
                     sin(phi),  cos(phi), 0, py;
                     0,         0,        1, 0;
                     0,         0,        0, 1];

    dh_table = table({'MCP'; 'PIP'; 'DIP'}, a, d, alpha, theta, ...
        'VariableNames', {'Joint','a_m','d_m','alpha_rad','theta_rad'});

    K = struct();
    K.link_lengths_m = a;
    K.dh_table = dh_table;
    K.A = A;
    K.T_0i = T_0i;
    K.T_03 = T;
    K.T_03_closed_form = T_closed_form;
    K.joint_positions_m = joint_positions_m;
    K.joint_positions_mm = joint_positions_m(:,1:2) * 1000;
    K.tip_position_m = [px; py; 0];
    K.tip_position_mm = [px; py] * 1000;
    K.tip_angle_rad = phi;
    K.tip_angle_deg = rad2deg(phi);
end

function A = dh_standard(a, alpha, d, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,   sa,     ca,    d;
         0,   0,      0,     1];
end