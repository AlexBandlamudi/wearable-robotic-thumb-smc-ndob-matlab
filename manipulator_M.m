function M = manipulator_M(q1, q2, q3)
% Inertia matrix M(q) for the 3-DOF planar robotic thumb.
% r=[50,35,25]mm, m=[15,10,6]g, J_i=m_i*r_i^2/3, CoM at r_i/2.

    r1 = 0.050;  r2 = 0.035;  r3 = 0.025;
    m1 = 0.015;  m2 = 0.010;  m3 = 0.006;
    J1 = (1/3)*m1*r1^2;
    J2 = (1/3)*m2*r2^2;
    J3 = (1/3)*m3*r3^2;
    c1 = r1/2;  c2 = r2/2;  c3 = r3/2;

    M11 = J1 + J2 + J3 ...
        + m1*c1^2 ...
        + m2*(r1^2 + c2^2 + 2*r1*c2*cos(q2)) ...
        + m3*(r1^2 + r2^2 + c3^2 ...
              + 2*r1*r2*cos(q2) + 2*r1*c3*cos(q2+q3) + 2*r2*c3*cos(q3));

    M22 = J2 + J3 ...
        + m2*c2^2 ...
        + m3*(r2^2 + c3^2 + 2*r2*c3*cos(q3));

    M33 = J3 + m3*c3^2;

    M12 = J2 + J3 ...
        + m2*(c2^2 + r1*c2*cos(q2)) ...
        + m3*(r2^2 + c3^2 + r1*r2*cos(q2) + r1*c3*cos(q2+q3) + 2*r2*c3*cos(q3));

    M13 = J3 ...
        + m3*(c3^2 + r1*c3*cos(q2+q3) + r2*c3*cos(q3));

    M23 = J3 ...
        + m3*(c3^2 + r2*c3*cos(q3));

    M = [M11, M12, M13;
         M12, M22, M23;
         M13, M23, M33];
end
