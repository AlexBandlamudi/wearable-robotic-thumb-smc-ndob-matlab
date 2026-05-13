function [C_vec, G_vec] = manipulator_CG(q1, q2, q3, dq1, dq2, dq3)
% Coriolis-centrifugal vector C(q,qdot)*qdot and gravity vector G(q).
% Derived via Christoffel symbols from M(q). g=9.81 along -Y.

    r1 = 0.050;  r2 = 0.035;  r3 = 0.025;
    m1 = 0.015;  m2 = 0.010;  m3 = 0.006;
    c1 = r1/2;   c2 = r2/2;   c3 = r3/2;
    g  = 9.81;

    s1   = sin(q1);
    s12  = sin(q1+q2);
    s123 = sin(q1+q2+q3);
    s2   = sin(q2);
    s23  = sin(q2+q3);
    s3   = sin(q3);

    a2  = m2*r1*c2*sin(q2)  + m3*(r1*r2*sin(q2) + r1*c3*sin(q2+q3));
    a3  = m3*(r1*c3*sin(q2+q3) + r2*c3*sin(q3));
    b3  = m3*r2*c3*sin(q3);

    C1 = -2*a2*dq1*dq2 - a2*dq2^2 ...
         - 2*a3*(dq1+dq2)*dq3 - a3*dq3^2;

    C2 =  a2*dq1^2 ...
         - 2*b3*(dq1+dq2)*dq3 - b3*dq3^2;

    p3 = m3*r1*c3*sin(q2+q3);
    C3 =  p3*dq1^2 + b3*(dq1+dq2)^2;

    C_vec = [C1; C2; C3];

    G1 = g*(m1*c1*s1 + m2*(r1*s1 + c2*s12) + m3*(r1*s1 + r2*s12 + c3*s123));
    G2 = g*(m2*c2*s12 + m3*(r2*s12 + c3*s123));
    G3 = g*m3*c3*s123;

    G_vec = [G1; G2; G3];
end
