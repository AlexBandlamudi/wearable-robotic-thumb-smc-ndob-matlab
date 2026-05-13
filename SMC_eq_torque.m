function tau_eq = SMC_eq_torque(q, qdot, qref, dqref, ddqref, Lambda)
% Equivalent control torque for model-based SMC (Slotine-Li reference dynamics).

    e_tilde  = q    - qref;
    de_tilde = qdot - dqref;

    qdot_r  = dqref  - Lambda * e_tilde;
    qddot_r = ddqref - Lambda * de_tilde;

    M_mat            = manipulator_M(q(1), q(2), q(3));
    [C_r_vec, G_vec] = manipulator_CG(q(1), q(2), q(3), qdot_r(1), qdot_r(2), qdot_r(3));
    tau_eq = M_mat * qddot_r + C_r_vec + G_vec;
end
