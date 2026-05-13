function qddot = plant_dynamics(q, qdot, tau_total)
% Solves the three-link thumb equation of motion for joint acceleration.
% Returns qddot = M(q)^{-1} * (tau_total - C(q,qdot)*qdot - G(q)).

    M_mat          = manipulator_M(q(1), q(2), q(3));
    [C_vec, G_vec] = manipulator_CG(q(1), q(2), q(3), qdot(1), qdot(2), qdot(3));
    qddot          = M_mat \ (tau_total - C_vec - G_vec);
end
