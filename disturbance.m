function tau_d = disturbance(t)
% Background disturbance + sudden load at t=10s. Returns [N·m].
% Used by Simulink model only; runners call their own inline version.

    tau_bg = [ 0.0020 *sin(t)     + 0.0005 *sin(200*pi*t);
               0.0010 *cos(2*t)   + 0.0005 *sin(200*pi*t);
               0.00015*sin(0.5*t) + 0.00005*sin(200*pi*t)];

    sigma    = 0.5 * (1 + tanh( (t - 10.0) / 0.1 ));
    F_load   = [-0.0030;  -0.0030;  -0.00020];
    tau_load = sigma * F_load;

    tau_d = tau_bg + tau_load;
end
