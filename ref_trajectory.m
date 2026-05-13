function [qr, dqr, ddqr] = ref_trajectory(t)
% 3-phase reference: CLOSE (quintic, 0-5s) | HOLD (5-10s) | SUSTAIN-LOAD (10-15s).
% Used by Simulink model only; runners use their own ref_full_cycle.

    q0 = [0.10; 0.10; 0.05];   % open / extended
    qf = [0.70; 1.20; 0.85];   % curled grip
    dq = qf - q0;

    T1 = 5;   % end of CLOSE phase

    if t <= 0
        s = 0; ds = 0; dds = 0;
    elseif t < T1
        u   = t / T1;
        s   =          10*u^3 - 15*u^4 +  6*u^5;
        ds  = (1/T1)   *( 30*u^2 - 60*u^3 + 30*u^4);
        dds = (1/T1^2) *( 60*u   -180*u^2 +120*u^3);
    else
        s = 1; ds = 0; dds = 0;
    end

    qr   = q0 + s   * dq;
    dqr  =      ds  * dq;
    ddqr =      dds * dq;
end
