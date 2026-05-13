function gain_sweep_fn()
% Parametric gain search for optimal joint 3 tracking
fprintf('Gain sweep for Joint 3 tracking\n\n');

best_e3 = inf; best_K3=0; best_d=0;

for K3_mNm = [20 40 60 80]
    for delta_r = [0.06 0.10 0.15]
        Lambda = diag([8, 7, 8]);
        K      = diag([0.040, 0.020, K3_mNm/1000]);
        delta  = delta_r;

        dt=0.0005; T=15; t=(0:dt:T)'; N=length(t);
        x0=[0.45;0.60;0.30;0;0;0];
        X=zeros(N,6); X(1,:)=x0';
        for k=1:N-1
            X(k+1,:)=rk4_step(X(k,:)',t(k),dt,Lambda,K,delta)';
        end
        q=X(:,1:3);
        qref_all=zeros(N,3);
        q0c=[0.45;0.60;0.30]; qfc=[1.00;0.80;0.40]; dq=q0c-qfc;
        for k=1:N, qref_all(k,:)=(qfc+(4/3)*dq*exp(-t(k))-(1/3)*dq*exp(-4*t(k)))'; end
        idx_ss=t>=13;
        e_ss3=mean(abs(q(idx_ss,3)-qref_all(idx_ss,3)));
        e_ss1=mean(abs(q(idx_ss,1)-qref_all(idx_ss,1)));
        fprintf('K3=%d mNm  delta=%.2f  SS_e1=%.4f  SS_e3=%.4f rad\n', ...
            K3_mNm, delta_r, e_ss1, e_ss3);
        if e_ss3 < best_e3
            best_e3=e_ss3; best_K3=K3_mNm; best_d=delta_r;
        end
    end
end
fprintf('\nBest: K3=%d mNm, delta=%.2f, e3_ss=%.4f rad\n', best_K3, best_d, best_e3);
end

function xnext=rk4_step(x,t,dt,Lambda,K,delta)
    k1=thumb_xdot(x,t,Lambda,K,delta);
    k2=thumb_xdot(x+dt/2*k1,t+dt/2,Lambda,K,delta);
    k3=thumb_xdot(x+dt/2*k2,t+dt/2,Lambda,K,delta);
    k4=thumb_xdot(x+dt*k3,t+dt,Lambda,K,delta);
    xnext=x+(dt/6)*(k1+2*k2+2*k3+k4);
end

function xd=thumb_xdot(x,t,Lambda,K,delta)
    q=x(1:3); qd=x(4:6);
    q0=[0.45;0.60;0.30]; qf=[1.00;0.80;0.40]; dqv=q0-qf;
    e1=exp(-t); e4=exp(-4*t);
    qref=qf+(4/3)*dqv*e1-(1/3)*dqv*e4;
    dqref=-(4/3)*dqv*e1+(4/3)*dqv*e4;
    ddqref=(4/3)*dqv*e1-(16/3)*dqv*e4;
    et=q-qref; det=qd-dqref;
    s=det+Lambda*et;
    qd_r=dqref-Lambda*et; qdd_r=ddqref-Lambda*det;
    M=thumb_M(q(1),q(2),q(3));
    [Cv_r,Gv]=thumb_CG(q(1),q(2),q(3),qd_r(1),qd_r(2),qd_r(3));
    tau_eq=M*qdd_r+Cv_r+Gv;
    sat_v=zeros(3,1);
    for i=1:3
        if abs(s(i))<delta, sat_v(i)=s(i)/delta; else, sat_v(i)=sign(s(i)); end
    end
    tau=tau_eq-K*sat_v;
    td=[0.008*sin(t)+0.002*sin(200*pi*t); 0.005*cos(2*t)+0.002*sin(200*pi*t); 0.003*sin(0.5*t)+0.001*sin(200*pi*t)];
    [Cv,Gv2]=thumb_CG(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    qddot=M\(tau+td-Cv-Gv2);
    xd=[qd;qddot];
end

function M=thumb_M(q1,q2,q3)
    r1=0.050;r2=0.035;r3=0.025;m1=0.015;m2=0.010;m3=0.006;
    J1=(1/3)*m1*r1^2;J2=(1/3)*m2*r2^2;J3=(1/3)*m3*r3^2;c1=r1/2;c2=r2/2;c3=r3/2;
    M11=J1+J2+J3+m1*c1^2+m2*(r1^2+c2^2+2*r1*c2*cos(q2))+m3*(r1^2+r2^2+c3^2+2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
    M22=J2+J3+m2*c2^2+m3*(r2^2+c3^2+2*r2*c3*cos(q3));
    M33=J3+m3*c3^2;
    M12=J2+J3+m2*(c2^2+r1*c2*cos(q2))+m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
    M13=J3+m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));
    M23=J3+m3*(c3^2+r2*c3*cos(q3));
    M=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
end

function [Cv,Gv]=thumb_CG(q1,q2,q3,qd1,qd2,qd3)
    r1=0.050;r2=0.035;r3=0.025;m1=0.015;m2=0.010;m3=0.006;
    c1=r1/2;c2=r2/2;c3=r3/2;g=9.81;
    a2=m2*r1*c2*sin(q2)+m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));
    b3=m3*r2*c3*sin(q3); p3=m3*r1*c3*sin(q2+q3);
    C1=-2*a2*qd1*qd2-a2*qd2^2-2*(p3+b3)*(qd1+qd2)*qd3-(p3+b3)*qd3^2;
    C2= a2*qd1^2-2*b3*(qd1+qd2)*qd3-b3*qd3^2;
    C3= p3*qd1^2+b3*(qd1+qd2)^2;   % corrected cross-term
    Cv=[C1;C2;C3];
    s1=sin(q1);s12=sin(q1+q2);s123=sin(q1+q2+q3);
    Gv=[g*(m1*c1*s1+m2*(r1*s1+c2*s12)+m3*(r1*s1+r2*s12+c3*s123));
        g*(m2*c2*s12+m3*(r2*s12+c3*s123));
        g*m3*c3*s123];
end
fprintf('Gain sweep for Joint 3 tracking\n\n');

best_e3 = inf; best_K3=0; best_d=0;

for K3_mNm = [20 40 60 80]
    for delta_r = [0.06 0.10 0.15]
        Lambda = diag([8, 7, 8]);
        K      = diag([0.040, 0.020, K3_mNm/1000]);
        delta  = delta_r;

        dt=0.0005; T=15; t=(0:dt:T)'; N=length(t);
        x0=[0.45;0.60;0.30;0;0;0];
        X=zeros(N,6); X(1,:)=x0';
        for k=1:N-1
            X(k+1,:)=rk4_step(X(k,:)',t(k),dt,Lambda,K,delta)';
        end
        q=X(:,1:3);
        qref_all=zeros(N,3);
        q0c=[0.45;0.60;0.30]; qfc=[1.00;0.80;0.40]; dq=q0c-qfc;
        for k=1:N, qref_all(k,:)=(qfc+(4/3)*dq*exp(-t(k))-(1/3)*dq*exp(-4*t(k)))'; end
        idx_ss=t>=13;
        e_ss3=mean(abs(q(idx_ss,3)-qref_all(idx_ss,3)));
        e_ss1=mean(abs(q(idx_ss,1)-qref_all(idx_ss,1)));
        tau_pk3=0;
        fprintf('K1/K2/K3=%d/%d/%d mNm  delta=%.2f  SS_e1=%.4f  SS_e3=%.4f rad\n', ...
            40, 20, K3_mNm, delta_r, e_ss1, e_ss3);
        if e_ss3 < best_e3
            best_e3=e_ss3; best_K3=K3_mNm; best_d=delta_r;
        end
    end
end
fprintf('\nBest: K3=%d mNm, delta=%.2f, e3_ss=%.4f rad\n', best_K3, best_d, best_e3);
