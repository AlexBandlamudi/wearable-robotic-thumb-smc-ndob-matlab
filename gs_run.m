% Gain sweep script - pure script, no function definitions, everything inlined
% Tests K3=[20,40,60,80] mNm and delta=[0.06,0.10,0.15]
r1=0.050;r2=0.035;r3=0.025;m1=0.015;m2=0.010;m3=0.006;
c1=r1/2;c2=r2/2;c3=r3/2;g=9.81;
J1=(1/3)*m1*r1^2;J2=(1/3)*m2*r2^2;J3=(1/3)*m3*r3^2;

K3_vals=[0.020 0.040 0.060 0.080];
delta_vals=[0.06 0.10 0.15];
K1=0.040; K2=0.020;
Lam=[8 0 0;0 7 0;0 0 8];
q0v=[0.45;0.60;0.30]; qfv=[1.00;0.80;0.40]; dqv=q0v-qfv;
dt=0.0005; T=15; t=(0:dt:T)'; N=length(t);

best_e3=inf; best_K3=0; best_d=0;
for K3=K3_vals
  for delta=delta_vals
    Km=diag([K1 K2 K3]);
    x=zeros(6,1); x(1:3)=q0v;
    X=zeros(N,6); X(1,:)=x';
    for k=1:N-1
      xk=X(k,:)'; tk=t(k);
      q1=xk(1);q2=xk(2);q3=xk(3);qd1=xk(4);qd2=xk(5);qd3=xk(6);
      e1=exp(-tk); e4=exp(-4*tk);
      qr1=qfv(1)+(4/3)*dqv(1)*e1-(1/3)*dqv(1)*e4;
      qr2=qfv(2)+(4/3)*dqv(2)*e1-(1/3)*dqv(2)*e4;
      qr3=qfv(3)+(4/3)*dqv(3)*e1-(1/3)*dqv(3)*e4;
      dqr1=-(4/3)*dqv(1)*e1+(4/3)*dqv(1)*e4;
      dqr2=-(4/3)*dqv(2)*e1+(4/3)*dqv(2)*e4;
      dqr3=-(4/3)*dqv(3)*e1+(4/3)*dqv(3)*e4;
      ddqr1=(4/3)*dqv(1)*e1-(16/3)*dqv(1)*e4;
      ddqr2=(4/3)*dqv(2)*e1-(16/3)*dqv(2)*e4;
      ddqr3=(4/3)*dqv(3)*e1-(16/3)*dqv(3)*e4;
      s1=(qd1-dqr1)+Lam(1,1)*(q1-qr1);
      s2=(qd2-dqr2)+Lam(2,2)*(q2-qr2);
      s3=(qd3-dqr3)+Lam(3,3)*(q3-qr3);
      qdr1=dqr1-Lam(1,1)*(q1-qr1);
      qdr2=dqr2-Lam(2,2)*(q2-qr2);
      qdr3=dqr3-Lam(3,3)*(q3-qr3);
      qddr1=ddqr1-Lam(1,1)*(qd1-dqr1);
      qddr2=ddqr2-Lam(2,2)*(qd2-dqr2);
      qddr3=ddqr3-Lam(3,3)*(qd3-dqr3);
      % M matrix
      M11=J1+J2+J3+m1*c1^2+m2*(r1^2+c2^2+2*r1*c2*cos(q2))+m3*(r1^2+r2^2+c3^2+2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
      M22=J2+J3+m2*c2^2+m3*(r2^2+c3^2+2*r2*c3*cos(q3));M33=J3+m3*c3^2;
      M12=J2+J3+m2*(c2^2+r1*c2*cos(q2))+m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
      M13=J3+m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));M23=J3+m3*(c3^2+r2*c3*cos(q3));
      Mv=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
      % CG with qdot_r
      a2=m2*r1*c2*sin(q2)+m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));b3=m3*r2*c3*sin(q3);p3=m3*r1*c3*sin(q2+q3);
      Cr1=-2*a2*qdr1*qdr2-a2*qdr2^2-2*(p3+b3)*(qdr1+qdr2)*qdr3-(p3+b3)*qdr3^2;
      Cr2=a2*qdr1^2-2*b3*(qdr1+qdr2)*qdr3-b3*qdr3^2;
      Cr3=p3*qdr1^2+b3*(qdr1+qdr2)^2;
      s123v=sin(q1+q2+q3);s12v=sin(q1+q2);s1v=sin(q1);
      Gr1=g*(m1*c1*s1v+m2*(r1*s1v+c2*s12v)+m3*(r1*s1v+r2*s12v+c3*s123v));
      Gr2=g*(m2*c2*s12v+m3*(r2*s12v+c3*s123v));Gr3=g*m3*c3*s123v;
      tau_eq=Mv*[qddr1;qddr2;qddr3]+[Cr1;Cr2;Cr3]+[Gr1;Gr2;Gr3];
      sv=[s1;s2;s3]; sat_v=zeros(3,1);
      for ii=1:3; if abs(sv(ii))<delta,sat_v(ii)=sv(ii)/delta;else,sat_v(ii)=sign(sv(ii));end;end
      tau=tau_eq-Km*sat_v;
      td=[0.008*sin(tk)+0.002*sin(200*pi*tk);0.005*cos(2*tk)+0.002*sin(200*pi*tk);0.003*sin(0.5*tk)+0.001*sin(200*pi*tk)];
      % CG with qdot actual
      Cv1=-2*a2*qd1*qd2-a2*qd2^2-2*(p3+b3)*(qd1+qd2)*qd3-(p3+b3)*qd3^2;
      Cv2=a2*qd1^2-2*b3*(qd1+qd2)*qd3-b3*qd3^2;Cv3=p3*qd1^2+b3*(qd1+qd2)^2;
      Gv1=Gr1;Gv2=Gr2;Gv3=Gr3;
      qdd=Mv\(tau+td-[Cv1;Cv2;Cv3]-[Gv1;Gv2;Gv3]);
      X(k+1,:)=xk'+dt*[qd1 qd2 qd3 qdd(1) qdd(2) qdd(3)];
    end
    q3=X(:,3);
    qref3=qfv(3)+(4/3)*dqv(3)*exp(-t)-(1/3)*dqv(3)*exp(-4*t);
    idx=t>=13;
    e_rms3=sqrt(mean((q3(idx)-qref3(idx)).^2));
    e_ss3=mean(abs(q3(idx)-qref3(idx)));
    fprintf('K3=%3.0f mNm  delta=%.2f  RMS_e3=%.4f  SS_e3=%.4f  final_q3=%.4f vs ref=%.4f\n', ...
        K3*1000, delta, e_rms3, e_ss3, q3(end), qref3(end));
    if e_ss3 < best_e3, best_e3=e_ss3; best_K3=K3; best_d=delta; end
  end
end
fprintf('\nBest: K3=%.0f mNm, delta=%.2f, e3_ss=%.4f rad\n', best_K3*1000, best_d, best_e3);
