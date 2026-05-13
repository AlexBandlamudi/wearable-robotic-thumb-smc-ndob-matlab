% RK4 gain sweep — no function definitions (inline dynamics), T=8s for speed
r1=0.050;r2=0.035;r3=0.025;m1=0.015;m2=0.010;m3=0.006;
c1=r1/2;c2=r2/2;c3=r3/2;g=9.81;
J1=(1/3)*m1*r1^2;J2=(1/3)*m2*r2^2;J3=(1/3)*m3*r3^2;
q0v=[0.45;0.60;0.30]; qfv=[1.00;0.80;0.40]; dqv=q0v-qfv;
K1=0.012; K2=0.008;
Lam1=8; Lam2=7; Lam3=8;
dt=0.0005; T=8; t=(0:dt:T)'; N=length(t);

for K3=[0.005 0.008 0.012 0.016]
 for delta=[0.02 0.03 0.05]
  x=zeros(6,1); x(1:3)=q0v; X=zeros(N,6); X(1,:)=x';
  for k=1:N-1
   xk=X(k,:)'; tk=t(k);
   % --- dynamics inline (used 4x for RK4) ---
   % k1
   q1=xk(1);q2=xk(2);q3=xk(3);qd1=xk(4);qd2=xk(5);qd3=xk(6);
   e1=exp(-tk);e4=exp(-4*tk);
   qr1=qfv(1)+(4/3)*dqv(1)*e1-(1/3)*dqv(1)*e4; qr2=qfv(2)+(4/3)*dqv(2)*e1-(1/3)*dqv(2)*e4; qr3=qfv(3)+(4/3)*dqv(3)*e1-(1/3)*dqv(3)*e4;
   dqr1=-(4/3)*dqv(1)*e1+(4/3)*dqv(1)*e4; dqr2=-(4/3)*dqv(2)*e1+(4/3)*dqv(2)*e4; dqr3=-(4/3)*dqv(3)*e1+(4/3)*dqv(3)*e4;
   ddqr1=(4/3)*dqv(1)*e1-(16/3)*dqv(1)*e4; ddqr2=(4/3)*dqv(2)*e1-(16/3)*dqv(2)*e4; ddqr3=(4/3)*dqv(3)*e1-(16/3)*dqv(3)*e4;
   s1v=(qd1-dqr1)+Lam1*(q1-qr1); s2v=(qd2-dqr2)+Lam2*(q2-qr2); s3v=(qd3-dqr3)+Lam3*(q3-qr3);
   qdr1=dqr1-Lam1*(q1-qr1); qdr2=dqr2-Lam2*(q2-qr2); qdr3=dqr3-Lam3*(q3-qr3);
   qddr1=ddqr1-Lam1*(qd1-dqr1); qddr2=ddqr2-Lam2*(qd2-dqr2); qddr3=ddqr3-Lam3*(qd3-dqr3);
   M11=J1+J2+J3+m1*c1^2+m2*(r1^2+c2^2+2*r1*c2*cos(q2))+m3*(r1^2+r2^2+c3^2+2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M22=J2+J3+m2*c2^2+m3*(r2^2+c3^2+2*r2*c3*cos(q3));M33=J3+m3*c3^2;
   M12=J2+J3+m2*(c2^2+r1*c2*cos(q2))+m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M13=J3+m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));M23=J3+m3*(c3^2+r2*c3*cos(q3));
   Mv=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
   a2=m2*r1*c2*sin(q2)+m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));b3v=m3*r2*c3*sin(q3);p3v=m3*r1*c3*sin(q2+q3);
   Cr1=-2*a2*qdr1*qdr2-a2*qdr2^2-2*(p3v+b3v)*(qdr1+qdr2)*qdr3-(p3v+b3v)*qdr3^2;
   Cr2=a2*qdr1^2-2*b3v*(qdr1+qdr2)*qdr3-b3v*qdr3^2; Cr3=p3v*qdr1^2+b3v*(qdr1+qdr2)^2;
   s123v=sin(q1+q2+q3);s12v=sin(q1+q2);s1s=sin(q1);
   Gr1=g*(m1*c1*s1s+m2*(r1*s1s+c2*s12v)+m3*(r1*s1s+r2*s12v+c3*s123v));Gr2=g*(m2*c2*s12v+m3*(r2*s12v+c3*s123v));Gr3=g*m3*c3*s123v;
   tau_eq=Mv*[qddr1;qddr2;qddr3]+[Cr1;Cr2;Cr3]+[Gr1;Gr2;Gr3];
   if abs(s1v)<delta,sa1=s1v/delta;else,sa1=sign(s1v);end
   if abs(s2v)<delta,sa2=s2v/delta;else,sa2=sign(s2v);end
   if abs(s3v)<delta,sa3=s3v/delta;else,sa3=sign(s3v);end
   tau=tau_eq-[K1*sa1;K2*sa2;K3*sa3];
   td=[0.008*sin(tk)+0.002*sin(200*pi*tk);0.005*cos(2*tk)+0.002*sin(200*pi*tk);0.003*sin(0.5*tk)+0.001*sin(200*pi*tk)];
   Cv1=-2*a2*qd1*qd2-a2*qd2^2-2*(p3v+b3v)*(qd1+qd2)*qd3-(p3v+b3v)*qd3^2;
   Cv2=a2*qd1^2-2*b3v*(qd1+qd2)*qd3-b3v*qd3^2; Cv3=p3v*qd1^2+b3v*(qd1+qd2)^2;
   f1=Mv\(tau+td-[Cv1;Cv2;Cv3]-[Gr1;Gr2;Gr3]); k1=[qd1;qd2;qd3;f1];
   % k2
   x2=xk+0.5*dt*k1; q1=x2(1);q2=x2(2);q3=x2(3);qd1=x2(4);qd2=x2(5);qd3=x2(6); t2=tk+0.5*dt;
   e1=exp(-t2);e4=exp(-4*t2);
   qr1=qfv(1)+(4/3)*dqv(1)*e1-(1/3)*dqv(1)*e4; qr2=qfv(2)+(4/3)*dqv(2)*e1-(1/3)*dqv(2)*e4; qr3=qfv(3)+(4/3)*dqv(3)*e1-(1/3)*dqv(3)*e4;
   dqr1=-(4/3)*dqv(1)*e1+(4/3)*dqv(1)*e4; dqr2=-(4/3)*dqv(2)*e1+(4/3)*dqv(2)*e4; dqr3=-(4/3)*dqv(3)*e1+(4/3)*dqv(3)*e4;
   ddqr1=(4/3)*dqv(1)*e1-(16/3)*dqv(1)*e4; ddqr2=(4/3)*dqv(2)*e1-(16/3)*dqv(2)*e4; ddqr3=(4/3)*dqv(3)*e1-(16/3)*dqv(3)*e4;
   s1v=(qd1-dqr1)+Lam1*(q1-qr1); s2v=(qd2-dqr2)+Lam2*(q2-qr2); s3v=(qd3-dqr3)+Lam3*(q3-qr3);
   qdr1=dqr1-Lam1*(q1-qr1); qdr2=dqr2-Lam2*(q2-qr2); qdr3=dqr3-Lam3*(q3-qr3);
   qddr1=ddqr1-Lam1*(qd1-dqr1); qddr2=ddqr2-Lam2*(qd2-dqr2); qddr3=ddqr3-Lam3*(qd3-dqr3);
   M11=J1+J2+J3+m1*c1^2+m2*(r1^2+c2^2+2*r1*c2*cos(q2))+m3*(r1^2+r2^2+c3^2+2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M22=J2+J3+m2*c2^2+m3*(r2^2+c3^2+2*r2*c3*cos(q3));M33=J3+m3*c3^2;
   M12=J2+J3+m2*(c2^2+r1*c2*cos(q2))+m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M13=J3+m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));M23=J3+m3*(c3^2+r2*c3*cos(q3));
   Mv=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
   a2=m2*r1*c2*sin(q2)+m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));b3v=m3*r2*c3*sin(q3);p3v=m3*r1*c3*sin(q2+q3);
   Cr1=-2*a2*qdr1*qdr2-a2*qdr2^2-2*(p3v+b3v)*(qdr1+qdr2)*qdr3-(p3v+b3v)*qdr3^2;
   Cr2=a2*qdr1^2-2*b3v*(qdr1+qdr2)*qdr3-b3v*qdr3^2; Cr3=p3v*qdr1^2+b3v*(qdr1+qdr2)^2;
   s123v=sin(q1+q2+q3);s12v=sin(q1+q2);s1s=sin(q1);
   Gr1=g*(m1*c1*s1s+m2*(r1*s1s+c2*s12v)+m3*(r1*s1s+r2*s12v+c3*s123v));Gr2=g*(m2*c2*s12v+m3*(r2*s12v+c3*s123v));Gr3=g*m3*c3*s123v;
   tau_eq=Mv*[qddr1;qddr2;qddr3]+[Cr1;Cr2;Cr3]+[Gr1;Gr2;Gr3];
   if abs(s1v)<delta,sa1=s1v/delta;else,sa1=sign(s1v);end
   if abs(s2v)<delta,sa2=s2v/delta;else,sa2=sign(s2v);end
   if abs(s3v)<delta,sa3=s3v/delta;else,sa3=sign(s3v);end
   tau=tau_eq-[K1*sa1;K2*sa2;K3*sa3];
   td=[0.008*sin(t2)+0.002*sin(200*pi*t2);0.005*cos(2*t2)+0.002*sin(200*pi*t2);0.003*sin(0.5*t2)+0.001*sin(200*pi*t2)];
   Cv1=-2*a2*qd1*qd2-a2*qd2^2-2*(p3v+b3v)*(qd1+qd2)*qd3-(p3v+b3v)*qd3^2;
   Cv2=a2*qd1^2-2*b3v*(qd1+qd2)*qd3-b3v*qd3^2; Cv3=p3v*qd1^2+b3v*(qd1+qd2)^2;
   f2=Mv\(tau+td-[Cv1;Cv2;Cv3]-[Gr1;Gr2;Gr3]); k2=[qd1;qd2;qd3;f2];
   % k3
   x3=xk+0.5*dt*k2; q1=x3(1);q2=x3(2);q3=x3(3);qd1=x3(4);qd2=x3(5);qd3=x3(6);
   e1=exp(-t2);e4=exp(-4*t2);
   qr1=qfv(1)+(4/3)*dqv(1)*e1-(1/3)*dqv(1)*e4; qr2=qfv(2)+(4/3)*dqv(2)*e1-(1/3)*dqv(2)*e4; qr3=qfv(3)+(4/3)*dqv(3)*e1-(1/3)*dqv(3)*e4;
   dqr1=-(4/3)*dqv(1)*e1+(4/3)*dqv(1)*e4; dqr2=-(4/3)*dqv(2)*e1+(4/3)*dqv(2)*e4; dqr3=-(4/3)*dqv(3)*e1+(4/3)*dqv(3)*e4;
   ddqr1=(4/3)*dqv(1)*e1-(16/3)*dqv(1)*e4; ddqr2=(4/3)*dqv(2)*e1-(16/3)*dqv(2)*e4; ddqr3=(4/3)*dqv(3)*e1-(16/3)*dqv(3)*e4;
   s1v=(qd1-dqr1)+Lam1*(q1-qr1); s2v=(qd2-dqr2)+Lam2*(q2-qr2); s3v=(qd3-dqr3)+Lam3*(q3-qr3);
   qdr1=dqr1-Lam1*(q1-qr1); qdr2=dqr2-Lam2*(q2-qr2); qdr3=dqr3-Lam3*(q3-qr3);
   qddr1=ddqr1-Lam1*(qd1-dqr1); qddr2=ddqr2-Lam2*(qd2-dqr2); qddr3=ddqr3-Lam3*(qd3-dqr3);
   M11=J1+J2+J3+m1*c1^2+m2*(r1^2+c2^2+2*r1*c2*cos(q2))+m3*(r1^2+r2^2+c3^2+2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M22=J2+J3+m2*c2^2+m3*(r2^2+c3^2+2*r2*c3*cos(q3));M33=J3+m3*c3^2;
   M12=J2+J3+m2*(c2^2+r1*c2*cos(q2))+m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M13=J3+m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));M23=J3+m3*(c3^2+r2*c3*cos(q3));
   Mv=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
   a2=m2*r1*c2*sin(q2)+m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));b3v=m3*r2*c3*sin(q3);p3v=m3*r1*c3*sin(q2+q3);
   Cr1=-2*a2*qdr1*qdr2-a2*qdr2^2-2*(p3v+b3v)*(qdr1+qdr2)*qdr3-(p3v+b3v)*qdr3^2;
   Cr2=a2*qdr1^2-2*b3v*(qdr1+qdr2)*qdr3-b3v*qdr3^2; Cr3=p3v*qdr1^2+b3v*(qdr1+qdr2)^2;
   s123v=sin(q1+q2+q3);s12v=sin(q1+q2);s1s=sin(q1);
   Gr1=g*(m1*c1*s1s+m2*(r1*s1s+c2*s12v)+m3*(r1*s1s+r2*s12v+c3*s123v));Gr2=g*(m2*c2*s12v+m3*(r2*s12v+c3*s123v));Gr3=g*m3*c3*s123v;
   tau_eq=Mv*[qddr1;qddr2;qddr3]+[Cr1;Cr2;Cr3]+[Gr1;Gr2;Gr3];
   if abs(s1v)<delta,sa1=s1v/delta;else,sa1=sign(s1v);end
   if abs(s2v)<delta,sa2=s2v/delta;else,sa2=sign(s2v);end
   if abs(s3v)<delta,sa3=s3v/delta;else,sa3=sign(s3v);end
   tau=tau_eq-[K1*sa1;K2*sa2;K3*sa3];
   td=[0.008*sin(t2)+0.002*sin(200*pi*t2);0.005*cos(2*t2)+0.002*sin(200*pi*t2);0.003*sin(0.5*t2)+0.001*sin(200*pi*t2)];
   Cv1=-2*a2*qd1*qd2-a2*qd2^2-2*(p3v+b3v)*(qd1+qd2)*qd3-(p3v+b3v)*qd3^2;
   Cv2=a2*qd1^2-2*b3v*(qd1+qd2)*qd3-b3v*qd3^2; Cv3=p3v*qd1^2+b3v*(qd1+qd2)^2;
   f3=Mv\(tau+td-[Cv1;Cv2;Cv3]-[Gr1;Gr2;Gr3]); k3=[qd1;qd2;qd3;f3];
   % k4
   x4=xk+dt*k3; q1=x4(1);q2=x4(2);q3=x4(3);qd1=x4(4);qd2=x4(5);qd3=x4(6); t4=tk+dt;
   e1=exp(-t4);e4=exp(-4*t4);
   qr1=qfv(1)+(4/3)*dqv(1)*e1-(1/3)*dqv(1)*e4; qr2=qfv(2)+(4/3)*dqv(2)*e1-(1/3)*dqv(2)*e4; qr3=qfv(3)+(4/3)*dqv(3)*e1-(1/3)*dqv(3)*e4;
   dqr1=-(4/3)*dqv(1)*e1+(4/3)*dqv(1)*e4; dqr2=-(4/3)*dqv(2)*e1+(4/3)*dqv(2)*e4; dqr3=-(4/3)*dqv(3)*e1+(4/3)*dqv(3)*e4;
   ddqr1=(4/3)*dqv(1)*e1-(16/3)*dqv(1)*e4; ddqr2=(4/3)*dqv(2)*e1-(16/3)*dqv(2)*e4; ddqr3=(4/3)*dqv(3)*e1-(16/3)*dqv(3)*e4;
   s1v=(qd1-dqr1)+Lam1*(q1-qr1); s2v=(qd2-dqr2)+Lam2*(q2-qr2); s3v=(qd3-dqr3)+Lam3*(q3-qr3);
   qdr1=dqr1-Lam1*(q1-qr1); qdr2=dqr2-Lam2*(q2-qr2); qdr3=dqr3-Lam3*(q3-qr3);
   qddr1=ddqr1-Lam1*(qd1-dqr1); qddr2=ddqr2-Lam2*(qd2-dqr2); qddr3=ddqr3-Lam3*(qd3-dqr3);
   M11=J1+J2+J3+m1*c1^2+m2*(r1^2+c2^2+2*r1*c2*cos(q2))+m3*(r1^2+r2^2+c3^2+2*r1*r2*cos(q2)+2*r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M22=J2+J3+m2*c2^2+m3*(r2^2+c3^2+2*r2*c3*cos(q3));M33=J3+m3*c3^2;
   M12=J2+J3+m2*(c2^2+r1*c2*cos(q2))+m3*(r2^2+c3^2+r1*r2*cos(q2)+r1*c3*cos(q2+q3)+2*r2*c3*cos(q3));
   M13=J3+m3*(c3^2+r1*c3*cos(q2+q3)+r2*c3*cos(q3));M23=J3+m3*(c3^2+r2*c3*cos(q3));
   Mv=[M11 M12 M13;M12 M22 M23;M13 M23 M33];
   a2=m2*r1*c2*sin(q2)+m3*(r1*r2*sin(q2)+r1*c3*sin(q2+q3));b3v=m3*r2*c3*sin(q3);p3v=m3*r1*c3*sin(q2+q3);
   Cr1=-2*a2*qdr1*qdr2-a2*qdr2^2-2*(p3v+b3v)*(qdr1+qdr2)*qdr3-(p3v+b3v)*qdr3^2;
   Cr2=a2*qdr1^2-2*b3v*(qdr1+qdr2)*qdr3-b3v*qdr3^2; Cr3=p3v*qdr1^2+b3v*(qdr1+qdr2)^2;
   s123v=sin(q1+q2+q3);s12v=sin(q1+q2);s1s=sin(q1);
   Gr1=g*(m1*c1*s1s+m2*(r1*s1s+c2*s12v)+m3*(r1*s1s+r2*s12v+c3*s123v));Gr2=g*(m2*c2*s12v+m3*(r2*s12v+c3*s123v));Gr3=g*m3*c3*s123v;
   tau_eq=Mv*[qddr1;qddr2;qddr3]+[Cr1;Cr2;Cr3]+[Gr1;Gr2;Gr3];
   if abs(s1v)<delta,sa1=s1v/delta;else,sa1=sign(s1v);end
   if abs(s2v)<delta,sa2=s2v/delta;else,sa2=sign(s2v);end
   if abs(s3v)<delta,sa3=s3v/delta;else,sa3=sign(s3v);end
   tau=tau_eq-[K1*sa1;K2*sa2;K3*sa3];
   td=[0.008*sin(t4)+0.002*sin(200*pi*t4);0.005*cos(2*t4)+0.002*sin(200*pi*t4);0.003*sin(0.5*t4)+0.001*sin(200*pi*t4)];
   Cv1=-2*a2*qd1*qd2-a2*qd2^2-2*(p3v+b3v)*(qd1+qd2)*qd3-(p3v+b3v)*qd3^2;
   Cv2=a2*qd1^2-2*b3v*(qd1+qd2)*qd3-b3v*qd3^2; Cv3=p3v*qd1^2+b3v*(qd1+qd2)^2;
   f4=Mv\(tau+td-[Cv1;Cv2;Cv3]-[Gr1;Gr2;Gr3]); k4=[qd1;qd2;qd3;f4];
   X(k+1,:)=(xk+(dt/6)*(k1+2*k2+2*k3+k4))';
  end
  q3=X(:,3); q1v=X(:,1); q2v=X(:,2);
  qref3=qfv(3)+(4/3)*dqv(3)*exp(-t)-(1/3)*dqv(3)*exp(-4*t);
  qref1=qfv(1)+(4/3)*dqv(1)*exp(-t)-(1/3)*dqv(1)*exp(-4*t);
  qref2=qfv(2)+(4/3)*dqv(2)*exp(-t)-(1/3)*dqv(2)*exp(-4*t);
  idx=t>=6;
  e1_ss=mean(abs(q1v(idx)-qref1(idx))); e2_ss=mean(abs(q2v(idx)-qref2(idx))); e3_ss=mean(abs(q3(idx)-qref3(idx)));
  fprintf('K3=%3.0fmNm delta=%.2f | e_ss: J1=%.4f J2=%.4f J3=%.4f | q3_end=%.4f ref=%.4f\n', ...
      K3*1000,delta,e1_ss,e2_ss,e3_ss,q3(end),qref3(end));
 end
end
