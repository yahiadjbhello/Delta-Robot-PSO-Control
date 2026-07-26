function [q,dq,ddq] = quintic1( q0,qf,dq0,dqf,ddq0,ddqf,t0,tf,t )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
   % tf=tf-ti;
   % a0=qi; a1=dpi;
   % a2=(3*(qf-qi)/(tf^2))-(dpf+2*dpi)/tf;
   % a3=((dpf+dpi)/tf^2)-2*(qf-qi)/tf^3;
   % q = a0+a1*t+a2*t.^2+a3*t.^3;
 
   B=[ 1    t0   t0^2   t0^3    t0^4     t0^5
       0    1    2*t0   3*t0.^2 4*t0.^3  5*t0.^4
       0    0    2      6*t0    12*t0.^2 20*t0.^3
       1    tf   tf^2   tf^3    tf^4     tf^5
       0    1    2*tf   3*tf.^2 4*tf.^3  5*tf.^4
       0    0    2      6*tf    12*tf.^2 20*tf.^3];
   C=[q0 dq0 ddq0 qf dqf ddqf]';
   A=B\C;
   a0=A(1);
   a1=A(2);
   a2=A(3);
   a3=A(4);
   a4=A(5);
   a5=A(6);
  q=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4+a5*t.^5 ;
  dq =a1+2*a2*t + 3*a3*t.^2+4*a4*t.^3+5*a5*t.^4;
  ddq = 2*a2 + 6*a3*t + 12*a4*t.^2+20*a5*t.^3;
end

