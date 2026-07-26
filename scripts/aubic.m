function [ q ] = aubic( qi,qf,dpi,dpf,ti,tf,t )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    tf=tf-ti;
    a0=qi; a1=dpi;
    a2=(3*(qf-qi)/(tf^2))-(dpf+2*dpi)/tf;
    a3=((dpf+dpi)/tf^2)-2*(qf-qi)/tf^3;
    q = a0+a1*t+a2*t.^2+a3*t.^3;

end

