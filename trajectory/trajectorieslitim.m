density
DELTA_ROBOT_X_DataFile
epsilon = 1e-3;
q1_0=deg2rad(160);
q2_0=deg2rad(160);
q3_0=deg2rad(160);
n=12;
w=2;
R=0.2;
s=0.05;
t = 0:s:n;
ttt = deg2rad(160);
Ti=[ttt;ttt;ttt];
    zd = -0.3;
    xd = 0;
    yd = 0;
for i=1:numel(t)
    %--------------------------%
    %path circle
    zd = -0.3+0.01*t(i);
    yd = R*sin(w*t(i));
    xd = R*cos(w*t(i));
%    
    %----------------------------%
    %path square

%     if t(i)<2 
%         yd = 0.15/2*t(i);
%         xd = 0.15/2*t(i)-0.15;
%     end
%     if t(i)<4 && t(i)>=2
%         yd = -0.15/2*(t(i)-2)+0.15;
%         xd = 0.15/2*(t(i)-2);
%     end
%     if t(i)<6 && t(i)>=4
%         yd =  -0.15/2*(t(i)-4);
%         xd = -0.15/2*(t(i)-4)+0.15;
%     end
%     if t(i)<8 && t(i)>=6
%         yd = 0.15/2*(t(i)-6)-0.15;
%         xd = -0.15/2*(t(i)-6);
%     end
    %----------------------%
     Tf = IGM(xd,yd,zd);
    [Tf(1),dTf(1),ddTf(1)]  = quintic1(Ti(1),Tf(1),0,0,0,0,0,s*i,t(i));
    [Tf(2),dTf(2),ddTf(2)] = quintic1( Ti(2),Tf(2),0,0,0,0,0,s*i,t(i));
    [Tf(3),dTf(3),ddTf(3)] = quintic1( Ti(3),Tf(3),0,0,0,0,0,s*i,t(i));
    Ti=Tf;
    X = FGM(Tf(1),Tf(2),Tf(3));
    xx(i)=X(1);
    yy(i)=X(2);
    zz(i)=X(3);
    q1(i)=Tf(1);
    q2(i)=Tf(2);
    q3(i)=Tf(3);
    dq1(i)=dTf(1);
    ddq1(i)=ddTf(1);
    dq2(i)=dTf(2);
    ddq2(i)=ddTf(2);
    dq3(i)=dTf(3);
    ddq3(i)=ddTf(3);
end

xdd = timeseries(xx,t);
ydd = timeseries(yy,t);
zdd = timeseries(zz,t);

q1d = timeseries(q1,t);
q2d = timeseries(q2,t);
q3d = timeseries(q3,t);

dq1d = timeseries(dq1,t);
dq2d = timeseries(dq2,t);
dq3d = timeseries(dq3,t);

ddq1d = timeseries(ddq1,t);
ddq2d = timeseries(ddq2,t);
ddq3d = timeseries(ddq3,t);

lamda1=50
lamda2=50
lamda3=50
K1=1000
K2=1000
K3=1000

figure;
plot3(xx,yy,zz,'LineWidth',1.5);
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Trajectoire cylindrique (hélice)');