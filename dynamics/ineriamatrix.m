 clear all
 clc
 syms x y z dx dy dz t1 t2 t3 dt1 dt2 dt3
    
    sb= 0.41268;%base equilateral triangle side
    sp= 0.086;%platform equilateral triangle side
    L=0.150;%upper legs length
    
    wb=sb*sqrt(3)/6; 
    wp=sp*sqrt(3)/6; 
    up=sp*sqrt(3)/3; 
    
    %inettial parameters
    mcy=1; %payload
    mn1= 0.037178;  %platform
    mab= 0.00150711; %lower arm
    mab=mab*2; %lower arm full weight
    mb =0.088043;%upper arm
    mc = 0.0194735+2*0.0152885;%elbow's weight
    mn = mn1+mcy+3*mc;%platform + payload 
    mng= mn+3*mab; 
    mnt= mn+3*mab+3*mb;%total mass
    g=9.81;
    
    Im=0;%is the inertia of the motor
    Ibt=Im+L^2*(mb/3+mc+2*mab/3);
    Ib=diag(Ibt*ones(1,3));% mass matrix of the DELTA
    Gbg=L*(mb/2+mc+mab/2)*g*cos([t1;t2;t3]);
    
    a=wb-up;
    b=.5*(sp-sqrt(3)*wb);
    c=wp-.5*wb;
    

%jacobien
    AX=[x y z;2*x 2*y 2*z;2*x 2*y 2*z];
    AT=L*[0 cos(t1) sin(t1);-sqrt(3)*cos(t2) -cos(t2) 2*sin(t2);...
        sqrt(3)*cos(t3) -cos(t3) 2*sin(t3)];
    Ac=[0 a 0;2*b 2*c 0;-2*b 2*c 0];
    Bs=L*diag([(y+a)*sin(t1) -(sqrt(3)*(x+b)+y+c)*sin(t2) ...
    (sqrt(3)*(x-b)-y-c)*sin(t3)]);
    Bc=L*diag([-z*cos(t1) -2*z*cos(t2) -2*z*cos(t3)]);
    
    A=AX+AT+Ac;
    B=Bs+Bc;
    J=A\B;%jacobien
  %derivative of the jacobien   
    dAX=[dx dy dz;2*dx 2*dy 2*dz;2*dx 2*dy 2*dz];
    dAT=[0 -L*dt1*sin(t1) L*dt1*cos(t1);3^(1/2)*L*dt2*sin(t2)  L*dt2*...
        sin(t2) 2*L*dt2*cos(t2);-3^(1/2)*L*dt3*sin(t3) L*dt3*...
        sin(t3) 2*L*dt3*cos(t3)];
    dBs=[L*dy*sin(t1)+L*dt1*cos(t1)*(a + y) 0 0;0 -L*sin(t2)*(dy+...
        3^(1/2)*dx)-L*dt2*cos(t2)*(c+y+3^(1/2)*(b+x)) 0;0 0 -L*sin(t3)*...
        (dy-3^(1/2)*dx)-L*dt3*cos(t3)*(c+y+3^(1/2)*(b-x))];
    dBc=[L*dt1*sin(t1)*z-L*dz*cos(t1) 0 0;0 2*L*dt2*sin(t2)*z-2*L*dz*...
        cos(t2) 0;0 0 2*L*dt3*sin(t3)*z-2*L*dz*cos(t3)];
   
    dA=dAX+dAT;
    dB=dBs+dBc;
        dJ=-inv(A)*dA*inv(A)*B+A\dB;%dJacobien

        M = Ib+ mnt*(J'*J);
        G=J'*mng*[0;0;g]-Gbg;
        C=J'*mnt*dJ*[t1;t2;t3];
        H=G+C;


