function P  = FGM( th1,th2,th3 )
    sb= 0.41268;%base equilateral triangle side
    sp= 0.086;%platform equilateral triangle side
    L=0.150;%upper legs length
    l = 0.300;
    wb=sb*sqrt(3)/6; 
    wp=sp*sqrt(3)/6; 
    up=sp*sqrt(3)/3; 

    %Representation of E1, E2, E3 in the fixed base frame
    e1=[(wb-up)+L*cos(th1) 0 -L*sin(th1)];
    e2=[(wb-up)+L*cos(th2) 0 -L*sin(th2)];
    e3=[(wb-up)+L*cos(th3) 0 -L*sin(th3)];
    E1=rot_0Z(0)*e1';
    E2=rot_0Z(2*pi/3)*e2'; 
    E3=rot_0Z(4*pi/3)*e3';
    %calculation of cartisien position using steps of sec(2.4)
    M=[E1';E2';E3'];
    M1=-M;
    N=zeros(2,3);
    V=zeros(2);
    for i=1:2
        for j=1:3
            N(i,j)=2*(M1(1,j)-M1(i+1,j));
        end
        V(i)=((M1(i+1,1)^2+M1(i+1,2)^2+M1(i+1,3)^2)-...
            (M1(1,1)^2+M1(1,2)^2+M1(1,3)^2));
    end
    
    D=N(1,1)*N(2,2)-N(2,1)*N(1,2);
    f1=(N(2,2)*V(1)-N(1,2)*V(2))/D;
    f2=(N(1,1)*V(2)-N(2,1)*V(1))/D;
    f3=(N(1,2)*N(2,3)-N(1,3)*N(2,2))/D;%fx
    f4=(N(2,1)*N(1,3)-N(1,1)*N(2,3))/D;%fy
    E=(f3^2 + f4^2 + 1);
    F=(2*M1(3,3) + 2*f3*(f1 + M1(3,1)) + 2*f4*(f2 + M1(3,2)));
    G=(f1 + M1(3,1))^2 + (f2 + M1(3,2))^2 + M1(3,3)^2-l^2;
    d = F^2-4*E*G;
    if d < 0
        z = nan;
        x = nan;
        y = nan;
        S = -1;
    else
        z=(-F-sqrt(d))/(2*E);
        x=f1+f3*z;
        y=f2+f4*z;
        if (z == 0)&&(x==0)&&(y==0)
            S = -1;
        else 
            S=0;
        end
    end
    P=[x;y;z];
    P=rot_0Z(-pi/2)*P;
   
end