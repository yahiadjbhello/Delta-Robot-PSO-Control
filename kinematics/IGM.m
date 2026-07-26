function [q]= IGM(x,y,z)
    
    sb= 0.41268;%base equilateral triangle side
    sp= 0.086;%platform equilateral triangle side
    L=0.150;%upper legs length
    l = 0.3;
    wb=sb*sqrt(3)/6; 
    wp=sp*sqrt(3)/6; 
    up=sp*sqrt(3)/3; 
    
a = wb - up;
b = sp/2 - sqrt(3)/2*wb;
c = wp - wb/2;
H =[0;0;0];
G = [0;0;0];
F = [0;0;0];
t = [0;0;0];
t1 = [0;0;0];
q = [0;0;0];
q1= [0;0;0];
H(1) = 2*L*(y+a);
H(2) = -L*(sqrt(3)*(x+b) + y + c);
H(3) = L*(sqrt(3)*(x-b)-y-c);

G(1)= x^2 + y^2 + z^2 + a^2 + L^2 + 2*y*a -l^2;
G(2)= x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*(x*b+y*c) - l^2;
G(3)= x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2*(-x*b+y*c) - l^2;

F(1)= 2*z*L;
F(2)= 2*z*L;
F(3)= 2*z*L;

for i = 1:3
    t(i) = (-F(i) + sqrt(H(i)^2 + F(i)^2 - G(i)^2))/(G(i)-H(i));
    t1(i) = (-F(i) - sqrt(H(i)^2 + F(i)^2 - G(i)^2))/(G(i)-H(i));
    q(i) = 2*atan(t(i));
    q1(i) = 2*atan(t1(i));
    if q(i)<= 0 
        q(i) = 2*pi + q(i);
   
end
end
