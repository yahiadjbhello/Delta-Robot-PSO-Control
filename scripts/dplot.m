x=Xi(:,1);
y=Xi(:,2);
z=Xi(:,3);
x1=Xo(:,1);
y1=Xo(:,2);
z1=Xo(:,3);
% tau1=Tau(:,1);
% tau2=Tau(:,2);
% tau3=Tau(:,3);
% theta1=theta(:,1);
% theta2=theta(:,2);
% theta3=theta(:,3);

figure(1)
grid on
plot3(x,y,z)
xlabel('x (meters)');
ylabel('y (meters)');
zlabel('z (meters)');
hold on
plot3(x1,y1,z1)
xlabel('x (meters)');
ylabel('y (meters)');
zlabel('z (meters)');
legend('The Desired Path','The Robot path')
% 
% figure(2)
% subplot(3,1,1)
% plot(tau1)
% subplot(3,1,2)
% plot(tau2)
% subplot(3,1,3)
% plot(tau3)