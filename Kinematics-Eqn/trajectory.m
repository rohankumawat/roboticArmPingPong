theta1 = pi/4;
theta2 = pi/2;
tf = 2;
t = 0:0.1:2;

a0 = theta1;
a1 = 0;
a2 = 3*(theta2 - theta1)/tf^2;
a3 = -2*(theta2 - theta1)/tf^3;

%disp(a0)
%disp(a1)
%disp(a2)
%disp(a3)

%theta_tp = (a0 + a1*t + a2*t.^2 + a3*t.^3)*180/pi;
%disp(theta_tp)

jointVel = 3*a3*t.^2 + 2*a2*t+a1;
disp(jointVel)

jointAcc = 6*a3*t+2*a2;

figure;plot(t,jointVel,'LineWidth',2)