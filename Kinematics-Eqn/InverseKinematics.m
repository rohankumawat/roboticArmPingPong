clc;
clear;


%% Joint angles
ja = [0,0,0,0,0];

%% D-H Table
theta = [ja(1), ja(2), ja(3)+pi/2, ja(4), ja(5)-pi/2];
a =[0.0, 0.18, 0.0, 0.0, 0.1];
d = [0.05, 0.0, 0.0, 0.28, 0.0];
alpha = [pi/2, 0, pi/2, -pi/2, 0];

%% Link Generation
L(1) = Link([theta(1), d(1),  a(1), alpha(1)], 'standrad');
L(2) = Link([theta(2), d(2),  a(2), alpha(2)], 'standrad');
L(3) = Link([theta(3), d(3),  a(3), alpha(3)], 'standrad');
L(4) = Link([theta(4), d(4),  a(4), alpha(4)], 'standrad');
L(5) = Link([theta(5), d(5),  a(5), alpha(5)], 'standrad');

R = SerialLink(L);
R.name = "RAGO";


%% End effector position

%pos = input("Enter the end effector position: ");

pos = [0.4, 0.0, 0.145];

x = pos(1);
y = pos(2);
z = pos(3);



l1 = sqrt(x^2 + y^2);
l2 = z - d(1);
l3 = sqrt(l1^2 + l2^2);

t = [];
t(1) = atan2(y,x);
t(2) = atan2(l2,l1) - acos((a(2)^2 + l3^2 - d(4)^2)/(2*a(2)*l3));
t(3) = acos((l3^2 - a(2)^2 -d(4)^2)/(2*a(2)*d(4)));


d = rad2deg(t)

