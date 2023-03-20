clc;
clear;


%% Joint angles


% ja = input("Enter joint angles (ja1->ja5): ");

ja = [0,0,0,0,0];

%% D-H Table
theta = [ja(1), ja(2)+pi/2, ja(3)-pi/2, ja(4), ja(5)+pi/2];
a =[0.0, 0.17275, 0.0, 0.0, 0.095]; %% A5 temp
d = [0.02, 0.0, 0.0, 0.258, 0.0];
alpha = [pi/2, 0, -pi/2, pi/2, 0];

%% Link Generation
L(1) = Link([theta(1), d(1),  a(1), alpha(1)], 'standrad');
L(2) = Link([theta(2), d(2),  a(2), alpha(2)], 'standrad');
L(3) = Link([theta(3), d(3),  a(3), alpha(3)], 'standrad');
L(4) = Link([theta(4), d(4),  a(4), alpha(4)], 'standrad');
L(5) = Link([theta(5), d(5),  a(5), alpha(5)], 'standrad');

R = SerialLink(L)
R.name = "RAGO";

t = [0, 0,0,0,0];
R.plot(theta+t);

tmat = R.fkine(theta+t)
% disp(tm0_5);

R.teach
%% Test
% 
% t1 = [0, pi/4, pi/3, pi/2, pi/1.5];
% t2 = [0, pi/4, pi/3, pi/2, pi/2];
% for i= 1:5
% 
% R.plot([t1(i),t2(i),theta(3),theta(4),theta(5)]);
% 
% tmat = R.fkine([t1(i),t2(i),theta(3),theta(4),theta(5)])
% pause(1);
% end


