%% Full State Feedback Controller
% Simple & Poweful
% Feedback controller (K = regulator) hopes to regulate the system back
% to the origin.

clc; clear all;

%% open-loop eigevalues

% constants
m = 2;
g = 9.8;
h = 1;
v = 5;
J = 5;
b = 3;
D = 4;

% matrices
A = [0 1;m*g*h/J 0];
B = [0;1];
C = [m*v^2*h/b, v*D/(b*J)];
D = [0 0]';

% find eigenvalues
[V,D]=eig(A)

%------------ Design full state feedback controller ----------------
%% verify the controllability of the system
Pc = ctrb(A,B);
rank(Pc)


%% solve the initial value problem for a 1st order differential equation
% t = time interval
% x = initial condition
[t,x]=ode45(@ssmodel1,[0 10],[-2 4]);


%% plot
plot(t,x, 'LineWidth', 2)
legend('x_1','x_2')
xlabel('Time (sec)')
ylabel('x(t)')
title('Full State Feedback Controller')
grid


%% function
function dx = ssmodel1(t,x)  % state-space model

% constants
m = 2;
g = 9.8;
h = 1;
J = 5;

% matrices
A = [0 1;m*g*h/J 0];
B = [0;1];

% find eigenvalues
open_loop_poles=eig(A);

% choose closed-loop eigenvalues/poles: -2, 2
 desired_closed_loop_poles = [-2, 2];
 %desired_closed_loop_poles = [-2, open_loop_poles(2)]; 

K = acker(A,B,desired_closed_loop_poles); % pole placement design 
u = -K*x;
dx = A*x + B*u;

end




