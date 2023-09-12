% BIKE PROJECT - EGR 556
% SLIDING MODE CONTROL FOR STABILIZATION OF AN AUTONOMOUS BIKE

close all;
clc;

% Bike parameters
% mass (kg)
m = 92;

% height of center of mass
h= 1.028;

% Moment of inertia w.r.t X axis
J = 3.28; % rear frame

% Inertia product w.r.t XZ axes
D = 0.603; % rear frame

% Acceleration due to gravity
g= 9.8;

% State space model of Bike

% initial conditions [ psi, psi_dot ]
x0 = [ 0.011; 0.0001];  % 0.63 degrees - 0.011 radian

% state space model
global A2
A2 = ((m*g*h)/J);

% Reference 
sim_time = [ 0 60];
global psie_dot
psie_dot = 0;
global i
i =0;
global U
U = zeros(293,1);
[t,x] = ode45(@ODEfunction,sim_time, x0,psie_dot);

% PLOTTING GRAPHS

%plot 1 - states vs time
figure(1)
hold on
set(gca,'Fontsize',10);
grid on
% plot the states
plot(t,x(:,1),'k','Linewidth',2) 
plot(t,x(:,2),'b','Linewidth',2) 
% add initial values of states to the plot
plot(t(1),x0(1,1),'k-o','Markersize',10,'Linewidth',2) 
plot(t(1),x0(2,1),'b-o','Markersize',10,'Linewidth',2) 
legend('Roll angle (psi) ','Roll angle derevative (psi_dot)'); 
xlabel('Time') 
ylabel('State') 
title('Sliding mode control - Stabilizing Bike - Plot of States vs Time ')

st = size(t);
%plot 2 - control input time
figure(2)
hold on
set(gca,'Fontsize',10);
grid on
% plot the states
plot(t,U(1:st),'r','Linewidth',2)
legend('lamda = 1 and K = 0.01');
xlabel('Time') 
ylabel('Control input (U) ') 
title('Sliding mode control - Stabilizing Bike - Plot of Control Input vs Time ')

% ODE FUNCTION 

function [dx,U] = ODEfunction(t, x, psie_dot)

global i
global U
global A2
global psie_dot
    
i = i+1;
% system dynamics
A = [ 0 1; A2 0 ];
B = [ 0; 1];

% Reference / set point
Xref = [0; 0];

psie = x(1) - Xref(1);
psiref_dot = 0;
A2X = A(2)*x(1) + A(4)*x(2);

lamda = 0.1;
k = 0.1;
s = psie_dot + (lamda*psie);
delta_temp = -inv(B(2))*( A2X - psiref_dot + (lamda*psie_dot) );
 
U(i) = delta_temp - (k*sign(s));

% system output
X_dot = A*x + B*U(i);
psie_dot = X_dot(1);
dx = X_dot;

end
