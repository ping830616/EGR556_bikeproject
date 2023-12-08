clear all
clc

%% bicycle parameters
a = 4; % distance from a vertical line through the CM to P1
b = 1; % wheel base
c = 1; % trail

Rrw = 0.35; % (m)
Rfw = 0.35; % (m)

% mass (kg)
mB = 87; % rear frame 
mH = 2; % front frame 
mF = 1.5; % front wheel
mR = 1.5; % rear wheel
m = mB+mH+mF+mR; % total mass

% center of mass (m) and human body mass center x coordinate
xB = 0.492; % rear frame 
xH = 0.866; % front frame
xF = b; % front wheel
xR = 0; % rear wheel

% center of mass (m) and human body mass center x coordinate
zB = 1.028; % rear frame 
zH = 0.676; % front frame
zF = Rfw; % front wheel
zR = Rrw; % rear wheel


%% the front fork
t = [7,2,1,3,5,7]; % time
V = m./t; % velocity
g = 9.81; % gravity
ang = 90; % head angle
roll = 15; % roll angle
steer = 10; % steer angle

%%% the small angle influences tilt angle
rollf = roll-steer*cos(ang);  % front fork roll angle
steerf = steer*cos(ang);  % front fork steer angle

%%% a static torque balance
Nf = (a*m*g)/b;   % vertical component of the forces
Ff = (a*m*V.^2)/(b^2)*steerf;   % horizontal component of the forces

T = (Ff+Nf*rollf)*c*sin(ang);  % external torque

Vsa = sqrt(b*g*cot(ang));  % self-alignment velocity


% the torque balance for the front fork assembly
k1 = b^2./((V.^2*sin(ang)-b*g*cos(ang))*m*a*c*sin(ang));
k2 = (b*g)./(V.^2*sin(ang)-b*g*cos(ang));
steer = k1.*T-k2.*rollf












