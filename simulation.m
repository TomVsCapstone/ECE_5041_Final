% Tom Krenalka
% Brad Tomcho
% ECE 5041 Final Project
% 
% Brushless DC Machine Simulation
% 
% 

clear;
clc;

%% Ideal Machine Parameters

Rs = 0.1;           % resistance of stator windings
Lq = 150E-6;        % inductance of q phase
Ld = 250E-6;        % inductance of d phase
Lss = (Lq + Ld)/2;  % self inductance
Lamba_m = 0.5;      % 
P = 4;              % number of poles
J_bldc = 0.2;       % Inertia of the the machine only
J_total = 6.7;      % Intertia of the machine and vehicle

Bm_bldc = 0.002;    % Frictional loss of bldc
Bm_total = 0.167;   % Frictional loss of vehicle and bldc

%% Part D: BLDC Only Simulation (1)

Vm = 205*sqrt(2);           % Peak voltage
w = 2*pi*60;                % Electrical frequency
wr = 0;                     % Starting rotor speed = 0
t = 0;                      % start time at t=0

Va = Vm*cos(w*t);           % phase A voltage
Vb = Vm*cos(w*t - 120);     % phase B voltage
Vc = Vm*cos(w*t + 120);     % phase C voltage

% Setup state space matrices for integration
u = [Va
    Vb
    Vc];                    % Inputs in ABC

K = (2/3)*[cos(w*t) cos(w*t - 120) cos(w*t + 120)
    sin (w*t) sin(w*t - 120) sin(w*t + 120)
    1/2 1/2 1/2];           % Transformation matrix

uqd = K*u;                  % Input in DQ0 reference frame

Aqd = [-Rs/Lss -wr -Lamba_m/Lss 0
       wr -Rs/Lss 0 0
       (1/J_bldc)*(3/2)*(P/2)^(2)*Lamba_m 0 -Bm_bldc/J_bldc 0
       0 0 1 0];
   
Bqd = [1/Lss 0 0
       0 1/Lss 0
       0 0 -P/(2*J_bldc)
       0 0 0];

