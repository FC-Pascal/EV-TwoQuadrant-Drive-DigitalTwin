%% EV Motor Digital Twin - Initialization Script
clear; clc;

% 1. Electrical Parameters (48V, 500W PMDC Motor)
V_bus = 48;         % DC Bus / Battery Voltage (V)
Ra = 0.25;          % Armature Resistance (Ohms)
La = 0.0015;        % Armature Inductance (H) - 1.5 mH

% 2. Mechanical Parameters
J = 0.015;          % Rotor Inertia (kg.m^2) - Includes load inertia
B = 0.001;          % Viscous Friction Coefficient (N.m.s/rad)

% 3. Electromechanical Coupling
% In SI units, Torque Constant (Kt) equals Back-EMF Constant (Ke)
Kt = 0.35;          % Torque Constant (N.m/A)
Ke = 0.35;          % Back-EMF Constant (V/(rad/s))

% 4. Simulation Parameters
Ts_plant = 1e-6;    % 1 microsecond sample time for physical plant (Simscape)
Ts_ctrl = 1e-4;     % 100 microsecond sample time for embedded C-controller

disp('Motor Parameters Loaded Successfully.');