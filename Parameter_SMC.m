clear all;
clc;

%% =========================================================
%  Aerodynamic Drag Coefficients
%  F_drag = -K1 * v * |v|  (translational drag)
%  M_drag = -K2 * w * |w|  (rotational drag)
% =========================================================
K1 = 0.01;  % Translational drag coefficient [N/(m/s)] — velocity-squared air resistance
K2 = 0;     % Rotational drag coefficient [N·m/(rad/s)] — neglected in this model

%% =========================================================
%  UAV Physical Parameters
% =========================================================
m   = 0.8;      % Drone mass [kg]
g   = 9.81;     % Gravitational acceleration [m/s^2]
l   = 0.33/2;   % Moment arm: distance from propeller center to drone center [m]
Jx  = 0.005;    % Moment of inertia about x-axis (roll)  [kg·m^2]
Jy  = 0.005;    % Moment of inertia about y-axis (pitch) [kg·m^2]
Jz  = 0.009;    % Moment of inertia about z-axis (yaw)   [kg·m^2]
                %   Jz > Jx, Jy because the rotational mass is distributed
                %   farther from the z-axis in the symmetric X-frame layout

%% =========================================================
%  Motor & PWM Parameters
%  PX4 "mixing" converts virtual control inputs (τ) to per-motor PWM signals:
%  v_i = (1000 * M * [τT; τP; τR; τY] - PWM_min) / (PWM_max - PWM_min)
% =========================================================
PWM_min = 1000;  % Minimum PWM signal [μs]
PWM_max = 2000;  % Maximum PWM signal [μs]
Tmax    = 4;     % Maximum thrust per motor [N]
Qmax    = 0.05;  % Maximum reaction torque per motor [N·m]

%% =========================================================
%  SMC Attitude Controller — Roll & Pitch (φ, θ)
%
%  Sliding surfaces (Eq. 21, 26):
%    s_φ = a3*e_φ + ė_φ
%    s_θ = a3*e_θ + ė_θ
%
%  Lyapunov stability conditions:
%    a3 - (2√2·l·Tmax / Jx)·k6 = 0  →  k6 = Jx·a3 / (√8·l·Tmax)
%    k5 > 0
%
%  k5 : gain on sat(s/ε) — governs switching magnitude and convergence speed
%  k6 : gain on ė term  — determined analytically from the stability condition
% =========================================================
a3 = 8.1910;    % Sliding surface slope for roll & pitch — PSO-tuned (Table 7)
k5 = 0.0388;    % Saturation function gain for roll & pitch control laws (Eq. 22, 27)

% k6 computed from stability condition  a3 = (2√2·l·Tmax / J)·k6
% Roll (Jx):
k6 = Jx * a3 / (l * sqrt(8) * Tmax);
% Pitch (Jy) — overwrites above; Jx == Jy here but stated explicitly for clarity:
%  k6 = Jy * a3 / (l * sqrt(8) * Tmax);

epsilon_1_a = 0.5;  % Boundary layer thickness ε for roll & pitch sliding surfaces
                    %   |s| < ε : sat(s/ε) = s/ε  → linear region, suppresses chattering
                    %   |s| ≥ ε : sat(s/ε) = sign(s) → hard switching

%% =========================================================
%  SMC Attitude Controller — Yaw (ψ)
%
%  Sliding surface (Eq. 31):
%    s_ψ = a4*e_ψ + ė_ψ
%
%  Lyapunov stability conditions:
%    a4 - (4·Qmax / Jz)·k8 = 0  →  k8 = a4·Jz / (4·Qmax)
%    k7 > 0
%
%  k7 : gain on sat(s_ψ/ε) in the yaw control law (Eq. 32)
%  k8 : gain on ė_ψ term   — determined analytically from the stability condition
% =========================================================
a4 = 2.7640;    % Sliding surface slope for yaw — PSO-tuned (Table 7)
k7 = 0.3147;    % Saturation function gain for yaw control law (Eq. 32)

% k8 computed from stability condition  a4 = (4·Qmax / Jz)·k8
k8 = a4 * Jz / (4 * Qmax);

epsilon_1_b = 0.5;  % Boundary layer thickness ε for yaw sliding surface

%% =========================================================
%  SMC Position Controller — X & Y
%
%  Sliding surface:
%    s = a1*e + ė
%
%  Control input u1:
%    u1 = k1'·sat(s/ε) + k2·ė + c·s - (drag compensation) - (FF acceleration)
%
%  Adaptive switching gain:
%    k1' = max(k1,  b1|e| + b2|ė|)
%    → scales up when error is large to accelerate reaching phase
%    → falls back to k1 near the surface to limit chattering
%
%  Lyapunov stability conditions:
%    a1 - k2 = 0  →  k2 = a1
%    k1' > 0,  c > 0
% =========================================================
a1 = 3.5524;    % Sliding surface slope for X & Y position — PSO-tuned
k1 = 0.001;     % Minimum switching gain (lower bound of adaptive k1')

% k2 set equal to a1 to satisfy stability condition  a1 - k2 = 0
k2 = a1;        % Gain on ė in the X & Y control laws

epsilon_2 = 0.5;  % Boundary layer thickness ε for X & Y position controllers

% Adaptive gain coefficients for k1' = max(k1, b1|e| + b2|ė|)
b_1x = 0.7426;  % Weight on position error |e_x|
b_2x = 1.4305;  % Weight on velocity error |ė_x|
c_x  = 0.2871;  % Damping gain on sliding variable s_x  (must be > 0)

b_1y = 0.5930;  % Weight on position error |e_y|
b_2y = 1.0895;  % Weight on velocity error |ė_y|
c_y  = 0.3867;  % Damping gain on sliding variable s_y  (must be > 0)

%% =========================================================
%  SMC Altitude Controller — Z
%
%  Sliding surface:
%    s = a2*e + ė
%
%  Control input τT:
%    τT = [m/(4cosθcosφ·Tmax)]·k3'·sat(s/ε)
%         + k4·ė/(cosθcosφ)
%         + (gravity & drag compensation terms) + 1
%
%  Adaptive switching gain:
%    k3' = max(k3,  b3|e| + b4|ė|)
%
%  Lyapunov stability conditions:
%    a2 - (4·Tmax/m)·k4 = 0  →  k4 = m·a2 / (4·Tmax)
%    k3' > 0
% =========================================================
a2 = 10.000;    % Sliding surface slope for altitude Z — PSO-tuned
k3 = 0.1;       % Minimum switching gain (lower bound of adaptive k3')

% k4 computed from stability condition  a2 = (4·Tmax/m)·k4
% Verify: m*a2/(4*Tmax) = 0.8*10/(4*4) = 0.5  ✓
k4 = 0.5000;    % Gain on ė_z in the altitude control law

epsilon_2_b = 0.5;  % Boundary layer thickness ε for altitude controller

b3 = 0.6373;  % Weight on altitude error  |e_z|
b4 = 0.0223;  % Weight on vertical velocity error |ė_z|

%% =========================================================
%  Disturbance Observer (DOB)
%
%  Example for X-axis:
%    d_ob  = z_in + k_ob * ẋ
%    ż_in  = -k_ob * d_ob + k_ob * (1/m)[u1·Thrust + K1·ẋ|ẋ|]
%
%  Error dynamics:  ė* = -k_ob·e* + ḋ
%    → Bounded ḋ  ⟹  bounded estimation error e*  (for any k_ob > 0)
%    → ḋ → 0      ⟹  e* → 0  (steady-state disturbance fully rejected)
%
%  The estimated disturbance d_ob is fed back into the control law
%  to actively cancel the disturbance effect.
%
%  Note: paper uses k_ob = 5; set to 1 here — increase if observer
%        convergence is too slow, but watch for noise amplification.
% =========================================================
k_ob = 1;       % Observer convergence gain — larger = faster convergence, more noise sensitivity