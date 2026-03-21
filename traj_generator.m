% generate_ref_traj_functions.m
% Define reference trajectory symbolically and generate MATLAB function files.
%
% Requirements:
%   - Trajectory must be C4 (four-times continuously differentiable).
%   - psi must be defined as a function of t (e.g. t*0 for constant yaw)
%     to avoid dimension mismatch in generated function files.
%
% Output files: ref_pos.m, ref_vel.m, ref_acc.m, ref_jerk.m, ref_snap.m
% These are called by ref_traj.m inside the Simulink model.

syms t real

%% ── Trajectory definition ───────────────────────────────────────────────
% Modify x, y, z, psi to change the reference trajectory.

s  = t * pi / 20;
cs = cos(s);
ss = sin(s);

x   = 0.5 * cs;
y   = 0.5 * ss * cs;
z   = 3 - (2 * cs);
psi = t * 0;   % Set as function of t. Use t*0 for constant yaw = 0.
               % For time-varying yaw, define psi as a symbolic expression.

%% ── Compute derivatives up to snap ─────────────────────────────────────
pos  = [x; y; z; psi];
vel  = diff(pos,  t);   % velocity
acc  = diff(vel,  t);   % acceleration
jerk = diff(acc,  t);   % jerk
snap = diff(jerk, t);   % snap — required for DF feedforward

%% ── Save as MATLAB function files ───────────────────────────────────────
matlabFunction(pos,  'Vars', {t}, 'File', 'ref_pos');
matlabFunction(vel,  'Vars', {t}, 'File', 'ref_vel');
matlabFunction(acc,  'Vars', {t}, 'File', 'ref_acc');
matlabFunction(jerk, 'Vars', {t}, 'File', 'ref_jerk');
matlabFunction(snap, 'Vars', {t}, 'File', 'ref_snap');

fprintf('Done. Function files generated: ref_pos, ref_vel, ref_acc, ref_jerk, ref_snap\n');