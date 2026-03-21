% result_plotter.m
% Loads simulation data from Excel and visualizes tracking performance.
% Plots: 3D trajectory, position time-history, velocity time-history,
%        acceleration time-history, and position tracking error.

clc; clear; close all;

%% ── 0. Configuration ────────────────────────────────────────────────────
filename = 'simulation_result_data.xlsx';  % ← change as needed

%% ── 1. Load data ────────────────────────────────────────────────────────
T  = readtable(filename);

t      = T.Time;

x      = T.X;      x_ref  = T.X_ref;
y      = T.Y;      y_ref  = T.Y_ref;
z      = T.Z;      z_ref  = T.Z_ref;

dx     = T.dX;     dx_ref = T.dX_ref;
dy     = T.dY;     dy_ref = T.dY_ref;
dz     = T.dZ;     dz_ref = T.dZ_ref;

ddx    = T.ddX;    ddx_ref = T.ddX_ref;
ddy    = T.ddY;    ddy_ref = T.ddY_ref;
ddz    = T.ddZ;    ddz_ref = T.ddZ_ref;

%% ── 2. Tracking error ───────────────────────────────────────────────────
ex  = x  - x_ref;
ey  = y  - y_ref;
ez  = z  - z_ref;
e3  = sqrt(ex.^2 + ey.^2 + ez.^2);   % 3D position error norm

fprintf('--- Tracking Performance ---\n');
fprintf('RMS error  | x: %.4f m  y: %.4f m  z: %.4f m\n', ...
    rms(ex), rms(ey), rms(ez));
fprintf('Max error  | x: %.4f m  y: %.4f m  z: %.4f m\n', ...
    max(abs(ex)), max(abs(ey)), max(abs(ez)));
fprintf('RMS 3D norm: %.4f m\n', rms(e3));

%% ── 3. Figure 1: 3D trajectory ──────────────────────────────────────────
figure('Name', '3D Trajectory', 'NumberTitle', 'off');
plot3(x_ref, y_ref, z_ref, 'b--', 'LineWidth', 2, 'DisplayName', 'Reference');
hold on;
plot3(x, y, z, 'r',  'LineWidth', 1.5, 'DisplayName', 'Actual');
plot3(x(1), y(1), z(1), 'ko', 'MarkerFaceColor', 'k', ...
    'MarkerSize', 6, 'DisplayName', 'Start');
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory Tracking');
legend('Location', 'best');
view(45, 30);

%% ── 4. Figure 2: Position time-history ──────────────────────────────────
figure('Name', 'Position', 'NumberTitle', 'off');
labels = {'X [m]', 'Y [m]', 'Z [m]'};
actual = {x, y, z};
ref    = {x_ref, y_ref, z_ref};

for i = 1:3
    subplot(3, 1, i);
    plot(t, ref{i},    'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, actual{i}, 'r',   'LineWidth', 1.2, 'DisplayName', 'Actual');
    grid on;
    ylabel(labels{i});
    if i == 1; legend('Location', 'best'); end
end
xlabel('Time [s]');
sgtitle('Position Tracking');

%% ── 5. Figure 3: Velocity time-history ──────────────────────────────────
figure('Name', 'Velocity', 'NumberTitle', 'off');
labels_v = {'dX [m/s]', 'dY [m/s]', 'dZ [m/s]'};
actual_v = {dx, dy, dz};
ref_v    = {dx_ref, dy_ref, dz_ref};

for i = 1:3
    subplot(3, 1, i);
    plot(t, ref_v{i},    'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, actual_v{i}, 'r',   'LineWidth', 1.2, 'DisplayName', 'Actual');
    grid on;
    ylabel(labels_v{i});
    if i == 1; legend('Location', 'best'); end
end
xlabel('Time [s]');
sgtitle('Velocity Tracking');

%% ── 6. Figure 4: Acceleration time-history ──────────────────────────────
figure('Name', 'Acceleration', 'NumberTitle', 'off');
labels_a = {'ddX [m/s²]', 'ddY [m/s²]', 'ddZ [m/s²]'};
actual_a = {ddx, ddy, ddz};
ref_a    = {ddx_ref, ddy_ref, ddz_ref};

for i = 1:3
    subplot(3, 1, i);
    plot(t, ref_a{i},    'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, actual_a{i}, 'r',   'LineWidth', 1.2, 'DisplayName', 'Actual');
    grid on;
    ylabel(labels_a{i});
    if i == 1; legend('Location', 'best'); end
end
xlabel('Time [s]');
sgtitle('Acceleration Tracking');

%% ── 7. Figure 5: Position tracking error ────────────────────────────────
figure('Name', 'Tracking Error', 'NumberTitle', 'off');

subplot(4, 1, 1);
plot(t, ex, 'r', 'LineWidth', 1.2); grid on;
ylabel('e_x [m]'); yline(0, 'k--');

subplot(4, 1, 2);
plot(t, ey, 'g', 'LineWidth', 1.2); grid on;
ylabel('e_y [m]'); yline(0, 'k--');

subplot(4, 1, 3);
plot(t, ez, 'b', 'LineWidth', 1.2); grid on;
ylabel('e_z [m]'); yline(0, 'k--');

subplot(4, 1, 4);
plot(t, e3, 'k', 'LineWidth', 1.2); grid on;
ylabel('||e|| [m]');
xlabel('Time [s]');

sgtitle('Position Tracking Error');

%% ── 8. Export figures ───────────────────────────────────────────────────
% Uncomment to save figures as PNG
% exportgraphics(figure(1), 'results/trajectory_3d.png',   'Resolution', 300);
% exportgraphics(figure(2), 'results/position.png',         'Resolution', 300);
% exportgraphics(figure(3), 'results/velocity.png',         'Resolution', 300);
% exportgraphics(figure(4), 'results/acceleration.png',     'Resolution', 300);
% exportgraphics(figure(5), 'results/tracking_error.png',   'Resolution', 300);