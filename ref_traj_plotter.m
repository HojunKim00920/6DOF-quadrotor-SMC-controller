% Load data from Excel file
T = readtable('simulation_result_data.xlsx');

% Extract reference trajectory
x_ref = T.X_ref;
y_ref = T.Y_ref;
z_ref = T.Z_ref;

% Visualize 3D trajectory
figure;
plot3(x_ref, y_ref, z_ref, 'b', 'LineWidth', 1);
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Reference Trajectory');
axis equal;