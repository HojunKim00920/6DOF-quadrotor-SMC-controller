% === Time ===
t = out.tout;

% % === Position ===
x     = out.x_SMC_diff_and_x_ref.Data(:,1);
x_ref = out.x_SMC_diff_and_x_ref.Data(:,2);
y     = out.y_SMC_diff_and_y_ref.Data(:,1);
y_ref = out.y_SMC_diff_and_y_ref.Data(:,2);
z     = out.z_SMC_diff_and_z_ref.Data(:,1);
z_ref = out.z_SMC_diff_and_z_ref.Data(:,2);

% % === Velocity ===
dx     = out.dx_SMC_diff_and_dx_ref.Data(:,1);
dx_ref = out.dx_SMC_diff_and_dx_ref.Data(:,2);
dy     = out.dy_SMC_diff_and_dy_ref.Data(:,1);
dy_ref = out.dy_SMC_diff_and_dy_ref.Data(:,2);
dz     = out.dz_SMC_diff_and_dz_ref.Data(:,1);
dz_ref = out.dz_SMC_diff_and_dz_ref.Data(:,2);

% % === Acceleration ===
ddx     = out.ddx_SMC_diff_and_ddx_ref.Data(:,1);
ddx_ref = out.ddx_SMC_diff_and_ddx_ref.Data(:,2);
ddy     = out.ddy_SMC_diff_and_ddy_ref.Data(:,1);
ddy_ref = out.ddy_SMC_diff_and_ddy_ref.Data(:,2);
ddz     = out.ddz_SMC_diff_and_ddz_ref.Data(:,1);
ddz_ref = out.ddz_SMC_diff_and_ddz_ref.Data(:,2);

% === Build table ===
T = table(t, ...
    x, x_ref, y, y_ref, z, z_ref, ...
    dx, dx_ref, dy, dy_ref, dz, dz_ref, ...
    ddx, ddx_ref, ddy, ddy_ref, ddz, ddz_ref, ...
    'VariableNames', {
        'Time', ...
        'X', 'X_ref', 'Y', 'Y_ref', 'Z', 'Z_ref', ...
        'dX', 'dX_ref', 'dY', 'dY_ref', 'dZ', 'dZ_ref', ...
        'ddX', 'ddX_ref', 'ddY', 'ddY_ref', 'ddZ', 'ddZ_ref'});

% === Save to Excel ===
writetable(T, 'simulation_result_data.xlsx');