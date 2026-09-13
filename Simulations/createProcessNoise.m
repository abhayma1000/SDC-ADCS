function Q = createProcessNoise(dt)

% TODO need correct values for these variance values
gyro_var = 0.001 * dt;
gyro_bias_var = 9.1e-1;
gyro_var_diag = diag([gyro_var; gyro_var; gyro_var]);
gyro_bias_var_diag = diag([gyro_bias_var; gyro_bias_var; gyro_bias_var]);

Q = zeros(3, 3);
Q(1:3, 1:3) = (gyro_var_diag * dt) + (gyro_bias_var_diag * (dt^3 / 3.0));


%{

mag_bias_var_diag = diag([1^2; 1^2; 1^2]);



Q_d = zeros(13, 13);



Q_d(1:3, 1:3) = (gyro_var_diag * dt) + (gyro_bias_var_diag * (dt^3 / 3.0));
Q_d(1:3, 10:12) = -1.0 * gyro_bias_var_diag * (dt^2 / 2.0);


Q_d(10:12, 1:3) = -1.0 * gyro_bias_var_diag * (dt^2 / 2.0);
Q_d(10:12, 10:12) = gyro_bias_var_diag * (dt^2 / 2.0);


Q_d(16:18, 16:18) = mag_bias_var_diag * dt;

Q_d(19, 19) = baro_bias_var * dt;

%}

