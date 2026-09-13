function phi = createProcessModel(prioriState, gyro_meas, dt)
    F = zeros(3, 3);
    F(1:3, 1:3) = -1.0 * skewSymmetric(gyro_meas);

    phi = eye(3) + (F * dt) + (0.5 * F * F * dt^2);
    
end
