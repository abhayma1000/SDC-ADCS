qT = mission.SimOutput.dynamics_output.q_b2icrf.Data(12:end, :);
%q_Est = squeeze(mission.SimOutput.navigation.nav_mekf.mekf.state.Data(1:4, :, 12:end))';
%q_Est = squeeze(mission.SimOutput.navigation.nav_mekf.rawdog_triad.state.Data(1:4, :, 12:end))';
q_Est = squeeze(mission.SimOutput.navigation.nav_mekf.rawdog_prop.state.Data(1:4, :, 12:end))';

%% Calculate stuff

sm_err = zeros(size(qT, 1), 3);
eul_err = zeros(size(qT, 1), 3);
for i = 1:size(qT, 1)
    qT_i = qT(i, :);
    q_Est_i = q_Est(i, :);
    this_q_err = quatmultiply(quatinv(q_Est_i), qT_i);
    %q_err(i, :) = this_q_err;
    sm_err(i, :) = this_q_err(2:4);

    eul_err(i, :) = wrapTo180(rad2deg(quat2eul(qT_i, "XYZ")) - rad2deg(quat2eul(q_Est_i, "XYZ")));
end


%% Euler angles

{
%plot(mission.SimOutput.tout(12:end, :), eul_err(:, 1), 'r', 'DisplayName', 'Error'); hold on;
%plot(mission.SimOutput.tout(12:end, :), eul_err(:, 2), 'g', 'DisplayName', 'Error');
%plot(mission.SimOutput.tout(12:end, :), eul_err(:, 3), 'b', 'DisplayName', 'Error');
}

%% Quat error (I think)


plot(mission.SimOutput.tout(12:end, :), sm_err(:, 1), 'r', 'DisplayName', 'Error'); hold on;
plot(mission.SimOutput.tout(12:end, :), sm_err(:, 2), 'g', 'DisplayName', 'Error');
plot(mission.SimOutput.tout(12:end, :), sm_err(:, 3), 'b', 'DisplayName', 'Error'); hold off;



%plot(mission.SimOutput.tout(12:end, :), -1.0 * squeeze(mission.SimOutput.navigation.od_kf.P.Data(1, 1, 12:end)));
%plot(mission.SimOutput.tout(12:end, :), 1.0 * squeeze(mission.SimOutput.navigation.od_kf.P.Data(1, 1, 12:end))); hold off

legend('X', 'Y', 'Z', 'X+ bounds', 'X- bounds')

ylim([-1 1]);