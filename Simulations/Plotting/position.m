difference = mission.SimOutput.dynamics_output.position_icrf.Data(12:end, :) - squeeze(mission.SimOutput.navigation.od_kf.state.Data(1:3, :, 12:end))';
plot(mission.SimOutput.tout(12:end, :), difference(:, 1), 'r', 'DisplayName', 'Error'); hold on;
plot(mission.SimOutput.tout(12:end, :), difference(:, 2), 'g', 'DisplayName', 'Error');
plot(mission.SimOutput.tout(12:end, :), difference(:, 3), 'b', 'DisplayName', 'Error');

plot(mission.SimOutput.tout(12:end, :), -1.0 * squeeze(mission.SimOutput.navigation.od_kf.P.Data(1, 1, 12:end)));
plot(mission.SimOutput.tout(12:end, :), 1.0 * squeeze(mission.SimOutput.navigation.od_kf.P.Data(1, 1, 12:end))); hold off

xlabel('Time (s)');
ylabel('Position Error (ICRF)');
title('Position Error Analysis');

legend('X', 'Y', 'Z', 'X+ bounds', 'X- bounds')
