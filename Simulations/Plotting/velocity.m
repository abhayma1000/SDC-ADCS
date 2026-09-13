difference = mission.SimOutput.dynamics_output.velocity_icrf.Data(12:end, :) - squeeze(mission.SimOutput.navigation.od_kf.state.Data(4:6, :, 12:end))';
plot(mission.SimOutput.tout(12:end, :), difference(:, 1), 'r', 'DisplayName', 'Error'); hold on;
plot(mission.SimOutput.tout(12:end, :), difference(:, 2), 'g', 'DisplayName', 'Error');
plot(mission.SimOutput.tout(12:end, :), difference(:, 3), 'b', 'DisplayName', 'Error'); hold off

legend('X', 'Y', 'Z')
