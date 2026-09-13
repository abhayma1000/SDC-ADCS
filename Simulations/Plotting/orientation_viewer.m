

simDuration = 5;
fps = 60;
numFrames = simDuration * fps;

quat = mission.SimOutput.dynamics_output.q_b2icrf.Data(12:end, 1:4);
%jd_time = out.dynamics.time.Data(:, :);
%utc_time = datetime(jd_time,'convertfrom','juliandate');
%pos = out.dynamics.position.Data(:, :);
% TODO incorporate position somehow
N = size(quat, 1);





%display(R_BT)

idx = round(linspace(1, N, numFrames));


figure;
ax = axes;
grid(ax, 'on');
axis(ax, [-1 1 -1 1 -1 1]);
view(3);
title('NED Animation');


h = poseplot(quaternion(eye(3), 'rotmat', 'frame'), [0 0 0], 'Parent', ax);


v = VideoWriter('C:\Users\jajli\Videos\something2.avi');
v.FrameRate = fps;
open(v);


for k = 2:length(idx)
    utc_time_vec = datevec(utc_time(idx(k)));
    lla = eci2lla(pos(idx(k), :), utc_time_vec);
    R_TE = dcmecef2ned(lla(1), lla(2));
    R_EI = dcmeci2ecef('IAU-2000/2006', utc_time_vec);
    wgs84 = wgs84Ellipsoid('kilometer');
    %R_TE = ned2ecef(0, 0, 0, lla(1), lla(2), lla(3), wgs84);
    %R_EI = ecef2eci(utc_time, [0, 0, 0], [0, 0, 0]);
    R_BT = R_TE' * R_EI' * quat2dcm(quat(idx(k), :));
    q_BT = dcm2quat(R_BT);
    
    q_orientation = quaternion(q_BT);
    %quat = ned2ecef(0, 0, 0, ) * ecef2eci()

    %quat = 
    set(h, 'Orientation', q_orientation);
    drawnow limitrate;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);