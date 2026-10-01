%% For if we are facing earth/sun subject to some confidence level
% Returns some boolean (yes or no)

sigma = 1.0; % Confidence level. 1.0->66%, 2.0->95%, 3.0->99.7%

orientation_IB = [1, 0, 0, 0];

% TODO define some body axes at some point
camera_vec_body = [1, 0, 0]'; % Define camera direction in body frame (unit)

camera_vec_inertial = quat2rotm(orientation_IB) * camera_vec_body;

%% Earth specific stuff

pos = [123131, 223912, 943299]'; % TODO replace in sim. Random for now
pos_normed = -1.0 * pos / norm(pos); % Multiply by -1 to flip vec direction

theta = acos(dot(camera_vec_inertial, pos_normed));

% Axis not really important, but good to have
axis = cross(camera_vec_inertial, pos_normed);
axis = norm(axis);

P = [0.001, 0.002, 0.003]'; % TODO replace in sim. P defined in inertial frame, so need to rot
P_body = quat2rotm(orientation_IB)' * P;
P_added_theta = hypot(abs(P_body(2, 1)), abs(P_body(3, 1))); % Euclid dist. Small angle approx

rad_earth = 6378100; % radius of earth (m) according to Chat
fov_earth = deg2rad(atan(rad_earth / pos));
fov_camera = deg2rad(60); % TODO: Ask what is the fov of the camera, validate correct ideas

% I think this math works out. Angles are weird tho so idk
if (abs(theta) + (deg2rad(fov_earth) / 2.0) + (sigma * P_added_theta) < (fov_camera / 2.0))
    disp("Open camera");
else
    disp("Keep camera closed");
end


%% Sun specific stuff
pos_sun = [123131, 223912, 943299]';
pos_to_sun = pos_sun - pos_; % TODO replace in sim. Random for now
pos_to_sun_normed = -1.0 * pos_to_sun / norm(pos_to_sun); % Multiply by -1 to flip vec direction

theta_sun = acos(dot(camera_vec_inertial, pos_to_sun_normed));

% Axis not really important, but good to have
axis_sun = cross(camera_vec_inertial, pos_to_sun_normed);
axis_sun = norm(axis_sun);

P = [0.001, 0.002, 0.003]'; % TODO replace in sim. P defined in inertial frame, so need to rot
P_body = quat2rotm(orientation_IB)' * P;

rad_sun = 695700000; % radius of sun (m) according to Chat
fov_sun = deg2rad(atan(rad_sun / pos));
fov_camera = deg2rad(60); % TODO: Ask what is the fov of the camera, validate correct ideas

% I think this math works out. Angles are weird tho so idk
if (abs(theta_sun) - (deg2rad(fov_sun) / 2.0) - (sigma * P_added_theta) > (fov_camera / 2.0))
    disp("Close camera");
else
    disp("Keep camera open");
end



%{
% Old logic, probably wrong
if (theta < deg2rad(fov_earth / 2.0) - deg2rad(fov_camera / 2.0) - (sigma * P_added_theta))
    disp("Open camera");
else
    disp("Keep camera closed");
end
%}





