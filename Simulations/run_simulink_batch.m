function run_simulink_batch(paramsFile, outFile)

params = jsondecode(fileread(paramsFile));


mission.mdl = "SimulinkModel";
open_system(mission.mdl);


w_earth = [0; 0; 7.2921159 * 10^(-5)]; 

pod_kf_initial_P = [1e2; 1e2; 1e2; 1e2; 1e2; 1e2];
nav_mekf_initial_P = [1e-2; 1e-2; 1e-2]; % Note just the quat for now
R_mag = diag([1; 2; 3]); % TODO change later to actual values
R_sun = diag([1; 2; 3]); % TODO change later to actual values

mission.StartDate = datetime(2025,1,1,12,0,0);

if isfield(params, 'duration_hours')
    mission.Duration = hours(params.duration_hours);
else
    mission.Duration = hours(0.025);
end

clock_drift_rate = 0.5; % [seconds/day] Rough estimate

mission.Satellite.blk = mission.mdl + "/Dynamics/Spacecraft Dynamics";
mission.Satellite.SemiMajorAxis  = params.semi_major_axis_m;  % meters
mission.Satellite.Eccentricity   = params.eccentricity;       % unitless
mission.Satellite.Inclination    = params.inclination_deg;    % degrees
mission.Satellite.ArgOfPeriapsis = params.arg_periapsis_deg;  % degrees
mission.Satellite.RAAN           = params.raan_deg;           % degrees
mission.Satellite.TrueAnomaly    = params.true_anomaly_deg;   % degrees

if isfield(params, 'initial_quaternion')
    q0 = reshape(params.initial_quaternion, 1, 4);
    mission.Satellite.q0 = q0 / norm(q0);
else
    random_quat = rand(1, 4);
    mission.Satellite.q0 = random_quat / norm(random_quat);
end

initial_q_IB = mission.Satellite.q0;

if isfield(params, 'tumble_rate_deg_s')
    mission.Satellite.pqr = reshape(params.tumble_rate_deg_s, 1, 3); % deg/s
else
    mission.Satellite.pqr = [10, 5, 2.5]; % deg/s
end

mass = 0.25; % [kg]
inertia_tensor = [0.2273, 0, 0; 0, 0.2273, 0; 0, 0, .0040]; % Change to actual inertia tensor

set_param(mission.Satellite.blk, ...
    "startDate",      string(juliandate(mission.StartDate)), ...
    "stateFormatNum", "Orbital elements", ...
    "orbitType",      "Keplerian", ...
    "semiMajorAxis",  string(mission.Satellite.SemiMajorAxis), ...
    "eccentricity",   string(mission.Satellite.Eccentricity), ...
    "inclination",    string(mission.Satellite.Inclination), ...
    "raan",           string(mission.Satellite.RAAN), ...
    "argPeriapsis",   string(mission.Satellite.ArgOfPeriapsis), ...
    "trueAnomaly",    string(mission.Satellite.TrueAnomaly));
set_param(mission.Satellite.blk, ...
    "attitudeFormat", "Quaternion", ...
    "attitudeFrame",  "ICRF", ...
    "attitude",       mat2str(mission.Satellite.q0), ...
    "attitudeRate",   mat2str(mission.Satellite.pqr));

set_param(mission.Satellite.blk, ...
    "gravityModel", "Spherical Harmonics", ...
    "earthSH",      "EGM2008", ... % Earth spherical harmonic potential model
    "shDegree",     "120", ... % Spherical harmonic model degree and order
    "useEOPs",      "on", ... % Use EOP's in ECI to ECEF transformations
    "eopFile",      "aeroiersdata.mat"); % EOP data file

set_param(mission.Satellite.blk, "useGravGrad", "on");

%% Solver settings
set_param(mission.mdl, ...
    "SolverType", "Variable-step", ...
    "SolverName", "VariableStepAuto", ...
    "RelTol",     "0.5e-5", ...
    "AbsTol",     "1e-5", ...
    "MaxStep",    "0.1", ...
    "MinStep",    "0.01", ...
    "StopTime",   string(seconds(mission.Duration)));

set_param(mission.mdl, ...
    "SaveOutput", "on", ...
    "OutputSaveName", "yout", ...
    "SaveFormat", "Dataset", ...
    "DatasetSignalFormat", "timetable");

base_vars = {'mission', 'w_earth', 'pod_kf_initial_P', 'nav_mekf_initial_P', ...
    'R_mag', 'R_sun', 'clock_drift_rate', 'mass', 'inertia_tensor', 'initial_q_IB'};
for k = 1:numel(base_vars)
    assignin('base', base_vars{k}, eval(base_vars{k}));
end

%% Run model
mission.SimOutput = sim(mission.mdl);

% Keep the base-workspace copy of mission in sync (sim's output is only
% assigned to the local copy above)
assignin('base', 'mission', mission);

%% Save results
tout = mission.SimOutput.tout;
yout = mission.SimOutput.yout;
orbit_params = params;

[outDir, ~, ~] = fileparts(outFile);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

save(outFile, 'tout', 'yout', 'orbit_params', '-v7.3');

fprintf('Saved results for "%s" to %s\n', params.name, outFile);

close_system(mission.mdl, 0);
evalin('base', ['clear ' strjoin(base_vars, ' ') ';']);

end
