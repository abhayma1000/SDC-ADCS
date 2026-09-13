function run_simulink_batch(paramsFile, outFile)

params = jsondecode(fileread(paramsFile));

mission.mdl = "SimulinkModel";
open_system(mission.mdl);

w_earth = [0; 0; 7.2921159 * 10^(-5)];

pod_kf_initial_P = [1e2; 1e2; 1e2; 1e2; 1e2; 1e2];
nav_mekf_initial_P = [1e-2; 1e-2; 1e-2];
R_mag = diag([1; 2; 3]);
R_sun = diag([1; 2; 3]);

mission.StartDate = datetime(2025,1,1,12,0,0);

if isfield(params, 'duration_hours')
    mission.Duration = hours(params.duration_hours);
else
    mission.Duration = hours(0.025);
end

clock_drift_rate = 0.5;

mission.Satellite.blk = mission.mdl + "/Dynamics/Spacecraft Dynamics";
mission.Satellite.SemiMajorAxis  = params.semi_major_axis_m;
mission.Satellite.Eccentricity   = params.eccentricity;
mission.Satellite.Inclination    = params.inclination_deg;
mission.Satellite.ArgOfPeriapsis = params.arg_periapsis_deg;
mission.Satellite.RAAN           = params.raan_deg;
mission.Satellite.TrueAnomaly    = params.true_anomaly_deg;

if isfield(params, 'initial_quaternion')
    q0 = reshape(params.initial_quaternion, 1, 4);
    mission.Satellite.q0 = q0 / norm(q0);
else
    random_quat = rand(1, 4);
    mission.Satellite.q0 = random_quat / norm(random_quat);
end

initial_q_IB = mission.Satellite.q0;

if isfield(params, 'tumble_rate_deg_s')
    mission.Satellite.pqr = reshape(params.tumble_rate_deg_s, 1, 3);
else
    mission.Satellite.pqr = [10, 5, 2.5];
end

mass = 0.25;
inertia_tensor = [0.2273, 0, 0; 0, 0.2273, 0; 0, 0, .0040];

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
    "earthSH",      "EGM2008", ...
    "shDegree",     "120", ...
    "useEOPs",      "on", ...
    "eopFile",      "aeroiersdata.mat");

set_param(mission.Satellite.blk, "useGravGrad", "on");

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

mission.SimOutput = sim(mission.mdl);

assignin('base', 'mission', mission);

tout = mission.SimOutput.tout;
yout = mission.SimOutput.yout;
orbit_params = params;

nav_mekf_time         = mission.SimOutput.navigation.nav_mekf.mekf.state.Time;
nav_mekf_state        = squeeze(mission.SimOutput.navigation.nav_mekf.mekf.state.Data)';
nav_mekf_P            = squeeze(mission.SimOutput.navigation.nav_mekf.mekf.P.Data);

nav_rawdog_triad_time  = mission.SimOutput.navigation.nav_mekf.rawdog_triad.state.Time;
nav_rawdog_triad_state = squeeze(mission.SimOutput.navigation.nav_mekf.rawdog_triad.state.Data)';
nav_rawdog_triad_P     = squeeze(mission.SimOutput.navigation.nav_mekf.rawdog_triad.P.Data);

nav_rawdog_prop_time   = mission.SimOutput.navigation.nav_mekf.rawdog_prop.state.Time;
nav_rawdog_prop_state  = squeeze(mission.SimOutput.navigation.nav_mekf.rawdog_prop.state.Data)';
nav_rawdog_prop_P      = squeeze(mission.SimOutput.navigation.nav_mekf.rawdog_prop.P.Data);

nav_od_kf_time         = mission.SimOutput.navigation.od_kf.state.Time;
nav_od_kf_state        = squeeze(mission.SimOutput.navigation.od_kf.state.Data)';
nav_od_kf_P            = squeeze(mission.SimOutput.navigation.od_kf.P.Data);

[outDir, ~, ~] = fileparts(outFile);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

save(outFile, 'tout', 'yout', 'orbit_params', ...
    'nav_mekf_time', 'nav_mekf_state', 'nav_mekf_P', ...
    'nav_rawdog_triad_time', 'nav_rawdog_triad_state', 'nav_rawdog_triad_P', ...
    'nav_rawdog_prop_time', 'nav_rawdog_prop_state', 'nav_rawdog_prop_P', ...
    'nav_od_kf_time', 'nav_od_kf_state', 'nav_od_kf_P', ...
    '-v7.3');

fprintf('Saved results for "%s" to %s\n', params.name, outFile);

close_system(mission.mdl, 0);
evalin('base', ['clear ' strjoin(base_vars, ' ') ';']);

end
