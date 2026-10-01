% Note: Basically copied from https://www.mathworks.com/help/aeroblks/analyzing-spacecraft-attitude-profiles-with-satellite-scenario.html
% But change for our specific scenario


%% Open file

mission.mdl = "SimulinkModel";
open_system(mission.mdl);

%% Earth parameters

w_earth = [0; 0; 7.2921159 * 10^(-5)]; % Note: Used for GPS velocity simulation, not high accuracy incl. nutation, etc.

%% Initial EKF parameters

pod_kf_initial_P = [1e2; 1e2; 1e2; 1e2; 1e2; 1e2];

nav_mekf_initial_P = [1e-2; 1e-2; 1e-2]; % Note just the quat for now

R_mag = diag([1; 2; 3]); % TODO change later to actual values

R_sun = diag([1; 2; 3]); % TODO change later to actual values

%% Initial mission parameters

mission.StartDate = datetime(2025,1,1,12,0,0);
datetime(2025,1,1,12,0,0)
%mission.Duration = days(1.5);
mission.Duration = hours(0.025);

%% Clock offset

clock_drift_rate = 0.5; % [seconds/day] Rough estimate

%% Sat properties

mission.Satellite.blk = mission.mdl + "/Dynamics/Spacecraft Dynamics";
mission.Satellite.SemiMajorAxis  = 6865000; % meters
mission.Satellite.Eccentricity   = 0.0002105; % unitless
mission.Satellite.Inclination    = 97.4; % degrees
mission.Satellite.ArgOfPeriapsis = 0; % degrees
mission.Satellite.RAAN           = 0; % degrees
mission.Satellite.TrueAnomaly    = 0; % degrees

random_quat = rand(1, 4);
initial_q_IB = random_quat / norm(random_quat);
mission.Satellite.q0 = initial_q_IB;
mission.Satellite.pqr = [10, 5, 2.5]; % deg/s Change to desired tumble rate

%% Initialize sat dynamics properties

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
    "MinStep", "0.01", ...
    "StopTime",   string(seconds(mission.Duration)));
%"MinStep",    "auto", ... % Theoretically, but takes soooo long

set_param(mission.mdl, ...
    "SaveOutput", "on", ...
    "OutputSaveName", "yout", ...
    "SaveFormat", "Dataset", ...
    "DatasetSignalFormat", "timetable");

%% Run model

mission.SimOutput = sim(mission.mdl);

scenario = satelliteScenario(mission.StartDate, ...
    mission.StartDate + mission.Duration, 60);

%% Create ground station

gsWPI = groundStation(scenario, 42.2746, -71.8068, Name="WPI");

%% Add sat to scenario

mission.Satellite.Ephemeris = retime(mission.SimOutput.yout{1}.Values, ...
    seconds(uniquetol(mission.SimOutput.tout, .0001)));
%mission.Satellite.Ephemeris = retime(mission.SimOutput.navigation{1}.Values, ...
    %seconds(uniquetol(mission.SimOutput.tout, .0001)));
% Replace with this one day when navigation works
sat = satellite(scenario, mission.Satellite.Ephemeris, ...
    "CoordinateFrame", "inertial", "Name", "GOAT-1");

%% Add Atnenna

snsr = conicalSensor(sat, MaxViewAngle=60, MountingLocation=[0 0 10], Name="Antenna");
fieldOfView(snsr);

acWPI = access(snsr, gsWPI);

mission.Satellite.AttitudeProfile = retime(mission.SimOutput.yout{3}.Values, ...
    seconds(uniquetol(mission.SimOutput.tout, .0001)));
%mission.Satellite.attitudeProfile = retime(mission.SimOutput.yout{3}.Values, ...
    %seconds(uniquetol(mission.SimOutput.tout, .0001)));
% Replace with this one day when navigation works
pointAt(sat, mission.Satellite.AttitudeProfile, ...
    "CoordinateFrame", "inertial", "Format", "quaternion", "ExtrapolationMethod", "nadir");

%% Add Camera

cam = conicalSensor(sat, MaxViewAngle=60, MountingLocation=[0 10 10], MountingAngles = [0; 30; 60], Name="Camera");
fieldOfView(cam);


%% Add Sun Sensor

sun_snsr = conicalSensor(sat, MaxViewAngle=60, MountingLocation = [0 5 0], MountingAngles = [30; 15; 90], Name="Sun Sensor");
% fieldOfView(sun_snsr);

%% View sat 3D model

%viewer1 = satelliteScenarioViewer(scenario);

%sat.Visual3DModel = "SmallSat.glb";
%coordinateAxes(sat, Scale=2);
%camtarget(viewer1, sat);