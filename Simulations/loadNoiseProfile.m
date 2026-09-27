function noise = loadNoiseProfile(noiseProfileFile)

if nargin < 1 || isempty(noiseProfileFile)
    noiseProfileFile = fullfile(fileparts(mfilename('fullpath')), 'noise_profile.json');
end

if ~isfile(noiseProfileFile)
    warning('loadNoiseProfile:missingFile', ...
        'noise_profile.json not found at %s -- running with all sensor noise disabled.', ...
        noiseProfileFile);
    noise = struct('sensors', struct());
    return
end

noise = jsondecode(fileread(noiseProfileFile));

if ~isfield(noise, 'sensors')
    warning('loadNoiseProfile:missingSensors', ...
        '%s has no "sensors" section -- running with all sensor noise disabled.', ...
        noiseProfileFile);
    noise.sensors = struct();
end

end
