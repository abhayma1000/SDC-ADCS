function applySensorNoise(mission, noise)

sensorsPath = mission.mdl + "/Embedded Computer/Sensors";
busCreatorPort = sprintf("Bus\nCreator");

apply_spliced_block(sensorsPath, "Create Mag", "corrupt_mag_noise", busCreatorPort, 1, get_field(noise, "magnetometer"), "mag", 101);
apply_spliced_block(sensorsPath, "Create Sun Sensor", "corrupt_sun", busCreatorPort, 5, get_field(noise, "sun_sensor"), "sun", 202);
apply_gps_noise(sensorsPath, busCreatorPort, get_field(noise, "gps"), 303);
apply_spliced_block(sensorsPath, "Create Clock", "corrupt_clock", busCreatorPort, 4, get_field(noise, "clock"), "clock", 404);

if isfield(noise.sensors, "star_tracker")
    [b, sg] = extract_bias_sigma(noise.sensors.star_tracker);
    if b ~= 0 || sg ~= 0
        warning('applySensorNoise:starTrackerUnused', ...
            'star_tracker noise values are set, but the star field is not consumed anywhere downstream in this model yet, so it has no effect. Skipping.');
    end
end

end

function s = get_field(noise, name)
if isfield(noise.sensors, name)
    s = noise.sensors.(name);
else
    s = struct();
end
end

function [bias, sigma] = extract_bias_sigma(s)
bias = 0;
sigma = 0;
if isfield(s, "bias") && isfield(s.bias, "value") && noiseFieldActive(s.bias.value)
    bias = s.bias.value;
end
if isfield(s, "white_noise") && isfield(s.white_noise, "std_dev") && noiseFieldActive(s.white_noise.std_dev)
    sigma = s.white_noise.std_dev;
end
end

function apply_spliced_block(sensorsPath, sourceBlockName, prefix, busCreatorPort, busInputIndex, s, busSignalName, seedBase)
try
    [bias, sigma] = extract_bias_sigma(s);
    if bias == 0 && sigma == 0
        return
    end

    sysStr = char(sensorsPath);
    srcPort = char(sourceBlockName) + "/1";
    dstPort = char(busCreatorPort) + "/" + string(busInputIndex);

    noisePath = sensorsPath + "/" + prefix + "_noise";
    biasPath = sensorsPath + "/" + prefix + "_bias";
    sumPath = sensorsPath + "/" + prefix + "_sum";

    add_block('simulink/Sources/Band-Limited White Noise', char(noisePath));
    set_param(char(noisePath), 'Cov', mat2str(sigma^2));
    set_param(char(noisePath), 'seed', mat2str(seedBase));
    set_param(char(noisePath), 'Ts', '0.1');

    add_block('simulink/Sources/Constant', char(biasPath));
    set_param(char(biasPath), 'Value', mat2str(bias));

    add_block('simulink/Math Operations/Sum', char(sumPath));
    set_param(char(sumPath), 'Inputs', '+++');

    add_line(sysStr, char(srcPort), char(prefix) + "_sum/1");
    add_line(sysStr, char(prefix) + "_noise/1", char(prefix) + "_sum/2");
    add_line(sysStr, char(prefix) + "_bias/1", char(prefix) + "_sum/3");

    delete_line(sysStr, char(srcPort), char(dstPort));
    outLine = add_line(sysStr, char(prefix) + "_sum/1", char(dstPort));
    set_param(outLine, 'Name', char(busSignalName));
catch ME
    warning('applySensorNoise:spliceFailed', ...
        'Could not splice a noise block after %s -- that sensor is running noise-free this run. %s', ...
        sourceBlockName, ME.message);
end
end

function apply_gps_noise(sensorsPath, busCreatorPort, s, seedBase)
try
    [bias, sigma] = extract_bias_sigma(s);
    if bias == 0 && sigma == 0
        return
    end

    sysStr = char(sensorsPath);
    dstPort = char(busCreatorPort) + "/3";

    selPath = sensorsPath + "/gps_noise_select";
    add_block('simulink/Signal Routing/Bus Selector', char(selPath));
    set_param(char(selPath), 'OutputSignals', 'pos,vel');
    add_line(sysStr, 'Create GPS/1', 'gps_noise_select/1');

    noisePath = sensorsPath + "/gps_pos_noise";
    biasPath = sensorsPath + "/gps_pos_bias";
    sumPath = sensorsPath + "/gps_pos_sum";

    add_block('simulink/Sources/Band-Limited White Noise', char(noisePath));
    set_param(char(noisePath), 'Cov', mat2str(sigma^2));
    set_param(char(noisePath), 'seed', mat2str(seedBase));
    set_param(char(noisePath), 'Ts', '0.1');

    add_block('simulink/Sources/Constant', char(biasPath));
    set_param(char(biasPath), 'Value', mat2str(bias));

    add_block('simulink/Math Operations/Sum', char(sumPath));
    set_param(char(sumPath), 'Inputs', '+++');

    add_line(sysStr, 'gps_noise_select/1', 'gps_pos_sum/1');
    add_line(sysStr, 'gps_pos_noise/1', 'gps_pos_sum/2');
    add_line(sysStr, 'gps_pos_bias/1', 'gps_pos_sum/3');

    rebusPath = sensorsPath + "/gps_noise_rebus";
    add_block('simulink/Signal Routing/Bus Creator', char(rebusPath));
    set_param(char(rebusPath), 'Inputs', '2');
    posLine = add_line(sysStr, 'gps_pos_sum/1', 'gps_noise_rebus/1');
    set_param(posLine, 'Name', 'pos');
    add_line(sysStr, 'gps_noise_select/2', 'gps_noise_rebus/2');

    delete_line(sysStr, 'Create GPS/1', dstPort);
    outLine = add_line(sysStr, 'gps_noise_rebus/1', dstPort);
    set_param(outLine, 'Name', 'gps');
catch ME
    warning('applySensorNoise:gpsSpliceFailed', ...
        'Could not splice GPS position noise -- GPS is running noise-free this run. %s', ME.message);
end
end
