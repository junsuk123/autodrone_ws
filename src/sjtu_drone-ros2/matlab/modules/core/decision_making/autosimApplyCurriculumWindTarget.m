function scenarioCfg = autosimApplyCurriculumWindTarget(cfg, scenarioCfg, targetRisk, scenarioId)
    profile = autosimGetKmaWindProfile(cfg);
    if isempty(profile) || ~isfield(profile, 'speed_sec') || isempty(profile.speed_sec)
        return;
    end

    idxPool = [];
    switch string(targetRisk)
        case "unsafe"
            if isfield(profile, 'unsafe_idx')
                idxPool = double(profile.unsafe_idx(:));
            end
        otherwise
            if isfield(profile, 'safe_idx')
                idxPool = double(profile.safe_idx(:));
            end
    end

    idxPool = idxPool(isfinite(idxPool) & idxPool >= 1 & idxPool <= numel(profile.speed_sec));
    if isempty(idxPool)
        idxPool = (1:numel(profile.speed_sec))';
    end

    pickMode = "sequential";
    if isfield(cfg.wind, 'kma_pick_mode')
        pickMode = string(cfg.wind.kma_pick_mode);
    end

    if pickMode == "random"
        seedBase = autosimClampNaN(cfg.wind.random_seed_base, 0);
        seed = mod(seedBase + 104729 * max(1, scenarioId) + 7919 * numel(idxPool), 2147483646) + 1;
        stream = RandStream('mt19937ar', 'Seed', seed);
        pick = idxPool(randi(stream, numel(idxPool)));
    else
        k = mod(max(1, scenarioId) - 1, numel(idxPool)) + 1;
        pick = idxPool(k);
    end

    scenarioCfg.wind_speed = profile.speed_sec(pick);
    scenarioCfg.wind_dir = profile.dir_sec(pick);
    scenarioCfg.wind_profile_offset_sec = max(0, round(pick) - 1);
end


