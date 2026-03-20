function [windSpeed, windDir] = autosimPickScenarioWind(cfg, scenarioId)
    windSpeed = autosimRandRange(cfg.wind.speed_min, cfg.wind.speed_max);
    windDir = autosimRandRange(cfg.wind.direction_min, cfg.wind.direction_max);

    src = "random";
    if isfield(cfg.wind, 'source')
        src = string(cfg.wind.source);
    end

    if src ~= "kma_csv"
        return;
    end

    profile = autosimGetKmaWindProfile(cfg);
    if isempty(profile)
        return;
    end

    profileN = numel(profile.speed_sec);
    if profileN < 1
        return;
    end

    idx = autosimResolveKmaProfileIndex(cfg, scenarioId, profileN);
    windSpeed = profile.speed_sec(idx);
    windDir = profile.dir_sec(idx);
    cap = autosimGetWindCommandCap(cfg);
    if isfinite(cap) && cap > 0
        windSpeed = min(windSpeed, cap);
    end
end


