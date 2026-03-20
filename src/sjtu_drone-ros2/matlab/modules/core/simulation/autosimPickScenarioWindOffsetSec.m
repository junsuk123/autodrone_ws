function offsetSec = autosimPickScenarioWindOffsetSec(cfg, scenarioId)
    offsetSec = 0;

    src = "random";
    if isfield(cfg.wind, 'source')
        src = string(cfg.wind.source);
    end
    if src ~= "kma_csv"
        return;
    end

    profile = autosimGetKmaWindProfile(cfg);
    if isempty(profile) || ~isfield(profile, 'speed_sec') || isempty(profile.speed_sec)
        return;
    end

    n = numel(profile.speed_sec);
    idx = autosimResolveKmaProfileIndex(cfg, scenarioId, n);
    offsetSec = max(0, idx - 1);
end


