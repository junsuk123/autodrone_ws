function idx = autosimResolveKmaProfileIndex(cfg, scenarioId, n)
    idx = 1;
    if nargin < 3 || n < 1
        return;
    end

    pickMode = "sequential";
    if isfield(cfg, 'wind') && isfield(cfg.wind, 'kma_pick_mode')
        pickMode = string(cfg.wind.kma_pick_mode);
    end

    if pickMode == "random"
        seedBase = 0;
        if isfield(cfg, 'wind') && isfield(cfg.wind, 'random_seed_base') && isfinite(cfg.wind.random_seed_base)
            seedBase = double(cfg.wind.random_seed_base);
        end
        seed = mod(seedBase + 104729 * max(1, scenarioId), 2147483646) + 1;
        stream = RandStream('mt19937ar', 'Seed', seed);
        idx = randi(stream, n);
        return;
    end

    step = max(1, floor(n / max(1, cfg.scenario.count)));
    idx = mod((max(1, scenarioId) - 1) * step, n) + 1;
end


