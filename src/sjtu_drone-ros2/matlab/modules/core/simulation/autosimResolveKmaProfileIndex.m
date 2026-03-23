function idx = autosimResolveKmaProfileIndex(cfg, scenarioId, n)
    idx = 1;
    if nargin < 3 || n < 1
        return;
    end

    pickMode = "sequential";
    if isfield(cfg, 'wind') && isfield(cfg.wind, 'kma_pick_mode')
        pickMode = string(cfg.wind.kma_pick_mode);
    end

    if pickMode == "random_start_cycle" || pickMode == "index_cycle_random_start"
        startIdx = autosimResolveCycleStartIndex(cfg, n);
        idx = mod((startIdx - 1) + (max(1, scenarioId) - 1), n) + 1;
        return;
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

function startIdx = autosimResolveCycleStartIndex(cfg, n)
    persistent cacheN cacheSeed cacheStartIdx;

    seedBase = 0;
    if isfield(cfg, 'wind') && isfield(cfg.wind, 'random_seed_base') && isfinite(cfg.wind.random_seed_base)
        seedBase = double(cfg.wind.random_seed_base);
    end

    runSig = 0;
    if isfield(cfg, 'paths') && isfield(cfg.paths, 'run_id')
        rid = char(string(cfg.paths.run_id));
        if ~isempty(rid)
            runSig = sum(double(rid));
        end
    end

    seed = mod(seedBase + 7919 * runSig + 104729 * n, 2147483646) + 1;
    if isempty(cacheStartIdx) || isempty(cacheN) || isempty(cacheSeed) || cacheN ~= n || cacheSeed ~= seed
        stream = RandStream('mt19937ar', 'Seed', seed);
        cacheStartIdx = randi(stream, n);
        cacheN = n;
        cacheSeed = seed;
    end

    startIdx = cacheStartIdx;
end


