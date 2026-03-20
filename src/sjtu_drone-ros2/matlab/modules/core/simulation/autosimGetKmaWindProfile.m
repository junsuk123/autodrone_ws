function profile = autosimGetKmaWindProfile(cfg)
    persistent cachedPath cachedProfile

    profile = [];
    if ~isfield(cfg.wind, 'kma_csv')
        return;
    end

    csvPath = char(cfg.wind.kma_csv);
    if isempty(csvPath) || ~isfile(csvPath)
        return;
    end

    if ~isempty(cachedPath) && strcmp(cachedPath, csvPath) && ~isempty(cachedProfile)
        profile = cachedProfile;
        return;
    end

    try
        T = readtable(csvPath, 'PreserveVariableNames', true, 'TextType', 'string');
    catch
        return;
    end

    if isempty(T) || width(T) < 2
        return;
    end

    speedIdx = autosimFindColumnIndex(T.Properties.VariableNames, cfg.wind.kma_speed_column, {'wind_speed','speed','ws','windspd'});
    dirIdx = autosimFindColumnIndex(T.Properties.VariableNames, cfg.wind.kma_direction_column, {'wind_dir','direction','wd','winddir'});

    if speedIdx < 1
        speedIdx = autosimFindColumnIndexContains(T.Properties.VariableNames, {'(m/s)','m/s','speed','windspeed','wind_spd'});
    end
    if dirIdx < 1
        dirIdx = autosimFindColumnIndexContains(T.Properties.VariableNames, {'(deg)','deg','direction','winddir','wind_dir'});
    end

    if speedIdx < 1 || dirIdx < 1
        return;
    end

    timeIdx = autosimFindColumnIndex(T.Properties.VariableNames, cfg.wind.kma_time_column, {'time','timestamp','datetime','date'});
    if timeIdx < 1
        timeIdx = autosimFindColumnIndexContains(T.Properties.VariableNames, {'time','timestamp','date','일시'});
    end

    speed = autosimToNumeric(T{:, speedIdx});
    dir = autosimToNumeric(T{:, dirIdx});

    if timeIdx > 0
        tSec = autosimParseTimeColumnSec(T{:, timeIdx});
    else
        tSec = (0:numel(speed)-1).' * 60.0;
    end

    if numel(tSec) ~= numel(speed)
        tSec = (0:numel(speed)-1).' * 60.0;
    end

    mask = isfinite(speed) & isfinite(dir) & isfinite(tSec);
    tSec = tSec(mask);
    speed = speed(mask);
    dir = dir(mask);
    if isempty(speed) || numel(speed) < 2
        return;
    end

    [tSec, keepIdx] = unique(tSec, 'stable');
    speed = speed(keepIdx);
    dir = dir(keepIdx);
    if numel(speed) < 2
        return;
    end

    if isfield(cfg.wind, 'kma_speed_scale') && isfinite(cfg.wind.kma_speed_scale)
        speed = speed * cfg.wind.kma_speed_scale;
    end
    if isfield(cfg.wind, 'kma_direction_offset_deg') && isfinite(cfg.wind.kma_direction_offset_deg)
        dir = dir + cfg.wind.kma_direction_offset_deg;
    end

    speed = max(0.0, speed);
    dir = mod(dir + 180.0, 360.0) - 180.0;

    secGrid = (0:1:floor(tSec(end))).';
    if numel(secGrid) < 2
        secGrid = (0:1:60).';
    end

    speedSec = interp1(tSec, speed, secGrid, 'pchip', 'extrap');

    dirCos = cosd(dir);
    dirSin = sind(dir);
    dirCosSec = interp1(tSec, dirCos, secGrid, 'pchip', 'extrap');
    dirSinSec = interp1(tSec, dirSin, secGrid, 'pchip', 'extrap');
    dirSec = atan2d(dirSinSec, dirCosSec);

    noiseStdSpeed = 0.0;
    noiseStdDir = 0.0;
    if isfield(cfg.wind, 'kma_interp_noise_std_speed') && isfinite(cfg.wind.kma_interp_noise_std_speed)
        noiseStdSpeed = max(0.0, cfg.wind.kma_interp_noise_std_speed);
    end
    if isfield(cfg.wind, 'kma_interp_noise_std_dir_deg') && isfinite(cfg.wind.kma_interp_noise_std_dir_deg)
        noiseStdDir = max(0.0, cfg.wind.kma_interp_noise_std_dir_deg);
    end

    if noiseStdSpeed > 0
        speedSec = speedSec + noiseStdSpeed * randn(size(speedSec));
    end
    if noiseStdDir > 0
        dirSec = dirSec + noiseStdDir * randn(size(dirSec));
    end

    speedSec = max(0.0, speedSec);
    cap = autosimGetWindCommandCap(cfg);
    if isfinite(cap) && cap > 0
        speedSec = min(speedSec, cap);
    end
    dirSec = mod(dirSec + 180.0, 360.0) - 180.0;

    % Build risk-ranked pools so curriculum can sample calm vs gust-heavy windows.
    ds = [0.0; abs(diff(speedSec))];
    ddir = [0.0; abs(diff(dirSec))];
    ddir = min(ddir, 360.0 - ddir);

    speedNorm = autosimNormalizeVector(speedSec);
    dsNorm = autosimNormalizeVector(ds);
    ddirNorm = autosimNormalizeVector(ddir);
    riskScore = autosimClamp(0.60 * speedNorm + 0.28 * dsNorm + 0.12 * ddirNorm, 0.0, 1.0);

    qSafe = [0.05, 0.45];
    qUnsafe = [0.70, 0.98];
    minPool = 30;
    if isfield(cfg, 'curriculum')
        if isfield(cfg.curriculum, 'safe_pool_quantile') && numel(cfg.curriculum.safe_pool_quantile) >= 2
            qSafe = sort(double(cfg.curriculum.safe_pool_quantile(1:2)));
        end
        if isfield(cfg.curriculum, 'unsafe_pool_quantile') && numel(cfg.curriculum.unsafe_pool_quantile) >= 2
            qUnsafe = sort(double(cfg.curriculum.unsafe_pool_quantile(1:2)));
        end
        if isfield(cfg.curriculum, 'min_pool_size') && isfinite(cfg.curriculum.min_pool_size)
            minPool = max(5, round(cfg.curriculum.min_pool_size));
        end
    end

    qSafe = autosimClamp(qSafe, 0.0, 1.0);
    qUnsafe = autosimClamp(qUnsafe, 0.0, 1.0);
    safeLo = prctile(riskScore, 100 * qSafe(1));
    safeHi = prctile(riskScore, 100 * qSafe(2));
    unsafeLo = prctile(riskScore, 100 * qUnsafe(1));
    unsafeHi = prctile(riskScore, 100 * qUnsafe(2));

    safeIdx = find(riskScore >= safeLo & riskScore <= safeHi);
    unsafeIdx = find(riskScore >= unsafeLo & riskScore <= unsafeHi);
    if numel(safeIdx) < minPool
        [~, ord] = sort(riskScore, 'ascend');
        safeIdx = ord(1:min(numel(ord), minPool));
    end
    if numel(unsafeIdx) < minPool
        [~, ord] = sort(riskScore, 'descend');
        unsafeIdx = ord(1:min(numel(ord), minPool));
    end

    profile = struct( ...
        'speed', speed(:), ...
        'dir', dir(:), ...
        't_sec', tSec(:), ...
        'sec_grid', secGrid(:), ...
        'speed_sec', speedSec(:), ...
        'dir_sec', dirSec(:), ...
        'risk_score', riskScore(:), ...
        'safe_idx', safeIdx(:), ...
        'unsafe_idx', unsafeIdx(:));
    cachedPath = csvPath;
    cachedProfile = profile;
end


