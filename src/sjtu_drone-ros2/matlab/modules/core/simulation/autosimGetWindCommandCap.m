function cap = autosimGetWindCommandCap(cfg)
    if autosimIsModuleEnabled(cfg, 'wind_engine')
        try
            cap = autosim_wind_sim('get_cap', cfg);
            if isfinite(cap)
                return;
            end
        catch
        end
    end

    cap = inf;
    if isfield(cfg, 'wind') && isfield(cfg.wind, 'speed_max') && isfinite(cfg.wind.speed_max) && cfg.wind.speed_max > 0
        cap = min(cap, cfg.wind.speed_max);
    end
    if isfield(cfg, 'wind') && isfield(cfg.wind, 'hover_limit_mps') && isfinite(cfg.wind.hover_limit_mps) && cfg.wind.hover_limit_mps > 0
        cap = min(cap, cfg.wind.hover_limit_mps);
    end
    if ~isfinite(cap)
        cap = nan;
    end
end


