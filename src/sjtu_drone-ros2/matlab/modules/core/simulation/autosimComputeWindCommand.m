function [speedCmd, dirCmd] = autosimComputeWindCommand(cfg, scenarioCfg, tNow, windArmed)
    if autosimIsModuleEnabled(cfg, 'wind_engine')
        try
            [speedCmd, dirCmd] = autosim_wind_sim('compute_command', cfg, scenarioCfg, tNow, windArmed);
            if isfinite(speedCmd) && isfinite(dirCmd)
                return;
            end
        catch
        end
    end

    if ~windArmed || ~cfg.wind.enable
        speedCmd = 0.0;
        dirCmd = 0.0;
        return;
    end

    baseSpeed = max(0.0, scenarioCfg.wind_speed);
    baseDir = scenarioCfg.wind_dir;

    useProfileDirect = false;
    if isfield(cfg.wind, 'source') && cfg.wind.source == "kma_csv"
        profile = autosimGetKmaWindProfile(cfg);
        if ~isempty(profile) && isfield(profile, 'speed_sec') && ~isempty(profile.speed_sec)
            offsetSec = 0;
            if isfield(scenarioCfg, 'wind_profile_offset_sec') && isfinite(scenarioCfg.wind_profile_offset_sec)
                offsetSec = round(scenarioCfg.wind_profile_offset_sec);
            end
            nSec = numel(profile.speed_sec);
            idx = mod(max(0, round(tNow)) + offsetSec, nSec) + 1;
            baseSpeed = profile.speed_sec(idx);
            baseDir = profile.dir_sec(idx);
            if isfield(cfg.wind, 'kma_use_profile_direct') && cfg.wind.kma_use_profile_direct
                useProfileDirect = true;
            end
        end
    end

    ramp = 1.0;
    if isfield(cfg.wind, 'model_ramp_sec') && isfinite(cfg.wind.model_ramp_sec) && cfg.wind.model_ramp_sec > 0
        ramp = autosimClamp(tNow / cfg.wind.model_ramp_sec, 0.0, 1.0);
    end

    if useProfileDirect
        speedCmd = max(0.0, ramp * baseSpeed);
    else
        gustScale = 1.0;
        if isfield(scenarioCfg, 'gust_amp_scale') && isfinite(scenarioCfg.gust_amp_scale)
            gustScale = max(0.0, scenarioCfg.gust_amp_scale);
        end
        gustAmp = baseSpeed * cfg.wind.model_gust_amp_ratio * gustScale;
        gust = gustAmp * sin(2.0 * pi * cfg.wind.model_gust_freq_hz * tNow);
        noise = cfg.wind.model_noise_std_speed * randn();
        speedCmd = max(0.0, ramp * (baseSpeed + gust + noise));
    end

    cap = autosimGetWindCommandCap(cfg);
    if isfinite(cap) && cap > 0
        speedCmd = min(speedCmd, cap);
    end

    if useProfileDirect
        dirCmd = baseDir;
    else
        dirOscAmp = cfg.wind.model_dir_osc_amp_deg;
        dirScale = 1.0;
        if isfield(scenarioCfg, 'dir_osc_scale') && isfinite(scenarioCfg.dir_osc_scale)
            dirScale = max(0.0, scenarioCfg.dir_osc_scale);
        end
        dirOscAmp = dirOscAmp * dirScale;
        if isfield(cfg.wind, 'source') && cfg.wind.source == "kma_csv"
            dirOscAmp = 0.5 * dirOscAmp;
        end
        dirOsc = dirOscAmp * sin(2.0 * pi * cfg.wind.model_dir_osc_freq_hz * tNow + pi/4.0);
        dirNoise = cfg.wind.model_dir_noise_std_deg * randn();
        dirCmd = baseDir + dirOsc + dirNoise;
    end
    dirCmd = mod(dirCmd + 180.0, 360.0) - 180.0;
end


