function onto = autosimBuildOntologyState(windObs, droneObs, tagObs, cfg)
    if autosimIsModuleEnabled(cfg, 'ontology_engine')
        try
            onto = autosim_ontology_engine('build_state', windObs, droneObs, tagObs, cfg);
            if isstruct(onto)
                return;
            end
        catch
        end
    end

    dt = max(1e-3, autosimClampNaN(windObs.dt, 0.2));

    wsHist = windObs.wind_speed_hist(:);
    wdHist = windObs.wind_dir_hist(:);

    repN = max(3, round(cfg.ontology.wind_condition_window_sec / dt));
    wsRep = autosimTail(wsHist, repN);
    wdRep = autosimTail(wdHist, repN);

    windSpeedRep = autosimNanMean(wsRep);
    if ~isfinite(windSpeedRep)
        windSpeedRep = autosimClampNaN(windObs.wind_speed, 0.0);
    end
    windVelObs = autosimVizField(windObs, 'wind_velocity', [windSpeedRep; 0.0]);
    windVelObs = double(windVelObs(:));
    if isempty(windVelObs)
        windVelObs = [windSpeedRep; 0.0];
    elseif numel(windVelObs) == 1
        windVelObs = [windVelObs(1); 0.0];
    end
    windAccObs = autosimVizField(windObs, 'wind_acceleration', [0.0; 0.0]);
    windAccObs = double(windAccObs(:));
    if isempty(windAccObs)
        windAccObs = [0.0; 0.0];
    elseif numel(windAccObs) == 1
        windAccObs = [windAccObs(1); 0.0];
    end
    windDirRep = autosimCircularMeanDeg(wdRep);
    if ~isfinite(windDirRep)
        windDirRep = autosimClampNaN(windObs.wind_direction, 0.0);
    end
    [windDirChangeRad, windDirChangeRisk] = autosimComputeWindDirectionChangeRisk(windObs, cfg);

    baseN = max(8, round(cfg.ontology.gust_base_window_sec / dt));
    burstN = max(3, round(cfg.ontology.gust_burst_window_sec / dt));
    wsBase = autosimTail(wsHist, baseN);
    wsBurst = autosimTail(wsHist, burstN);

    vBase = autosimNanMean(wsBase);
    vPeak = autosimNanMax(wsBurst);
    deltaPeak = max(0.0, vPeak - vBase);
    burstDvdt = diff(wsBurst) / dt;
    dvdtPeak = autosimNanMax(abs(burstDvdt));

    gustActive = (deltaPeak >= cfg.ontology.gust_delta_min) || (dvdtPeak >= cfg.ontology.gust_dvdt_min);
    gustIntensity = autosimClamp( ...
        0.60 * autosimNormalize01(deltaPeak, 0.0, cfg.ontology.gust_delta_high) + ...
        0.40 * autosimNormalize01(dvdtPeak, 0.0, cfg.ontology.gust_dvdt_high), 0.0, 1.0);
    if ~gustActive
        gustLevel = 'none';
    elseif gustIntensity < 0.5
        gustLevel = 'weak';
    else
        gustLevel = 'strong';
    end

    temporal = autosimBuildTemporalSemanticState(windObs, droneObs, tagObs, gustIntensity, cfg);
    rollAbs = abs(autosimClampNaN(droneObs.roll, 0.0));
    pitchAbs = abs(autosimClampNaN(droneObs.pitch, 0.0));
    windRiskComp = autosimBuildWindRiskState(windVelObs(1:2), windAccObs(1:2), cfg, rollAbs, pitchAbs, windDirChangeRisk);

    onto = struct();
    onto.entities = struct();
    onto.entities.WindCondition = struct( ...
        'wind_speed', windSpeedRep, ...
        'wind_direction', windDirRep, ...
        'wind_velocity_vec', windVelObs(1:2), ...
        'wind_velocity_x', windVelObs(1), ...
        'wind_velocity_y', windVelObs(2), ...
        'wind_speed_norm', autosimNormalize01(hypot(windVelObs(1), windVelObs(2)), 0.0, max(1.0, cfg.wind.speed_max)), ...
        'wind_body_force_x', windRiskComp.F_wx, ...
        'wind_body_force_y', windRiskComp.F_wy, ...
        'wind_body_force', windRiskComp.F_body, ...
        'wind_body_risk', windRiskComp.r_body, ...
        'wind_gust_risk', windRiskComp.r_gust, ...
        'wind_dir_change', windDirChangeRad, ...
        'wind_dir_change_risk', windRiskComp.r_dir_change, ...
        'wind_risk_raw', windRiskComp.r_wind, ...
        'thrust_margin', windRiskComp.F_cap);

    onto.entities.Gust = struct( ...
        'active', gustActive, ...
        'intensity', gustIntensity, ...
        'delta_peak', deltaPeak, ...
        'dvdt_peak', dvdtPeak, ...
        'wind_acceleration_vec', windAccObs(1:2), ...
        'wind_acceleration_x', windAccObs(1), ...
        'wind_acceleration_y', windAccObs(2), ...
        'wind_acc_norm', autosimNormalize01(hypot(windAccObs(1), windAccObs(2)), 0.0, max(1.0, cfg.ontology.gust_dvdt_high)), ...
        'level', gustLevel);
    onto.entities.TemporalPattern = temporal;

    onto.entities.DroneState = struct( ...
        'position', droneObs.position, ...
        'roll', droneObs.roll, ...
        'pitch', droneObs.pitch, ...
        'abs_attitude', max(abs(droneObs.roll), abs(droneObs.pitch)), ...
        'vz', droneObs.velocity(3));

    onto.entities.TagObservation = struct( ...
        'detected', tagObs.detected, ...
        'u_norm', tagObs.u_norm, ...
        'v_norm', tagObs.v_norm, ...
        'u_pred', tagObs.u_pred, ...
        'v_pred', tagObs.v_pred, ...
        'jitter_px', tagObs.jitter_px, ...
        'stability_score', tagObs.stability_score, ...
        'detection_continuity', tagObs.detection_continuity, ...
        'centered', tagObs.centered);

    onto.entities.LandingContext = struct( ...
        'landing_area_size', cfg.ontology.landing_area_size, ...
        'obstacle_presence', cfg.ontology.obstacle_presence, ...
        'wind_speed_caution', cfg.ontology.wind_caution_speed, ...
        'wind_speed_unsafe', cfg.ontology.wind_unsafe_speed);

    onto.semantic_state = struct();
    onto.semantic_state.Environment = struct( ...
        'wind_speed', windSpeedRep, ...
        'wind_direction', windDirRep, ...
        'gust_active', gustActive, ...
        'gust_intensity', gustIntensity, ...
        'gust_level', string(gustLevel), ...
        'wind_body_force_x', windRiskComp.F_wx, ...
        'wind_body_force_y', windRiskComp.F_wy, ...
        'wind_body_force', windRiskComp.F_body, ...
        'wind_body_risk', windRiskComp.r_body, ...
        'wind_gust_risk', windRiskComp.r_gust, ...
        'wind_dir_change', windDirChangeRad, ...
        'wind_dir_change_risk', windRiskComp.r_dir_change, ...
        'wind_risk_raw', windRiskComp.r_wind, ...
        'thrust_margin', windRiskComp.F_cap, ...
        'wind_pattern', string(temporal.wind_pattern), ...
        'wind_persistence', temporal.wind_persistence, ...
        'wind_variability', temporal.wind_variability);
    onto.semantic_state.DroneState = struct( ...
        'position', droneObs.position, ...
        'roll', droneObs.roll, ...
        'pitch', droneObs.pitch, ...
        'abs_attitude', max(abs(droneObs.roll), abs(droneObs.pitch)), ...
        'vz', droneObs.velocity(3), ...
        'control_load', temporal.control_load, ...
        'control_difficulty', string(temporal.control_difficulty));
    onto.semantic_state.VisualState = struct( ...
        'detected', tagObs.detected, ...
        'u_norm', tagObs.u_norm, ...
        'v_norm', tagObs.v_norm, ...
        'jitter_px', tagObs.jitter_px, ...
        'stability_score', tagObs.stability_score, ...
        'centered', tagObs.centered, ...
        'visual_pattern', string(temporal.visual_pattern), ...
        'alignment_trend', string(temporal.alignment_trend));
    onto.semantic_state.LandingContext = struct( ...
        'landing_area_size', cfg.ontology.landing_area_size, ...
        'obstacle_presence', cfg.ontology.obstacle_presence);
end

function comp = autosimBuildWindRiskState(windVelocityVec, windAccelerationVec, cfg, rollAbs, pitchAbs, dirChangeRisk)
    vel = double(windVelocityVec(:));
    if isempty(vel)
        vel = [0.0; 0.0];
    elseif numel(vel) < 2
        vel = [autosimClampNaN(vel(1), 0.0); 0.0];
    else
        vel = vel(1:2);
    end
    vel(~isfinite(vel)) = 0.0;

    acc = double(windAccelerationVec(:));
    if isempty(acc)
        acc = [0.0; 0.0];
    elseif numel(acc) < 2
        acc = [autosimClampNaN(acc(1), 0.0); 0.0];
    else
        acc = acc(1:2);
    end
    acc(~isfinite(acc)) = 0.0;

    if nargin < 6 || ~isfinite(double(dirChangeRisk))
        dirChangeRisk = 0.0;
    end

    model = autosimWindRiskModel(cfg);
    c_tilt = cos(abs(double(rollAbs))) * cos(abs(double(pitchAbs)));
    c_tilt = max(model.c_min, min(1.0, c_tilt));
    T_req = (model.mass_kg * model.gravity_mps2) / max(c_tilt, model.c_min);
    if ~isfinite(T_req)
        T_req = model.mass_kg * model.gravity_mps2;
    end
    F_cap = max(model.F_min, model.max_total_thrust_n - T_req);
    if ~isfinite(F_cap)
        F_cap = model.F_min;
    end

    vx = vel(1);
    vy = vel(2);
    ax = acc(1);
    ay = acc(2);
    F_wx = 0.5 * model.rho * model.c_x * model.S_x * vx * abs(vx);
    F_wy = 0.5 * model.rho * model.c_y * model.S_y * vy * abs(vy);
    if ~isfinite(F_wx)
        F_wx = 0.0;
    end
    if ~isfinite(F_wy)
        F_wy = 0.0;
    end
    F_body = hypot(F_wx, F_wy);
    if ~isfinite(F_body)
        F_body = 0.0;
    end

    r_body = autosimClamp(F_body / max(F_cap, 1e-6), 0.0, 1.0);
    a_w = hypot(ax, ay);
    if ~isfinite(a_w)
        a_w = 0.0;
    end
    r_gust = autosimClamp(a_w / max(model.a_thr, 1e-6), 0.0, 1.0);
    r_dir_change = autosimClamp(dirChangeRisk, 0.0, 1.0);
    % Keep each risk independent; aggregate without additive weighting.
    r_wind = max([r_body, r_gust, r_dir_change]);

    comp = struct('F_wx', F_wx, 'F_wy', F_wy, 'F_body', F_body, 'F_cap', F_cap, ...
        'r_body', r_body, 'r_gust', r_gust, 'r_dir_change', r_dir_change, 'r_wind', r_wind);
end

function model = autosimWindRiskModel(cfg)
    model = struct('rho', 1.225, 'c_x', 1.10, 'c_y', 1.10, 'S_x', 0.075, 'S_y', 0.075, ...
        'c_min', 0.2, 'F_min', 0.5, 'a_thr', 1.2, 'w_body', 0.7, 'w_gust', 0.3, ...
        'mass_kg', 1.4, 'gravity_mps2', 9.81, 'max_total_thrust_n', 24.0);
    if isstruct(cfg) && isfield(cfg, 'ontology') && isstruct(cfg.ontology) && isfield(cfg.ontology, 'wind_risk_model') && isstruct(cfg.ontology.wind_risk_model)
        orm = cfg.ontology.wind_risk_model;
        model.rho = max(1e-6, autosimStateField(orm, 'rho', model.rho));
        model.c_min = max(1e-6, autosimStateField(orm, 'c_min', model.c_min));
        model.F_min = max(1e-6, autosimStateField(orm, 'F_min', model.F_min));
        model.a_thr = max(1e-6, autosimStateField(orm, 'a_thr', model.a_thr));
        model.w_body = max(0.0, autosimStateField(orm, 'w_body', model.w_body));
        model.w_gust = max(0.0, autosimStateField(orm, 'w_gust', model.w_gust));
        cX = autosimStateField(orm, 'c_x', nan);
        cY = autosimStateField(orm, 'c_y', nan);
        sX = autosimStateField(orm, 'S_x', nan);
        sY = autosimStateField(orm, 'S_y', nan);
        if isfinite(cX)
            model.c_x = max(1e-6, cX);
        end
        if isfinite(cY)
            model.c_y = max(1e-6, cY);
        end
        if isfinite(sX)
            model.S_x = max(1e-6, sX);
        end
        if isfinite(sY)
            model.S_y = max(1e-6, sY);
        end
    end
    if isstruct(cfg) && isfield(cfg, 'wind') && isstruct(cfg.wind) && isfield(cfg.wind, 'physics') && isstruct(cfg.wind.physics)
        p = cfg.wind.physics;
        cd = max(1e-6, autosimStateField(p, 'drag_coefficient', model.c_x));
        area = max(1e-6, autosimStateField(p, 'frontal_area_m2', model.S_x));
        model.rho = max(1e-6, autosimStateField(p, 'air_density_kgpm3', model.rho));
        model.c_x = max(1e-6, autosimStateField(p, 'drag_coefficient_x', cd));
        model.c_y = max(1e-6, autosimStateField(p, 'drag_coefficient_y', cd));
        model.S_x = max(1e-6, autosimStateField(p, 'frontal_area_x_m2', area));
        model.S_y = max(1e-6, autosimStateField(p, 'frontal_area_y_m2', area));
        model.mass_kg = autosimStateField(p, 'mass_kg', model.mass_kg);
        model.gravity_mps2 = autosimStateField(p, 'gravity_mps2', model.gravity_mps2);
        model.max_total_thrust_n = autosimStateField(p, 'max_total_thrust_n', model.max_total_thrust_n);
        model.F_min = max(model.F_min, max(0.0, autosimStateField(p, 'min_thrust_margin_n', model.F_min)));
    end
end

function v = autosimStateField(s, name, fallback)
    if isstruct(s) && isfield(s, name)
        vv = double(s.(name));
        if isfinite(vv)
            v = vv;
            return;
        end
    end
    v = fallback;
end

function [deltaThetaRad, risk] = autosimComputeWindDirectionChangeRisk(windObs, cfg)
    vxHist = double(autosimVizField(windObs, 'wind_vel_x_hist', nan));
    vyHist = double(autosimVizField(windObs, 'wind_vel_y_hist', nan));
    vxHist = vxHist(:);
    vyHist = vyHist(:);
    n = min(numel(vxHist), numel(vyHist));
    if n < 2
        deltaThetaRad = 0.0;
        risk = 0.0;
        return;
    end

    vNow = [vxHist(n); vyHist(n)];
    vPrev = [vxHist(n-1); vyHist(n-1)];
    if any(~isfinite(vNow)) || any(~isfinite(vPrev))
        deltaThetaRad = 0.0;
        risk = 0.0;
        return;
    end

    nNow = norm(vNow);
    nPrev = norm(vPrev);
    if nNow <= 1e-6 || nPrev <= 1e-6
        deltaThetaRad = 0.0;
        risk = 0.0;
        return;
    end

    cosTheta = dot(vNow, vPrev) / max(nNow * nPrev, 1e-12);
    cosTheta = min(1.0, max(-1.0, cosTheta));
    deltaThetaRad = acos(cosTheta);

    thetaThr = pi;
    if isstruct(cfg) && isfield(cfg, 'ontology') && isstruct(cfg.ontology) && ...
            isfield(cfg.ontology, 'wind_risk_model') && isstruct(cfg.ontology.wind_risk_model) && ...
            isfield(cfg.ontology.wind_risk_model, 'theta_thr_rad') && isfinite(cfg.ontology.wind_risk_model.theta_thr_rad)
        thetaThr = max(1e-6, double(cfg.ontology.wind_risk_model.theta_thr_rad));
    end
    risk = autosimClamp(deltaThetaRad / thetaThr, 0.0, 1.0);
end


