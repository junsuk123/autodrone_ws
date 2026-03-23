function semantic = autosimOntologyReasoning(onto, cfg)
    if autosimIsModuleEnabled(cfg, 'ontology_engine')
        try
            semantic = autosim_ontology_engine('reason', onto, cfg);
            if isstruct(semantic)
                return;
            end
        catch
        end
    end

    w = onto.entities.WindCondition;
    g = onto.entities.Gust;
    tp = onto.entities.TemporalPattern;
    d = onto.entities.DroneState;
    t = onto.entities.TagObservation;
    c = onto.entities.LandingContext;

    windVelocity = autosimClampNaN(w.wind_speed, 0.0);
    windAcceleration = autosimClampNaN(g.dvdt_peak, 0.0);
    windVelocityVec = [windVelocity; 0.0];
    if isfield(w, 'wind_velocity_vec')
        vv = double(w.wind_velocity_vec(:));
        if numel(vv) >= 2
            windVelocityVec = vv(1:2);
        elseif numel(vv) == 1
            windVelocityVec = [vv(1); 0.0];
        end
    end
    windAccelerationVec = [windAcceleration; 0.0];
    if isfield(g, 'wind_acceleration_vec')
        va = double(g.wind_acceleration_vec(:));
        if numel(va) >= 2
            windAccelerationVec = va(1:2);
        elseif numel(va) == 1
            windAccelerationVec = [va(1); 0.0];
        end
    end
    windVelocity = hypot(windVelocityVec(1), windVelocityVec(2));
    windAcceleration = hypot(windAccelerationVec(1), windAccelerationVec(2));

    rollAbs = abs(autosimClampNaN(d.roll, 0.0));
    pitchAbs = abs(autosimClampNaN(d.pitch, 0.0));
    windDirChangeRisk = autosimClampNaN(autosimVizField(w, 'wind_dir_change_risk', 0.0), 0.0);
    windDirChange = autosimClampNaN(autosimVizField(w, 'wind_dir_change', 0.0), 0.0);
    windRiskComp = autosimComputeWindRiskComponents(windVelocityVec, windAccelerationVec, cfg, rollAbs, pitchAbs, windDirChangeRisk);
    condScore = windRiskComp.r_wind;
    windRiskRuleEnc = windRiskComp.r_wind;

    if t.detected && isfinite(t.u_norm) && isfinite(t.v_norm)
        errNow = sqrt((t.u_norm - cfg.control.target_u)^2 + (t.v_norm - cfg.control.target_v)^2);
        if isfinite(t.u_pred) && isfinite(t.v_pred)
            errPred = sqrt((t.u_pred - cfg.control.target_u)^2 + (t.v_pred - cfg.control.target_v)^2);
        else
            errPred = errNow;
        end
        detCont = autosimClampNaN(t.detection_continuity, 0.0);
        alignRuleEnc = autosimClamp(1.0 - ( ...
            0.55 * autosimNormalize01(errNow, 0.0, cfg.agent.max_tag_error_before_land) + ...
            0.25 * autosimNormalize01(errPred, 0.0, cfg.agent.max_tag_error_before_land) + ...
            0.10 * tp.alignment_drift + ...
            0.10 * tp.tag_error_volatility + ...
            0.20 * (1.0 - detCont)), 0.0, 1.0);
    else
        errNow = nan;
        errPred = nan;
        alignRuleEnc = 0.0;
    end

    jitterN = autosimNormalize01(t.jitter_px, 0.0, cfg.ontology.tag_jitter_unsafe_px);
    stabScore = autosimClampNaN(t.stability_score, 0.0);
    detCont = autosimClampNaN(t.detection_continuity, 0.0);
    visualRuleEnc = autosimClamp( ...
        0.26 * double(t.detected) + 0.22 * (1.0 - jitterN) + 0.20 * stabScore + 0.14 * detCont + 0.10 * (1.0 - tp.visual_dropout) + 0.08 * (1.0 - tp.tag_error_volatility), 0.0, 1.0);
    if ~t.detected
        visualRuleEnc = 0.6 * visualRuleEnc;
    end

    attStab = 1.0 - autosimNormalize01(d.abs_attitude, 0.0, deg2rad(cfg.thresholds.final_attitude_max_deg));
    windRiskEnc = windRiskRuleEnc;
    alignEnc = alignRuleEnc;
    visualEnc = visualRuleEnc;

    if isfield(cfg, 'ontology_ai') && isfield(cfg.ontology_ai, 'enable') && cfg.ontology_ai.enable
        aiFeatWind = [ ...
            condScore, ...
            autosimClampNaN(g.intensity, 0.0), ...
            tp.wind_persistence, ...
            tp.wind_variability, ...
            tp.wind_direction_shift, ...
            tp.wind_direction_spread, ...
            tp.control_load, ...
            tp.visual_dropout ...
        ];
        windRiskAiEnc = autosimLinearSigmoid(aiFeatWind, cfg.ontology_ai.wind_w, cfg.ontology_ai.wind_b, 0.5);

        aiFeatAlign = [ ...
            autosimNormalize01(errNow, 0.0, cfg.agent.max_tag_error_before_land), ...
            autosimNormalize01(errPred, 0.0, cfg.agent.max_tag_error_before_land), ...
            detCont, ...
            tp.alignment_drift, ...
            tp.tag_error_volatility, ...
            double(t.detected), ...
            1.0 - jitterN, ...
            1.0 - tp.control_load ...
        ];
        alignAiEnc = autosimLinearSigmoid(aiFeatAlign, cfg.ontology_ai.align_w, cfg.ontology_ai.align_b, 0.0);

        aiFeatVisual = [ ...
            double(t.detected), ...
            jitterN, ...
            stabScore, ...
            detCont, ...
            tp.visual_dropout, ...
            tp.tag_error_volatility, ...
            autosimClampNaN(g.intensity, 0.0), ...
            tp.control_load ...
        ];
        visualAiEnc = autosimLinearSigmoid(aiFeatVisual, cfg.ontology_ai.visual_w, cfg.ontology_ai.visual_b, 0.0);

        rw = autosimClampNaN(cfg.ontology_ai.rule_weight, 0.60);
        rw = autosimClamp(rw, 0.0, 1.0);
        aw = 1.0 - rw;

        windRiskEnc = autosimClamp(rw * windRiskRuleEnc + aw * windRiskAiEnc, 0.0, 1.0);
        alignEnc = autosimClamp(rw * alignRuleEnc + aw * alignAiEnc, 0.0, 1.0);
        visualEnc = autosimClamp(rw * visualRuleEnc + aw * visualAiEnc, 0.0, 1.0);
    end

    windRisk = autosimRiskLevel3(windRiskEnc);
    windPattern = string(tp.wind_pattern);
    alignmentTrend = string(tp.alignment_trend);
    visualPattern = string(tp.visual_pattern);
    controlDifficulty = string(tp.control_difficulty);

    if alignEnc >= 0.70
        alignState = 'aligned';
    else
        alignState = 'misaligned';
    end
    if visualEnc >= cfg.ontology.tag_stability_score_warn
        visualState = 'stable';
    else
        visualState = 'unstable';
    end
    if windRiskEnc < 0.30 && tp.control_load < 0.35 && tp.visual_dropout < 0.20
        environmentState = 'favorable';
    elseif windRiskEnc < 0.65 && tp.control_load < 0.70
        environmentState = 'monitoring';
    else
        environmentState = 'adverse';
    end

    droneVzNorm = autosimNormalize01(abs(d.vz), 0.0, cfg.agent.no_model_max_abs_vz);
    droneStability = autosimClamp(0.48 * attStab + 0.22 * (1.0 - droneVzNorm) + 0.30 * (1.0 - tp.control_load), 0.0, 1.0);
    if droneStability >= 0.70
        droneState = 'stable';
    elseif droneStability >= 0.40
        droneState = 'recovering';
    else
        droneState = 'unstable';
    end

    if strcmp(environmentState, 'favorable') && strcmp(droneState, 'stable') && strcmp(visualState, 'stable')
        semanticRelation = 'supportive';
    elseif strcmp(environmentState, 'adverse') || strcmp(droneState, 'unstable')
        semanticRelation = 'conflicting';
    else
        semanticRelation = 'conditional';
    end

    landingFeasibility = autosimClamp( ...
        0.40 * (1.0 - windRiskEnc) + ...
        0.25 * alignEnc + ...
        0.20 * visualEnc + ...
        0.15 * droneStability, 0.0, 1.0);

    if landingFeasibility >= cfg.agent.semantic_land_threshold
        semanticIntegration = 'AttemptLanding';
        finalDecision = 'AttemptLanding';
    elseif landingFeasibility <= cfg.agent.semantic_abort_threshold
        semanticIntegration = 'HoldLanding';
        finalDecision = 'HoldLanding';
    else
        semanticIntegration = 'HoldLanding';
        finalDecision = 'HoldLanding';
    end

    semantic = struct();
    semantic.environment_state = environmentState;
    semantic.drone_state = droneState;
    semantic.wind_risk = windRisk;
    semantic.wind_pattern = windPattern;
    semantic.alignment_state = alignState;
    semantic.alignment_trend = alignmentTrend;
    semantic.visual_state = visualState;
    semantic.visual_pattern = visualPattern;
    semantic.control_difficulty = controlDifficulty;
    semantic.landing_feasibility = landingFeasibility;
    semantic.final_decision = finalDecision;
    semantic.isSafeForLanding = strcmp(finalDecision, 'AttemptLanding');
    semantic.wind_velocity = windVelocity;
    semantic.wind_acceleration = windAcceleration;
    semantic.wind_velocity_x = windVelocityVec(1);
    semantic.wind_velocity_y = windVelocityVec(2);
    semantic.wind_acceleration_x = windAccelerationVec(1);
    semantic.wind_acceleration_y = windAccelerationVec(2);
    semantic.wind_body_force_x = windRiskComp.F_wx;
    semantic.wind_body_force_y = windRiskComp.F_wy;
    semantic.wind_body_force = windRiskComp.F_body;
    semantic.wind_body_risk = windRiskComp.r_body;
    semantic.wind_gust_risk = windRiskComp.r_gust;
    semantic.wind_dir_change = windDirChange;
    semantic.wind_dir_change_risk = windRiskComp.r_dir_change;
    semantic.wind_risk_raw = windRiskComp.r_wind;
    semantic.thrust_margin = windRiskComp.F_cap;
    semantic.wind_risk_enc = windRiskEnc;
    semantic.alignment_enc = alignEnc;
    semantic.visual_enc = visualEnc;

end

function risk = autosimComputeWindRiskComponents(windVelocityVec, windAccelerationVec, cfg, rollAbs, pitchAbs, dirChangeRisk)
    if nargin < 5 || ~isfinite(double(rollAbs))
        rollAbs = 0.0;
    end
    if nargin < 6 || ~isfinite(double(pitchAbs))
        pitchAbs = 0.0;
    end
    if nargin < 7 || ~isfinite(double(dirChangeRisk))
        dirChangeRisk = 0.0;
    end

    velVec = double(windVelocityVec(:));
    if isempty(velVec)
        velVec = [0.0; 0.0];
    elseif numel(velVec) < 2
        velVec = [velVec(1); 0.0];
    else
        velVec = velVec(1:2);
    end
    velVec(~isfinite(velVec)) = 0.0;

    accVec = double(windAccelerationVec(:));
    if isempty(accVec)
        accVec = [0.0; 0.0];
    elseif numel(accVec) < 2
        accVec = [accVec(1); 0.0];
    else
        accVec = accVec(1:2);
    end
    accVec(~isfinite(accVec)) = 0.0;

    model = autosimGetWindRiskModel(cfg, rollAbs, pitchAbs);
    vx = velVec(1);
    vy = velVec(2);
    ax = accVec(1);
    ay = accVec(2);

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

    r_body = autosimClamp(F_body / max(F_cap, 1e-6), 0.0, 1.0);
    a_w = hypot(ax, ay);
    if ~isfinite(a_w)
        a_w = 0.0;
    end
    r_gust = autosimClamp(a_w / max(model.a_thr, 1e-6), 0.0, 1.0);

    r_dir_change = autosimClamp(dirChangeRisk, 0.0, 1.0);
    % Keep each risk independent; aggregate without additive weighting.
    r_wind = max([r_body, r_gust, r_dir_change]);

    risk = struct();
    risk.F_wx = F_wx;
    risk.F_wy = F_wy;
    risk.F_body = F_body;
    risk.F_cap = F_cap;
    risk.r_body = r_body;
    risk.r_gust = r_gust;
    risk.r_dir_change = r_dir_change;
    risk.r_wind = r_wind;
end

function model = autosimGetWindRiskModel(cfg, rollAbs, pitchAbs)
    if nargin < 2 || ~isfinite(double(rollAbs))
        rollAbs = 0.0;
    end
    if nargin < 3 || ~isfinite(double(pitchAbs))
        pitchAbs = 0.0;
    end

    model = struct();
    model.rho = 1.225;
    model.c_x = 1.10;
    model.c_y = 1.10;
    model.S_x = 0.075;
    model.S_y = 0.075;
    model.c_min = 0.2;
    model.F_min = 0.5;
    model.a_thr = 1.2;
    model.w_body = 0.7;
    model.w_gust = 0.3;
    model.mass_kg = 1.4;
    model.gravity_mps2 = 9.81;
    model.max_total_thrust_n = 24.0;

    if isstruct(cfg) && isfield(cfg, 'ontology') && isstruct(cfg.ontology) && isfield(cfg.ontology, 'wind_risk_model') && isstruct(cfg.ontology.wind_risk_model)
        orm = cfg.ontology.wind_risk_model;
        model.rho = max(1e-6, autosimWindLoadField(orm, 'rho', model.rho));
        model.c_min = max(1e-6, autosimWindLoadField(orm, 'c_min', model.c_min));
        model.F_min = max(1e-6, autosimWindLoadField(orm, 'F_min', model.F_min));
        model.a_thr = max(1e-6, autosimWindLoadField(orm, 'a_thr', model.a_thr));
        model.w_body = max(0.0, autosimWindLoadField(orm, 'w_body', model.w_body));
        model.w_gust = max(0.0, autosimWindLoadField(orm, 'w_gust', model.w_gust));

        cx = autosimWindLoadField(orm, 'c_x', nan);
        cy = autosimWindLoadField(orm, 'c_y', nan);
        sx = autosimWindLoadField(orm, 'S_x', nan);
        sy = autosimWindLoadField(orm, 'S_y', nan);
        if isfinite(cx)
            model.c_x = max(1e-6, cx);
        end
        if isfinite(cy)
            model.c_y = max(1e-6, cy);
        end
        if isfinite(sx)
            model.S_x = max(1e-6, sx);
        end
        if isfinite(sy)
            model.S_y = max(1e-6, sy);
        end
    end

    if ~isstruct(cfg) || ~isfield(cfg, 'wind') || ~isstruct(cfg.wind) || ~isfield(cfg.wind, 'physics') || ~isstruct(cfg.wind.physics)
        return;
    end
    p = cfg.wind.physics;
    model.rho = max(1e-6, autosimWindLoadField(p, 'air_density_kgpm3', model.rho));
    cd = max(1e-6, autosimWindLoadField(p, 'drag_coefficient', model.c_x));
    area = max(1e-6, autosimWindLoadField(p, 'frontal_area_m2', model.S_x));
    model.c_x = max(1e-6, autosimWindLoadField(p, 'drag_coefficient_x', cd));
    model.c_y = max(1e-6, autosimWindLoadField(p, 'drag_coefficient_y', cd));
    model.S_x = max(1e-6, autosimWindLoadField(p, 'frontal_area_x_m2', area));
    model.S_y = max(1e-6, autosimWindLoadField(p, 'frontal_area_y_m2', area));

    model.mass_kg = autosimWindLoadField(p, 'mass_kg', model.mass_kg);
    model.gravity_mps2 = autosimWindLoadField(p, 'gravity_mps2', model.gravity_mps2);
    model.max_total_thrust_n = autosimWindLoadField(p, 'max_total_thrust_n', model.max_total_thrust_n);
    model.F_min = max(model.F_min, max(0.0, autosimWindLoadField(p, 'min_thrust_margin_n', model.F_min)));

    cosTilt = cos(abs(double(rollAbs))) * cos(abs(double(pitchAbs)));
    cosTilt = min(1.0, max(model.c_min, cosTilt));
    requiredHoverThrust = (model.mass_kg * model.gravity_mps2) / max(cosTilt, model.c_min);

    margin = model.max_total_thrust_n - requiredHoverThrust;
    if ~(isfinite(margin) && (margin > 0))
        marginFlat = autosimWindLoadField(p, 'thrust_margin_n', model.max_total_thrust_n - model.mass_kg * model.gravity_mps2);
        if isfinite(marginFlat)
            margin = marginFlat;
        else
            margin = model.max_total_thrust_n - model.mass_kg * model.gravity_mps2;
        end
    end
    model.F_min = max(model.F_min, 1e-6);
    if isfinite(margin) && margin > model.F_min
        model.max_total_thrust_n = requiredHoverThrust + margin;
    end
end

function v = autosimWindLoadField(s, name, fallback)
    if isstruct(s) && isfield(s, name)
        vv = double(s.(name));
        if isfinite(vv)
            v = vv;
            return;
        end
    end
    v = fallback;
end


