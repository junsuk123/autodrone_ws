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
    windLoad = autosimComputeWindLoadRisk(windVelocityVec, windAccelerationVec, c.wind_speed_unsafe, cfg, rollAbs, pitchAbs);
    speedEqScore = autosimNormalize01(windLoad.equivalent_speed, c.wind_speed_caution, c.wind_speed_unsafe);
    dragLoadScore = autosimNormalize01(windLoad.drag_ratio, 0.60, 1.00);
    accScoreMag = autosimNormalize01(abs(windAcceleration), cfg.ontology.gust_dvdt_min, cfg.ontology.gust_dvdt_high);
    accScoreComp = autosimNormalize01(max(abs(windAccelerationVec(1)), abs(windAccelerationVec(2))), cfg.ontology.gust_dvdt_min, cfg.ontology.gust_dvdt_high);
    accScore = autosimClamp(0.60 * accScoreMag + 0.40 * accScoreComp, 0.0, 1.0);
    condScore = autosimClamp(0.60 * speedEqScore + 0.30 * dragLoadScore + 0.10 * accScore, 0.0, 1.0);
    windRiskRuleEnc = autosimClamp( ...
        0.26 * condScore + 0.22 * g.intensity + 0.22 * tp.wind_persistence + 0.14 * tp.wind_variability + 0.08 * tp.wind_direction_shift + 0.08 * tp.wind_direction_spread, 0.0, 1.0);

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
    semantic.wind_risk_enc = windRiskEnc;
    semantic.alignment_enc = alignEnc;
    semantic.visual_enc = visualEnc;

end

function risk = autosimComputeWindLoadRisk(windVelocityVec, windAccelerationVec, windUnsafe, cfg, rollAbs, pitchAbs)
    if nargin < 5 || ~isfinite(double(rollAbs))
        rollAbs = 0.0;
    end
    if nargin < 6 || ~isfinite(double(pitchAbs))
        pitchAbs = 0.0;
    end

    if ~(isfinite(windUnsafe) && (windUnsafe > 0))
        windUnsafe = 1.0;
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

    velMag = hypot(velVec(1), velVec(2));
    velComp = max(abs(velVec(1)), abs(velVec(2)));
    accMag = hypot(accVec(1), accVec(2));
    accComp = max(abs(accVec(1)), abs(accVec(2)));

    model = autosimGetWindLoadModel(cfg, rollAbs, pitchAbs);
    dynPressure = 0.5 * model.rho * model.cd * model.area;

    dragMag = dynPressure * (velMag ^ 2);
    dragComp = dynPressure * (velComp ^ 2);

    gustGain = 1.0 + min(0.5, 0.08 * max(0.0, max(accMag, accComp)));
    dragEff = max(dragMag, dragComp) * gustGain;
    dragRatio = dragEff / max(1e-6, model.drag_capacity_n);
    dragEqSpeed = windUnsafe * sqrt(max(0.0, dragRatio));

    risk = struct();
    risk.drag_ratio = max(0.0, dragRatio);
    risk.equivalent_speed = max([velMag, velComp, dragEqSpeed]);
end

function model = autosimGetWindLoadModel(cfg, rollAbs, pitchAbs)
    if nargin < 2 || ~isfinite(double(rollAbs))
        rollAbs = 0.0;
    end
    if nargin < 3 || ~isfinite(double(pitchAbs))
        pitchAbs = 0.0;
    end

    model = struct();
    model.rho = 1.225;
    model.cd = 1.10;
    model.area = 0.075;
    model.drag_capacity_n = 6.0;

    if ~isstruct(cfg) || ~isfield(cfg, 'wind') || ~isstruct(cfg.wind) || ~isfield(cfg.wind, 'physics') || ~isstruct(cfg.wind.physics)
        return;
    end

    p = cfg.wind.physics;
    model.rho = max(1e-6, autosimWindLoadField(p, 'air_density_kgpm3', model.rho));
    model.cd = max(1e-6, autosimWindLoadField(p, 'drag_coefficient', model.cd));
    model.area = max(1e-6, autosimWindLoadField(p, 'frontal_area_m2', model.area));

    massKg = autosimWindLoadField(p, 'mass_kg', 1.4);
    grav = autosimWindLoadField(p, 'gravity_mps2', 9.81);
    maxThrust = autosimWindLoadField(p, 'max_total_thrust_n', 24.0);
    minMargin = max(0.0, autosimWindLoadField(p, 'min_thrust_margin_n', 0.5));

    cosTilt = cos(abs(double(rollAbs))) * cos(abs(double(pitchAbs)));
    cosTilt = min(1.0, max(0.15, cosTilt));
    requiredHoverThrust = (massKg * grav) / cosTilt;

    margin = maxThrust - requiredHoverThrust;
    if ~(isfinite(margin) && (margin > 0))
        marginFlat = autosimWindLoadField(p, 'thrust_margin_n', maxThrust - massKg * grav);
        if isfinite(marginFlat)
            margin = marginFlat;
        else
            margin = maxThrust - massKg * grav;
        end
    end
    margin = max(1e-6, max(margin, minMargin));

    model.drag_capacity_n = margin;
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


