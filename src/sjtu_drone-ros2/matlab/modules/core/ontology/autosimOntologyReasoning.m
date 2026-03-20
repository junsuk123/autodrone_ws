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
    velScore = autosimNormalize01(windVelocity, c.wind_speed_caution, c.wind_speed_unsafe);
    accScore = autosimNormalize01(abs(windAcceleration), cfg.ontology.gust_dvdt_min, cfg.ontology.gust_dvdt_high);
    condScore = autosimClamp(0.72 * velScore + 0.28 * accScore, 0.0, 1.0);
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
    riskCtx = ...
        0.28 * windRiskRuleEnc + ...
        0.16 * (1.0 - alignRuleEnc) + ...
        0.16 * (1.0 - visualRuleEnc) + ...
        0.12 * (1.0 - attStab) + ...
        0.15 * tp.control_load + ...
        0.13 * tp.visual_dropout + ...
        0.40 * double(c.obstacle_presence);
    contextRuleEnc = autosimClamp(1.0 - riskCtx, 0.0, 1.0);

    windRiskEnc = windRiskRuleEnc;
    alignEnc = alignRuleEnc;
    visualEnc = visualRuleEnc;
    contextEnc = contextRuleEnc;

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

        aiFeatContext = [ ...
            windRiskAiEnc, ...
            alignAiEnc, ...
            visualAiEnc, ...
            attStab, ...
            tp.control_load, ...
            tp.wind_persistence, ...
            tp.visual_dropout, ...
            1.0 - double(c.obstacle_presence) ...
        ];
        contextAiEnc = autosimLinearSigmoid(aiFeatContext, cfg.ontology_ai.context_w, cfg.ontology_ai.context_b, 0.5);

        rw = autosimClampNaN(cfg.ontology_ai.rule_weight, 0.60);
        rw = autosimClamp(rw, 0.0, 1.0);
        aw = 1.0 - rw;

        windRiskEnc = autosimClamp(rw * windRiskRuleEnc + aw * windRiskAiEnc, 0.0, 1.0);
        alignEnc = autosimClamp(rw * alignRuleEnc + aw * alignAiEnc, 0.0, 1.0);
        visualEnc = autosimClamp(rw * visualRuleEnc + aw * visualAiEnc, 0.0, 1.0);
        contextEnc = autosimClamp(rw * contextRuleEnc + aw * contextAiEnc, 0.0, 1.0);
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
    if contextEnc >= 0.70
        contextState = 'safe';
    elseif contextEnc >= 0.40
        contextState = 'caution';
    else
        contextState = 'unsafe';
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

    if strcmp(environmentState, 'favorable') && strcmp(droneState, 'stable') && strcmp(visualState, 'stable') && strcmp(contextState, 'safe')
        semanticRelation = 'supportive';
    elseif strcmp(contextState, 'unsafe') || strcmp(environmentState, 'adverse') || strcmp(droneState, 'unstable')
        semanticRelation = 'conflicting';
    else
        semanticRelation = 'conditional';
    end

    landingFeasibility = autosimClamp( ...
        0.30 * (1.0 - windRiskEnc) + ...
        0.20 * alignEnc + ...
        0.20 * visualEnc + ...
        0.15 * contextEnc + ...
        0.15 * droneStability, 0.0, 1.0);

    if landingFeasibility >= cfg.agent.semantic_land_threshold && strcmp(contextState, 'safe')
        semanticIntegration = 'AttemptLanding';
        finalDecision = 'AttemptLanding';
    elseif landingFeasibility <= cfg.agent.semantic_abort_threshold || strcmp(contextState, 'unsafe')
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
    semantic.landing_context = contextState;
    semantic.control_difficulty = controlDifficulty;
    semantic.semantic_relation = semanticRelation;
    semantic.semantic_integration = semanticIntegration;
    semantic.landing_feasibility = landingFeasibility;
    semantic.final_decision = finalDecision;
    semantic.isSafeForLanding = strcmp(finalDecision, 'AttemptLanding');
    semantic.wind_velocity = windVelocity;
    semantic.wind_acceleration = windAcceleration;
    semantic.wind_risk_enc = windRiskEnc;
    semantic.alignment_enc = alignEnc;
    semantic.visual_enc = visualEnc;
    semantic.context_enc = contextEnc;

end


