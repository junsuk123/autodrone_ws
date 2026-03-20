function [cfg, info] = autosimApplyWindPhysicsLimits(cfg)
    info = struct('applied', false, 'hover_limit_mps', nan, 'landing_limit_mps', nan, 'thrust_margin_n', nan, ...
        'mass_kg', nan, 'max_total_thrust_n', nan, 'mass_source', "", 'thrust_source', "");
    if ~isfield(cfg, 'wind') || ~isfield(cfg.wind, 'physics') || ~isstruct(cfg.wind.physics)
        return;
    end

    p = cfg.wind.physics;
    enabled = true;
    if isfield(p, 'enable')
        enabled = logical(p.enable);
    end
    if ~enabled
        return;
    end

    p = autosimReadPhysicsFromDroneFiles(p);
    cfg.wind.physics = p;

    massKg = autosimWindField(p, 'mass_kg', 1.4);
    g = autosimWindField(p, 'gravity_mps2', 9.81);
    tMax = autosimWindField(p, 'max_total_thrust_n', 24.0);
    rho = autosimWindField(p, 'air_density_kgpm3', 1.225);
    cd = autosimWindField(p, 'drag_coefficient', 1.10);
    area = autosimWindField(p, 'frontal_area_m2', 0.075);
    minMargin = autosimWindField(p, 'min_thrust_margin_n', 0.5);
    landingFactor = autosimWindField(p, 'landing_limit_factor', 0.5);

    margin = tMax - massKg * g;
    if ~isfinite(margin)
        return;
    end
    margin = max(margin, max(0.0, minMargin));

    denom = rho * cd * area;
    if ~(isfinite(denom) && denom > 0)
        return;
    end

    hoverLimit = sqrt(max(0.0, 2.0 * margin / denom));
    landingLimit = max(0.0, landingFactor) * hoverLimit;
    if ~(isfinite(hoverLimit) && isfinite(landingLimit))
        return;
    end

    cfg.wind.hover_limit_mps = hoverLimit;
    cfg.wind.landing_limit_mps = landingLimit;
    cfg.wind.physics.thrust_margin_n = margin;

    if isfield(p, 'apply_to_agent_gate') && logical(p.apply_to_agent_gate)
        gateRatio = autosimWindField(p, 'agent_gate_ratio', 1.0);
        gateLimit = max(0.0, gateRatio * landingLimit);
        if ~isfield(cfg, 'agent') || ~isstruct(cfg.agent)
            cfg.agent = struct();
        end
        if isfield(cfg.agent, 'no_model_max_wind_speed') && isfinite(cfg.agent.no_model_max_wind_speed)
            cfg.agent.no_model_max_wind_speed = min(cfg.agent.no_model_max_wind_speed, gateLimit);
        else
            cfg.agent.no_model_max_wind_speed = gateLimit;
        end
    end

    if isfield(p, 'apply_to_ontology') && logical(p.apply_to_ontology)
        cautionRatio = autosimWindField(p, 'ontology_caution_ratio', 0.55);
        unsafeRatio = autosimWindField(p, 'ontology_unsafe_ratio', 0.90);
        windUnsafe = max(0.0, unsafeRatio * landingLimit);
        windCaution = max(0.0, cautionRatio * landingLimit);
        if ~isfield(cfg, 'ontology') || ~isstruct(cfg.ontology)
            cfg.ontology = struct();
        end
        cfg.ontology.wind_caution_speed = windCaution;
        cfg.ontology.wind_unsafe_speed = windUnsafe;
        cfg.ontology.wind_variability_warn = 0.12 * windUnsafe;
        cfg.ontology.wind_variability_high = 0.28 * windUnsafe;
    end

    info.applied = true;
    info.hover_limit_mps = hoverLimit;
    info.landing_limit_mps = landingLimit;
    info.thrust_margin_n = margin;
    info.mass_kg = massKg;
    info.max_total_thrust_n = tMax;
    info.mass_source = autosimWindStringField(p, 'source_mass', "default_mass");
    info.thrust_source = autosimWindStringField(p, 'source_max_force', "default_maxForce");
end


