function temporal = autosimBuildTemporalSemanticState(windObs, droneObs, tagObs, gustIntensity, cfg)
    dt = max(1e-3, autosimClampNaN(windObs.dt, 0.2));
    shortN = max(4, round(cfg.ontology.temporal_short_window_sec / dt));
    mediumN = max(shortN + 1, round(cfg.ontology.temporal_medium_window_sec / dt));
    longN = max(mediumN + 1, round(cfg.ontology.temporal_long_window_sec / dt));

    wsLong = autosimTail(autosimVizField(windObs, 'wind_speed_hist', windObs.wind_speed), longN);
    wsShort = autosimTail(autosimVizField(windObs, 'wind_speed_hist', windObs.wind_speed), shortN);
    wdLong = autosimTail(autosimVizField(windObs, 'wind_dir_hist', windObs.wind_direction), longN);
    wdShort = autosimTail(autosimVizField(windObs, 'wind_dir_hist', windObs.wind_direction), shortN);

    rollLong = autosimTail(autosimVizField(droneObs, 'roll_hist', droneObs.roll), longN);
    pitchLong = autosimTail(autosimVizField(droneObs, 'pitch_hist', droneObs.pitch), longN);
    vzLong = autosimTail(autosimVizField(droneObs, 'vz_hist', droneObs.velocity(3)), longN);

    errLong = autosimTail(autosimVizField(tagObs, 'err_hist', nan), longN);
    detLong = autosimTail(autosimVizField(tagObs, 'detected_hist', double(tagObs.detected)), longN);

    wsLong = wsLong(:);
    wsShort = wsShort(:);
    wdLong = wdLong(:);
    wdShort = wdShort(:);
    attLong = max(abs(rollLong(:)), abs(pitchLong(:)));
    vzLong = vzLong(:);
    errLong = errLong(:);
    detLong = detLong(:);

    windPersistenceStrong = autosimNanMean(double(wsLong >= cfg.ontology.wind_caution_speed));
    windPersistenceUnsafe = autosimNanMean(double(wsLong >= cfg.ontology.wind_unsafe_speed));
    windPersistence = autosimClamp( ...
        0.65 * autosimNormalize01(windPersistenceStrong, cfg.ontology.wind_persistent_warn_ratio, cfg.ontology.wind_persistent_high_ratio) + ...
        0.35 * autosimClampNaN(windPersistenceUnsafe, 0.0), 0.0, 1.0);
    windVariability = autosimNormalize01(autosimNanStd(wsLong), cfg.ontology.wind_variability_warn, cfg.ontology.wind_variability_high);
    windDirectionSpread = autosimNormalize01(autosimCircularSpreadDeg(wdLong), cfg.ontology.wind_direction_spread_warn_deg, cfg.ontology.wind_direction_spread_high_deg);
    windDirectionShiftDeg = abs(autosimWrapTo180(autosimCircularMeanDeg(wdShort) - autosimCircularMeanDeg(wdLong)));
    windDirectionShift = autosimNormalize01(windDirectionShiftDeg, cfg.ontology.wind_direction_shift_warn_deg, cfg.ontology.wind_direction_shift_high_deg);
    speedNorm = autosimNormalize01(autosimNanMean(wsShort), 0.0, cfg.wind.speed_max);

    attMeanDeg = rad2deg(autosimNanMean(attLong));
    attStdDeg = rad2deg(autosimNanStd(attLong));
    controlLoad = autosimClamp( ...
        0.42 * autosimNormalize01(attMeanDeg, cfg.ontology.control_attitude_warn_deg, cfg.ontology.control_attitude_high_deg) + ...
        0.28 * autosimNormalize01(attStdDeg, cfg.ontology.control_attitude_osc_warn_deg, cfg.ontology.control_attitude_osc_high_deg) + ...
        0.30 * autosimNormalize01(autosimNanStd(vzLong), cfg.ontology.control_vz_osc_warn, cfg.ontology.control_vz_osc_high), 0.0, 1.0);

    visualDropout = autosimClamp(1.0 - autosimClampNaN(autosimNanMean(detLong), double(tagObs.detected)), 0.0, 1.0);
    visualDropout = autosimNormalize01(visualDropout, cfg.ontology.visual_dropout_warn_ratio, cfg.ontology.visual_dropout_high_ratio);
    tagErrorVolatility = autosimNormalize01(autosimNanStd(errLong), cfg.ontology.tag_error_vol_warn, cfg.ontology.tag_error_vol_high);
    errSlope = autosimTemporalSlope(errLong, dt);
    alignmentDrift = autosimNormalize01(max(errSlope, 0.0), cfg.ontology.tag_error_drift_warn, cfg.ontology.tag_error_drift_high);

    temporal = struct();
    temporal.wind_persistence = autosimClampNaN(windPersistence, 0.0);
    temporal.wind_variability = autosimClampNaN(windVariability, 0.0);
    temporal.wind_direction_spread = autosimClampNaN(windDirectionSpread, 0.0);
    temporal.wind_direction_shift = autosimClampNaN(windDirectionShift, 0.0);
    temporal.control_load = autosimClampNaN(controlLoad, 0.0);
    temporal.visual_dropout = autosimClampNaN(visualDropout, 0.0);
    temporal.tag_error_volatility = autosimClampNaN(tagErrorVolatility, 0.0);
    temporal.alignment_drift = autosimClampNaN(alignmentDrift, 0.0);
    temporal.wind_pattern = autosimDescribeWindPattern(speedNorm, temporal.wind_persistence, autosimClampNaN(gustIntensity, 0.0), temporal.wind_variability, temporal.wind_direction_shift);
    temporal.control_difficulty = autosimDescribeControlDifficulty(temporal.control_load);
    temporal.visual_pattern = autosimDescribeVisualPattern(temporal.visual_dropout, temporal.tag_error_volatility, autosimClampNaN(tagObs.stability_score, 0.0));
    temporal.alignment_trend = autosimDescribeAlignmentTrend(errSlope, temporal.tag_error_volatility, autosimClampNaN(tagObs.detection_continuity, 0.0));
end


