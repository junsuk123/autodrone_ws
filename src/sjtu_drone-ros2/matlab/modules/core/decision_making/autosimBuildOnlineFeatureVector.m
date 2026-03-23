function feat = autosimBuildOnlineFeatureVector(z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, contact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR, semVec, cfg, windVelocity, windAcceleration)
    if nargin < 16
        semVec = [];
    end
    if nargin < 17
        cfg = [];
    end
    if nargin < 18
        windVelocity = [];
    end
    if nargin < 19
        windAcceleration = [];
    end

    if autosimIsModuleEnabled(cfg, 'ai_engine')
        try
            feat = autosim_ai_engine('build_online_features', z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, contact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR, semVec, cfg, windVelocity, windAcceleration);
            if isstruct(feat)
                return;
            end
        catch
        end
    end

    feat = struct();
    feat.mean_wind_speed = autosimNanMean(windSpeed);
    feat.max_wind_speed = autosimNanMax(windSpeed);
    if isempty(windVelocity)
        windVelocity = windSpeed;
    end
    if isempty(windAcceleration)
        windAcceleration = 0.0;
    end
    [feat.wind_velocity_x, feat.wind_velocity_y, feat.wind_velocity] = autosimResolveWindVectorFeatures(windVelocity, windSpeed);
    [feat.wind_acceleration_x, feat.wind_acceleration_y, feat.wind_acceleration] = autosimResolveWindVectorFeatures(windAcceleration, 0.0);
    feat.mean_abs_roll_deg = autosimNanMean(abs(rollDeg));
    feat.mean_abs_pitch_deg = autosimNanMean(abs(pitchDeg));
    feat.mean_abs_vz = autosimNanMean(abs(vz));
    feat.max_abs_vz = autosimNanMax(abs(vz));
    feat.mean_tag_error = autosimNanMean(tagErr);
    feat.max_tag_error = autosimNanMax(tagErr);

    feat.final_altitude = autosimNanLast(z);
    feat.final_abs_speed = autosimNanLast(speedAbs);
    feat.final_abs_roll_deg = autosimNanLast(abs(rollDeg));
    feat.final_abs_pitch_deg = autosimNanLast(abs(pitchDeg));
    feat.final_tag_error = autosimNanLast(tagErr);

    finalWindow = min(numel(z), 25);
    zTail = autosimTail(z, finalWindow);
    vzTail = autosimTail(vz, finalWindow);
    feat.stability_std_z = autosimNanStd(zTail);
    feat.stability_std_vz = autosimNanStd(vzTail);

    [~, feat.stability_std_vz_osc, feat.touchdown_accel_rms] = autosimCalcVzMetrics(vzTail, 0.2);
    feat.contact_count = sum(contact > 0);
    feat.mean_imu_ang_vel = autosimNanMean(imuAngVel);
    feat.max_imu_ang_vel = autosimNanMax(imuAngVel);
    feat.mean_imu_lin_acc = autosimNanMean(imuLinAcc);
    feat.max_imu_lin_acc = autosimNanMax(imuLinAcc);
    feat.max_contact_force = autosimNanMax(contactForce);
    feat.arm_force_imbalance = autosimNanMax([abs(armFL-armFR), abs(armRL-armRR)]);

    if ~isempty(semVec) && ~isempty(cfg) && isfield(cfg, 'ontology') && isfield(cfg.ontology, 'semantic_feature_names')
        semNames = string(cfg.ontology.semantic_feature_names);
        feat.wind_risk_enc = autosimSemGet(semVec, semNames, "wind_risk_enc", 0.0);
        feat.alignment_enc = autosimSemGet(semVec, semNames, "alignment_enc", 0.0);
        feat.visual_enc = autosimSemGet(semVec, semNames, "visual_enc", 0.0);
    end

    ontoFeat = autosimBuildOntologyInputFromFeatureStruct(feat, cfg);
    feat.onto_wind_condition = ontoFeat.onto_wind_condition;
    feat.onto_gust = ontoFeat.onto_gust;
    feat.onto_temporal_pattern = ontoFeat.onto_temporal_pattern;
    feat.onto_drone_state = ontoFeat.onto_drone_state;
    feat.onto_tag_observation = ontoFeat.onto_tag_observation;
end


