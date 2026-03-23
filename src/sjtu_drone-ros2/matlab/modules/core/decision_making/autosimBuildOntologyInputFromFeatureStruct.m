function ontoFeat = autosimBuildOntologyInputFromFeatureStruct(feat, cfg)
% Build ontology-entity AI input features from online observable feature struct.

windMaxRef = autosimCfgScalar(cfg, {'wind', 'speed_max'}, 3.0);
gustDvdtRef = autosimCfgScalar(cfg, {'ontology', 'gust_dvdt_high'}, 3.0);
gustDeltaRef = autosimCfgScalar(cfg, {'ontology', 'gust_delta_high'}, 2.0);
attMaxRef = autosimCfgScalar(cfg, {'thresholds', 'final_attitude_max_deg'}, 35.0);
tagErrRef = autosimCfgScalar(cfg, {'thresholds', 'final_tag_error_max'}, 0.9);
vzRef = autosimCfgScalar(cfg, {'agent', 'no_model_max_abs_vz'}, 0.9);
stdZRef = autosimCfgScalar(cfg, {'thresholds', 'final_stability_std_z_max'}, 0.45);
stdVzRef = autosimCfgScalar(cfg, {'thresholds', 'final_stability_std_vz_max'}, 0.55);

windMean = autosimField(feat, 'mean_wind_speed', 0.0);
windMax = autosimField(feat, 'max_wind_speed', windMean);
windVel = autosimField(feat, 'wind_velocity', windMean);
windVelX = autosimField(feat, 'wind_velocity_x', windVel);
windVelY = autosimField(feat, 'wind_velocity_y', 0.0);
windAcc = abs(autosimField(feat, 'wind_acceleration', 0.0));
windAccX = autosimField(feat, 'wind_acceleration_x', windAcc);
windAccY = autosimField(feat, 'wind_acceleration_y', 0.0);
rollAbs = autosimField(feat, 'mean_abs_roll_deg', 0.0);
pitchAbs = autosimField(feat, 'mean_abs_pitch_deg', 0.0);
vzAbs = autosimField(feat, 'mean_abs_vz', autosimField(feat, 'max_abs_vz', 0.0));
tagMean = autosimField(feat, 'mean_tag_error', 0.0);
tagMax = autosimField(feat, 'max_tag_error', tagMean);
stdZ = autosimField(feat, 'stability_std_z', 0.0);
stdVz = autosimField(feat, 'stability_std_vz', 0.0);

windRiskEnc = autosimField(feat, 'wind_risk_enc', nan);
windBodyRiskEnc = autosimField(feat, 'wind_body_risk_enc', nan);
windGustRiskEnc = autosimField(feat, 'wind_gust_risk_enc', nan);
windDirChangeRiskEnc = autosimField(feat, 'wind_dir_change_risk_enc', nan);
alignEnc = autosimField(feat, 'alignment_enc', nan);
visualEnc = autosimField(feat, 'visual_enc', nan);

nWind = autosimClip01(windMean / max(1e-6, windMaxRef));
nWindVel = autosimClip01(windVel / max(1e-6, windMaxRef));
nWindVelComp = autosimClip01(max(abs(windVelX), abs(windVelY)) / max(1e-6, windMaxRef));
nWindAcc = autosimClip01(windAcc / max(1e-6, gustDvdtRef));
nWindAccComp = autosimClip01(max(abs(windAccX), abs(windAccY)) / max(1e-6, gustDvdtRef));
nGustAmp = autosimClip01((windMax - windMean) / max(1e-6, gustDeltaRef));
nAtt = autosimClip01((rollAbs + pitchAbs) / max(1e-6, 2.0 * attMaxRef));
nVz = autosimClip01(vzAbs / max(1e-6, vzRef));
nTag = autosimClip01(tagMean / max(1e-6, tagErrRef));
nTagSpread = autosimClip01(abs(tagMax - tagMean) / max(1e-6, tagErrRef));
nStability = autosimClip01(0.5 * (stdZ / max(1e-6, stdZRef)) + 0.5 * (stdVz / max(1e-6, stdVzRef)));

windRiskFallback = autosimClip01(0.35 * nWind + 0.30 * nWindVel + 0.15 * nWindVelComp + 0.20 * max(nWindAcc, nWindAccComp));
if ~isfinite(windRiskEnc), windRiskEnc = windRiskFallback; end
if ~isfinite(windBodyRiskEnc), windBodyRiskEnc = autosimClip01(max(nWindVel, nWindVelComp)); end
if ~isfinite(windGustRiskEnc), windGustRiskEnc = autosimClip01(max(nWindAcc, nWindAccComp)); end
if ~isfinite(windDirChangeRiskEnc), windDirChangeRiskEnc = autosimClip01(nGustAmp); end
if ~isfinite(alignEnc), alignEnc = autosimClip01(1.0 - nTag); end
if ~isfinite(visualEnc), visualEnc = autosimClip01(1.0 - (0.70 * nTag + 0.30 * nTagSpread)); end

ontoFeat = struct();
ontoFeat.onto_wind_condition = autosimClip01(0.35 * nWind + 0.30 * nWindVel + 0.15 * nWindVelComp + 0.20 * windRiskEnc);
ontoFeat.onto_gust = autosimClip01(0.40 * nWindAcc + 0.20 * nWindAccComp + 0.40 * nGustAmp);
ontoFeat.onto_temporal_pattern = autosimClip01(0.60 * ontoFeat.onto_gust + 0.40 * nStability);
ontoFeat.onto_drone_state = autosimClip01(1.0 - (0.55 * nAtt + 0.45 * nVz));
ontoFeat.onto_tag_observation = autosimClip01(0.60 * visualEnc + 0.40 * (1.0 - nTag));
ontoFeat.onto_wind_body_risk = autosimClip01(windBodyRiskEnc);
ontoFeat.onto_wind_accel_risk = autosimClip01(windGustRiskEnc);
ontoFeat.onto_wind_dir_change_risk = autosimClip01(windDirChangeRiskEnc);
end

function v = autosimField(s, name, fallback)
if isstruct(s) && isfield(s, name)
    vv = double(s.(name));
    if isfinite(vv)
        v = vv;
        return;
    end
end
v = fallback;
end

function v = autosimCfgScalar(cfg, pathCells, fallback)
v = fallback;
try
    cur = cfg;
    for i = 1:numel(pathCells)
        key = pathCells{i};
        if ~isstruct(cur) || ~isfield(cur, key)
            return;
        end
        cur = cur.(key);
    end
    if isfinite(double(cur))
        v = double(cur);
    end
catch
    v = fallback;
end
end

function y = autosimClip01(x)
y = x;
if ~isfinite(y)
    y = 0.0;
end
y = max(0.0, min(1.0, y));
end