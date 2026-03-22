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
windAcc = abs(autosimField(feat, 'wind_acceleration', 0.0));
rollAbs = autosimField(feat, 'mean_abs_roll_deg', 0.0);
pitchAbs = autosimField(feat, 'mean_abs_pitch_deg', 0.0);
vzAbs = autosimField(feat, 'mean_abs_vz', autosimField(feat, 'max_abs_vz', 0.0));
tagMean = autosimField(feat, 'mean_tag_error', 0.0);
tagMax = autosimField(feat, 'max_tag_error', tagMean);
stdZ = autosimField(feat, 'stability_std_z', 0.0);
stdVz = autosimField(feat, 'stability_std_vz', 0.0);

windRiskEnc = autosimField(feat, 'wind_risk_enc', nan);
alignEnc = autosimField(feat, 'alignment_enc', nan);
visualEnc = autosimField(feat, 'visual_enc', nan);
contextEnc = autosimField(feat, 'context_enc', nan);

nWind = autosimClip01(windMean / max(1e-6, windMaxRef));
nWindVel = autosimClip01(windVel / max(1e-6, windMaxRef));
nWindAcc = autosimClip01(windAcc / max(1e-6, gustDvdtRef));
nGustAmp = autosimClip01((windMax - windMean) / max(1e-6, gustDeltaRef));
nAtt = autosimClip01((rollAbs + pitchAbs) / max(1e-6, 2.0 * attMaxRef));
nVz = autosimClip01(vzAbs / max(1e-6, vzRef));
nTag = autosimClip01(tagMean / max(1e-6, tagErrRef));
nTagSpread = autosimClip01(abs(tagMax - tagMean) / max(1e-6, tagErrRef));
nStability = autosimClip01(0.5 * (stdZ / max(1e-6, stdZRef)) + 0.5 * (stdVz / max(1e-6, stdVzRef)));

windRiskFallback = autosimClip01(0.65 * nWind + 0.35 * nWindAcc);
if ~isfinite(windRiskEnc), windRiskEnc = windRiskFallback; end
if ~isfinite(alignEnc), alignEnc = autosimClip01(1.0 - nTag); end
if ~isfinite(visualEnc), visualEnc = autosimClip01(1.0 - (0.70 * nTag + 0.30 * nTagSpread)); end
if ~isfinite(contextEnc), contextEnc = autosimClip01(0.60 * (1.0 - windRiskEnc) + 0.40 * visualEnc); end

ontoFeat = struct();
ontoFeat.onto_wind_condition = autosimClip01(0.45 * nWind + 0.35 * nWindVel + 0.20 * windRiskEnc);
ontoFeat.onto_gust = autosimClip01(0.55 * nWindAcc + 0.45 * nGustAmp);
ontoFeat.onto_temporal_pattern = autosimClip01(0.60 * ontoFeat.onto_gust + 0.40 * nStability);
ontoFeat.onto_drone_state = autosimClip01(1.0 - (0.55 * nAtt + 0.45 * nVz));
ontoFeat.onto_tag_observation = autosimClip01(0.60 * visualEnc + 0.40 * (1.0 - nTag));
ontoFeat.onto_landing_context = autosimClip01(0.45 * contextEnc + 0.25 * alignEnc + 0.15 * ontoFeat.onto_drone_state + 0.15 * (1.0 - windRiskEnc));
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