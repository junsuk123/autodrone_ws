function T = autosimEnsureOntologyFeatureColumns(T, cfg)
% Ensure ontology-entity model input columns exist on table T.
% Features are normalized to [0, 1] for stable training/inference behavior.

if isempty(T)
    return;
end

required = [ ...
    "onto_wind_condition", "onto_gust", "onto_temporal_pattern", ...
    "onto_drone_state", "onto_tag_observation", "onto_landing_context" ...
];

vars = string(T.Properties.VariableNames);
if all(ismember(required, vars))
    return;
end

n = height(T);
windMaxRef = autosimSafeCfgScalar(cfg, {'wind', 'speed_max'}, 3.0);
gustDvdtRef = autosimSafeCfgScalar(cfg, {'ontology', 'gust_dvdt_high'}, 3.0);
gustDeltaRef = autosimSafeCfgScalar(cfg, {'ontology', 'gust_delta_high'}, 2.0);
attMaxRef = autosimSafeCfgScalar(cfg, {'thresholds', 'final_attitude_max_deg'}, 35.0);
tagErrRef = autosimSafeCfgScalar(cfg, {'thresholds', 'final_tag_error_max'}, 0.9);
vzRef = autosimSafeCfgScalar(cfg, {'agent', 'no_model_max_abs_vz'}, 0.9);
stdZRef = autosimSafeCfgScalar(cfg, {'thresholds', 'final_stability_std_z_max'}, 0.45);
stdVzRef = autosimSafeCfgScalar(cfg, {'thresholds', 'final_stability_std_vz_max'}, 0.55);

windMean = autosimTblCol(T, 'mean_wind_speed', autosimTblCol(T, 'wind_speed_cmd', zeros(n, 1)));
windMax = autosimTblCol(T, 'max_wind_speed', windMean);
windVel = autosimTblCol(T, 'wind_velocity', windMean);
windAcc = abs(autosimTblCol(T, 'wind_acceleration', zeros(n, 1)));

rollAbs = autosimTblCol(T, 'mean_abs_roll_deg', autosimTblCol(T, 'final_abs_roll_deg', zeros(n, 1)));
pitchAbs = autosimTblCol(T, 'mean_abs_pitch_deg', autosimTblCol(T, 'final_abs_pitch_deg', zeros(n, 1)));
vzAbs = autosimTblCol(T, 'mean_abs_vz', autosimTblCol(T, 'max_abs_vz', zeros(n, 1)));

tagMean = autosimTblCol(T, 'mean_tag_error', autosimTblCol(T, 'final_tag_error', zeros(n, 1)));
tagMax = autosimTblCol(T, 'max_tag_error', tagMean);

stdZ = autosimTblCol(T, 'stability_std_z', zeros(n, 1));
stdVz = autosimTblCol(T, 'stability_std_vz', zeros(n, 1));

windRiskEnc = autosimTblCol(T, 'wind_risk_enc', nan(n, 1));
alignEnc = autosimTblCol(T, 'alignment_enc', nan(n, 1));
visualEnc = autosimTblCol(T, 'visual_enc', nan(n, 1));
contextEnc = autosimTblCol(T, 'context_enc', nan(n, 1));

if any(~isfinite(visualEnc)) && ismember('semantic_visual_state', T.Properties.VariableNames)
    visualEnc(~isfinite(visualEnc)) = autosimEncodeVisualState(T.semantic_visual_state(~isfinite(visualEnc)));
end
if any(~isfinite(contextEnc)) && ismember('semantic_landing_context', T.Properties.VariableNames)
    contextEnc(~isfinite(contextEnc)) = autosimEncodeLandingContext(T.semantic_landing_context(~isfinite(contextEnc)));
end
if any(~isfinite(alignEnc)) && ismember('semantic_relation', T.Properties.VariableNames)
    alignEnc(~isfinite(alignEnc)) = autosimEncodeSemanticRelation(T.semantic_relation(~isfinite(alignEnc)));
end

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
windRisk = autosimFillNaN(windRiskEnc, windRiskFallback);
align = autosimFillNaN(alignEnc, autosimClip01(1.0 - nTag));
visual = autosimFillNaN(visualEnc, autosimClip01(1.0 - (0.70 * nTag + 0.30 * nTagSpread)));
context = autosimFillNaN(contextEnc, autosimClip01(0.60 * (1.0 - windRisk) + 0.40 * visual));

ontoWindCondition = autosimClip01(0.45 * nWind + 0.35 * nWindVel + 0.20 * windRisk);
ontoGust = autosimClip01(0.55 * nWindAcc + 0.45 * nGustAmp);
ontoTemporalPattern = autosimClip01(0.60 * ontoGust + 0.40 * nStability);
ontoDroneState = autosimClip01(1.0 - (0.55 * nAtt + 0.45 * nVz));
ontoTagObservation = autosimClip01(0.60 * visual + 0.40 * (1.0 - nTag));
ontoLandingContext = autosimClip01(0.45 * context + 0.25 * align + 0.15 * ontoDroneState + 0.15 * (1.0 - windRisk));

T.onto_wind_condition = ontoWindCondition;
T.onto_gust = ontoGust;
T.onto_temporal_pattern = ontoTemporalPattern;
T.onto_drone_state = ontoDroneState;
T.onto_tag_observation = ontoTagObservation;
T.onto_landing_context = ontoLandingContext;
end

function v = autosimTblCol(T, name, fallback)
if ismember(name, T.Properties.VariableNames)
    col = T.(name);
    if isnumeric(col) || islogical(col)
        v = double(col);
    else
        v = str2double(string(col));
    end
    v(~isfinite(v)) = nan;
else
    v = fallback;
end
end

function v = autosimSafeCfgScalar(cfg, pathCells, fallback)
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

function y = autosimEncodeVisualState(x)
s = lower(strtrim(string(x)));
y = nan(size(s));
y(s == "stable") = 1.0;
y(s == "unstable") = 0.0;
end

function y = autosimEncodeLandingContext(x)
s = lower(strtrim(string(x)));
y = nan(size(s));
y(s == "safe") = 1.0;
y(s == "caution") = 0.5;
y(s == "unsafe") = 0.0;
end

function y = autosimEncodeSemanticRelation(x)
s = lower(strtrim(string(x)));
y = nan(size(s));
y(s == "supportive") = 1.0;
y(s == "conditional") = 0.5;
y(s == "conflicting") = 0.0;
end

function y = autosimFillNaN(x, fallback)
y = x;
mask = ~isfinite(y);
y(mask) = fallback(mask);
end

function y = autosimClip01(x)
y = x;
y(~isfinite(y)) = 0.0;
y = max(0.0, min(1.0, y));
end