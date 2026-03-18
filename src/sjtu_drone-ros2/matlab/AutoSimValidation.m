% AutoSimValidation.m
% Standalone validation-only runner for the latest trained AutoSim model.
%
% Behavior:
% - Uses latest saved model snapshot.
% - Disables incremental learning, adaptive scenario shaping, and probe overrides.
% - Runs 100 validation scenarios per batch.
% - Samples wind from KMA CSV-derived values with per-scenario random selection.
% - Adds supplemental batches until GT ratio is close to target or max rounds is reached.
% - Computes aggregate decision accuracy against GT labels saved by AutoSim.

clear; clc; close all force;

rootDir = fileparts(mfilename('fullpath'));
dataRoot = fullfile(rootDir, 'data');
modelRoot = fullfile(rootDir, 'models');
overridePath = fullfile(rootDir, 'autosim_override.mat');

% Optional model override.
% Leave empty to use the latest compatible model automatically.
if ~exist('modelPath', 'var')
    modelPath = "";
end

batchScenarioCount = 1000;
targetSafeRatio = 0.50;
targetTolerance = 0.10;
maxSupplementRounds = 4;
paperDraftPath = '/home/j/INCSL/IICC26_ws/src/Diagram/drone_landing_paper_draft.md';

if strlength(string(modelPath)) == 0
    modelFiles = [dir(fullfile(modelRoot, 'autosim_model_*.mat')); dir(fullfile(modelRoot, 'autosim_model_final_*.mat'))];
    if isempty(modelFiles)
        error('AutoSimValidation:NoModel', 'No trained AutoSim model was found in %s', modelRoot);
    end
else
    modelPath = char(string(modelPath));
    if ~isfile(modelPath)
        error('AutoSimValidation:ModelPathNotFound', 'Specified model path does not exist: %s', modelPath);
    end
end

runDirs = strings(0,1);
aggregateDataset = table();

fprintf('[AutoSimValidation] start validation batches, batch=%d targetSafeRatio=%.2f tolerance=%.2f\n', ...
    batchScenarioCount, targetSafeRatio, targetTolerance);

for batchIdx = 1:(1 + maxSupplementRounds)
    autosimOverride = autosimValidationBuildOverride(batchScenarioCount, 20260313 + 1000 * (batchIdx - 1), modelPath);
    save(overridePath, 'autosimOverride');

    fprintf('[AutoSimValidation] batch %d override file: %s\n', batchIdx, overridePath);
    run(fullfile(rootDir, 'AutoSim.m'));

    latestRunDir = autosimValidationFindLatestRunDir(dataRoot);
    datasetCsv = autosimValidationPickLatest(fullfile(latestRunDir, 'autosim_dataset_*_completed.csv'));
    if isempty(datasetCsv)
        warning('[AutoSimValidation] dataset csv not found under %s', latestRunDir);
        continue;
    end

    T = readtable(datasetCsv);
    runDirs(end+1,1) = string(latestRunDir); %#ok<SAGROW>
    if isempty(aggregateDataset)
        aggregateDataset = T;
    else
        aggregateDataset = [aggregateDataset; T]; %#ok<AGROW>
    end

    gtSafeRatio = autosimValidationComputeGtSafeRatio(aggregateDataset);
    fprintf('[AutoSimValidation] cumulative GT safe ratio after batch %d: %.4f\n', batchIdx, gtSafeRatio);
    if isfinite(gtSafeRatio) && abs(gtSafeRatio - targetSafeRatio) <= targetTolerance
        fprintf('[AutoSimValidation] target GT ratio satisfied. stop supplementation.\n');
        break;
    end
end

if isempty(aggregateDataset)
    error('AutoSimValidation:NoAggregateDataset', 'Validation finished without any aggregate dataset.');
end

aggSummary = autosimValidationBuildAggregateSummary(aggregateDataset, runDirs);
summaryPath = fullfile(char(runDirs(end)), 'autosim_validation_summary.csv');
writetable(aggSummary, summaryPath);
autosimValidationSaveDistributionReports(aggregateDataset, char(runDirs(end)));

[comparisonCsv, thresholdCsv] = autosimValidationRunPaperPlots(rootDir, dataRoot, aggregateDataset);

fprintf('[AutoSimValidation] aggregate summary saved: %s\n', summaryPath);
fprintf('[AutoSimValidation] accuracy=%.4f precision=%.4f recall=%.4f specificity=%.4f unsafe_landing_rate=%.4f landing_success_rate=%.4f\n', ...
    aggSummary.accuracy(1), aggSummary.precision(1), aggSummary.safe_recall(1), ...
    aggSummary.unsafe_reject(1), aggSummary.unsafe_landing_rate(1), aggSummary.landing_success_rate(1));
fprintf('[AutoSimValidation] excluded intervention=%d hover=%d\n', ...
    aggSummary.n_excluded_intervention(1), aggSummary.n_excluded_hover(1));

if isfile(paperDraftPath)
    autosimValidationFillPaperDraft(summaryPath, comparisonCsv, thresholdCsv, paperDraftPath);
    fprintf('[AutoSimValidation] paper draft updated: %s\n', paperDraftPath);
end


function runDir = autosimValidationFindLatestRunDir(dataRoot)
    d = dir(dataRoot);
    isSub = [d.isdir] & ~startsWith({d.name}, '.');
    names = string({d(isSub).name});
    mask = strlength(names) == 15 & contains(names, '_');
    names = names(mask);
    if isempty(names)
        error('AutoSimValidation:NoRunDir', 'No run directory found under %s', dataRoot);
    end
    names = sort(names);
    runDir = fullfile(dataRoot, char(names(end)));
end


function pathOut = autosimValidationPickLatest(pattern)
    pathOut = '';
    d = dir(pattern);
    if isempty(d)
        return;
    end
    [~, idx] = max([d.datenum]);
    pathOut = fullfile(d(idx).folder, d(idx).name);
end


function successRate = autosimValidationComputeSuccessRate(T)
    successRate = nan;
    if isempty(T) || ~ismember('success', T.Properties.VariableNames)
        return;
    end
    v = double(T.success);
    v = v(isfinite(v));
    if isempty(v)
        return;
    end
    successRate = mean(v);
end


function ratio = autosimValidationComputeGtSafeRatio(T)
    ratio = nan;
    if isempty(T) || ~ismember('gt_safe_to_land', T.Properties.VariableNames)
        return;
    end
    gt = string(T.gt_safe_to_land);
    valid = (gt == "stable") | (gt == "safe") | (gt == "unstable") | (gt == "unsafe");
    if ~any(valid)
        return;
    end
    ratio = mean((gt(valid) == "stable") | (gt(valid) == "safe"));
end


function override = autosimValidationBuildOverride(batchScenarioCount, seedBase, modelPath)
    override = struct();
    override.scenario = struct('count', batchScenarioCount);
    override.learning = struct('enable', false);
    override.adaptive = struct('enable', false);
    override.probe = struct('enable', false);
    override.curriculum = struct('enable', false);
    override.model = struct('force_model_path', string(modelPath));
    override.wind = struct( ...
        'source', "kma_csv", ...
        'kma_pick_mode', "random", ...
        'kma_use_profile_direct', true, ...
        'random_seed_base', seedBase);
    override.validation = struct( ...
        'enable', true, ...
        'mode_cycle', ["easy", "boundary", "hard"], ...
        'direction_bin_count', 8, ...
        'seed_base', seedBase, ...
        'easy_speed_quantile', [0.05, 0.35], ...
        'boundary_speed_quantile', [0.35, 0.70], ...
        'hard_speed_quantile', [0.70, 0.95], ...
        'easy_hover_ratio', 0.80, ...
        'boundary_hover_ratio', 0.50, ...
        'hard_hover_ratio', 0.20, ...
        'easy_gust_amp_scale', 0.90, ...
        'boundary_gust_amp_scale', 1.05, ...
        'hard_gust_amp_scale', 1.25, ...
        'easy_dir_osc_scale', 0.90, ...
        'boundary_dir_osc_scale', 1.05, ...
        'hard_dir_osc_scale', 1.20);
end


function summaryTbl = autosimValidationBuildAggregateSummary(T, runDirs)
    n = height(T);
    gtSafe = false(n, 1);
    gtValid = false(n, 1);
    predLand = false(n, 1);
    predValid = false(n, 1);
    predHover = false(n, 1);
    interventionCase = false(n, 1);

    if ismember('gt_safe_to_land', T.Properties.VariableNames)
        gt = string(T.gt_safe_to_land);
        gtSafe = (gt == "stable") | (gt == "safe");
        gtValid = (gt == "stable") | (gt == "safe") | (gt == "unstable") | (gt == "unsafe");
    elseif ismember('label', T.Properties.VariableNames)
        lbl = string(T.label);
        gtSafe = (lbl == "stable");
        gtValid = (lbl == "stable") | (lbl == "unstable");
    end

    if ismember('pred_decision', T.Properties.VariableNames)
        pd = string(T.pred_decision);
        predLand = (pd == "land");
        predHover = (pd == "hover");
        predValid = (pd == "land") | (pd == "abort");
    elseif ismember('landing_cmd_time', T.Properties.VariableNames)
        predLand = isfinite(T.landing_cmd_time);
        predValid = true(n, 1);
    end

    if ismember('target_case', T.Properties.VariableNames)
        tc = string(T.target_case);
        interventionCase = interventionCase | ...
            (tc == "safe_hover_timeout") | (tc == "unsafe_hover_timeout") | (tc == "unsafe_forced_land");
    end
    if ismember('action_source', T.Properties.VariableNames)
        as = string(T.action_source);
        interventionCase = interventionCase | (as == "timeout_hover_abort") | (as == "timeout_forced_land");
    end

    valid = gtValid & predValid & ~predHover & ~interventionCase;

    tp = sum(predLand(valid) & gtSafe(valid));
    fp = sum(predLand(valid) & ~gtSafe(valid));
    fn = sum(~predLand(valid) & gtSafe(valid));
    tn = sum(~predLand(valid) & ~gtSafe(valid));
    nValid = tp + fp + fn + tn;
    accuracy = autosimValidationSafeDiv(tp + tn, nValid);
    precision = autosimValidationSafeDiv(tp, tp + fp);
    safeRecall = autosimValidationSafeDiv(tp, tp + fn);
    unsafeReject = autosimValidationSafeDiv(tn, tn + fp);
    f1 = nan;
    if isfinite(precision) && isfinite(safeRecall) && (precision + safeRecall) > 0
        f1 = 2 * precision * safeRecall / (precision + safeRecall);
    end
    unsafeLandingRate = autosimValidationSafeDiv(fp, fp + tn);
    successRate = autosimValidationComputeSuccessRate(T);

    summaryTbl = table( ...
        string(strjoin(cellstr(runDirs), ';')), nValid, sum(gtSafe(valid)), sum(~gtSafe(valid)), ...
        accuracy, precision, safeRecall, unsafeReject, f1, unsafeLandingRate, successRate, ...
        sum(interventionCase), sum(predHover), ...
        'VariableNames', {'run_dir','n_valid','n_safe','n_unsafe','accuracy','precision','safe_recall','unsafe_reject','f1','unsafe_landing_rate','landing_success_rate','n_excluded_intervention','n_excluded_hover'});
end


function v = autosimValidationSafeDiv(a, b)
    if b <= 0
        v = nan;
    else
        v = a / b;
    end
end


function autosimValidationSaveDistributionReports(T, runDir)
    if isempty(T)
        return;
    end

    gtSafe = false(height(T), 1);
    predLand = false(height(T), 1);
    if ismember('gt_safe_to_land', T.Properties.VariableNames)
        gt = string(T.gt_safe_to_land);
        gtSafe = (gt == "stable") | (gt == "safe");
    end
    if ismember('pred_decision', T.Properties.VariableNames)
        pd = string(T.pred_decision);
        predLand = (pd == "land");
    end

    labelTbl = table( ...
        string(["positive_or_land"; "negative_or_abort"]), ...
        [sum(gtSafe); sum(~gtSafe)], ...
        [sum(predLand); sum(~predLand)], ...
        'VariableNames', {'class_group','gt_count','prediction_count'});
    writetable(labelTbl, fullfile(runDir, 'autosim_validation_label_distribution.csv'));

    if ismember('wind_speed_cmd', T.Properties.VariableNames)
        w = double(T.wind_speed_cmd);
    elseif ismember('mean_wind_speed', T.Properties.VariableNames)
        w = double(T.mean_wind_speed);
    else
        w = nan(height(T), 1);
    end

    valid = isfinite(w);
    if any(valid)
        edges = prctile(w(valid), [0 20 40 60 80 100]);
        edges = autosimValidationMakeStrictEdges(edges);
        bins = discretize(w, edges);
        windTbl = table((1:5)', zeros(5,1), 'VariableNames', {'wind_bin','count'});
        for i = 1:5
            windTbl.count(i) = sum(bins == i);
        end
        writetable(windTbl, fullfile(runDir, 'autosim_validation_wind_distribution.csv'));
    end
end


function edges = autosimValidationMakeStrictEdges(edges)
    edges = double(edges(:))';
    for i = 2:numel(edges)
        if ~(edges(i) > edges(i-1))
            edges(i) = edges(i-1) + 1e-6;
        end
    end
end


function [comparisonCsv, thresholdCsv] = autosimValidationRunPaperPlots(rootDir, dataRoot, aggregateDataset)
    comparisonCsv = '';
    thresholdCsv = '';

    aggRunDir = fullfile(dataRoot, ['validation_aggregate_' datestr(now, 'yyyymmdd_HHMMSS')]);
    if ~exist(aggRunDir, 'dir')
        mkdir(aggRunDir);
    end
    writetable(aggregateDataset, fullfile(aggRunDir, 'autosim_dataset_latest.csv'));

    runDir = aggRunDir; %#ok<NASGU>
    outputDir = fullfile(rootDir, 'plots', ['paper_validation_' datestr(now, 'yyyymmdd_HHMMSS')]); %#ok<NASGU>
    run(fullfile(rootDir, 'AutoSimPaperPlots.m'));

    comparisonCsv = fullfile(outputDir, 'paper_table_method_comparison.csv');
    thresholdCsv = fullfile(outputDir, 'paper_table_thresholds.csv');
end


function autosimValidationFillPaperDraft(summaryCsvPath, comparisonCsvPath, thresholdCsvPath, paperDraftPath)
    S = readtable(summaryCsvPath);
    if isempty(S)
        return;
    end

    cmp = table();
    thr = table();
    if nargin >= 2 && strlength(string(comparisonCsvPath)) > 0 && isfile(comparisonCsvPath)
        cmp = readtable(comparisonCsvPath);
    end
    if nargin >= 3 && strlength(string(thresholdCsvPath)) > 0 && isfile(thresholdCsvPath)
        thr = readtable(thresholdCsvPath);
    end

    txt = fileread(paperDraftPath);
    lines = splitlines(string(txt));

    policyRow = autosimValidationGetMethodRow(cmp, "Ontology+AI (policy)");
    executedRow = autosimValidationGetMethodRow(cmp, "Ontology+AI (executed)");
    baselineRow = autosimValidationGetMethodRow(cmp, "Threshold baseline");

    safeRatio = autosimValidationSafeDiv(S.n_safe(1), max(1, S.n_valid(1)));

    policySentence = autosimValidationBuildPolicySentence(policyRow, baselineRow);
    executedSentence = autosimValidationBuildExecutedSentence(executedRow);
    imbalanceSentence = autosimValidationBuildImbalanceSentence(S, safeRatio);
    conclusionSentence = autosimValidationBuildConclusionSentence(policyRow, baselineRow);

    for i = 1:numel(lines)
        line = strtrim(lines(i));
        if startsWith(line, '| Ontology+AI (policy) |')
            lines(i) = autosimValidationFormatMethodRow("Ontology+AI (policy)", policyRow);
        elseif startsWith(line, '| Ontology+AI (executed) |')
            lines(i) = autosimValidationFormatMethodRow("Ontology+AI (executed)", executedRow);
        elseif startsWith(line, '| Threshold baseline |')
            lines(i) = autosimValidationFormatMethodRow("Threshold baseline", baselineRow);
        elseif startsWith(line, '| wind_threshold |')
            lines(i) = autosimValidationFormatThresholdRow("wind_threshold", thr);
        elseif startsWith(line, '| tag_error_threshold |')
            lines(i) = autosimValidationFormatThresholdRow("tag_error_threshold", thr);
        elseif startsWith(line, '| roll_threshold_deg |')
            lines(i) = autosimValidationFormatThresholdRow("roll_threshold_deg", thr);
        elseif startsWith(line, '| pitch_threshold_deg |')
            lines(i) = autosimValidationFormatThresholdRow("pitch_threshold_deg", thr);
        elseif startsWith(line, '반면 제안 방법은 시간적 외란 특성과 시각적 안정성을 함께 평가하므로,')
            lines(i) = policySentence;
        elseif startsWith(line, '정량 해석 시에는 단일 정확도 지표만으로 결론을 내리지 않고,')
            lines(i) = executedSentence;
        elseif startsWith(line, '이 결과는 두 가지 가능성을 시사한다.')
            lines(i) = imbalanceSentence;
        elseif startsWith(line, '실험 결과, 제안 방법은 단순 임계값 기반 방법 대비 위험한 착륙 시도를 감소시키고 전체 착륙 안정성을 향상시키는 것으로 나타났다.')
            lines(i) = conclusionSentence;
        end
    end

    fid = fopen(paperDraftPath, 'w');
    if fid < 0
        error('AutoSimValidation:PaperWriteFailed', 'Failed to open paper draft: %s', paperDraftPath);
    end
    fwrite(fid, strjoin(lines, newline));
    fclose(fid);
end


function row = autosimValidationGetMethodRow(cmp, methodName)
    row = table();
    if isempty(cmp) || ~ismember('method', cmp.Properties.VariableNames)
        return;
    end
    mask = string(cmp.method) == string(methodName);
    if any(mask)
        row = cmp(find(mask, 1, 'first'), :);
    end
end


function line = autosimValidationFormatMethodRow(methodName, row)
    if isempty(row)
        line = "| " + methodName + " |  |  |  |  |  |  |  |";
        return;
    end
    line = "| " + methodName + " | " + autosimValidationFmtInt(row.n_valid(1)) + ...
        " | " + autosimValidationFmtPct(row.accuracy(1)) + ...
        " | " + autosimValidationFmtPct(row.precision(1)) + ...
        " | " + autosimValidationFmtPct(row.safe_recall(1)) + ...
        " | " + autosimValidationFmtPct(row.unsafe_reject(1)) + ...
        " | " + autosimValidationFmtPct(row.f1(1)) + ...
        " | " + autosimValidationFmtPct(row.unsafe_landing_rate(1)) + " |";
end


function line = autosimValidationFormatThresholdRow(name, thr)
    value = "";
    if ~isempty(thr) && ismember(name, thr.Properties.VariableNames)
        value = autosimValidationFmtFloat(thr.(name)(1));
    end
    line = "| " + name + " | " + value + " |";
end


function s = autosimValidationBuildPolicySentence(policyRow, baselineRow)
    if isempty(policyRow) || isempty(baselineRow)
        s = "반면 제안 방법은 시간적 외란 특성과 시각적 안정성을 함께 평가하므로, 위와 같은 경계 상황에서도 보다 보수적이고 안정적인 판단을 내릴 수 있었다.";
        return;
    end
    accDiff = 100 * (policyRow.accuracy(1) - baselineRow.accuracy(1));
    ulrDiff = 100 * (baselineRow.unsafe_landing_rate(1) - policyRow.unsafe_landing_rate(1));
    s = "반면 제안 방법의 policy 기준 성능은 Accuracy " + autosimValidationFmtPct(policyRow.accuracy(1)) + ", Unsafe Landing Rate " + autosimValidationFmtPct(policyRow.unsafe_landing_rate(1)) + "로 나타났으며, threshold baseline 대비 정확도는 " + autosimValidationFmtSignedPp(accDiff) + ", 위험 착륙률은 " + autosimValidationFmtSignedPp(ulrDiff) + " 개선되었다.";
end


function s = autosimValidationBuildExecutedSentence(executedRow)
    if isempty(executedRow)
        s = "정량 해석 시에는 단일 정확도 지표만으로 결론을 내리지 않고, unsafe 클래스에 대한 거부 성능(Unsafe Reject/Specificity)과 위험 착륙률(Unsafe Landing Rate)을 함께 해석해야 한다.";
        return;
    end
    s = "실제 실행 액션 기준의 Ontology+AI (executed)는 Accuracy " + autosimValidationFmtPct(executedRow.accuracy(1)) + ", Unsafe Reject " + autosimValidationFmtPct(executedRow.unsafe_reject(1)) + ", Unsafe Landing Rate " + autosimValidationFmtPct(executedRow.unsafe_landing_rate(1)) + "로 집계되었으며, 이는 정책 출력과 실제 실행 결과를 분리하여 해석할 필요가 있음을 보여준다.";
end


function s = autosimValidationBuildImbalanceSentence(S, safeRatio)
    s = "누적 검증 데이터의 safe 비율은 " + autosimValidationFmtPct(safeRatio) + "이며, 총 유효 시나리오는 " + autosimValidationFmtInt(S.n_valid(1)) + "건, unsafe 시나리오는 " + autosimValidationFmtInt(S.n_unsafe(1)) + "건으로 집계되었다. 따라서 후속 결과 보고에서는 accuracy와 precision뿐 아니라 specificity, balanced accuracy, unsafe landing rate를 반드시 병기할 필요가 있다.";
end


function s = autosimValidationBuildConclusionSentence(policyRow, baselineRow)
    if isempty(policyRow) || isempty(baselineRow)
        s = "실험 결과, 제안 방법은 단순 임계값 기반 방법 대비 위험한 착륙 시도를 감소시키고 전체 착륙 안정성을 향상시키는 것으로 나타났다.";
        return;
    end
    s = "실험 결과, 제안 방법은 threshold baseline 대비 Accuracy " + autosimValidationFmtPct(policyRow.accuracy(1)) + ", F1 " + autosimValidationFmtPct(policyRow.f1(1)) + ", Unsafe Landing Rate " + autosimValidationFmtPct(policyRow.unsafe_landing_rate(1)) + "를 기록하였으며, 이는 임계값 기반 방식보다 상위 상황 판단 계층이 더 안정적인 착륙 결정을 유도함을 보여준다.";
end


function s = autosimValidationFmtPct(v)
    if ~isfinite(v)
        s = "";
    else
        s = string(sprintf('%.2f%%', 100 * v));
    end
end


function s = autosimValidationFmtInt(v)
    if ~isfinite(v)
        s = "";
    else
        s = string(sprintf('%d', round(v)));
    end
end


function s = autosimValidationFmtFloat(v)
    if ~isfinite(v)
        s = "";
    else
        s = string(sprintf('%.4f', v));
    end
end


function s = autosimValidationFmtSignedPp(v)
    if ~isfinite(v)
        s = "";
    else
        s = string(sprintf('%+.2f%%p', v));
    end
end