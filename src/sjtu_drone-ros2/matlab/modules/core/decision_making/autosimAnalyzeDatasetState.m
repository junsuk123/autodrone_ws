function state = autosimAnalyzeDatasetState(cfg, results, traceStore, learningHistory)
    if nargin < 1 || isempty(results)
        summaryTbl = table();
    else
        summaryTbl = autosimSummaryTable(results);
    end

    dEval = autosimEvaluateDecisionMetrics(summaryTbl);

    state = struct();
    state.nStable = 0;
    state.nUnstable = 0;
    state.stableRatio = 0.5;
    state.decisionAccuracy = autosimClampNaN(dEval.accuracy, 0.0);
    state.unsafeLandingRate = autosimClampNaN(dEval.unsafe_landing_rate, 0.0);
    state.falseNegativeRate = autosimSafeDivide(dEval.fn, dEval.fn + dEval.tp);
    state.boundarySampleRatio = 0.0;
    state.recentUnstableRatio = 0.5;
    state.tp = dEval.tp;
    state.fp = dEval.fp;
    state.fn = dEval.fn;
    state.tn = dEval.tn;
    state.nValid = dEval.n_valid;
    state.nScenarios = 0;
    state.nProbeEpisodes = 0;
    state.nUnsafeProbeEpisodes = 0;
    state.recentProbeRatio = 0.0;
    state.recentUnsafeProbeRatio = 0.0;
    state.recentExploitProbeRatio = 0.0;
    state.recentBoundaryProbeRatio = 0.0;
    state.recentHardNegativeProbeRatio = 0.0;
    state.caseNames = ["safe_land", "safe_hover_timeout", "unsafe_hover_timeout", "unsafe_forced_land"];
    state.caseCounts = zeros(1, 4);
    state.nCaseLabeled = 0;

    if isempty(summaryTbl)
        return;
    end

    state.nScenarios = height(summaryTbl);

    if ismember('label', summaryTbl.Properties.VariableNames)
        lbl = string(summaryTbl.label);
        state.nStable = sum(lbl == "stable");
        state.nUnstable = sum(lbl == "unstable");
        nAll = state.nStable + state.nUnstable;
        if nAll > 0
            state.stableRatio = state.nStable / nAll;
        end

        w = min(20, numel(lbl));
        if w > 0
            recent = lbl(max(1, end-w+1):end);
            state.recentUnstableRatio = sum(recent == "unstable") / numel(recent);
        end
    end

    if ismember('probe_episode', summaryTbl.Properties.VariableNames)
        probeEpisode = logical(autosimToNumeric(summaryTbl.probe_episode));
        state.nProbeEpisodes = sum(probeEpisode);

        unsafeLabel = false(height(summaryTbl), 1);
        if ismember('gt_safe_to_land', summaryTbl.Properties.VariableNames)
            gtLbl = string(summaryTbl.gt_safe_to_land);
            unsafeLabel = (gtLbl == "unstable") | (gtLbl == "unsafe");
        elseif ismember('label', summaryTbl.Properties.VariableNames)
            unsafeLabel = string(summaryTbl.label) == "unstable";
        end
        state.nUnsafeProbeEpisodes = sum(probeEpisode & unsafeLabel);

        recentWindow = 10;
        if isfield(cfg, 'adaptive') && isfield(cfg.adaptive, 'recent_window') && isfinite(cfg.adaptive.recent_window)
            recentWindow = max(3, round(cfg.adaptive.recent_window));
        end
        wProbe = min(recentWindow, height(summaryTbl));
        if wProbe > 0
            recentIdx = max(1, height(summaryTbl) - wProbe + 1):height(summaryTbl);
            recentProbe = probeEpisode(recentIdx);
            recentUnsafe = unsafeLabel(recentIdx);
            state.recentProbeRatio = sum(recentProbe) / numel(recentProbe);
            state.recentUnsafeProbeRatio = sum(recentProbe & recentUnsafe) / numel(recentProbe);

            if ismember('scenario_policy', summaryTbl.Properties.VariableNames)
                recentPolicy = string(summaryTbl.scenario_policy(recentIdx));
                state.recentExploitProbeRatio = autosimPolicyProbeRatio(recentProbe, recentPolicy, "exploit");
                state.recentBoundaryProbeRatio = autosimPolicyProbeRatio(recentProbe, recentPolicy, "boundary_validation");
                state.recentHardNegativeProbeRatio = autosimPolicyProbeRatio(recentProbe, recentPolicy, "hard_negative");
            end
        end
    end

    if ismember('target_case', summaryTbl.Properties.VariableNames)
        tc = string(summaryTbl.target_case);
        tc = lower(strtrim(tc));
        caseNames = lower(string(state.caseNames(:)));
        counts = zeros(size(caseNames));
        for ci = 1:numel(caseNames)
            counts(ci) = sum(tc == caseNames(ci));
        end
        state.caseCounts = counts(:).';
        state.nCaseLabeled = sum(state.caseCounts);
    end

    boundaryBySemantic = false(height(summaryTbl), 1);
    boundaryByProb = false(height(summaryTbl), 1);
    if ~isempty(traceStore) && ismember('scenario_id', traceStore.Properties.VariableNames)
        if ismember('sem_alignment_enc', traceStore.Properties.VariableNames)
            ids = summaryTbl.scenario_id;
            semMeans = nan(height(summaryTbl), 1);
            for i = 1:height(summaryTbl)
                sid = ids(i);
                v = traceStore.sem_alignment_enc(traceStore.scenario_id == sid);
                v = v(isfinite(v));
                if ~isempty(v)
                    semMeans(i) = mean(v);
                end
            end
            boundaryBySemantic = isfinite(semMeans) & (abs(semMeans - 0.5) <= 0.10);
        end

        if ismember('pred_stable_prob', traceStore.Properties.VariableNames)
            ids = summaryTbl.scenario_id;
            probMeans = nan(height(summaryTbl), 1);
            for i = 1:height(summaryTbl)
                sid = ids(i);
                v = traceStore.pred_stable_prob(traceStore.scenario_id == sid);
                v = v(isfinite(v));
                if ~isempty(v)
                    probMeans(i) = mean(v);
                end
            end
            boundaryByProb = isfinite(probMeans) & (abs(probMeans - 0.5) <= 0.10);
        end
    end

    boundaryMask = boundaryBySemantic | boundaryByProb;
    if any(boundaryMask)
        state.boundarySampleRatio = sum(boundaryMask) / numel(boundaryMask);
    elseif state.nValid > 0
        % Fallback: no trace boundary signal, approximate by uncertain outcomes.
        state.boundarySampleRatio = min(1.0, (state.fp + state.fn) / state.nValid);
    end

    if nargin >= 3 && ~isempty(learningHistory) && ismember('stable_ratio', learningHistory.Properties.VariableNames)
        r = learningHistory.stable_ratio(end);
        if isfinite(r)
            state.stableRatio = r;
        end
    end
end


