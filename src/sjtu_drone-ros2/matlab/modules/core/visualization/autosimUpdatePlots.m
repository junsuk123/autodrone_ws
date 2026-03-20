function plotState = autosimUpdatePlots(plotState, results, learningHistory)
    if isempty(plotState) || ~isfield(plotState, 'fig') || ~isgraphics(plotState.fig)
        return;
    end

    sIdx = numel(results);
    summaryTbl = autosimSummaryTable(results);
    dEval = autosimEvaluateDecisionMetrics(summaryTbl);
    dExec = autosimEvaluateDecisionMetrics(autosimBuildDecisionTable(summaryTbl, 'executed_action'));

    unsafeGtCount = 0;
    probeCount = 0;
    if ~isempty(summaryTbl)
        if ismember('gt_safe_to_land', summaryTbl.Properties.VariableNames)
            gtLbl = string(summaryTbl.gt_safe_to_land);
            unsafeGtCount = sum((gtLbl == "unstable") | (gtLbl == "unsafe"));
        elseif ismember('label', summaryTbl.Properties.VariableNames)
            unsafeGtCount = sum(string(summaryTbl.label) == "unstable");
        end
        if ismember('probe_episode', summaryTbl.Properties.VariableNames)
            probeCount = sum(logical(autosimToNumeric(summaryTbl.probe_episode)));
        end
    end

    if dEval.n_valid > 0
        addpoints(plotState.accLine, sIdx, dEval.accuracy);
        addpoints(plotState.precLine, sIdx, dEval.precision);
        addpoints(plotState.recLine, sIdx, dEval.recall);
        addpoints(plotState.unsafeLine, sIdx, dEval.unsafe_landing_rate);
        if dExec.n_valid > 0
            addpoints(plotState.execUnsafeLine, sIdx, dExec.unsafe_landing_rate);
        end

        addpoints(plotState.tpLine, sIdx, dEval.tp);
        addpoints(plotState.fpLine, sIdx, dEval.fp);
        addpoints(plotState.fnLine, sIdx, dEval.fn);
        addpoints(plotState.tnLine, sIdx, dEval.tn);
        addpoints(plotState.unsafeGtLine, sIdx, unsafeGtCount);
        addpoints(plotState.probeLine, sIdx, probeCount);
    end

    drawnow limitrate nocallbacks;
end


