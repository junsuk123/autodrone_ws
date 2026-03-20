function windCtx = autosimBuildScenarioWindContext(summaryTbl, traceStore, scenarioIds)
    n = numel(scenarioIds);
    windCtx = struct();
    windCtx.speed_cmd = nan(n, 1);
    windCtx.mean_speed = nan(n, 1);
    windCtx.max_speed = nan(n, 1);
    windCtx.direction_deg = nan(n, 1);
    windCtx.characteristic = repmat("unknown", n, 1);

    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    [isMatched, loc] = ismember(scenarioIds, summaryTbl.scenario_id);
    matchedIdx = find(isMatched);
    if any(isMatched)
        matchedLoc = loc(isMatched);
        if ismember('wind_speed_cmd', summaryTbl.Properties.VariableNames)
            windCtx.speed_cmd(matchedIdx) = double(summaryTbl.wind_speed_cmd(matchedLoc));
        end
        if ismember('mean_wind_speed', summaryTbl.Properties.VariableNames)
            windCtx.mean_speed(matchedIdx) = double(summaryTbl.mean_wind_speed(matchedLoc));
        end
        if ismember('max_wind_speed', summaryTbl.Properties.VariableNames)
            windCtx.max_speed(matchedIdx) = double(summaryTbl.max_wind_speed(matchedLoc));
        end
        if ismember('wind_dir_cmd', summaryTbl.Properties.VariableNames)
            windCtx.direction_deg(matchedIdx) = double(summaryTbl.wind_dir_cmd(matchedLoc));
        end
        if ismember('semantic_environment', summaryTbl.Properties.VariableNames)
            baseLabels = string(summaryTbl.semantic_environment(matchedLoc));
            validLabelMask = strlength(strtrim(baseLabels)) > 0 & baseLabels ~= "unknown";
            windCtx.characteristic(matchedIdx(validLabelMask)) = baseLabels(validLabelMask);
        end
    end

    if isempty(traceStore) || ~ismember('scenario_id', traceStore.Properties.VariableNames)
        return;
    end

    hasWindSpeed = ismember('wind_speed', traceStore.Properties.VariableNames);
    hasCmdSpeed = ismember('wind_cmd_speed', traceStore.Properties.VariableNames);
    hasCmdDir = ismember('wind_cmd_dir', traceStore.Properties.VariableNames);
    hasWindRisk = ismember('semantic_wind_risk', traceStore.Properties.VariableNames);
    hasEnvironment = ismember('semantic_environment', traceStore.Properties.VariableNames);

    for i = 1:n
        rows = (traceStore.scenario_id == scenarioIds(i));
        if ~any(rows)
            continue;
        end

        if hasWindSpeed && (~isfinite(windCtx.mean_speed(i)) || ~isfinite(windCtx.max_speed(i)))
            windVals = double(traceStore.wind_speed(rows));
            windVals = windVals(isfinite(windVals));
            if ~isempty(windVals)
                if ~isfinite(windCtx.mean_speed(i))
                    windCtx.mean_speed(i) = mean(windVals);
                end
                if ~isfinite(windCtx.max_speed(i))
                    windCtx.max_speed(i) = max(windVals);
                end
            end
        end

        if hasCmdSpeed && ~isfinite(windCtx.speed_cmd(i))
            cmdSpeedVals = double(traceStore.wind_cmd_speed(rows));
            cmdSpeedVals = cmdSpeedVals(isfinite(cmdSpeedVals));
            if ~isempty(cmdSpeedVals)
                windCtx.speed_cmd(i) = cmdSpeedVals(find(isfinite(cmdSpeedVals), 1, 'last'));
            end
        end

        if hasCmdDir && ~isfinite(windCtx.direction_deg(i))
            dirVals = double(traceStore.wind_cmd_dir(rows));
            dirVals = dirVals(isfinite(dirVals));
            if ~isempty(dirVals)
                windCtx.direction_deg(i) = dirVals(find(isfinite(dirVals), 1, 'last'));
            end
        end

        label = "unknown";
        if hasWindRisk
            label = autosimLastMeaningfulLabel(traceStore.semantic_wind_risk(rows));
        end
        if (strlength(label) == 0 || label == "unknown") && hasEnvironment
            label = autosimLastMeaningfulLabel(traceStore.semantic_environment(rows));
        end
        if strlength(label) > 0 && label ~= "unknown"
            windCtx.characteristic(i) = label;
        end
    end
end


