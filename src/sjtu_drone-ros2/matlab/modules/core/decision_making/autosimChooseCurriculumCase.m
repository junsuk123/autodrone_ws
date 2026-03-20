function targetCase = autosimChooseCurriculumCase(cfg, datasetState, scenarioId)
    caseNames = string(cfg.curriculum.target_case_names(:));
    if isempty(caseNames)
        targetCase = "safe_land";
        return;
    end

    ratios = double(cfg.curriculum.target_case_ratio(:));
    if numel(ratios) ~= numel(caseNames) || any(~isfinite(ratios))
        ratios = ones(numel(caseNames), 1);
    end
    ratios = max(0.0, ratios);
    if sum(ratios) <= 0
        ratios = ones(numel(caseNames), 1);
    end
    ratios = ratios / sum(ratios);

    if isfield(datasetState, 'caseCounts') && numel(datasetState.caseCounts) == numel(caseNames)
        counts = double(datasetState.caseCounts(:));
        counts(~isfinite(counts)) = 0;
    else
        counts = zeros(numel(caseNames), 1);
    end
    total = sum(counts);

    if total <= 0 && isfield(cfg.curriculum, 'bootstrap_cycle') && cfg.curriculum.bootstrap_cycle
        idx = mod(max(1, scenarioId) - 1, numel(caseNames)) + 1;
        targetCase = caseNames(idx);
        return;
    end

    deficit = ratios * max(1.0, total + 1.0) - counts;
    [~, idx] = max(deficit + 1e-6 * (1:numel(deficit))');
    targetCase = caseNames(idx);
end


