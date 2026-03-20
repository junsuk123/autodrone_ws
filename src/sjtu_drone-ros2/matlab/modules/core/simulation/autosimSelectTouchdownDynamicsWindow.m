function vzTouch = autosimSelectTouchdownDynamicsWindow(vz, stateVal, cfg)
    dt = max(cfg.scenario.sample_period_sec, 1e-3);
    preN = max(3, round(1.2 / dt));
    postN = max(4, round(1.6 / dt));
    n = numel(vz);

    vzTouch = autosimTail(vz, preN + postN);
    if n < 3 || numel(stateVal) ~= n
        return;
    end

    landedIdx = find(isfinite(stateVal) & (stateVal == cfg.thresholds.land_state_value));
    if isempty(landedIdx)
        return;
    end

    segStart = 1;
    d = diff(landedIdx);
    jumpIdx = find(d > 1, 1, 'last');
    if ~isempty(jumpIdx)
        segStart = jumpIdx + 1;
    end
    landedTail = landedIdx(segStart:end);
    if isempty(landedTail)
        return;
    end

    tdIdx = landedTail(1);
    i0 = max(1, tdIdx - preN);
    i1 = min(n, tdIdx + postN);
    vzSel = vz(i0:i1);
    vzSel = vzSel(isfinite(vzSel));
    if numel(vzSel) >= 4
        vzTouch = vzSel;
    end
end


