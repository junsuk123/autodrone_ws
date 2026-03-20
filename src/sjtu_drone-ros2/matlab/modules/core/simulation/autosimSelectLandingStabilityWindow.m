function [zStable, vzStable] = autosimSelectLandingStabilityWindow(z, vz, stateVal, cfg)
    dt = max(cfg.scenario.sample_period_sec, 1e-3);
    tailN = max(3, round(2.4 / dt));

    zStable = autosimTail(z, tailN);
    vzStable = autosimTail(vz, tailN);

    landedIdx = find(isfinite(stateVal) & (stateVal == cfg.thresholds.land_state_value));
    if numel(landedIdx) < 3
        return;
    end

    % Use the final contiguous landed segment to avoid including descent dynamics.
    segStart = 1;
    d = diff(landedIdx);
    jumpIdx = find(d > 1, 1, 'last');
    if ~isempty(jumpIdx)
        segStart = jumpIdx + 1;
    end
    landedTail = landedIdx(segStart:end);
    if numel(landedTail) < 3
        return;
    end

    settleSkipN = round(0.8 / dt);
    if numel(landedTail) > (settleSkipN + 2)
        landedTail = landedTail((settleSkipN + 1):end);
    end

    useN = min(numel(landedTail), tailN);
    idxUse = landedTail(end-useN+1:end);

    zSel = z(idxUse);
    vzSel = vz(idxUse);
    zSel = zSel(isfinite(zSel));
    vzSel = vzSel(isfinite(vzSel));
    if numel(zSel) >= 3
        zStable = zSel;
    end
    if numel(vzSel) >= 3
        vzStable = vzSel;
    end
end


