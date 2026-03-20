function autosimPrintStats(results, idx, totalCount, learnInfo)
    labels = strings(numel(results), 1);
    for i = 1:numel(results)
        labels(i) = string(results(i).label);
    end

    nStable = sum(labels == "stable");
    nUnstable = sum(labels == "unstable");
    nValid = nStable + nUnstable;

    if nValid > 0
        fprintf('[AUTOSIM] Stats %d/%d: stable=%d (%.1f%%), unstable=%d (%.1f%%)\n', ...
            idx, totalCount, nStable, 100*nStable/nValid, nUnstable, 100*nUnstable/nValid);
    else
        fprintf('[AUTOSIM] Stats %d/%d: no valid labels\n', idx, totalCount);
    end

    if learnInfo.model_updated
        fprintf('[AUTOSIM] Model updated with n=%d, stableRatio=%.2f, path=%s\n', ...
            learnInfo.n_train, learnInfo.stable_ratio, learnInfo.model_path);
    else
        if isfield(learnInfo, 'skip_reason') && strlength(string(learnInfo.skip_reason)) > 0
            fprintf('[AUTOSIM] Model update skipped (%s): n=%d, stable=%d, unstable=%d\n', ...
                string(learnInfo.skip_reason), learnInfo.n_train, learnInfo.n_stable, learnInfo.n_unstable);
        else
            fprintf('[AUTOSIM] Model not updated yet (waiting for labeled scenario data).\n');
        end
    end
end


