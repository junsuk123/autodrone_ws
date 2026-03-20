function v = autosimVizSemScore(semVec, idx, semantic, fieldName, fallback)
    v = fallback;
    if numel(semVec) >= idx
        cand = semVec(idx);
        if isfinite(cand)
            v = cand;
            return;
        end
    end
    if isstruct(semantic) && isfield(semantic, fieldName)
        cand = semantic.(fieldName);
        if isnumeric(cand) && isfinite(cand)
            v = cand;
        end
    end
end


