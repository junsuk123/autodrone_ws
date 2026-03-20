function v = autosimSemGet(semVec, semNames, key, fallback)
    idx = find(semNames == key, 1);
    if isempty(idx) || idx > numel(semVec) || ~isfinite(semVec(idx))
        v = fallback;
    else
        v = double(semVec(idx));
    end
end


