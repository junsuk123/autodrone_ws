function v = autosimVizField(S, fieldName, fallback)
    v = fallback;
    if nargin < 3
        fallback = nan;
        v = fallback;
    end
    if isstruct(S) && isfield(S, fieldName)
        v = S.(fieldName);
    end
end


