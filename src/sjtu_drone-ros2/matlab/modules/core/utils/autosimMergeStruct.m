function base = autosimMergeStruct(base, override)
    if ~isstruct(override)
        return;
    end

    fn = fieldnames(override);
    for i = 1:numel(fn)
        key = fn{i};
        if isfield(base, key) && isstruct(base.(key)) && isstruct(override.(key))
            base.(key) = autosimMergeStruct(base.(key), override.(key));
        else
            base.(key) = override.(key);
        end
    end
end


