function s = autosimWindStringField(st, fieldName, fallback)
    s = string(fallback);
    if isfield(st, fieldName)
        s = string(st.(fieldName));
    end
end


