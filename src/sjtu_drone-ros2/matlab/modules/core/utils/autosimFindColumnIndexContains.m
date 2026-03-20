function idx = autosimFindColumnIndexContains(varNames, tokens)
    idx = -1;
    if nargin < 2 || isempty(tokens)
        return;
    end

    names = lower(string(varNames));
    for i = 1:numel(tokens)
        tok = lower(string(tokens{i}));
        hit = find(contains(names, tok), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end
end


