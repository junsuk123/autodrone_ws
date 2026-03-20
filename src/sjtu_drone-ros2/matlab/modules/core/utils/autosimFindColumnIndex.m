function idx = autosimFindColumnIndex(varNames, preferredName, fallbackNames)
    idx = -1;
    if nargin < 3
        fallbackNames = {};
    end

    names = string(varNames);
    if ~isempty(preferredName)
        hit = find(lower(names) == lower(string(preferredName)), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end

    for i = 1:numel(fallbackNames)
        hit = find(lower(names) == lower(string(fallbackNames{i})), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end
end


