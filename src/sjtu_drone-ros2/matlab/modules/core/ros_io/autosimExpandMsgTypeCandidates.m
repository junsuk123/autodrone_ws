function candidatesOut = autosimExpandMsgTypeCandidates(candidatesIn)
    candidatesOut = strings(0,1);
    if isempty(candidatesIn)
        return;
    end

    for i = 1:numel(candidatesIn)
        t = string(strtrim(candidatesIn(i)));
        if strlength(t) == 0
            continue;
        end

        candidatesOut(end+1,1) = t; %#ok<AGROW>

        if contains(t, "/msg/")
            candidatesOut(end+1,1) = replace(t, "/msg/", "/"); %#ok<AGROW>
        else
            parts = split(t, "/");
            if numel(parts) == 2
                candidatesOut(end+1,1) = parts(1) + "/msg/" + parts(2); %#ok<AGROW>
            end
        end
    end

    candidatesOut = unique(candidatesOut(strlength(candidatesOut) > 0), 'stable');
end


