function tf = autosimLooksLikeMissingMatlabMsgSupport(errText)
    tf = false;
    s = lower(string(errText));
    if strlength(s) == 0
        return;
    end

    tf = contains(s, "message type") || contains(s, "unknown message") || contains(s, "not found") || ...
        contains(s, "cannot resolve") || contains(s, "interface") || contains(s, "does not exist");
end


