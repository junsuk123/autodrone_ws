function tf = autosimIsUserInterrupt(ME)
    tf = false;

    try
        id = lower(string(ME.identifier));
    catch
        id = "";
    end

    try
        msg = lower(string(ME.message));
    catch
        msg = "";
    end

    tf = contains(id, "operationterminatedbyuser") || ...
         contains(id, "interrupted") || ...
         contains(msg, "operation terminated by user") || ...
         contains(msg, "interrupt") || ...
         contains(msg, "ctrl+c");

    if tf
        return;
    end

    try
        causes = ME.cause;
        for i = 1:numel(causes)
            if autosimIsUserInterrupt(causes{i})
                tf = true;
                return;
            end
        end
    catch
    end
end


