function x = autosimTryReceive(sub, timeout)
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end


