function autosimReleaseLock(lockPath)
    try
        if isfile(lockPath)
            delete(lockPath);
        end
    catch
    end
end


