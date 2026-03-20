function autosimHandleStopFigureClose(src, reason)
    autosimRequestStop(reason);
    try
        delete(src);
    catch
    end
end


