function autosimMaximizeFigure(fig)
    try
        set(fig, 'WindowState', 'maximized');
    catch
        try
            scr = get(0, 'ScreenSize');
            set(fig, 'Units', 'pixels', 'Position', scr);
        catch
        end
    end
end


