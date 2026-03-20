function autosimPlaceFigureRight(fig, sizeFracWH, centerFracXY)
    if nargin < 2 || numel(sizeFracWH) ~= 2
        sizeFracWH = [0.40, 0.55];
    end
    if nargin < 3 || numel(centerFracXY) ~= 2
        centerFracXY = [0.75, 0.50];
    end

    try
        scr = get(0, 'ScreenSize');
        w = max(640, round(scr(3) * sizeFracWH(1)));
        h = max(420, round(scr(4) * sizeFracWH(2)));

        cx = scr(1) + round(scr(3) * centerFracXY(1));
        cy = scr(2) + round(scr(4) * centerFracXY(2));
        x = min(max(scr(1), cx - round(w/2)), scr(1) + scr(3) - w);
        y = min(max(scr(2), cy - round(h/2)), scr(2) + scr(4) - h);

        set(fig, 'Units', 'pixels', 'Position', [x, y, w, h]);
    catch
    end
end


