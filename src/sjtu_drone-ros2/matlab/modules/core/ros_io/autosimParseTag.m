function [detected, u, v, err] = autosimParseTag(msg)
    detected = false;
    u = nan;
    v = nan;
    err = nan;

    try
        d = double(msg.data);
        if numel(d) < 4
            return;
        end

        detected = d(1) > 0.5;
        if ~detected
            return;
        end

        cx = d(3);
        cy = d(4);
        w = 640.0;
        h = 480.0;

        u = (cx - w/2.0) / (w/2.0);
        v = (cy - h/2.0) / (h/2.0);
        err = sqrt(u*u + v*v);
    catch
    end
end


