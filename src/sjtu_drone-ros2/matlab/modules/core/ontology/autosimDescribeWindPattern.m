function label = autosimDescribeWindPattern(speedNorm, persistence, gustIntensity, variability, directionShift)
    if gustIntensity >= 0.62 && directionShift >= 0.35
        label = "gust_front";
    elseif persistence >= 0.65 && speedNorm >= 0.45
        label = "persistent_strong";
    elseif variability >= 0.55 || directionShift >= 0.60
        label = "turbulent_shift";
    elseif speedNorm <= 0.25 && persistence <= 0.20
        label = "steady_calm";
    else
        label = "steady_flow";
    end
end


