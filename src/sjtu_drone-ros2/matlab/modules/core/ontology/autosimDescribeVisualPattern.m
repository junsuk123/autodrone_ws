function label = autosimDescribeVisualPattern(dropout, errVolatility, stabilityScore)
    if dropout >= 0.60
        label = "intermittent_lock";
    elseif errVolatility >= 0.50 || stabilityScore < 0.55
        label = "drifting_lock";
    else
        label = "locked_on";
    end
end


