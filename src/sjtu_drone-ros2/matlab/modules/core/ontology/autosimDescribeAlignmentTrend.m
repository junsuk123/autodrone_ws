function label = autosimDescribeAlignmentTrend(errSlope, errVolatility, detCont)
    if isfinite(errSlope) && errSlope <= -0.020 && detCont >= 0.55
        label = "converging";
    elseif errSlope >= 0.020 || errVolatility >= 0.55
        label = "diverging";
    else
        label = "steady_offset";
    end
end


