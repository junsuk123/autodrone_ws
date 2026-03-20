function level = autosimRiskLevel3(score01)
    s = autosimClampNaN(score01, 1.0);
    if s < 0.33
        level = 'low';
    elseif s < 0.66
        level = 'medium';
    else
        level = 'high';
    end
end


