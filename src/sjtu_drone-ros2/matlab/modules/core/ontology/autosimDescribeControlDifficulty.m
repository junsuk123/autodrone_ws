function label = autosimDescribeControlDifficulty(controlLoad)
    if controlLoad >= 0.68
        label = "hard_to_hold";
    elseif controlLoad >= 0.38
        label = "corrective_hold";
    else
        label = "easy_to_hold";
    end
end


