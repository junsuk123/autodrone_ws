function ratio = autosimPolicyProbeRatio(probeEpisode, policyLabels, policyName)
    mask = string(policyLabels) == string(policyName);
    if ~any(mask)
        ratio = 0.0;
    else
        ratio = sum(probeEpisode(mask)) / sum(mask);
    end
end


