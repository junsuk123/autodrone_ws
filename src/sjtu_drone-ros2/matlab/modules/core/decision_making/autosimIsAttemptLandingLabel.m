function tf = autosimIsAttemptLandingLabel(x)
    tf = autosimNormalizeActionLabel(x) == "AttemptLanding";
end


