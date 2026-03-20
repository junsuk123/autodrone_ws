function windAcc = autosimComputeWindAcceleration(windSpeedHist, dt)
% Compute WindAcceleration from wind speed history.
% WindAcceleration = rate of change of wind speed (dv/dt in m/s^2)
%
% Uses linear trend of recent samples to estimate acceleration with noise robustness.
    
    windSpeedHist = double(windSpeedHist(:));
    windSpeedHist = windSpeedHist(isfinite(windSpeedHist));
    
    if numel(windSpeedHist) < 2 || ~isfinite(dt) || dt <= 0
        windAcc = 0.0;
        return;
    end
    
    % Use most recent 60% of samples to capture current trend
    windowSize = max(2, round(0.6 * numel(windSpeedHist)));
    recentSamples = windSpeedHist(max(1, end-windowSize+1):end);
    
    if numel(recentSamples) < 2
        windAcc = 0.0;
        return;
    end
    
    % Linear fit: acceleration is the slope of wind speed vs time
    t = (0:numel(recentSamples)-1)' * dt;
    p = polyfit(t, recentSamples, 1);
    windAcc = p(1);  % Slope in m/s per second
end


