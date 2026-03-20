function [sub, msgType, diag] = autosimCreateBumperSubscriber(node, cfg, allowTopicProbe)
    sub = [];
    msgType = "";
    diag = struct('msg_unsupported', false, 'last_error', "");

    if nargin < 3
        allowTopicProbe = false;
    end

    retryCount = 1;
    retryIntervalSec = 0.0;
    logMissingMsgSupport = true;
    if isfield(cfg, 'ros')
        if isfield(cfg.ros, 'bumper_subscribe_retry_count') && isfinite(cfg.ros.bumper_subscribe_retry_count)
            retryCount = max(1, round(cfg.ros.bumper_subscribe_retry_count));
        end
        if isfield(cfg.ros, 'bumper_subscribe_retry_interval_sec') && isfinite(cfg.ros.bumper_subscribe_retry_interval_sec)
            retryIntervalSec = max(0.0, cfg.ros.bumper_subscribe_retry_interval_sec);
        end
        if isfield(cfg.ros, 'bumper_log_missing_msg_support')
            logMissingMsgSupport = logical(cfg.ros.bumper_log_missing_msg_support);
        end
    end

    lastErr = "";

    for attempt = 1:retryCount
        candidates = strings(0,1);
        if allowTopicProbe
            detectedType = autosimDetectTopicType(cfg, cfg.topics.bumpers);
            if strlength(detectedType) > 0
                candidates(end+1,1) = string(detectedType); %#ok<AGROW>
            end
        end

        if isfield(cfg, 'ros') && isfield(cfg.ros, 'bumper_msg_type_candidates')
            try
                candidates = [candidates; string(cfg.ros.bumper_msg_type_candidates(:))]; %#ok<AGROW>
            catch
            end
        end

        if isfield(cfg, 'ros') && isfield(cfg.ros, 'bumper_msg_type')
            oneType = string(cfg.ros.bumper_msg_type);
            if strlength(oneType) > 0
                candidates = [candidates; oneType]; %#ok<AGROW>
            end
        end

        candidates = unique(candidates(strlength(candidates) > 0), 'stable');
        candidates = autosimExpandMsgTypeCandidates(candidates);
        for i = 1:numel(candidates)
            cand = char(candidates(i));
            try
                sub = ros2subscriber(node, cfg.topics.bumpers, cand);
                msgType = string(cand);
                return;
            catch ME
                lastErr = string(ME.message);
            end
        end

        % Fallback: let MATLAB infer topic type from ROS graph when possible.
        try
            sub = ros2subscriber(node, cfg.topics.bumpers);
            msgType = "auto";
            return;
        catch ME
            lastErr = string(ME.message);
        end

        if attempt < retryCount && retryIntervalSec > 0
            pause(retryIntervalSec);
        end
    end

    diag.last_error = lastErr;
    if autosimLooksLikeMissingMatlabMsgSupport(lastErr)
        diag.msg_unsupported = true;
        if logMissingMsgSupport
            warning(['[AUTOSIM] MATLAB ROS2 message support appears missing for bumper topic type. ' ...
                'Build interfaces with ros2genmsg (e.g., your ROS2 ws/src) and restart MATLAB. ' ...
                'Last error: %s'], lastErr);
        end
    end
end


