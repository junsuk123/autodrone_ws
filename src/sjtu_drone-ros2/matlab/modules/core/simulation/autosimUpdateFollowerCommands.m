function [pidXFollowers, pidYFollowers, tagLostFollowers, lastTagUFollowers, lastTagVFollowers, lastTagDetectFollowers, haveLastTagFollowers, tagRxCountFollowers, stateFollowers, followerDiag] = ...
    autosimUpdateFollowerCommands(cfg, rosCtx, tk, dtCtrl, recvTimeoutSec, cmdPrimaryX, cmdPrimaryY, ...
    pidXFollowers, pidYFollowers, tagLostFollowers, lastTagUFollowers, lastTagVFollowers, ...
    lastTagDetectFollowers, haveLastTagFollowers, tagRxCountFollowers, stateFollowers)
% autosimUpdateFollowerCommands
% Follower drones track their own landing tags instead of mirroring primary cmd_vel.

followerDiag = autosimInitFollowerDiag();

if ~isfield(rosCtx, 'pubCmdFollowers') || isempty(rosCtx.pubCmdFollowers)
    return;
end
if ~isfield(rosCtx, 'subTagFollowers') || isempty(rosCtx.subTagFollowers)
    return;
end
if ~isfield(rosCtx, 'subStateFollowers') || isempty(rosCtx.subStateFollowers)
    return;
end
if ~isfield(rosCtx, 'subPoseFollowers') || isempty(rosCtx.subPoseFollowers)
    return;
end

nFollowers = min([numel(rosCtx.pubCmdFollowers), numel(rosCtx.subTagFollowers), numel(rosCtx.subStateFollowers), numel(rosCtx.subPoseFollowers)]);
if nFollowers < 1
    return;
end

followerDiag.expected_count = nFollowers;

cmdXAll = nan(nFollowers, 1);
cmdYAll = nan(nFollowers, 1);
poseReadyMask = false(nFollowers, 1);
stateReadyMask = false(nFollowers, 1);
tagDetectedMask = false(nFollowers, 1);
flyingMask = false(nFollowers, 1);

multiCount = 1;
if isfield(cfg, 'runtime') && isfield(cfg.runtime, 'multi_drone_count')
    multiCount = max(1, round(double(cfg.runtime.multi_drone_count)));
end
spacingM = 3.0;
if isfield(cfg, 'runtime') && isfield(cfg.runtime, 'multi_drone_spacing_m')
    spacingM = max(0.5, double(cfg.runtime.multi_drone_spacing_m));
end

flyingStates = [1, 2];
if isfield(cfg.control, 'takeoff_state_values') && ~isempty(cfg.control.takeoff_state_values)
    fs = unique(double(cfg.control.takeoff_state_values(:)'));
    fs = fs(isfinite(fs));
    if ~isempty(fs)
        flyingStates = fs;
    end
end

for i = 1:nFollowers
    poseMsg = autosimTryReceive(rosCtx.subPoseFollowers{i}, recvTimeoutSec);
    xNow = nan;
    yNow = nan;
    zNow = nan;
    if ~isempty(poseMsg) && isfield(poseMsg, 'position')
        xNow = double(poseMsg.position.x);
        yNow = double(poseMsg.position.y);
        zNow = double(poseMsg.position.z);
        poseReadyMask(i) = true;
    end

    stateMsg = autosimTryReceive(rosCtx.subStateFollowers{i}, recvTimeoutSec);
    if ~isempty(stateMsg) && isfield(stateMsg, 'data')
        stateFollowers(i) = double(stateMsg.data);
        stateReadyMask(i) = true;
    end

    isFlyingFollower = false;
    if isfinite(stateFollowers(i))
        isFlyingFollower = any(abs(stateFollowers(i) - flyingStates) < 1e-9);
        if ~isFlyingFollower && isfinite(zNow)
            isFlyingFollower = zNow >= cfg.control.flying_altitude_threshold;
        end
    elseif isfinite(zNow)
        isFlyingFollower = zNow >= cfg.control.flying_altitude_threshold;
    end
    flyingMask(i) = isFlyingFollower;

    [hasFreshTag, tagDetected, uTag, vTag, ~, tagRxCountNow] = autosimReadTagInput( ...
        rosCtx.subTagFollowers{i}, recvTimeoutSec, false, "", tagRxCountFollowers(i));
    tagRxCountFollowers(i) = tagRxCountNow;

    if hasFreshTag && tagDetected && isfinite(uTag) && isfinite(vTag)
        lastTagUFollowers(i) = uTag;
        lastTagVFollowers(i) = vTag;
        lastTagDetectFollowers(i) = tk;
        haveLastTagFollowers(i) = true;
        tagDetectedMask(i) = true;
    elseif haveLastTagFollowers(i) && ((tk - lastTagDetectFollowers(i)) <= cfg.control.tag_hold_timeout_sec)
        tagDetected = true;
        uTag = lastTagUFollowers(i);
        vTag = lastTagVFollowers(i);
        tagDetectedMask(i) = true;
    else
        tagDetected = false;
        uTag = nan;
        vTag = nan;
    end

    followerIdx = i + 1;
    if isfield(rosCtx, 'follower_namespaces') && numel(rosCtx.follower_namespaces) >= i
        nsTxt = char(string(rosCtx.follower_namespaces(i)));
        m = regexp(nsTxt, '(\d+)$', 'tokens', 'once');
        if ~isempty(m)
            idxParsed = str2double(m{1});
            if isfinite(idxParsed) && idxParsed >= 1
                followerIdx = round(idxParsed);
            end
        end
    end
    [homeX, homeY, ~] = autosimComputeSpawnPose(followerIdx, multiCount, spacingM);

    if isFlyingFollower
        [cmdXf, cmdYf, pidXFollowers(i), pidYFollowers(i), tagLostFollowers(i)] = autosimComputeTagTrackingCommand( ...
            cfg, tk, dtCtrl, xNow, yNow, false, nan, nan, tagDetected, uTag, vTag, ...
            pidXFollowers(i), pidYFollowers(i), tagLostFollowers(i), homeX, homeY);

        % If follower has no tag observation, optionally mirror primary XY command.
        if isfield(cfg, 'control') && isfield(cfg.control, 'follower_cmd_fallback_to_primary') && cfg.control.follower_cmd_fallback_to_primary
            if ~tagDetected && isfinite(cmdPrimaryX) && isfinite(cmdPrimaryY)
                cmdXf = cmdPrimaryX;
                cmdYf = cmdPrimaryY;
            end
        end
    else
        cmdXf = 0.0;
        cmdYf = 0.0;
        pidXFollowers(i) = autosimPidInit();
        pidYFollowers(i) = autosimPidInit();
        tagLostFollowers(i) = nan;
    end

    msg = ros2message(rosCtx.pubCmdFollowers{i});
    msg.linear.x = cmdXf;
    msg.linear.y = cmdYf;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    send(rosCtx.pubCmdFollowers{i}, msg);

    cmdXAll(i) = cmdXf;
    cmdYAll(i) = cmdYf;
    if stateReadyMask(i) && ~isnan(stateFollowers(i))
        followerDiag.last_state_values(i) = stateFollowers(i);
    end
end

followerDiag.pose_ready_count = nnz(poseReadyMask);
followerDiag.state_ready_count = nnz(stateReadyMask);
followerDiag.tag_detect_count = nnz(tagDetectedMask);
followerDiag.flying_count = nnz(flyingMask);
if any(isfinite(cmdXAll)) || any(isfinite(cmdYAll))
    followerDiag.cmd_xy_rms = sqrt(autosimNanMean(cmdXAll.^2 + cmdYAll.^2));
else
    followerDiag.cmd_xy_rms = nan;
end
followerDiag.cmd_x_mean = autosimNanMean(cmdXAll);
followerDiag.cmd_y_mean = autosimNanMean(cmdYAll);
followerDiag.last_cmd_x = cmdXAll;
followerDiag.last_cmd_y = cmdYAll;
end

function d = autosimInitFollowerDiag()
d = struct();
d.expected_count = 0;
d.pose_ready_count = 0;
d.state_ready_count = 0;
d.tag_detect_count = 0;
d.flying_count = 0;
d.cmd_xy_rms = nan;
d.cmd_x_mean = nan;
d.cmd_y_mean = nan;
d.last_state_values = nan(0, 1);
d.last_cmd_x = nan(0, 1);
d.last_cmd_y = nan(0, 1);
end
