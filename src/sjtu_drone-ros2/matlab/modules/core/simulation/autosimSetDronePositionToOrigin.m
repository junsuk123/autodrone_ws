function autosimSetDronePositionToOrigin(rosCtx, hoverHeightM, scenarioId)
    persistent warnedMissingMsgSupport

    if ~isfield(rosCtx, 'node') || isempty(rosCtx.node)
        return;
    end

    svcName = '/gazebo/set_model_state';
    try
        client = ros2svcclient(rosCtx.node, svcName, 'gazebo_msgs/srv/SetModelState');
        if isempty(client)
            return;
        end

        req = ros2message(client);
        req.model_state.model_name = 'drone';
        req.model_state.pose.position.x = 0.0;
        req.model_state.pose.position.y = 0.0;
        req.model_state.pose.position.z = double(hoverHeightM);
        req.model_state.pose.orientation.x = 0.0;
        req.model_state.pose.orientation.y = 0.0;
        req.model_state.pose.orientation.z = 0.0;
        req.model_state.pose.orientation.w = 1.0;
        req.model_state.twist.linear.x = 0.0;
        req.model_state.twist.linear.y = 0.0;
        req.model_state.twist.linear.z = 0.0;
        req.model_state.twist.angular.x = 0.0;
        req.model_state.twist.angular.y = 0.0;
        req.model_state.twist.angular.z = 0.0;
        req.model_state.reference_frame = 'world';

        resp = call(client, req, 'Timeout', 2.0);
        if ~isempty(resp)
            fprintf('[AUTOSIM] Scenario %d drone position reset to origin (0.0, 0.0, %.2f).\\n', scenarioId, hoverHeightM);
        end
    catch ME
        if autosimLooksLikeMissingMatlabMsgSupport(ME.message)
            if isempty(warnedMissingMsgSupport) || ~warnedMissingMsgSupport
                warnedMissingMsgSupport = true;
                fprintf('[AUTOSIM] Gazebo set_model_state MATLAB message type is unavailable; skipping pose-origin reset.\n');
            end
            return;
        end
        warning('[AUTOSIM] Scenario %d gazebo set_model_state failed: %s', scenarioId, ME.message);
    end
end


