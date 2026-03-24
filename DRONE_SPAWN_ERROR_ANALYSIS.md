# MATLAB Drone Spawn & Launch Error Handling Analysis

**Date**: 2026-03-24  
**Scope**: AutoSim drone spawn, launch, and ROS/Gazebo integration error handling

---

## Executive Summary

The MATLAB AutoSim codebase **does NOT have explicit `autosimSpawnDrone` or `autosimStartGazebo` functions**. Instead, drone spawning is delegated to the ROS 2 launch system via `sjtu_drone_bringup.launch.py`. Error handling is distributed across:

1. **Launch-phase errors**: caught in `autosimStartLaunch()`
2. **Reset/recovery errors**: caught in `autosimResetSimulationForScenario()`
3. **ROS communication errors**: caught in `autosimTryReceive()` and soft-reset logic
4. **Cleanup/pre-launch errors**: caught in `autosimCleanupProcesses()` and `autosimCleanupGazebo()`
5. **Service-level errors**: caught in `autosimSoftReset()` and `autosimSetDronePositionToOrigin()`

---

## 1. Primary Launch Function: `autosimStartLaunch()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimStartLaunch.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimStartLaunch.m#L1)

### Function Signature
```matlab
function info = autosimStartLaunch(cfg, scenarioCfg, scenarioId)
```

### Launch Command Template

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/orchestration/autosimDefaultConfig.m](src/sjtu_drone-ros2/matlab/modules/core/orchestration/autosimDefaultConfig.m#L49)

```bash
source /opt/ros/humble/setup.bash && \
source /home/j/INCSL/IICC26_ws/install/setup.bash && \
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py \
  drone_namespace:=%s \
  multi_drone_count:=%d \
  multi_drone_spacing_m:=%0.2f \
  multi_drone_namespace_prefix:=%s \
  multi_drone_spawn_tags:=%s \
  multi_drone_use_world_tag_as_first:=%s \
  takeoff_hover_height:=%0.2f \
  use_gui:=%s use_rviz:=%s use_teleop:=%s \
  use_apriltag:=true \
  apriltag_camera:=%s/bottom \
  apriltag_tags:=tags \
  [... apriltag bridge and type config ...]
```

### Error Handling in `autosimStartLaunch()`

#### **Error Point 1: Stale Process Cleanup Failure** [L10-20]

```matlab
% Lines 10-20
preSnap = autosimGetActiveProcessSnapshot();
if strlength(strtrim(preSnap)) > 0
    fprintf('[AUTOSIM] Stale process snapshot before launch:\n%s\n', preSnap);
    autosimCleanupProcesses(cfg);
    preSnap2 = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(preSnap2)) > 0
        error('Cleanup did not converge before launch start. Remaining processes:\n%s', preSnap2);
    end
end
```

**Failure Mode**: 
- Old Gazebo/ROS processes still running from previous scenario
- ROS daemon nodes in stale state (zombie processes)
- Port conflicts (11345, 11346, etc. still bound)

**Current Recovery**:
- Calls `autosimCleanupProcesses(cfg)` once
- If processes persist → **hard error** (no retry)

**Missing Logic**:
- No exponential backoff retry
- No per-process timeout validation
- No alternative cleanup strategies

---

#### **Error Point 2: Launch Command Execution Failure** [L68-74]

```matlab
% Lines 68-74
bashCmd = sprintf('bash -i -c "%s > \\\"%s\\\" 2>&1 & echo $!"', escCmd, escLog);
[st, out] = system(bashCmd);
if st ~= 0
    error('Launch failed: %s', out);
end
```

**Failure Modes**:
1. **ROS environment not sourced properly** → `command not found: ros2`
2. **Package not built/installed** → `sjtu_drone_bringup not found`
3. **Gazebo/Ignition not installed** → Gazebo fails to spawn
4. **Parameter type mismatch** → Launch XML parsing fails
5. **Resource exhaustion** → Cannot allocate memory/ports
6. **Namespace conflicts** → Multiple `/drone` namespaces

**Current Recovery**:
- System call returns to bash
- Exit code captured
- Error message from stderr printed
- **No retry or fallback**

**Launch Log Location**:
```
${cfg.paths.log_dir}/autosim_launch_s{scenarioId}_{timestamp}.log
```

**Missing Logic**:
- No parsing of launch log for specific failure reasons
- No retry on transient failures (port conflicts)
- No timeout for launch startup completion
- No validation that Gazebo actually spawned successfully

---

### Configuration Parameters Related to Launch

**File**: [src/sjtu_drone-ros2/matlab/modules/core/orchestration/autosimDefaultConfig.m](src/sjtu_drone-ros2/matlab/modules/core/orchestration/autosimDefaultConfig.m#L49)

```matlab
cfg.launch.warmup_sec = 10.0;              % Wait after launch before first ops
cfg.launch.ready_timeout_sec = 15.0;       % [UNUSED] Gazebo readiness timeout
cfg.process.reuse_simulation_with_reset = true;  % Reuse Gazebo or relaunch
cfg.process.cleanup_scope = "global";      % "global", "instance", "none"
cfg.process.kill_settle_sec = 0.2;         % Pause after pkill
cfg.process.cleanup_verify_timeout_sec = 8.0;   % Wait for proc cleanup
cfg.process.soft_reset_service_timeout_sec = 6.0;
```

---

## 2. Simulation Reset & Recovery: `autosimResetSimulationForScenario()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimResetSimulationForScenario.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimResetSimulationForScenario.m#L1)

### Reset Strategy
When `cfg.process.reuse_simulation_with_reset == true`:
1. **Force land** (publish land command + wait for landed state)
2. **Soft reset** (call `/reset_world` or `/reset_simulation` service)
3. **Topic-based reset** (publish reset message if soft reset fails/missing)
4. **Wait for landed state** before next scenario
5. **Optional: Teleport drone to origin** via `/gazebo/set_model_state` service
6. **Force takeoff** (publish takeoff + wait for flying state)

### Error Handling Points

#### **Error Point 3: Pre-reset Land Publish Failure** [L105-111]

```matlab
% Lines 105-111
if forceLandBeforeReset
    if isfield(rosCtx, 'pubLand') && ~isempty(rosCtx.pubLand)
        try
            for i = 1:nLandPub
                autosimSendToFleet(rosCtx, 'land', rosCtx.msgLand);
                pause(dtLandPub);
            end
        catch ME
            warning('[AUTOSIM] Pre-reset land publish failed for scenario %d: %s', scenarioId, ME.message);
        end
    end
end
```

**Failure Mode**: ROS publisher dead or topic not available  
**Current Recovery**: Warning logged, continues anyway  
**Missing Logic**: No fallback landing method

---

#### **Error Point 4: Soft Reset Service Call Failure** [See `autosimSoftReset()` below]

Reset attempts `/reset_world` → `/reset_simulation` with timeouts.

---

#### **Error Point 5: Topic-Based Reset Fallback Failure** [L136-143]

```matlab
% Lines 136-143
doTopicReset = (~softResetOK) || softResetFallbackToTopic;
if doTopicReset
    if ~hasTopicResetPub
        warning('[AUTOSIM] Reset publisher unavailable and soft reset failed/disabled for scenario %d.', scenarioId);
        return;  % FAILS HARD HERE
    end
    try
        for i = 1:nPub
            autosimSendToFleet(rosCtx, 'reset', rosCtx.msgReset);
            pause(dtPub);
        end
    catch ME
        warning('[AUTOSIM] Reset publish failed for scenario %d: %s', scenarioId, ME.message);
        return;  % FAILS HERE TOO
    end
end
```

**Failure Mode**: Reset publisher unavailable OR Reset message publish fails  
**Current Recovery**: Warning, then **hard return (reset_ok=false)**  
**Impact**: AutoSim tries to relaunch entire simulation

---

#### **Error Point 6: Gazebo set_model_state Teleport Failure** [See `autosimSetDronePositionToOrigin()` below]

Optional pose reset to origin. If unavailable (missing MATLAB message support), skips silently.

---

#### **Error Point 7: Post-reset Takeoff Failure** [L185-199]

```matlab
% Lines 185-199
if takeoffAfterReset
    if ~(isfield(rosCtx, 'pubTakeoff') && ~isempty(rosCtx.pubTakeoff))
        warning('[AUTOSIM] Takeoff publisher unavailable after reset for scenario %d.', scenarioId);
        return;  % FAILS
    end

    try
        for i = 1:nTakeoffPub
            autosimSendToFleet(rosCtx, 'takeoff', rosCtx.msgTakeoff);
            pause(dtTakeoffPub);
        end
    catch ME
        warning('[AUTOSIM] Post-reset takeoff publish failed for scenario %d: %s', scenarioId, ME.message);
        return;  % FAILS
    end

    if flyingTimeoutSec > 0
        flyingNow = autosimWaitForStateValue(rosCtx, flyingStateValue, flyingTimeoutSec);
        if ~flyingNow
            warning('[AUTOSIM] Scenario %d failed to reach flying state after reset/takeoff.', scenarioId);
            return;  % FAILS
        end
    end
end
```

**Failure Modes**:
1. Takeoff publisher not available
2. Takeoff message publish fails
3. Drone state does not reach "flying" within timeout

**Current Recovery**: Warnings, then return false  
**Timeout**: configurable `cfg.process.takeoff_wait_flying_timeout_sec` (default 8.0s)  
**Missing Logic**: No retry, no intermediate state verification

---

## 3. Soft Reset Service: `autosimSoftReset()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimSoftReset.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimSoftReset.m#L1)

### Reset Service Discovery & Calling

```matlab
% Lines 44-74
try
    [stList, svcs] = system(listCmd);
    if stList ~= 0
        warning('[AUTOSIM] s%03d soft reset: service list failed.', scenarioId);
        return;
    end

    hasResetWorld = contains(string(svcs), "/reset_world");
    hasResetSim = contains(string(svcs), "/reset_simulation");
    
    targets = strings(0,1);
    if hasResetWorld
        targets(end+1,1) = "/reset_world";
    end
    if hasResetSim
        targets(end+1,1) = "/reset_simulation";
    end

    if isempty(targets)
        fprintf('[AUTOSIM] s%03d soft reset: no reset service found.\n', scenarioId);
        return;
    end

    for i = 1:numel(targets)
        svc = char(targets(i));
        callCore = sprintf('timeout %.1fs ros2 service call %s std_srvs/srv/Empty {}', timeoutSec, svc);
        callCmd = sprintf('bash -i -c "%s"', autosimEscapeDq(callCore));
        
        fprintf('[AUTOSIM] s%03d soft reset: calling %s ...\n', scenarioId, svc);
        [stCall, outCall] = system(callCmd);
        if stCall == 0
            fprintf('[AUTOSIM] s%03d soft reset succeeded via %s\n', scenarioId, svc);
            softOK = true;
            return;
        end
        warning('[AUTOSIM] s%03d soft reset failed via %s: %s', scenarioId, svc, strtrim(outCall));
    end
catch ME
    warning('[AUTOSIM] s%03d soft reset error: %s', scenarioId, ME.message);
    softOK = false;
end
```

### Error Handling in Soft Reset

#### **Error Point 8: ROS 2 Service List Failure** [L30-32]

**Failure Mode**: `ros2 service list` command fails  
**Cause**: ROS environment not sourced, ROS daemon dead, permission issue  
**Current Recovery**: Warning logged, return false (fallback to topic reset)

---

#### **Error Point 9: No Reset Service Available** [L40-44]

**Failure Mode**: Neither `/reset_world` nor `/reset_simulation` available  
**Cause**: Gazebo plugin not loaded, Gazebo crashed, wrong domain ID  
**Current Recovery**: Printf warning, return false (fallback to topic reset)

---

#### **Error Point 10: Service Call Timeout** [L54-56]

```bash
timeout 6.0s ros2 service call /reset_world std_srvs/srv/Empty {}
```

**Failure Mode**: Service call blocks or doesn't respond within 6 seconds  
**Cause**: Gazebo hung, service implementation slow, network latency  
**Current Recovery**: Shell timeout kills command, outputs message  
**Timeout Config**: `cfg.process.soft_reset_service_timeout_sec` (default 6.0s)

---

#### **Error Point 11: Service Call Execution Error** [L63-64]

**Failure Mode**: Service call returns non-zero exit code  
**Cause**: Gazebo state corruption, service not actually callable  
**Current Recovery**: Warning logged, tries next service in targets list, then returns false

---

## 4. Drone Position Teleport: `autosimSetDronePositionToOrigin()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimSetDronePositionToOrigin.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimSetDronePositionToOrigin.m#L1)

### Service Call Details

```matlab
% Lines 6-35
svcName = '/gazebo/set_model_state';
try
    client = ros2svcclient(rosCtx.node, svcName, 'gazebo_msgs/srv/SetModelState');
    if isempty(client)
        return;  % Silent failure
    end

    req = ros2message(client);
    req.model_state.model_name = 'drone';
    req.model_state.pose.position.x = 0.0;
    req.model_state.pose.position.y = 0.0;
    req.model_state.pose.position.z = double(hoverHeightM);
    % ... orientation and twist set to identity/zero ...
    
    resp = call(client, req, 'Timeout', 2.0);
    if ~isempty(resp)
        fprintf('[AUTOSIM] Scenario %d drone position reset to origin (0.0, 0.0, %.2f).\n', scenarioId, hoverHeightM);
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
```

### Error Handling

#### **Error Point 12: Missing MATLAB Message Support** [L37-42]

**Failure Mode**: `gazebo_msgs/srv/SetModelState` not available in MATLAB's generated message library  
**Current Recovery**: Detects via `autosimLooksLikeMissingMatlabMsgSupport()` pattern match, logs once, continues  
**Detection Pattern**: 
```matlab
contains(err, "message type") || contains(err, "unknown message") || 
contains(err, "not found") || contains(err, "cannot resolve") || 
contains(err, "interface") || contains(err, "does not exist")
```

---

#### **Error Point 13: Service Call Timeout** [L31]

```matlab
resp = call(client, req, 'Timeout', 2.0);
```

**Failure Mode**: Gazebo service doesn't respond within 2 seconds  
**Cause**: Gazebo state corruption, service handler hung  
**Current Recovery**: Exception caught, warning logged

---

## 5. Message Reception & Timeouts: `autosimTryReceive()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/ros_io/autosimTryReceive.m](src/sjtu_drone-ros2/matlab/modules/core/ros_io/autosimTryReceive.m#L1)

### Function Implementation

```matlab
function x = autosimTryReceive(sub, timeout)
    if nargin < 2 || ~isfinite(timeout)
        timeout = 0.01;
    end
    timeout = max(0.0, double(timeout));
    if isempty(sub)
        x = [];
        return;
    end

    % Prefer non-blocking latest-sample access
    try
        if isprop(sub, 'LatestMessage')
            latest = sub.LatestMessage;
            if ~isempty(latest)
                x = latest;
                return;
            end
            if timeout <= 0.0
                x = [];
                return;
            end
        end
    catch
        % Fall through to receive() fallback
    end

    % Cap timeout to 5ms to prevent control-loop stalls
    timeout = min(timeout, 0.005);
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end
```

### Error Handling

#### **Error Point 14: Blocking Receive Timeout** [L29-32]

**Failure Mode**: `receive()` call hung waiting for new message when topic is sparse or dead  
**Cause**: Topic not being published, publisher crashed, Gazebo paused mid-scenario  
**Current Recovery (Fixed 2026-03-22)**: 
- Cap fallback `receive()` timeout to 5ms maximum
- Prevent control loop stalls
- Return empty array on timeout instead of blocking

**Known Issue** (from memory file):
> Long-run hang observed at wind subscriber receive in `autosimRunScenario` when `/wind_condition` remained stale (rx=0, age=Inf).  
> Mitigation: add startup-stale guard with `cfg.ros.wind_poll_disable_after_sec=2.5`

---

## 6. ROS State Machine: `autosimWaitForStateValue()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimWaitForStateValue.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimWaitForStateValue.m#L1)

### Function Implementation

```matlab
function ok = autosimWaitForStateValue(rosCtx, expectedState, timeoutSec)
    ok = false;
    if timeoutSec <= 0
        return;  % Timeout disabled
    end
    if ~isfield(rosCtx, 'subState') || isempty(rosCtx.subState)
        return;  % No subscriber
    end

    t0 = tic;
    while toc(t0) < timeoutSec
        stMsg = autosimTryReceive(rosCtx.subState, 0.10);
        if ~isempty(stMsg)
            try
                if double(stMsg.data) == expectedState
                    ok = true;
                    return;
                end
            catch
                % Message data format error, continue polling
            end
        end
        pause(0.02);
    end
end
```

### Error Handling

#### **Error Point 15: State Value Mismatch or Timeout** [L10-25]

**Failure Mode**: Drone state does not transition to expected value within timeout  
**Examples**:
- Waiting for `landed_state == 0` after reset takes >4 seconds → timeout
- Waiting for `flying_state == 1` after takeoff takes >8 seconds → timeout

**Current Recovery**:
```matlab
if ~landedNow
    warning('[AUTOSIM] Scenario %d did not confirm landed before reset; continuing with reset.', scenarioId);
else
    fprintf('[AUTOSIM] Scenario %d pre-reset landing confirmed.\n', scenarioId);
end
```

**Missing Logic**: 
- No intermediate state validation
- No per-state maximum wait times
- No check if drone crashed during state transition

---

## 7. Process Cleanup: `autosimCleanupProcesses()` & `autosimCleanupGazebo()`

**Location**: 
- [src/sjtu_drone-ros2/matlab/modules/core/utils/autosimCleanupProcesses.m](src/sjtu_drone-ros2/matlab/modules/core/utils/autosimCleanupProcesses.m#L1)
- [src/sjtu_drone-ros2/matlab/modules/core/orchestration/autosimCleanupGazebo.m](src/sjtu_drone-ros2/matlab/modules/core/orchestration/autosimCleanupGazebo.m#L1)

### Cleanup Strategy

#### Phase 1: Target-specific process kills
```bash
pkill -9 -f "[r]os2 launch sjtu_drone_bringup"
pkill -9 -f "[s]jtu_drone_bringup.launch.py"
pkill -9 -f "[s]pawn_drone"  # <-- Spawning script
pkill -9 gzserver
pkill -9 gzclient
```

#### Phase 2-3: Node-specific kills (3 passes with increasing delays)
```bash
pkill -9 -f "[r]obot_state_publisher"
pkill -9 -f "[j]oint_state_publisher"
pkill -9 -f "[a]priltag"
pkill -9 rviz2
# ... etc
```

#### Phase 4: Port verification

**Ports checked**:
```matlab
gazebo_ports = [11345, 11346, 11347, 11348, 13000:14000];
```

**Error Point 16: Gazebo Port Still In Use** [L57-75 in autosimCleanupGazebo]

```matlab
while cleanup_retry_count < max_retries
    ports_in_use = [];
    for port = gazebo_ports(1:4)
        try
            [st, ~] = system(sprintf('lsof -i :%d 2>/dev/null', port));
            if st == 0
                ports_in_use = [ports_in_use, port];
            end
        catch
        end
    end
    
    if ~isempty(ports_in_use)
        fprintf('[AUTOSIM]   - Ports still in use: %s\n', sprintf('%d ', ports_in_use));
        if cleanup_retry_count < max_retries - 1
            fprintf('[AUTOSIM]   - Retrying aggressive kill...\n');
            system('pkill -9 -f gazebo 2>/dev/null || true');
            pause(2.0);
            cleanup_retry_count = cleanup_retry_count + 1;
        else
            fprintf('[AUTOSIM]   - Warning: Some ports still in use, proceeding anyway\n');
            break;
        end
    else
        fprintf('[AUTOSIM]   ✓ All Gazebo ports are free\n');
        break;
    end
end
```

**Failure Mode**: Gazebo process exits but OS doesn't free port for ~30 seconds  
**Current Recovery**: 
- Retries up to max_retries=2 times
- Pauses 2 seconds between retries
- **Proceeds anyway if still bound** (may cause next launch to fail)

**Missing Logic**:
- Longer maximum wait (curr limit ~4s)
- SO_REUSEADDR socket option not configurable
- No check if old gzserver is actually dead vs just lingering socket

---

#### Phase 5: ROS environment cleanup

```bash
ros2 daemon stop
rm -rf ~/.ros/latest_run
```

**Known Issue** (from memory file, 2026-03-21):
> Immediate worker exit error: `Error creating the ROS 2 node`, `libmwros2s: the same interface may not be selected twice`, `rcl node's rmw handle is invalid`  
> Fix: default `AUTOSIM_ROS_LOCALHOST_ONLY` to `0` in run scripts; keep opt-in only for network-debug sessions.

---

## 8. Scenario-Level Exception Handling

**Location**: [src/sjtu_drone-ros2/matlab/AutoSim.m](src/sjtu_drone-ros2/matlab/AutoSim.m#L135)

```matlab
% Lines 135-175
try
    if preRunPauseSec > 0
        pause(preRunPauseSec);
    end
    [scenarioResult, scenarioTrace] = autosimRunScenario(cfg, scenarioCfg, scenarioId, model, rosCtx);
    scenarioResult.launch_pid = launchInfo.pid;
    scenarioResult.launch_log = launchInfo.log_file;
catch ME
    scenarioResult = autosimEmptyScenarioResult();
    scenarioResult.scenario_id = scenarioId;
    scenarioResult.label = "unstable";
    scenarioResult.success = false;
    if autosimIsUserInterrupt(ME)
        scenarioResult.failure_reason = "user_interrupt";
        runStatus = "interrupted";
    else
        scenarioResult.failure_reason = "runtime_exception";
    end
    scenarioResult.exception_message = string(ME.message);
    warning('[AUTOSIM] Scenario %d exception: %s', scenarioId, ME.message);
    fprintf(2, '[AUTOSIM] Scenario %d stack:\n%s\n', scenarioId, getReport(ME, 'extended', 'hyperlinks', 'off'));
end
```

### Recovery After Scenario Exception

**Option 1: Relaunch Simulation**  
If reset fails or scenario throws exception:
```matlab
if ~resetOk
    warning('[AUTOSIM] Reset path failed, relaunching simulation for scenario %d.', scenarioId);
    launchInfo = autosimStartLaunch(cfg, scenarioCfg, scenarioId);
    launchActive = true;
    preRunPauseSec = cfg.launch.warmup_sec;
end
```

**Option 2: Stop After Each Scenario**  
```matlab
if cfg.process.stop_after_each_scenario && (~isfield(cfg.process, 'reuse_simulation_with_reset') || ~cfg.process.reuse_simulation_with_reset)
    autosimCleanupProcesses(cfg, launchInfo.pid);
    pause(cfg.process.kill_settle_sec);
    launchActive = false;
end
```

---

## 9. ROS Connection Initialization: `autosimCreateRosContext()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/ros_io/autosimCreateRosContext.m](src/sjtu_drone-ros2/matlab/modules/core/ros_io/autosimCreateRosContext.m#L1)

### Error Point 17: AprilTag Callback Mode Fallback

```matlab
% Lines 15-26
tagCallbackRequested = isfield(cfg, 'ros') && isfield(cfg.ros, 'prioritize_tag_callback') && cfg.ros.prioritize_tag_callback;
if tagCallbackRequested
    tagCacheKey = regexprep(sprintf('autosim_tag_cache_%s', nodeName), '[^a-zA-Z0-9_]', '_');
    autosimInitTagStateCache(tagCacheKey);
    try
        rosCtx.subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray', ...
            @(varargin) autosimTagStateCallback(tagCacheKey, varargin{:}));
        rosCtx.tag_callback_enabled = true;
        rosCtx.tag_cache_key = string(tagCacheKey);
        fprintf('[AUTOSIM] AprilTag callback-priority subscriber enabled on %s\n', cfg.topics.tag_state);
    catch ME
        autosimClearTagStateCache(tagCacheKey);
        rosCtx.subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray');
        warning('[AUTOSIM] AprilTag callback mode unavailable; fallback to polling: %s', ME.message);
    end
end
```

**Failure Mode**: Callback subscriber fails (e.g., MATLAB version limitation)  
**Current Recovery**: Falls back to polling mode (non-blocking LatestMessage)  
**Impact**: Minor latency difference, but functionality preserved

---

### Error Point 18: Optional IMU Subscription

```matlab
% Lines 32-35
rosCtx.subImu = [];
if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_imu_subscription') && cfg.ros.enable_imu_subscription
    try
        rosCtx.subImu = ros2subscriber(node, cfg.topics.imu, 'sensor_msgs/msg/Imu');
    catch
        rosCtx.subImu = [];
    end
end
```

**Failure Mode**: IMU message type not available or topic doesn't exist  
**Current Recovery**: Silent failure, rosCtx.subImu remains empty, scenario continues

---

### Error Point 19: Optional Bumper Subscription

```matlab
% Lines 37-40
if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_bumper_subscription') && cfg.ros.enable_bumper_subscription
    [rosCtx.subBumpers, rosCtx.bumper_msg_type, bumperDiag] = autosimCreateBumperSubscriber(node, cfg, false);
    if isstruct(bumperDiag) && isfield(bumperDiag, 'msg_unsupported')
        rosCtx.bumper_msg_unsupported = logical(bumperDiag.msg_unsupported);
    end
end
```

**Failure Mode**: Bumper message type not available  
**Current Recovery**: Sets `bumper_msg_unsupported = true`, scenario reads but ignores bumper data  

---

## 10. Spawn Pose Calculation: `autosimComputeSpawnPose()`

**Location**: [src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimComputeSpawnPose.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimComputeSpawnPose.m#L1)

### Function

```matlab
function [x, y, z] = autosimComputeSpawnPose(indexOneBased, count, spacingM)
    % Match Gazebo multi-drone spawn grid to recover each drone's home pad.
    
    idx = max(1, round(double(indexOneBased)));
    n = max(1, round(double(count)));
    spacing = max(0.1, double(spacingM));
    
    if n <= 3
        cols = n;
    else
        cols = ceil(sqrt(double(n)));
    end
    rows = ceil(double(n) / double(cols));
    
    indexZeroBased = idx - 1;
    row = floor(double(indexZeroBased) / double(cols));
    col = mod(double(indexZeroBased), double(cols));
    
    xCenter = 0.5 * double(cols - 1);
    yCenter = 0.5 * double(rows - 1);
    
    x = (col - xCenter) * spacing;
    y = (yCenter - row) * spacing;
    z = 0.05;
end
```

**Error Handling**: None. Function deterministic, no external I/O.

**Spawn Grid Example** (3 drones, 10m spacing):
```
Drone 1: (-10, 0,  0.05)
Drone 2: (  0, 0,  0.05)
Drone 3: ( 10, 0,  0.05)
```

---

## 11. Critical Known Issues & Mitigation History

From [/memories/repo/matlab_ros_crash_notes.md](/memories/repo/matlab_ros_crash_notes.md):

### **Issue 1: MATLAB R2024b ROS Crash** (2026-03-11)
- **Root Cause**: Implicit destruction order of ROS2 subscribers/publishers in shared_ptr
- **Symptom**: Segfault in libmwrosinterprocessutil during AutoSim shutdown
- **Mitigation**: Added explicit `autosimCleanupRosHandles()` in reverse dependency order before node release
- **Location**: [modules/core/ros_io/autosimReleaseRosContext.m](src/sjtu_drone-ros2/matlab/modules/core/ros_io/autosimReleaseRosContext.m)

### **Issue 2: AutoSim Lock False Positive PID Reuse** (2026-03-16)
- **Root Cause**: `.lock` file stored only PID; PID reused by new process
- **Symptom**: AutoSim refused to start because `autosim.lock` pid matched unrelated process
- **Mitigation**: Enhanced lock check: PID + `/proc/<pid>/stat` start_ticks + cmdline validation
- **Location**: [AutoSim.m](src/sjtu_drone-ros2/matlab/AutoSim.m) lock initialization

### **Issue 3: ros2svcclient vs ros2svclient Typo** (2026-03-19)
- **Symptom**: `Undefined function 'ros2svclient'` in drone position reset
- **Mitigation**: Corrected to `ros2svcclient` (MATLAB ROS2 API correct spelling)
- **Location**: [autosimSetDronePositionToOrigin.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimSetDronePositionToOrigin.m#L9)

### **Issue 4: Gazebo Service Timeout Default Change** (2026-03-19)
- **Root Cause**: MATLAB R2024b ros2svcclient no longer accepts `Timeout` in constructor
- **Symptom**: Service calls hung indefinitely or failed
- **Mitigation**: Changed to `call(client, req, 'Timeout', 2.0)` in call invocation
- **Location**: [autosimSetDronePositionToOrigin.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimSetDronePositionToOrigin.m#L31)

### **Issue 5: Wind Topic Stale Hang** (2026-03-20)
- **Symptom**: Long-run hang at `autosimTryReceive(subWind)` when `/wind_condition` unpublished (rx=0, age=Inf)
- **Mitigation**: Added startup-stale guard:
  - `cfg.ros.wind_poll_disable_on_startup_stale = true`
  - `cfg.ros.wind_poll_disable_after_sec = 2.5`
- **Location**: [autosimRunScenario.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimRunScenario.m) wind polling

### **Issue 6: Global Pad Position Tracking During Tag Loss** (2026-03-20)
- **Symptom**: XY control diverges when AprilTag temporarily lost
- **Mitigation**: Cumulative mean world pad position estimate from tag observations + current drone pose
- **Fallback**: `xy_hold`, `landing_track` use estimated pad position (not tag-relative)
- **Location**: [autosimRunScenario.m](src/sjtu_drone-ros2/matlab/modules/core/simulation/autosimRunScenario.m) pose tracking

### **Issue 7: ROS_LOCALHOST_ONLY Interface Conflict** (2026-03-21)
- **Symptom**: Parallel worker exit with `Error creating the ROS 2 node`, `the same interface may not be selected twice`
- **Root Cause**: `ROS_LOCALHOST_ONLY=1` forced lo0 for all workers in same domain
- **Mitigation**: Default `AUTOSIM_ROS_LOCALHOST_ONLY=0` in run scripts; opt-in only for network-debug
- **Location**: `run_autosim_parallel.sh`, `run_autosim_domain_bridge.sh`

### **Issue 8: Multi-Drone Follower Loop Freeze** (2026-03-22)
- **Symptom**: Scenario never completes when follower drone topics sparse (e.g., only primary publishing)
- **Root Cause**: `autosimTryReceive -> receive` blocking indefinitely waiting for new state messages
- **Mitigation**: 
  - Prefer non-blocking `LatestMessage` property
  - Cap fallback `receive` timeout to 5ms maximum (prevent control-loop stalls)
- **Location**: [autosimTryReceive.m](src/sjtu_drone-ros2/matlab/modules/core/ros_io/autosimTryReceive.m#L29)

---

## 12. Identified Missing Error Handling & Retry Logic

### **Gap 1: No Retry Loop for Launch**
- Launch fails once → hard error
- Alternative: Implement exponential backoff retry with max 3 attempts
- **Suggested Code**:
```matlab
maxRetries = 3;
retryDelay = [1.0, 2.0, 5.0];  % seconds
for attempt = 1:maxRetries
    try
        [st, out] = system(bashCmd);
        if st == 0
            break;  % Success
        elseif attempt < maxRetries
            fprintf('[AUTOSIM] Launch attempt %d/%d failed, retrying in %.1fs...\n', attempt, maxRetries, retryDelay(attempt));
            pause(retryDelay(attempt));
        else
            error('Launch failed after %d attempts: %s', maxRetries, out);
        end
    catch ME
        if attempt < maxRetries
            pause(retryDelay(attempt));
        else
            rethrow(ME);
        end
    end
end
```

### **Gap 2: No Validation That Gazebo Spawned**
- Launch succeeds at shell level, but Gazebo/drones may not actually exist
- Alternative: Implement startup readiness check:
```matlab
% After launch, wait for key topics to begin publishing
topicsToCheck = {cfg.topics.state, cfg.topics.pose, cfg.topics.tag_state};
readinessTimeout = cfg.launch.ready_timeout_sec;  % Use existing param
readyOk = autosimWaitForTopicReadiness(rosCtx, topicsToCheck, readinessTimeout);
if ~readyOk
    error('Launch topics not ready after %.1fs', readinessTimeout);
end
```

### **Gap 3: No Per-Service Retry in Soft Reset**
- Service call hangs → timeout → warning → move to next service
- Alternative: Implement exponential backoff per service
```matlab
maxRetries = 2;
for svcIdx = 1:numel(targets)
    svc = char(targets(svcIdx));
    for retryIdx = 1:maxRetries
        % ... call service ...
        if success
            return;  % Success
        elseif retryIdx < maxRetries
            fprintf('[AUTOSIM] Retry %d/%d for %s\n', retryIdx, maxRetries, svc);
            pause(1.0 * retryIdx);  % Backoff
        end
    end
end
```

### **Gap 4: No Namespace Conflict Detection**
- If launch script has bug in namespace generation, both drones spawn as `/drone`
- Alternative: Validate namespace uniqueness at ROS context creation:
```matlab
% After creating ROS context, verify all follower namespaces are unique
nsSet = set(rosCtx.follower_namespaces);
if numel(nsSet) < numel(rosCtx.follower_namespaces)
    error('Duplicate namespaces detected; namespace conflict may prevent multi-drone operation');
end
```

### **Gap 5: No Gazebo Port Conflict Retry in Launch**
- Pre-launch check finds port 11345 busy, cleanup attempts to free it, launch proceeds
- But port may not be freed until kernel decides (30+ seconds)
- Alternative: Implement port binding retry in launch command:
```matlab
% Wrap launch in retry loop that checks/waits for port availability
maxPortRetries = 3;
for portRetry = 1:maxPortRetries
    if checkGazeboPortAvailable(cfg)
        break;  % Proceed with launch
    elseif portRetry < maxPortRetries
        fprintf('[AUTOSIM] Port not available, waiting before launch attempt %d/%d\n', portRetry, maxPortRetries);
        pause(5.0 * portRetry);
    else
        error('Gazebo port still bound after %d retries', maxPortRetries);
    end
end
```

### **Gap 6: No Message Type Validation at ROS Context Creation**
- Bumper/IMU message types detected as missing only at subscription attempt
- Alternative: Early detection during ROS context init:
```matlab
% Validate all required message types available before subscriptions
requiredMsgTypes = {'std_msgs/Int8', 'geometry_msgs/Pose', 'std_msgs/Float32MultiArray'};
for msgType = requiredMsgTypes
    try
        testMsg = ros2message(msgType);
    catch
        error('Required message type %s not available; ensure MATLAB message generation complete', char(msgType));
    end
end
```

### **Gap 7: No Drone State Consistency Check**
- After takeoff, assumes drone is flying without validating pose/velocity
- Alternative: Multi-sensor state validation:
```matlab
% Multi-check flying state: state==1 AND pose.z > 0.5 AND velocity.z > -0.05
ok = autosimWaitForCompoundStateCondition(rosCtx, {...
    @(st) eq(st.data, 1), ...  % state == flying
    @(pose) pose.position.z > 0.5, ...  % above ground
    @(vel) vel.linear.z > -0.05 ...     % not descending rapidly
}, 8.0);
```

---

## 13. Summary Table: Failure Points & Recovery

| # | Module | Failure Point | Symptom | Current Recovery | Retry? | Missing? |
|---|--------|---|---|---|---|---|
| 1 | autosimStartLaunch | Stale process cleanup fails | Processes still alive | Hard error | ❌ | Exponential backoff |
| 2 | autosimStartLaunch | Launch command fails | Shell returns non-zero | Hard error | ❌ | Retry loop + log parse |
| 3 | autosimResetSimulationForScenario | Land publish fails | ROS exception | Warning + continue | ✓ | Fallback land method |
| 4 | autosimResetSimulationForScenario | Soft reset unavailable | No /reset_world service | Topic reset fallback | ✓ | N/A |
| 5 | autosimResetSimulationForScenario | Topic reset fails | Publisher dead | Hard return false, relaunch | ✓ | Fallback to hard reset |
| 6 | autosimSetDronePositionToOrigin | Gazebo service timeout | Service hangs 2s | Warning only | ❌ | Retry + longer timeout |
| 7 | autosimResetSimulationForScenario | Takeoff fail + timeout | Drone not flying after 8s | Hard return false, relaunch | ✓ | Intermediate state check |
| 8 | autosimSoftReset | Service list fails | ros2 command error | Warning, return false | ❌ | Retry with setup sourcing |
| 9 | autosimSoftReset | Service timeout | 6s timeout on service call | Next service in list | ✓ | Per-service backoff |
| 10 | autosimSoftReset | No reset service exists | Neither /reset_world nor /reset_simulation | Return false (fallback to topic) | ✓ | N/A |
| 11 | autosimWaitForStateValue | State timeout | Drone state != expected after Xs | Warning only | ✓ | N/A |
| 12 | autosimSetDronePositionToOrigin | Missing MATLAB msg type | Message library incomplete | Silently skip + log once | ✓ | Early msg validation |
| 13 | autosimTryReceive | Receive timeout | **FIXED (2026-03-22)**: cap to 5ms | Non-blocking LatestMessage | ✓ | N/A |
| 14 | autosimCreateRosContext | Tag callback unavailable | MATLAB limitation | Fall back to polling | ✓ | N/A |
| 15 | autosimCreateRosContext | IMU/Bumper subscription fails | Message type unavailable | Silent skip | ✓ | N/A |
| 16 | autosimCleanupGazebo | Port still bound after cleanup | Socket not released | Warnings, proceed anyway | ✓ | Longer wait + SO_REUSEADDR |
| 17 | autosimStartLaunch | Gazebo spawning fails (silent) | No error, but drones missing | No validation | ❌ | **Topic readiness check** |
| 18 | Multi-drone | Namespace conflict | Duplicate /drone namespace | No detection | ❌ | Namespace uniqueness check |
| 19 | Wind | Topic stale hang | **FIXED (2026-03-20)**: wind_poll_disable_after_sec | Config: disable after 2.5s | ✓ | N/A |
| 20 | Multi-drone | Follower topic timeout | **FIXED (2026-03-22)**: receive to 5ms | Prevent ctrl-loop stalls | ✓ | N/A |

---

## 14. Recommendations

### **Priority 1: Critical (Launch Reliability)**
1. **Implement launch retry loop** with exponential backoff (3 attempts, 1s/2s/5s)
2. **Add Gazebo spawning validation** - wait for topic readiness after launch
3. **Enhance soft-reset error handling** - per-service retry, longer service timeout
4. **Add namespace conflict detection** - validate uniqueness of multi-drone namespaces

### **Priority 2: High (Reset Robustness)**
5. **Compound state validation** - multi-sensor check (state + pose + velocity)
6. **Fallback landing mechanism** - try reset again if takeoff fails
7. **Gazebo port retry** - implement binding retry with longer wait
8. **Early message type validation** - detect missing MATLAB messages at ROS context creation

### **Priority 3: Medium (Observability)**
9. **Parse launch logs** - extract specific failure reasons from Gazebo output
10. **Structured error reporting** - capture and report failure genealogy
11. **Timeout instrumentation** - log remaining time when state checks time out
12. **Gazebo state snapshot** - dump gazebo state on launch failure for debugging

---

## 15. Files Involved in Spawn/Launch Error Handling

```
src/sjtu_drone-ros2/matlab/
├── AutoSim.m                                    [Main scenario loop, exception handling]
├── AutoSimCollect.m                             [Collection orchestration, parallel launch]
├── AutoSimMain.m                                [Pipeline entrypoint, cleanup]
│
├── modules/core/simulation/
│   ├── autosimStartLaunch.m                     [Launch execution, process sanity check]
│   ├── autosimResetSimulationForScenario.m      [Reset sequence, state waits, timeouts]
│   ├── autosimSoftReset.m                       [Service discovery, soft reset calls]
│   ├── autosimSetDronePositionToOrigin.m        [Gazebo teleport service]
│   ├── autosimWaitForStateValue.m               [State polling with timeout]
│   ├── autosimComputeSpawnPose.m                [Spawn grid calculation (no errors)]
│   └── autosimRunScenario.m                     [Scenario execution, command sends, data receives]
│
├── modules/core/utils/
│   └── autosimCleanupProcesses.m                [Process termination, PID-specific cleanup]
│
├── modules/core/ros_io/
│   ├── autosimCreateRosContext.m                [ROS init, subscriber/publisher creation]
│   ├── autosimCreateBumperSubscriber.m          [Optional bumper subscription]
│   ├── autosimTryReceive.m                      [Non-blocking message reception (FIXED)]
│   ├── autosimReleaseRosContext.m               [ROS cleanup, handle release]
│   ├── autosimCleanupRosHandles.m               [Explicit cleanup order (FIXED)]
│   ├── autosimLooksLikeMissingMatlabMsgSupport.m [Message type detection]
│   └── autosimSendToFleet.m                     [Multi-drone publication wrapper]
│
├── modules/core/orchestration/
│   ├── autosimCleanupGazebo.m                   [Comprehensive Gazebo/ROS cleanup]
│   ├── autosimDefaultConfig.m                   [Launch template, timeout defaults]
│   ├── autosimIsUserInterrupt.m                 [User interrupt detection]
│   └── autosimMainOrchestrate.m                 [Parallel worker orchestration]
│
└── scripts/
    └── monitor_autosim_parallel.m               [Parallel session monitoring, launch log discovery]
```

---

**End of Analysis**
