function timerHandle = startWindPublisher(pubCfg)
% startWindPublisher Starts a MATLAB-based wind generator and publisher.
%
% timerHandle = startWindPublisher(pubCfg)
%
% pubCfg fields (all optional, sensible defaults provided):
%  rate - publish rate Hz (default 10)
%  steady_speed - steady wind speed (m/s)
%  steady_dir - steady wind direction (deg, 0 = +X)
%  dryden.Lu, dryden.Lv - length scales (m)
%  dryden.sigma_u, dryden.sigma_v - turbulence intensities (m/s)
%  shear.z_ref - reference height for shear (m)
%  shear.alpha - power-law exponent
%  gust.prob - probability per second to start a gust
%  gust.amp - typical gust amplitude (m/s)
%  pose_topic - pose topic to read altitude for shear (default '/drone/gt_pose')
%
% Returns a MATLAB timer handle. To stop: stop(timerHandle); delete(timerHandle);

if nargin < 1, pubCfg = struct(); end
cfg = pubCfg;
if ~isfield(cfg,'rate'), cfg.rate = 10; end
if ~isfield(cfg,'steady_speed'), cfg.steady_speed = 2.0; end
if ~isfield(cfg,'steady_dir'), cfg.steady_dir = 0.0; end
if ~isfield(cfg,'dryden'), cfg.dryden = struct(); end
if ~isfield(cfg.dryden,'Lu'), cfg.dryden.Lu = 200.0; end
if ~isfield(cfg.dryden,'Lv'), cfg.dryden.Lv = 200.0; end
if ~isfield(cfg.dryden,'sigma_u'), cfg.dryden.sigma_u = 0.5; end
if ~isfield(cfg.dryden,'sigma_v'), cfg.dryden.sigma_v = 0.5; end
if ~isfield(cfg,'shear'), cfg.shear = struct(); end
if ~isfield(cfg.shear,'z_ref'), cfg.shear.z_ref = 10.0; end
if ~isfield(cfg.shear,'alpha'), cfg.shear.alpha = 0.14; end
if ~isfield(cfg,'gust'), cfg.gust = struct(); end
if ~isfield(cfg.gust,'prob'), cfg.gust.prob = 0.02; end
if ~isfield(cfg.gust,'amp'), cfg.gust.amp = 3.0; end
if ~isfield(cfg,'pose_topic'), cfg.pose_topic = '/drone/gt_pose'; end
if ~isfield(cfg,'use_set_wind_service'), cfg.use_set_wind_service = true; end
if ~isfield(cfg,'topic_publish_mode'), cfg.topic_publish_mode = 'matlab'; end % 'matlab' | 'cli'
if ~isfield(cfg,'cli_setup_cmd'), cfg.cli_setup_cmd = 'source /home/j/INCSL/IICC26_ws/install/setup.bash'; end

% service client connection params
if ~isfield(cfg,'service_wait_timeout'), cfg.service_wait_timeout = 10.0; end % total seconds to wait for service
if ~isfield(cfg,'service_retry_max'), cfg.service_retry_max = 8; end
if ~isfield(cfg,'service_retry_interval'), cfg.service_retry_interval = 0.5; end % initial interval (sec), exponential backoff applied

% create a ROS2 node and service client
try
	node = ros2node('/matlab_wind_publisher');
catch
	error('Failed to create ros2 node for wind publisher. Ensure MATLAB ROS2 support and env are available.');
end
% create a service client for /set_wind only when enabled
client = [];
if cfg.use_set_wind_service
	try
		client = ros2serviceclient(node, '/set_wind', 'sjtu_drone_interfaces/srv/SetWind');
	catch
		error('Failed to create ros2 service client for /set_wind. Ensure service type is built and available.');
	end
end

% create a backup publisher to /wind_command in case service is unavailable
try
	pub = ros2publisher(node, '/wind_command', 'std_msgs/Float32MultiArray');
catch
	pub = [];
	warning('Failed to create backup publisher for /wind_command');
end

% Wait for service to become available / responsive (simple retry with backoff)
service_ready = false;
if cfg.use_set_wind_service
	try
		tstart = tic;
		retry = 0;
		interval = cfg.service_retry_interval;
		while true
			% prepare a small probe request (uses steady values but will be overwritten by real ticks)
			try
				probe = ros2message(client);
				probe.speed = single(cfg.steady_speed);
				probe.direction = single(cfg.steady_dir);
				resp = call(client, probe);
				if isstruct(resp) || (isobject(resp) && isprop(resp,'success'))
					service_ready = true;
					fprintf('[MATLAB] /set_wind service is available (responded after %d attempts).\n', retry+1);
					break;
				end
			catch
				% service not available yet or call failed
			end

			retry = retry + 1;
			if retry >= cfg.service_retry_max || toc(tstart) > cfg.service_wait_timeout
				break;
			end
			pause(interval);
			interval = min(5.0, interval * 2.0); % exponential backoff cap
		end
		if ~service_ready
			fprintf('[MATLAB] Warning: /set_wind service did not respond within timeout. Wind publisher will start and attempt calls; service calls will be ignored until available.\n');
		end
	catch ME
		warning('Error while waiting for /set_wind service: %s', ME.message);
		service_ready = false;
	end
else
	fprintf('[MATLAB] /set_wind service usage disabled. Publishing /wind_command continuously.\n');
end

% optionally subscribe to pose to read altitude for shear
poseSub = [];
try
	poseSub = ros2subscriber(node, cfg.pose_topic, 'geometry_msgs/Pose');
catch
	% if subscription fails, shear will use z=cfg.shear.z_ref
	poseSub = [];
end

% internal state for Dryden filters and gust
state.u_t = 0.0; state.v_t = 0.0; state.gust_amp = 0.0; state.gust_time = 0.0;
state.last_time = tic;

% timer callback
function tick(~,~)
	dt = toc(state.last_time);
	if dt <= 0, dt = 1.0/cfg.rate; end
	state.last_time = tic;

	% get altitude for shear
	z = cfg.shear.z_ref;
	if ~isempty(poseSub)
		try
			pmsg = receive(poseSub, 0.0);
			if ~isempty(pmsg)
				z = double(pmsg.position.z);
				if isnan(z) || ~isfinite(z), z = cfg.shear.z_ref; end
			end
		catch
			% ignore
		end
	end

	% steady wind (apply shear power law)
	Uref = cfg.steady_speed;
	zref = cfg.shear.z_ref;
	alpha = cfg.shear.alpha;
	if z <= 0, z = 0.1; end
	Usteady = Uref * (z / zref)^alpha;
	theta = cfg.steady_dir * pi/180.0;
	Ux = Usteady * cos(theta);
	Uy = Usteady * sin(theta);

	% approximate Dryden using exponential filter on white noise
	% a = exp(-U*dt/L)
	Uabs = max(0.1, Usteady);
	a_u = exp(-Uabs*dt / cfg.dryden.Lu);
	a_v = exp(-Uabs*dt / cfg.dryden.Lv);
	b_u = sqrt(1 - a_u^2) * cfg.dryden.sigma_u;
	b_v = sqrt(1 - a_v^2) * cfg.dryden.sigma_v;
	wn_u = randn()*1.0; wn_v = randn()*1.0;
	state.u_t = a_u * state.u_t + b_u * wn_u;
	state.v_t = a_v * state.v_t + b_v * wn_v;

	% gust: random Poisson events that create an exponentially decaying pulse
	if state.gust_time <= 0
		if rand() < cfg.gust.prob * dt
			state.gust_amp = cfg.gust.amp * (0.5 + rand());
			% random direction for gust relative to steady wind
			gdir = theta + (rand()-0.5)*pi/3;
			state.gust_vx = state.gust_amp * cos(gdir);
			state.gust_vy = state.gust_amp * sin(gdir);
			state.gust_time = 1.5 + rand()*1.5; % seconds duration
		end
	end
	gust_x = 0.0; gust_y = 0.0;
	if state.gust_time > 0
		% exponential decay
		decay = exp(-dt*1.5);
		state.gust_vx = state.gust_vx * decay;
		state.gust_vy = state.gust_vy * decay;
		gust_x = state.gust_vx; gust_y = state.gust_vy;
		state.gust_time = state.gust_time - dt;
	end

	% total wind vector
	Wx = Ux + state.u_t + gust_x;
	Wy = Uy + state.v_t + gust_y;
	w_speed = sqrt(Wx^2 + Wy^2);
	w_dir = atan2(Wy, Wx) * 180.0 / pi; % degrees

			% Prefer calling the /set_wind service; if it fails, fallback to publishing /wind_command
			did_service = false;
			if cfg.use_set_wind_service && ~isempty(client)
				try
					req = ros2message(client);
					req.speed = single(w_speed);
					req.direction = single(w_dir);
					resp = call(client, req);
					% if call returned a response structure or object with success field, consider it OK
					if ~isempty(resp)
						did_service = true;
						service_ready = true;
					end
				catch
					% service call failed; will fallback to topic publish below
					did_service = false;
				end
			end

			if ~did_service
				if strcmpi(cfg.topic_publish_mode, 'cli')
					publishWindByCli(w_speed, w_dir, cfg.cli_setup_cmd);
				elseif ~isempty(pub)
					try
						msg = ros2message(pub);
						msg.data = single([w_speed, w_dir]);
						send(pub, msg);
					catch
						% if even topic publish fails, ignore silently (MATLAB may not have ROS env)
					end
				end
			end
end

% create and start timer
period = 1.0 / cfg.rate;
timerHandle = timer('ExecutionMode','fixedRate','Period',period,'TimerFcn',@tick,'BusyMode','drop');
start(timerHandle);

fprintf('[MATLAB] Wind publisher running @ %.1f Hz (steady=%.2f m/s dir=%.1f deg, use_service=%d, topic_mode=%s)\n', cfg.rate, cfg.steady_speed, cfg.steady_dir, cfg.use_set_wind_service, cfg.topic_publish_mode);
end

function publishWindByCli(w_speed, w_dir, setupCmd)
% publishWindByCli Publishes one wind sample using ROS2 CLI command:
% ros2 topic pub /wind_command std_msgs/msg/Float32MultiArray "data: [speed, direction]" -1
topicCmd = sprintf('ros2 topic pub /wind_command std_msgs/msg/Float32MultiArray ''data: [%.6f, %.6f]'' -1', w_speed, w_dir);
if isempty(setupCmd)
	fullCmd = sprintf('bash -lc "%s"', topicCmd);
else
	fullCmd = sprintf('bash -lc "%s; %s"', setupCmd, topicCmd);
end

% Suppress CLI chatter in MATLAB output while preserving command behavior.
[status, out] = system([fullCmd ' > /dev/null 2>&1']);
if status ~= 0
	warning('CLI wind publish failed: %s', strtrim(out));
end
end
