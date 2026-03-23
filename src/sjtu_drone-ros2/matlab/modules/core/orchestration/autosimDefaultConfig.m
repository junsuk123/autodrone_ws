function cfg = autosimDefaultConfig()
    cfg = struct();

    ros2env = [ ...
        'cd /home/j/INCSL/IICC26_ws && ' ...
        'unset LD_LIBRARY_PATH && ' ...
        'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash' ...
    ];

    cfg.paths = struct();
    cfg.paths.ws = '/home/j/INCSL/IICC26_ws';
    cfg.paths.root = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab';
    cfg.paths.run_id = datestr(now, 'yyyymmdd_HHMMSS');
    cfg.paths.data_root = fullfile(cfg.paths.root, 'data');
    cfg.paths.log_root = fullfile(cfg.paths.root, 'logs');
    cfg.paths.plot_root = fullfile(cfg.paths.root, 'plots');
    cfg.paths.data_dir = fullfile(cfg.paths.data_root, cfg.paths.run_id);
    cfg.paths.model_dir = fullfile(cfg.paths.root, 'models');
    cfg.paths.plot_dir = fullfile(cfg.paths.plot_root, cfg.paths.run_id);
    cfg.paths.log_dir = fullfile(cfg.paths.log_root, cfg.paths.run_id);
    cfg.paths.lock_file = fullfile(cfg.paths.data_root, 'autosim.lock');

    cfg.runtime = struct();
    cfg.runtime.worker_id = 1;
    cfg.runtime.worker_count = 1;
    cfg.runtime.domain_id = nan;
    cfg.runtime.gazebo_port = nan;
    cfg.runtime.drone_namespace = '/drone';
    cfg.runtime.multi_drone_count = 4;
    cfg.runtime.multi_drone_spacing_m = 3.0;
    cfg.runtime.multi_drone_namespace_prefix = 'drone_w';
    cfg.runtime.multi_drone_spawn_tags = true;
    cfg.runtime.multi_drone_use_world_tag_as_first = false;
    cfg.runtime.primary_drone_index = 1;
    cfg.runtime.use_gpu = false;
    cfg.runtime.gpu_device = nan;
    cfg.runtime.launch_env_prefix = "";

    cfg.visualization = struct();
    cfg.visualization.enable_progress_plot = true;
    cfg.visualization.enable_scenario_live_view = false;

    cfg.launch = struct();
    cfg.launch.use_gui = true;
    cfg.launch.use_rviz = true;
    cfg.launch.use_teleop = false;
    cfg.launch.command_template = [ ...
        ros2env ' && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        'ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py ' ...
        'drone_namespace:=%s ' ...
        'multi_drone_count:=%d multi_drone_spacing_m:=%0.2f ' ...
        'multi_drone_namespace_prefix:=%s multi_drone_spawn_tags:=%s ' ...
        'multi_drone_use_world_tag_as_first:=%s ' ...
        'takeoff_hover_height:=%0.2f ' ...
        'takeoff_vertical_speed:=0.2 ' ...
        'use_gui:=%s use_rviz:=%s use_teleop:=%s ' ...
        'use_apriltag:=true apriltag_camera:=%s/bottom ' ...
        'apriltag_image:=image_raw apriltag_tags:=tags apriltag_type:=umich ' ...
        'apriltag_bridge_topic:=%s ' ...
        'apriltag_use_standalone_detector:=true ' ...
        'apriltag_bridge_use_target_id:=false ' ...
        'apriltag_bridge_target_id:=0' ...
    ];
    cfg.launch.warmup_sec = 10.0;
    cfg.launch.ready_timeout_sec = 15.0;

    cfg.scenario = struct();
    cfg.scenario.count = 1000;
    cfg.scenario.duration_sec = inf;
    cfg.scenario.pre_landing_timeout_sec = 75.0;
    cfg.scenario.max_collection_timeout_sec = 120.0;
    cfg.scenario.sample_period_sec = 0.1;
    cfg.scenario.analysis_stop_at_landing = true;
    cfg.scenario.post_land_observe_sec = 3.0;
    cfg.scenario.early_stop_after_landing_sec = 1.5;
    cfg.scenario.hover_height_min_m = 1.5;
    cfg.scenario.hover_height_max_m = 2.5;

    cfg.wind = struct();
    cfg.wind.enable = true;
    cfg.wind.update_period_sec = 0.1;
    cfg.wind.speed_min = 0.0;
    cfg.wind.speed_max = 3.0;
    cfg.wind.direction_min = -180.0;
    cfg.wind.direction_max = 180.0;
    cfg.wind.start_delay_after_hover_sec = 0.0;
    cfg.wind.start_require_tag_centered = true;
    cfg.wind.start_tag_center_hold_sec = 0.0;
    cfg.wind.start_force_after_hover_sec = 0.0;
    cfg.wind.model_ramp_sec = 0.0;
    cfg.wind.model_gust_amp_ratio = 0.25;
    cfg.wind.model_gust_freq_hz = 0.18;
    cfg.wind.model_noise_std_speed = 0.15;
    cfg.wind.model_dir_noise_std_deg = 2.5;
    cfg.wind.model_dir_osc_amp_deg = 10.0;
    cfg.wind.model_dir_osc_freq_hz = 0.09;
    cfg.wind.source = "kma_csv"; % "kma_csv" | "random"
    cfg.wind.kma_csv = fullfile(cfg.paths.data_root, 'OBS_ASOS_MI_20260309231031.csv');
    cfg.wind.kma_time_column = 'time';
    cfg.wind.kma_speed_column = 'wind_speed';
    cfg.wind.kma_direction_column = 'wind_dir';
    cfg.wind.kma_speed_scale = 1.0;
    cfg.wind.kma_direction_offset_deg = 0.0;
    cfg.wind.kma_interp_noise_std_speed = 0.10;
    cfg.wind.kma_interp_noise_std_dir_deg = 1.5;
    cfg.wind.kma_use_profile_direct = true;
    cfg.wind.kma_pick_mode = "sequential"; % "sequential" | "random"
    cfg.wind.random_seed_base = 20260313;
    cfg.wind.physics = struct();
    cfg.wind.physics.enable = true;
    cfg.wind.physics.mass_kg = 1.4;
    cfg.wind.physics.gravity_mps2 = 9.81;
    cfg.wind.physics.max_total_thrust_n = 24.0;
    cfg.wind.physics.air_density_kgpm3 = 1.225;
    cfg.wind.physics.drag_coefficient = 1.10;
    cfg.wind.physics.frontal_area_m2 = 0.075;
    cfg.wind.physics.min_thrust_margin_n = 0.5;
    cfg.wind.physics.motor_count = 4;
    cfg.wind.physics.arm_length_m = 0.18;
    cfg.wind.physics.center_of_pressure_arm_m = 0.10;
    cfg.wind.physics.control_reserve_ratio = 0.75;
    cfg.wind.physics.landing_limit_factor = 0.5;
    cfg.wind.physics.auto_load_from_drone_files = true;
    cfg.wind.physics.prefer_file_values = true;
    cfg.wind.physics.drone_yaml_path = fullfile(cfg.paths.ws, 'src', 'sjtu_drone-ros2', 'sjtu_drone_bringup', 'config', 'drone.yaml');
    cfg.wind.physics.drone_urdf_path = fullfile(cfg.paths.ws, 'src', 'sjtu_drone-ros2', 'sjtu_drone_description', 'urdf', 'sjtu_drone.urdf');
    cfg.wind.physics.drone_sdf_path = fullfile(cfg.paths.ws, 'src', 'sjtu_drone-ros2', 'sjtu_drone_description', 'models', 'sjtu_drone', 'sjtu_drone.sdf');
    cfg.wind.physics.apply_to_agent_gate = true;
    cfg.wind.physics.apply_to_ontology = true;
    cfg.wind.physics.ontology_caution_ratio = 0.55;
    cfg.wind.physics.ontology_unsafe_ratio = 0.90;
    cfg.wind.physics.agent_gate_ratio = 1.0;

    cfg.validation = struct();
    cfg.validation.enable = false;
    cfg.validation.mode_cycle = ["easy", "boundary", "hard"];
    cfg.validation.direction_bin_count = 8;
    cfg.validation.seed_base = 20260313;
    cfg.validation.easy_speed_quantile = [0.05, 0.35];
    cfg.validation.boundary_speed_quantile = [0.35, 0.70];
    cfg.validation.hard_speed_quantile = [0.70, 0.95];
    cfg.validation.easy_hover_ratio = 0.80;
    cfg.validation.boundary_hover_ratio = 0.50;
    cfg.validation.hard_hover_ratio = 0.20;
    cfg.validation.easy_gust_amp_scale = 0.90;
    cfg.validation.boundary_gust_amp_scale = 1.05;
    cfg.validation.hard_gust_amp_scale = 1.25;
    cfg.validation.easy_dir_osc_scale = 0.90;
    cfg.validation.boundary_dir_osc_scale = 1.05;
    cfg.validation.hard_dir_osc_scale = 1.20;

    cfg.pipeline = struct();
    % "joint" | "train_only" | "validate_only"
    cfg.pipeline.mode = "joint";

    cfg.control = struct();
    cfg.control.takeoff_retry_sec = 1.0;
    cfg.control.takeoff_broadcast_window_sec = 15.0;
    cfg.control.dronevel_mode_reassert_sec = 1.0;
    cfg.control.takeoff_state_values = [1, 2];
    cfg.control.takeoff_rearm_altitude_max_m = 0.8;
    cfg.control.hover_settle_sec = 3.0;
    cfg.control.flying_altitude_threshold = 1.0;
    cfg.control.hover_z_hold_enable = true;
    cfg.control.hover_z_hold_kp = 0.9;
    cfg.control.hover_z_hold_cmd_limit_mps = 0.25;
    cfg.control.hover_z_hold_deadband_m = 0.08;
    cfg.control.xy_kp = 1.75;
    cfg.control.xy_ki = 0.0;
    cfg.control.xy_kd = 1.75;
    cfg.control.xy_i_limit = 1.0;
    cfg.control.xy_cmd_limit = 0.7;
    cfg.control.target_u = 0.0;
    cfg.control.target_v = 0.0;
    cfg.control.publish_cmd_always = true;
    cfg.control.xy_map_sign_x_from_v = 1.0;
    cfg.control.xy_map_sign_y_from_u = 1.0;
    cfg.control.tag_center_deadband = 0.005;
    cfg.control.tag_hold_timeout_sec = 0.6;
    cfg.control.tag_predict_horizon_sec = 0.15;
    cfg.control.tag_predict_timeout_sec = 0.6;
    cfg.control.tag_history_len = 20;
    cfg.control.tag_min_predict_samples = 2;
    cfg.control.tag_fast_loop_enable = true;
    cfg.control.tag_fast_loop_period_sec = 0.02;
    cfg.control.pre_takeoff_require_tag_centered = false;
    cfg.control.pre_takeoff_tag_center_tolerance = 0.005;
    cfg.control.pre_takeoff_tag_center_hold_sec = 1.0;
    cfg.control.search_enable_spiral = true;
    cfg.control.search_spiral_cmd_max = 0.30;
    cfg.control.search_spiral_growth_per_sec = 0.08;
    cfg.control.search_spiral_omega_rad_sec = 1.2;
    cfg.control.search_spiral_start_radius = 0.04;
    cfg.control.pose_hold_enable = true;
    cfg.control.pose_hold_kp = 0.45;
    cfg.control.pose_hold_ki = 0.00;
    cfg.control.pose_hold_kd = 0.00;
    cfg.control.pose_hold_i_limit = 0.40;
    cfg.control.pose_hold_cmd_limit = 0.35;
    cfg.control.land_cmd_alt_m = 0.22;
    cfg.control.land_forced_timeout_sec = 30.0;
    cfg.control.hover_hold_abort_timeout_sec = 60.0;
    cfg.control.landing_use_z_tracking = true;
    cfg.control.landing_descent_rate_mps = 0.24;
    cfg.control.landing_descent_rate_near_ground_mps = 0.10;
    cfg.control.landing_near_ground_alt_m = 0.45;
    cfg.control.landing_min_target_alt_m = 0.05;
    cfg.control.landing_z_kp = 1.25;
    cfg.control.landing_z_cmd_limit_mps = 0.35;
    cfg.control.landing_lock_enable = true;
    cfg.control.landing_lock_min_stable_frames = 8;
    cfg.control.landing_lock_max_tag_error = 0.12;
    cfg.control.landing_lock_xy_follow_enable = true;
    cfg.control.landing_lock_xy_blend_alpha = 0.20;
    cfg.control.pad_global_tracking_enable = true;
    cfg.control.pad_global_tracking_scale_x_m_per_norm = 0.60;
    cfg.control.pad_global_tracking_scale_y_m_per_norm = 0.60;
    cfg.control.pad_global_tracking_obs_max_tag_error = 0.90;
    cfg.control.pad_global_tracking_min_samples = 5;
    cfg.control.pad_global_tracking_use_in_xy_hold = true;
    cfg.control.pad_global_tracking_use_in_fast_loop = true;
    cfg.control.pad_global_tracking_use_in_landing_track = true;
    cfg.control.follower_cmd_fallback_to_primary = true;

    cfg.agent = struct();
    cfg.agent.enable_model_decision = true;
    cfg.agent.semantic_only_mode = false;
    cfg.agent.semantic_land_threshold = 0.70;
    cfg.agent.semantic_abort_threshold = 0.40;
    cfg.agent.prob_land_threshold = 0.50;
    cfg.agent.model_uncertain_margin = 0.12;
    cfg.agent.model_uncertain_fallback_enable = true;
    cfg.agent.model_semantic_fusion_weight = 0.65;
    cfg.agent.online_feature_window_sec = 15.0;
    cfg.agent.adaptive_fusion_by_ontology = true;
    cfg.agent.fusion_semantic_boost_on_caution = 0.10;
    cfg.agent.fusion_semantic_boost_on_conflict = 0.20;
    cfg.agent.semantic_assist_enable = true;
    cfg.agent.semantic_assist_land_min = 0.78;
    cfg.agent.semantic_assist_abort_max = 0.28;
    cfg.agent.min_samples_before_decision = 8;
    cfg.agent.min_hover_eval_sec = 3.0;
    cfg.agent.min_altitude_before_land = 0.10;
    cfg.agent.max_tag_error_before_land = 0.70;
    cfg.agent.decision_cooldown_sec = 0.5;
    cfg.agent.block_landing_if_unstable = true;
    cfg.agent.freeze_xy_if_unstable = false;
    cfg.agent.no_model_fallback_enable = true;
    cfg.agent.no_model_min_samples_before_land = 6;
    cfg.agent.no_model_max_tag_error = 0.90;
    cfg.agent.no_model_eval_window_sec = 1.8;
    cfg.agent.no_model_max_abs_vz = 0.90;
    cfg.agent.no_model_max_z_osc_std = 0.30;
    cfg.agent.no_model_max_z_flip_rate_hz = 4.0;
    cfg.agent.no_model_max_xy_std = 0.24;
    cfg.agent.no_model_max_xy_speed_rms = 0.55;
    cfg.agent.no_model_max_xy_radius = 0.55;
    cfg.agent.no_model_max_abs_roll_pitch_deg = 35.0;
    cfg.agent.no_model_max_wind_speed = 10.0;
    cfg.agent.no_model_require_semantic_safe = false;
    cfg.agent.model_min_total_samples_for_use = 10;
    cfg.agent.model_min_class_samples_for_use = 1;
    cfg.agent.model_minority_ratio_for_use = 0.05;
    cfg.agent.ontology_guard_enable = true;
    cfg.agent.ontology_guard_context_min = 0.45;
    cfg.agent.ontology_guard_visual_min = 0.35;
    cfg.agent.ontology_guard_max_wind_risk = 0.80;
    cfg.agent.ontology_guard_block_conflicting_relation = true;

    cfg.learning = struct();
    cfg.learning.enable = true;
    cfg.learning.save_every_scenario = false;
    cfg.learning.update_every_n_scenarios = 2;
    cfg.learning.min_scenarios_before_first_update = 4;
    cfg.learning.bootstrap_min_samples = 1;
    cfg.learning.min_stable_samples_for_update = 2;
    cfg.learning.min_unstable_samples_for_update = 1;
    cfg.learning.minority_ratio_floor_for_update = 0.05;
    cfg.learning.force_update_after_stale_scenarios = 15;
    cfg.learning.tag_lock_error_max = 0.16;
    cfg.learning.tag_lock_hold_sec = 1.2;
    cfg.learning.random_landing_wait_min_sec = 1.0;
    cfg.learning.random_landing_wait_max_sec = 4.0;
    cfg.learning.random_cmd_duration_min_sec = 1.0;
    cfg.learning.random_cmd_duration_max_sec = 3.0;
    cfg.learning.random_xy_cmd_max = 0.35;

    cfg.adaptive = struct();
    cfg.adaptive.enable = true;
    cfg.adaptive.warmup_scenarios = 2;
    cfg.adaptive.target_unstable_ratio = 0.45;
    cfg.adaptive.target_boundary_ratio = 0.25;
    cfg.adaptive.boundary_margin_context = 0.10;
    cfg.adaptive.boundary_margin_prob = 0.10;
    cfg.adaptive.recent_window = 10;
    cfg.adaptive.base_exploit_prob = 0.45;
    cfg.adaptive.base_boundary_prob = 0.30;
    cfg.adaptive.base_hard_negative_prob = 0.25;
    cfg.adaptive.max_boundary_prob = 0.45;
    cfg.adaptive.max_hard_negative_prob = 0.40;
    cfg.adaptive.safe_probe_ratio_boost = 0.06;

    cfg.adaptive.exploit = struct();
    cfg.adaptive.exploit.wind_scale = 0.85;
    cfg.adaptive.exploit.gust_amp_scale = 0.85;
    cfg.adaptive.exploit.dir_osc_scale = 0.85;
    cfg.adaptive.exploit.hover_bias_m = 0.0;

    cfg.adaptive.boundary = struct();
    cfg.adaptive.boundary.wind_center_scale = 0.72;
    cfg.adaptive.boundary.wind_span_scale = 0.10;
    cfg.adaptive.boundary.gust_amp_scale = 1.00;
    cfg.adaptive.boundary.dir_osc_scale = 1.05;
    cfg.adaptive.boundary.hover_bias_m = 0.0;

    cfg.adaptive.hard_negative = struct();
    cfg.adaptive.hard_negative.wind_scale = 1.18;
    cfg.adaptive.hard_negative.wind_min_scale = 0.95;
    cfg.adaptive.hard_negative.gust_amp_scale = 1.35;
    cfg.adaptive.hard_negative.dir_osc_scale = 1.25;
    cfg.adaptive.hard_negative.hover_bias_m = -0.10;

    cfg.probe = struct();
    cfg.probe.enable = true;
    cfg.probe.exploit_prob = 0.03;
    cfg.probe.boundary_prob = 0.40;
    cfg.probe.hard_negative_prob = 0.82;
    cfg.probe.max_exploit_prob = 0.10;
    cfg.probe.max_boundary_prob = 0.70;
    cfg.probe.max_hard_negative_prob = 0.97;
    cfg.probe.target_recent_total_ratio = 0.30;
    cfg.probe.target_recent_exploit_ratio = 0.08;
    cfg.probe.target_recent_boundary_ratio = 0.55;
    cfg.probe.target_recent_hard_negative_ratio = 0.90;
    cfg.probe.target_recent_unsafe_probe_ratio = 0.18;
    cfg.probe.total_shortage_gain = 0.55;
    cfg.probe.policy_shortage_gain = 0.85;
    cfg.probe.unsafe_shortage_gain = 0.70;
    cfg.probe.high_fp_boost = 0.12;
    cfg.probe.bootstrap_episode_count = 10;
    cfg.probe.bootstrap_target_total_ratio = 0.40;
    cfg.probe.bootstrap_exploit_prob = 0.10;
    cfg.probe.bootstrap_boundary_prob = 0.65;
    cfg.probe.bootstrap_hard_negative_prob = 0.95;
    cfg.probe.bootstrap_gain = 0.60;
    cfg.probe.max_tag_error = 0.90;
    cfg.probe.min_altitude_before_land = 0.10;
    cfg.probe.require_tag_detected = true;
    cfg.probe.require_unsafe_signal = true;
    cfg.probe.allow_uncertain_signal = true;
    cfg.probe.only_when_policy_holds = true;

    cfg.curriculum = struct();
    cfg.curriculum.enable = true;
    cfg.curriculum.target_case_names = ["safe_land", "safe_hover_timeout", "unsafe_hover_timeout", "unsafe_forced_land"];
    cfg.curriculum.target_case_ratio = [0.60, 0.10, 0.20, 0.10];
    cfg.curriculum.bootstrap_cycle = true;
    cfg.curriculum.hover_timeout_sec = 30.0;
    cfg.curriculum.safe_pool_quantile = [0.05, 0.45];
    cfg.curriculum.unsafe_pool_quantile = [0.85, 1.00];
    cfg.curriculum.min_pool_size = 30;

    cfg.persistence = struct();
    cfg.persistence.checkpoint_mat = fullfile(cfg.paths.data_dir, 'autosim_checkpoint_latest.mat');
    cfg.persistence.checkpoint_csv = fullfile(cfg.paths.data_dir, 'autosim_dataset_latest.csv');
    cfg.persistence.trace_csv = fullfile(cfg.paths.data_dir, 'autosim_trace_latest.csv');

    cfg.process = struct();
    cfg.process.stop_after_each_scenario = false;
    cfg.process.reuse_simulation_with_reset = true;
    cfg.process.force_land_before_reset = true;
    cfg.process.force_land_publish_count = 3;
    cfg.process.force_land_publish_interval_sec = 0.20;
    cfg.process.force_land_wait_timeout_sec = 8.0;
    cfg.process.reset_publish_count = 3;
    cfg.process.reset_publish_interval_sec = 0.12;
    cfg.process.reset_settle_sec = 2.0;
    cfg.process.reset_wait_landed_timeout_sec = 4.0;
    cfg.process.takeoff_after_reset = true;
    cfg.process.takeoff_publish_count = 2;
    cfg.process.takeoff_publish_interval_sec = 0.20;
    cfg.process.takeoff_wait_flying_timeout_sec = 8.0;
    cfg.process.takeoff_settle_sec = 1.0;
    cfg.process.soft_reset_enable = true;
    cfg.process.soft_reset_fallback_to_topic = false;
    cfg.process.soft_reset_service_timeout_sec = 6.0;
    cfg.process.kill_settle_sec = 2.0;
    cfg.process.cleanup_verify_timeout_sec = 8.0;
    % global: legacy behavior, instance: only worker-owned launch tree cleanup.
    cfg.process.cleanup_scope = "global";

    cfg.thresholds = struct();
    cfg.thresholds.land_state_value = 0;
    cfg.thresholds.landed_altitude_max_m = 0.60;
    cfg.thresholds.final_speed_max_mps = 0.90;
    cfg.thresholds.final_attitude_max_deg = 35.0;
    cfg.thresholds.final_tag_error_max = 0.90;
    cfg.thresholds.final_stability_std_z_max = 0.45;
    cfg.thresholds.final_stability_std_vz_max = 0.55;
    cfg.thresholds.final_touchdown_vz_osc_max = 1.45;
    cfg.thresholds.final_touchdown_accel_rms_max = 3.30;
    cfg.thresholds.final_touchdown_abs_vz_max = 6.70;
    cfg.thresholds.final_imu_ang_vel_rms_max = 6.0;
    cfg.thresholds.final_imu_lin_acc_rms_max = 11.0;
    cfg.thresholds.final_contact_force_max_n = 50.0;
    cfg.thresholds.final_arm_force_imbalance_max_n = 40.0;
    cfg.thresholds.final_arm_force_peak_max_n = 35.0;
    cfg.thresholds.require_contact_metrics = true;
    cfg.thresholds.use_mass_scaled_impact_thresholds = true;
    cfg.thresholds.mass_scaled_contact_force_factor = 3.8;
    cfg.thresholds.mass_scaled_arm_imbalance_factor = 1.4;
    cfg.thresholds.mass_scaled_arm_peak_factor = 1.5;

    cfg.model = struct();
    % Ontology-entity input schema for AI model.
    cfg.model.schema_version = "decision_ontology_v1";
    cfg.model.feature_names = [ ...
        "onto_wind_condition", ...
        "onto_gust", ...
        "onto_temporal_pattern", ...
        "onto_drone_state", ...
        "onto_tag_observation" ...
    ];
    cfg.model.input_entities = [ ...
        "WindCondition", "Gust", "TemporalPattern", ...
        "DroneState", "TagObservation" ...
    ];
    cfg.model.prior_uniform_blend = 0.55;

    cfg.ontology = struct();
    cfg.ontology.landing_area_size = [3.0, 3.0];
    cfg.ontology.obstacle_presence = false;
    cfg.ontology.tag_jitter_warn_px = 8.0;
    cfg.ontology.tag_jitter_unsafe_px = 20.0;
    cfg.ontology.tag_stability_score_warn = 0.65;
    cfg.ontology.tag_min_samples = 5;
    cfg.ontology.wind_condition_window_sec = 1.0;
    cfg.ontology.gust_base_window_sec = 6.0;
    cfg.ontology.gust_burst_window_sec = 0.8;
    cfg.ontology.wind_caution_speed = 0.55 * cfg.wind.speed_max;
    cfg.ontology.wind_unsafe_speed = 0.90 * cfg.wind.speed_max;
    cfg.ontology.gust_delta_min = 0.8;
    cfg.ontology.gust_delta_high = 2.0;
    cfg.ontology.gust_dvdt_min = 1.2;
    cfg.ontology.gust_dvdt_high = 3.0;
    cfg.ontology.temporal_short_window_sec = 1.2;
    cfg.ontology.temporal_medium_window_sec = 3.0;
    cfg.ontology.temporal_long_window_sec = 6.0;
    cfg.ontology.wind_variability_warn = 0.12 * cfg.wind.speed_max;
    cfg.ontology.wind_variability_high = 0.28 * cfg.wind.speed_max;
    cfg.ontology.wind_direction_shift_warn_deg = 18.0;
    cfg.ontology.wind_direction_shift_high_deg = 50.0;
    cfg.ontology.wind_direction_spread_warn_deg = 20.0;
    cfg.ontology.wind_direction_spread_high_deg = 65.0;
    cfg.ontology.wind_persistent_warn_ratio = 0.35;
    cfg.ontology.wind_persistent_high_ratio = 0.70;
    cfg.ontology.control_attitude_warn_deg = 7.5;
    cfg.ontology.control_attitude_high_deg = 18.0;
    cfg.ontology.control_attitude_osc_warn_deg = 2.5;
    cfg.ontology.control_attitude_osc_high_deg = 8.0;
    cfg.ontology.control_vz_osc_warn = 0.10;
    cfg.ontology.control_vz_osc_high = 0.35;
    cfg.ontology.visual_dropout_warn_ratio = 0.15;
    cfg.ontology.visual_dropout_high_ratio = 0.45;
    cfg.ontology.tag_error_vol_warn = 0.03;
    cfg.ontology.tag_error_vol_high = 0.10;
    cfg.ontology.tag_error_drift_warn = 0.015;
    cfg.ontology.tag_error_drift_high = 0.060;
    cfg.ontology.semantic_feature_names = [ ...
        "wind_speed", "wind_velocity", "wind_acceleration", "wind_dir_norm", "roll_abs", "pitch_abs", ...
        "tag_u", "tag_v", "jitter", "stability_score", ...
        "wind_risk_enc", "alignment_enc", "visual_enc" ...
    ];

    % Ontology-AI fusion: ontology concepts are preserved, but concept scores are
    % co-estimated by a lightweight temporal encoder and fused with rule-based scores.
    cfg.ontology_ai = struct();
    cfg.ontology_ai.enable = true;
    cfg.ontology_ai.mode = "temporal_tcn_lite";
    cfg.ontology_ai.rule_weight = 0.58; % fused = rule_weight*rule + (1-rule_weight)*temporal_ai

    % temporal ontology encoder branch weights (features defined in autosimOntologyReasoning)
    cfg.ontology_ai.wind_w = [1.05, 1.20, 1.10, 0.85, 0.75, 0.55, 0.35, 0.40];
    cfg.ontology_ai.wind_b = -2.10;

    cfg.ontology_ai.align_w = [-1.20, -0.85, 0.95, -1.15, -0.75, 0.55, -0.25, -0.40];
    cfg.ontology_ai.align_b = 0.55;

    cfg.ontology_ai.visual_w = [1.10, -0.95, 0.90, 0.85, -1.15, -0.70, -0.20, -0.35];
    cfg.ontology_ai.visual_b = -0.10;

    cfg.modules = struct();
    cfg.modules.enable = true;
    cfg.modules.use_wind_engine = true;
    cfg.modules.use_ai_engine = true;
    cfg.modules.use_learning_engine = true;
    % Keep ontology engine optional until its logic fully matches legacy reasoning.
    cfg.modules.use_ontology_engine = false;

    cfg.topics = struct();
    droneNs = cfg.runtime.drone_namespace;
    cfg.topics.state = [droneNs '/state'];
    cfg.topics.pose = [droneNs '/gt_pose'];
    cfg.topics.vel = [droneNs '/gt_vel'];
    cfg.topics.imu = [droneNs '/imu'];
    cfg.topics.bumpers = [droneNs '/bumper_states'];
    cfg.topics.tag_state = [droneNs '/landing_tag_state'];
    cfg.topics.wind_condition = '/wind_condition';
    cfg.topics.wind_command = '/wind_command';
    cfg.topics.land_cmd = [droneNs '/land'];
    cfg.topics.takeoff_cmd = [droneNs '/takeoff'];
    cfg.topics.reset_cmd = [droneNs '/reset'];
    cfg.topics.cmd_vel = [droneNs '/cmd_vel'];

    cfg.ros = struct();
    cfg.ros.enable_imu_subscription = false;
    cfg.ros.enable_bumper_subscription = true;
    cfg.ros.bumper_msg_type_candidates = ["gazebo_msgs/msg/ContactsState", "gazebo_msgs/ContactsState", "ros_gz_interfaces/msg/Contacts"];
    cfg.ros.bumper_topic_type_detect_enable = true;
    cfg.ros.bumper_topic_type_detect_timeout_sec = 2.0;
    cfg.ros.bumper_subscribe_retry_count = 8;
    cfg.ros.bumper_subscribe_retry_interval_sec = 0.5;
    cfg.ros.bumper_skip_retry_if_msg_unsupported = true;
    cfg.ros.bumper_log_missing_msg_support = true;
    cfg.ros.prioritize_tag_callback = true;
    cfg.ros.receive_timeout_sec = 0.002;
    cfg.ros.wind_poll_disable_on_startup_stale = true;
    cfg.ros.wind_poll_disable_after_sec = 2.5;
    cfg.ros.health_log_enable = true;
    cfg.ros.health_log_period_sec = 1.0;

    cfg.shell = struct();
    cfg.shell.setup_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash' ...
    ];
    cfg.shell.land_cli_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        'ros2 topic pub ' cfg.topics.land_cmd ' std_msgs/msg/Empty {} --once' ...
    ];
end


