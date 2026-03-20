function autosimEnsureDirectories(cfg)
    dirs = {cfg.paths.data_root, cfg.paths.log_root, cfg.paths.plot_root, cfg.paths.data_dir, cfg.paths.model_dir, cfg.paths.plot_dir, cfg.paths.log_dir};
    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir')
            mkdir(dirs{i});
        end
    end
end


