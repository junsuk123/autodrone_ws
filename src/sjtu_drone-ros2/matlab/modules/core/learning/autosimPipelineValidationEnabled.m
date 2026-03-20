function tf = autosimPipelineValidationEnabled(cfg)
    tf = true;
    if isfield(cfg, 'pipeline') && isfield(cfg.pipeline, 'mode')
        mode = lower(string(cfg.pipeline.mode));
        tf = ~(mode == "train_only");
    end
end


