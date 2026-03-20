function tf = autosimPipelineTrainEnabled(cfg)
    tf = true;
    if isfield(cfg, 'pipeline') && isfield(cfg.pipeline, 'mode')
        mode = lower(string(cfg.pipeline.mode));
        tf = ~(mode == "validate_only");
    end
end


