function model = autosimCreatePlaceholderModel(cfg, reason)
    nFeat = numel(cfg.model.feature_names);
    model = struct();
    model.kind = "gaussian_nb";
    model.class_names = ["AttemptLanding"; "HoldLanding"];
    model.feature_names = cfg.model.feature_names;
    model.schema_version = string(cfg.model.schema_version);
    model.mu = zeros(2, nFeat);
    model.sigma2 = ones(2, nFeat);
    model.prior = [0.5; 0.5];
    model.placeholder = true;
    model.placeholder_reason = string(reason);
    model.created_at = string(datetime('now'));
end


