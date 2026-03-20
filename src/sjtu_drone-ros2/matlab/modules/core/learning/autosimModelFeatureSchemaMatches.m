function tf = autosimModelFeatureSchemaMatches(model, cfg)
    tf = false;
    if ~isfield(model, 'feature_names') || ~isfield(cfg, 'model') || ~isfield(cfg.model, 'feature_names')
        return;
    end

    modelFeat = string(model.feature_names(:));
    cfgFeat = string(cfg.model.feature_names(:));
    if numel(modelFeat) ~= numel(cfgFeat)
        return;
    end

    if ~all(modelFeat == cfgFeat)
        return;
    end

    if isfield(model, 'mu')
        mu = double(model.mu);
        if size(mu, 2) ~= numel(cfgFeat)
            return;
        end
    end

    if isfield(model, 'sigma2')
        sigma2 = double(model.sigma2);
        if size(sigma2, 2) ~= numel(cfgFeat)
            return;
        end
    end

    if isfield(cfg.model, 'schema_version') && isfield(model, 'schema_version')
        if string(model.schema_version) ~= string(cfg.model.schema_version)
            return;
        end
    end

    tf = true;
end


