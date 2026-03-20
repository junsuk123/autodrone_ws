function [model, info] = autosimLoadOrInitModel(cfg)
    if isfield(cfg, 'model') && isfield(cfg.model, 'force_model_path')
        forcedPath = string(cfg.model.force_model_path);
        if strlength(forcedPath) > 0
            modelPath = char(forcedPath);
            if ~isfile(modelPath)
                error('Forced model path does not exist: %s', modelPath);
            end

            S = load(modelPath, 'model');
            if ~isfield(S, 'model')
                error('Forced model file does not contain variable "model": %s', modelPath);
            end

            candidate = S.model;
            if ~isfield(candidate, 'feature_names')
                candidate.feature_names = cfg.model.feature_names;
            end

            if ~autosimModelFeatureSchemaMatches(candidate, cfg)
                error('Forced model schema is incompatible with current AutoSim config: %s', modelPath);
            end

            model = candidate;
            info = struct('source', string(modelPath));
            return;
        end
    end

    dd = dir(fullfile(cfg.paths.model_dir, 'autosim_model_*.mat'));
    if isempty(dd)
        dd = dir(fullfile(cfg.paths.model_dir, 'landing_model_*.mat'));
    end

    if isempty(dd)
        model = autosimCreatePlaceholderModel(cfg, 'cold_start');
        info = struct('source', "cold_start_placeholder");
        return;
    end

    [~, order] = sort([dd.datenum], 'descend');
    dd = dd(order);

    rejectedPath = "";
    for i = 1:numel(dd)
        modelPath = fullfile(dd(i).folder, dd(i).name);
        S = load(modelPath);
        if ~isfield(S, 'model')
            continue;
        end

        candidate = S.model;
        if ~isfield(candidate, 'feature_names')
            candidate.feature_names = cfg.model.feature_names;
        end

        if autosimModelFeatureSchemaMatches(candidate, cfg)
            model = candidate;
            info = struct('source', string(modelPath));
            return;
        end

        if strlength(rejectedPath) == 0
            rejectedPath = string(modelPath);
        end
    end

    model = autosimCreatePlaceholderModel(cfg, 'schema_mismatch');
    if strlength(rejectedPath) > 0
        warning('[AUTOSIM] Ignoring incompatible model schema: %s', rejectedPath);
        info = struct('source', "schema_mismatch_placeholder");
    else
        info = struct('source', "cold_start_placeholder");
    end
end


