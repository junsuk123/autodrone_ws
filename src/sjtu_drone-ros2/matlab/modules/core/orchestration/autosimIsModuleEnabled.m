function tf = autosimIsModuleEnabled(cfg, moduleName)
    tf = false;
    if nargin < 2
        return;
    end

    mn = lower(string(moduleName));
    if isempty(cfg)
        % Allow module use for utility/model functions where cfg is not threaded.
        baseEnable = true;
        useFlag = true;
    else
        baseEnable = ~(isfield(cfg, 'modules') && isfield(cfg.modules, 'enable') && ~logical(cfg.modules.enable));
        useFlag = true;
        if isfield(cfg, 'modules')
            switch mn
                case "wind_engine"
                    if isfield(cfg.modules, 'use_wind_engine')
                        useFlag = logical(cfg.modules.use_wind_engine);
                    end
                case "ai_engine"
                    if isfield(cfg.modules, 'use_ai_engine')
                        useFlag = logical(cfg.modules.use_ai_engine);
                    end
                case "learning_engine"
                    if isfield(cfg.modules, 'use_learning_engine')
                        useFlag = logical(cfg.modules.use_learning_engine);
                    end
                case "ontology_engine"
                    if isfield(cfg.modules, 'use_ontology_engine')
                        useFlag = logical(cfg.modules.use_ontology_engine);
                    end
            end
        end
    end

    if ~baseEnable || ~useFlag
        return;
    end

    switch mn
        case "wind_engine"
            tf = exist('autosim_wind_sim', 'file') == 2;
        case "ai_engine"
            tf = exist('autosim_ai_engine', 'file') == 2;
        case "learning_engine"
            tf = exist('autosim_learning_engine', 'file') == 2;
        case "ontology_engine"
            tf = exist('autosim_ontology_engine', 'file') == 2;
        otherwise
            tf = false;
    end
end


