function [cfg, info] = autosimApplyExternalOverride(cfg, rootDir)
    info = struct('loaded', false, 'path', "");
    if nargin < 2 || strlength(string(rootDir)) == 0
        return;
    end

    overridePath = fullfile(char(rootDir), 'autosim_override.mat');
    if ~isfile(overridePath)
        return;
    end

    S = load(overridePath);
    override = struct();
    if isfield(S, 'autosimOverride') && isstruct(S.autosimOverride)
        override = S.autosimOverride;
    elseif isfield(S, 'override') && isstruct(S.override)
        override = S.override;
    end

    if ~isempty(fieldnames(override))
        cfg = autosimMergeStruct(cfg, override);
        info.loaded = true;
        info.path = string(overridePath);
    end

    try
        delete(overridePath);
    catch
    end
end


