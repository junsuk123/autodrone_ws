function tf = autosimHasUsableModelParameters(model)
    tf = false;
    req = {'class_names', 'mu', 'sigma2', 'prior'};
    for i = 1:numel(req)
        if ~isfield(model, req{i})
            return;
        end
    end

    cls = string(model.class_names(:));
    hasActionClasses = all(ismember(["AttemptLanding"; "HoldLanding"], cls));
    hasLegacyClasses = all(ismember(["stable"; "unstable"], cls));
    if numel(cls) < 2 || (~hasActionClasses && ~hasLegacyClasses)
        return;
    end

    mu = double(model.mu);
    s2 = double(model.sigma2);
    pr = double(model.prior(:));

    if isempty(mu) || isempty(s2) || numel(pr) < 2
        return;
    end
    if any(size(mu) ~= size(s2))
        return;
    end
    if size(mu, 1) ~= numel(cls)
        return;
    end
    if any(~isfinite(mu), 'all') || any(~isfinite(s2), 'all') || any(~isfinite(pr))
        return;
    end
    if any(s2(:) <= 0)
        return;
    end

    tf = true;
end


