function [predLabel, predScore] = autosimPredictGaussianNB(model, X, cfg)
    if nargin < 3
        cfg = [];
    end

    if autosimIsModuleEnabled(cfg, 'ai_engine')
        try
            [predLabel, predScore] = autosim_ai_engine('predict_gnb', model, X);
            return;
        catch
        end
    end

    X = autosimSanitize(X);
    if isrow(X)
        X = reshape(X, 1, []);
    end

    n = size(X, 1);
    c = numel(model.class_names);
    logp = zeros(n, c);

    for i = 1:c
        mu = model.mu(i,:);
        s2 = model.sigma2(i,:);
        lp = -0.5 * sum(log(2*pi*s2) + ((X - mu).^2) ./ s2, 2);
        lp = lp + log(max(model.prior(i), eps));
        logp(:,i) = lp;
    end

    [mx, idx] = max(logp, [], 2);
    predLabel = strings(n,1);
    for k = 1:n
        predLabel(k) = string(model.class_names(idx(k)));
    end

    lse = zeros(n,1);
    for k = 1:n
        a = logp(k,:) - mx(k);
        lse(k) = mx(k) + log(sum(exp(a)));
    end
    ex = exp(logp - max(logp, [], 2));
    post = ex ./ max(sum(ex, 2), eps);
    cls = string(model.class_names(:));
    attemptIdx = find(cls == "AttemptLanding", 1, 'first');
    if isempty(attemptIdx)
        attemptIdx = find(cls == "stable", 1, 'first');
    end
    if isempty(attemptIdx)
        predScore = exp(mx - lse);
    else
        predScore = post(:, attemptIdx);
    end
end


