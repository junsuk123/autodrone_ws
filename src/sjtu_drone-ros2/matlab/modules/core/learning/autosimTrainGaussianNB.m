function model = autosimTrainGaussianNB(X, y, featureNames, priorUniformBlend, cfg)
    if nargin < 5
        cfg = [];
    end

    if autosimIsModuleEnabled(cfg, 'ai_engine')
        try
            model = autosim_ai_engine('train_gnb', X, y, featureNames, priorUniformBlend);
            if isstruct(model)
                model.kind = "gaussian_nb";
                model.created_at = string(datetime('now'));
                model.placeholder = false;
                return;
            end
        catch
        end
    end

    X = autosimSanitize(X);
    [X, useGpu] = autosimMaybeToGpu(X, cfg);
    cls = unique(y);
    nClass = numel(cls);
    nFeat = size(X, 2);

    if nargin < 4
        priorUniformBlend = 0.55;
    end

    mu = zeros(nClass, nFeat);
    sigma2 = ones(nClass, nFeat);
    prior = zeros(nClass, 1);

    for i = 1:nClass
        mask = (y == cls(i));
        Xi = X(mask, :);
        prior(i) = max(mean(mask), eps);
        mu(i,:) = mean(Xi, 1);
        sigma2(i,:) = var(Xi, 0, 1);
        sigma2(i, sigma2(i,:) < 1e-6) = 1e-6;
    end

    priorUniformBlend = autosimClamp(priorUniformBlend, 0.0, 1.0);
    prior = (1.0 - priorUniformBlend) * prior + priorUniformBlend * (ones(nClass, 1) / max(nClass, 1));
    prior = prior / max(sum(prior), eps);

    if useGpu
        mu = gather(mu);
        sigma2 = gather(sigma2);
        prior = gather(prior);
    end

    model = struct();
    model.kind = "gaussian_nb";
    model.class_names = cls;
    model.feature_names = featureNames;
    model.mu = mu;
    model.sigma2 = sigma2;
    model.prior = prior;
    model.created_at = string(datetime('now'));
    model.placeholder = false;
end

function [Xout, useGpu] = autosimMaybeToGpu(Xin, cfg)
    Xout = Xin;
    useGpu = false;
    try
        if ~(isstruct(cfg) && isfield(cfg, 'runtime') && isfield(cfg.runtime, 'use_gpu') && cfg.runtime.use_gpu)
            return;
        end
        if isfield(cfg.runtime, 'gpu_device') && isfinite(cfg.runtime.gpu_device)
            gpuDevice(max(1, round(cfg.runtime.gpu_device)));
        else
            gpuDevice();
        end
        Xout = gpuArray(Xin);
        useGpu = true;
    catch
        Xout = Xin;
        useGpu = false;
    end
end


