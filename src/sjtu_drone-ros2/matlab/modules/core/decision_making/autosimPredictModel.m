function [predLabel, predScore] = autosimPredictModel(model, featStruct, featureNames, cfg)
    if nargin < 4
        cfg = [];
    end

    if autosimIsModuleEnabled(cfg, 'ai_engine')
        try
            [predLabel, predScore] = autosim_ai_engine('predict_model', model, featStruct, featureNames);
            return;
        catch
        end
    end

    X = zeros(1, numel(featureNames));
    for i = 1:numel(featureNames)
        fn = char(featureNames(i));
        if isfield(featStruct, fn)
            X(i) = double(featStruct.(fn));
        end
    end

    [lbl, score] = autosimPredictGaussianNB(model, X, cfg);
    predLabel = lbl(1);
    predScore = score(1);
end


