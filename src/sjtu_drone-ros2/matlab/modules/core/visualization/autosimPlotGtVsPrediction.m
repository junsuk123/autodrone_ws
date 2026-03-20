function autosimPlotGtVsPrediction(summaryTbl, traceStore, model, cfg, outPngPath)
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    dTbl = autosimBuildDecisionTable(summaryTbl);
    dEval = autosimEvaluateDecisionMetrics(dTbl);
    if dEval.n_valid == 0
        return;
    end

    sid = dTbl.scenario_id;
    gtSafe = dTbl.gt_safe;
    predLand = dTbl.pred_land;
    windCtx = autosimBuildScenarioWindContext(summaryTbl, traceStore, sid);

    fig = figure('Name', 'AutoSim GT vs Decision', 'NumberTitle', 'off');
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    stem(ax1, sid, double(gtSafe), 'Color', [0.10 0.60 0.10], 'Marker', 'o', 'LineWidth', 1.0);
    hold(ax1, 'on');
    stem(ax1, sid, double(predLand), 'Color', [0.85 0.33 0.10], 'Marker', 'x', 'LineWidth', 1.0);
    mismatchMask = dTbl.valid & (gtSafe ~= predLand);
    if any(mismatchMask)
        scatter(ax1, sid(mismatchMask), 0.5 * ones(sum(mismatchMask), 1), 44, ...
            'd', 'MarkerFaceColor', [0.95 0.78 0.12], 'MarkerEdgeColor', [0.15 0.15 0.15], ...
            'LineWidth', 0.8);
    end
    ylim(ax1, [-0.1 1.1]);
    yticks(ax1, [0 1]);
    yticklabels(ax1, {'unsafe/hover', 'safe/land'});
    xlabel(ax1, 'scenario');
    ylabel(ax1, 'class');
    title(ax1, 'GT Safety vs Algorithm Decision (Land/Abort)');
    if any(mismatchMask)
        legend(ax1, {'GT safe(1)/unsafe(0)', 'Decision land(1)/abort(0)', 'decision mismatch'}, ...
            'Location', 'southoutside', 'Orientation', 'horizontal');
    else
        legend(ax1, {'GT safe(1)/unsafe(0)', 'Decision land(1)/abort(0)'}, ...
            'Location', 'southoutside', 'Orientation', 'horizontal');
    end
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    ax2LegendHandles = gobjects(0, 1);
    ax2LegendLabels = {};

    yyaxis(ax2, 'left');
    hold(ax2, 'on');
    hCmd = plot(ax2, sid, windCtx.speed_cmd, '--', 'LineWidth', 1.2, 'Color', [0.45 0.45 0.45]);
    hMean = plot(ax2, sid, windCtx.mean_speed, '-', 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);
    hMax = plot(ax2, sid, windCtx.max_speed, '-', 'LineWidth', 1.4, 'Color', [0.75 0.20 0.20]);
    ylabel(ax2, 'wind speed [m/s]');
    ax2LegendHandles(end+1:end+3,1) = [hCmd; hMean; hMax];
    ax2LegendLabels(end+1:end+3) = {'cmd speed', 'mean speed', 'max speed'};

    yyaxis(ax2, 'right');
    hold(ax2, 'on');
    dirMin = -180;
    dirMax = 180;
    if isfield(cfg, 'wind')
        if isfield(cfg.wind, 'direction_min') && isfinite(cfg.wind.direction_min)
            dirMin = cfg.wind.direction_min;
        end
        if isfield(cfg.wind, 'direction_max') && isfinite(cfg.wind.direction_max)
            dirMax = cfg.wind.direction_max;
        end
    end
    windCats = unique(windCtx.characteristic, 'stable');
    windCats = windCats(strlength(windCats) > 0);
    if isempty(windCats)
        windCats = "unknown";
    end
    windColors = lines(numel(windCats));
    for i = 1:numel(windCats)
        mask = (windCtx.characteristic == windCats(i)) & isfinite(windCtx.direction_deg);
        if ~any(mask)
            continue;
        end
        hCat = scatter(ax2, sid(mask), windCtx.direction_deg(mask), 34, ...
            'filled', 'MarkerFaceColor', windColors(i,:), 'MarkerEdgeColor', [0.12 0.12 0.12], ...
            'LineWidth', 0.6);
        ax2LegendHandles(end+1,1) = hCat;
        ax2LegendLabels{end+1} = char("dir | " + autosimWindLegendLabel(windCats(i))); %#ok<AGROW>
    end
    ylabel(ax2, 'wind direction [deg]');
    ylim(ax2, [dirMin dirMax]);
    xlabel(ax2, 'scenario');
    title(ax2, 'Per-Episode Wind Context');
    grid(ax2, 'on');
    legend(ax2, ax2LegendHandles, ax2LegendLabels, 'Location', 'southoutside', 'Orientation', 'horizontal');

    subtitle(tl, sprintf(['Decision-focused evaluation | valid=%d safe=%d unsafe=%d | ' ...
        'TP=%d FP=%d FN=%d TN=%d | Acc=%.3f Prec=%.3f SafeRec=%.3f UnsafeReject=%.3f UnsafeLand=%.3f | ' ...
        'mean wind=%.2f m/s max wind=%.2f m/s'], ...
        dEval.n_valid, dEval.n_safe, dEval.n_unsafe, dEval.tp, dEval.fp, dEval.fn, dEval.tn, ...
        dEval.accuracy, dEval.precision, dEval.recall, dEval.specificity, dEval.unsafe_landing_rate, ...
        autosimNanMean(windCtx.mean_speed), autosimNanMean(windCtx.max_speed)));

    exportgraphics(fig, outPngPath, 'Resolution', 150);
end


