function plotState = autosimInitPlots()
    plotState = struct();
    plotState.fig = figure('Name', 'AutoSim Decision Progress', 'NumberTitle', 'off');
    set(plotState.fig, 'CloseRequestFcn', @(src, evt) autosimHandleStopFigureClose(src, "progress_plot_closed"));
    autosimPlaceFigureRight(plotState.fig, [0.36, 0.46], [0.52, 0.51]);
    tl = tiledlayout(plotState.fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    plotState.ax1 = ax1;
    set(ax1, 'FontSize', 9);
    plotState.accLine = animatedline(ax1, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5);
    plotState.precLine = animatedline(ax1, 'Color', [0.20 0.60 0.20], 'LineWidth', 1.5);
    plotState.recLine = animatedline(ax1, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
    plotState.unsafeLine = animatedline(ax1, 'Color', [0.75 0.20 0.20], 'LineWidth', 1.6);
    plotState.execUnsafeLine = animatedline(ax1, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.6, 'LineStyle', '--');
    title(ax1, 'Decision Metrics Trend', 'FontSize', 10);
    xlabel(ax1, 'scenario', 'FontSize', 9);
    ylabel(ax1, 'score', 'FontSize', 9);
    ylim(ax1, [0 1]);
    grid(ax1, 'on');
    legend(ax1, {'accuracy', 'precision', 'safe recall', 'policy unsafe landing rate', 'executed unsafe landing rate'}, 'Location', 'best', 'FontSize', 8);

    ax2 = nexttile(tl, 2);
    plotState.ax2 = ax2;
    set(ax2, 'FontSize', 9);
    plotState.tpLine = animatedline(ax2, 'Color', [0.20 0.60 0.20], 'LineWidth', 1.4);
    plotState.fpLine = animatedline(ax2, 'Color', [0.75 0.20 0.20], 'LineWidth', 1.4);
    plotState.fnLine = animatedline(ax2, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4);
    plotState.tnLine = animatedline(ax2, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
    plotState.unsafeGtLine = animatedline(ax2, 'Color', [0.40 0.40 0.40], 'LineWidth', 1.4, 'LineStyle', '--');
    plotState.probeLine = animatedline(ax2, 'Color', [0.55 0.10 0.55], 'LineWidth', 1.4, 'LineStyle', ':');
    title(ax2, 'Decision Counts And Sample Mix', 'FontSize', 10);
    xlabel(ax2, 'scenario', 'FontSize', 9);
    ylabel(ax2, 'count', 'FontSize', 9);
    grid(ax2, 'on');
    legend(ax2, {'TP', 'FP', 'FN', 'TN', 'unsafe GT', 'probe episodes'}, 'Location', 'best', 'FontSize', 8);
end


