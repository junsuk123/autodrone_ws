function viz = autosimInitScenarioRealtimePlot(cfg, scenarioId, scenarioCfg)
    figName = 'AutoSim Ontology Live View';
    fig = findobj('Type', 'figure', 'Name', figName);
    if isempty(fig) || ~isgraphics(fig)
        fig = figure('Name', figName, 'NumberTitle', 'off');
    else
        figure(fig);
        clf(fig);
    end
    set(fig, 'CloseRequestFcn', @(src, evt) autosimHandleStopFigureClose(src, sprintf("scenario_plot_closed_s%03d", scenarioId)));
    autosimMaximizeFigure(fig);

    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    axSensor = nexttile(tl, 1);
    sensorBar = bar(axSensor, zeros(8,1), 0.65, 'FaceColor', [0.16 0.46 0.74]);
    ylim(axSensor, [0 1]);
    xticks(axSensor, 1:8);
    xticklabels(axSensor, {'wind','roll','pitch','altitude','vz','tag err','jitter','tag stable'});
    ylabel(axSensor, 'normalized');
    title(axSensor, 'Sensor Snapshot', 'Interpreter', 'none');
    grid(axSensor, 'on');
    sensorLabels = gobjects(8,1);
    for i = 1:8
        sensorLabels(i) = text(axSensor, i, 0.03, '', 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'Rotation', 90, 'FontSize', 7, 'Color', [0.15 0.15 0.15], ...
            'Interpreter', 'none');
    end

    axConcept = nexttile(tl, 2);
    conceptBar = bar(axConcept, zeros(5,1), 0.65, 'FaceColor', [0.20 0.62 0.38]);
    ylim(axConcept, [0 1]);
    xticks(axConcept, 1:5);
    xticklabels(axConcept, {'wind flow','alignment','visual lock','control','feasibility'});
    ylabel(axConcept, 'score');
    title(axConcept, 'Ontology Meaning', 'Interpreter', 'none');
    grid(axConcept, 'on');
    conceptLabels = gobjects(5,1);
    for i = 1:5
        conceptLabels(i) = text(axConcept, i, 0.03, '', 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'Rotation', 90, 'FontSize', 7, 'Color', [0.15 0.15 0.15], ...
            'Interpreter', 'none');
    end

    axTrend = nexttile(tl, 3);
    hold(axTrend, 'on');
    trendProb = animatedline(axTrend, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5);
    trendFeas = animatedline(axTrend, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
    trendCtx = animatedline(axTrend, 'Color', [0.30 0.30 0.30], 'LineWidth', 1.2, 'LineStyle', '--');
    yline(axTrend, cfg.agent.prob_land_threshold, ':', 'Color', [0.00 0.45 0.74]);
    yline(axTrend, cfg.agent.semantic_land_threshold, ':', 'Color', [0.85 0.33 0.10]);
    ylim(axTrend, [0 1]);
    xlabel(axTrend, 't [s]');
    ylabel(axTrend, 'readiness');
    title(axTrend, 'Landing Readiness Trend', 'Interpreter', 'none');
    legend(axTrend, {'decision confidence','safe-to-land score','semantic safety'}, 'Location', 'best', 'FontSize', 8);
    grid(axTrend, 'on');

    axFlow = nexttile(tl, 4);
    hold(axFlow, 'on');
    axis(axFlow, [0 1 0 1]);
    axis(axFlow, 'off');
    set(axFlow, 'XLim', [0 1], 'YLim', [0 1], 'XLimMode', 'manual', 'YLimMode', 'manual');
    flowTitle = title(axFlow, sprintf('Scenario %03d Ontology Flow (hover=%.2fm)', ...
        scenarioId, scenarioCfg.hover_height_m), 'FontSize', 10, 'Interpreter', 'none');

    text(axFlow, 0.13, 0.965, 'Sensors', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.18 0.36 0.64], 'Interpreter', 'none');
    text(axFlow, 0.47, 0.965, 'Meaning', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.20 0.55 0.30], 'Interpreter', 'none');
    text(axFlow, 0.86, 0.965, 'Result', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.66 0.44 0.14], 'Interpreter', 'none');

    summaryBox = rectangle(axFlow, 'Position', [0.30 0.855 0.40 0.06], 'Curvature', 0.08, ...
        'FaceColor', [0.93 0.94 0.96], 'EdgeColor', [0.45 0.48 0.55], 'LineWidth', 1.4);
    summaryText = text(axFlow, 0.50, 0.885, 'Decision Pending', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.15 0.15 0.18], ...
        'Interpreter', 'none');

    sensorPos = [0.13 0.76; 0.13 0.60; 0.13 0.44; 0.13 0.28];
    conceptPos = [0.47 0.76; 0.47 0.60; 0.47 0.44; 0.47 0.28];
    resultPos = [0.86 0.68; 0.86 0.50; 0.86 0.32];
    sensorRectPos = [0.03 0.70 0.20 0.11; 0.03 0.54 0.20 0.11; 0.03 0.38 0.20 0.11; 0.03 0.22 0.20 0.11];
    conceptRectPos = [0.37 0.70 0.20 0.11; 0.37 0.54 0.20 0.11; 0.37 0.38 0.20 0.11; 0.37 0.22 0.20 0.11];
    resultRectPos = [0.74 0.61 0.24 0.12; 0.74 0.43 0.24 0.12; 0.74 0.25 0.24 0.12];

    sensorNodes = gobjects(4,1);
    conceptNodes = gobjects(4,1);
    resultNodes = gobjects(3,1);
    sensorRects = gobjects(4,1);
    conceptRects = gobjects(4,1);
    resultRects = gobjects(3,1);
    for i = 1:4
        sensorRects(i) = rectangle(axFlow, 'Position', sensorRectPos(i,:), 'Curvature', 0.05, ...
            'FaceColor', [0.88 0.92 0.98], 'EdgeColor', [0.30 0.45 0.65], 'LineWidth', 1.2);
        sensorNodes(i) = text(axFlow, sensorPos(i,1), sensorPos(i,2), '', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.12 0.18 0.28], 'Interpreter', 'none');
        conceptRects(i) = rectangle(axFlow, 'Position', conceptRectPos(i,:), 'Curvature', 0.05, ...
            'FaceColor', [0.89 0.96 0.90], 'EdgeColor', [0.28 0.55 0.35], 'LineWidth', 1.2);
        conceptNodes(i) = text(axFlow, conceptPos(i,1), conceptPos(i,2), '', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.10 0.24 0.12], 'Interpreter', 'none');
    end
    for i = 1:3
        resultRects(i) = rectangle(axFlow, 'Position', resultRectPos(i,:), 'Curvature', 0.05, ...
            'FaceColor', [0.97 0.93 0.85], 'EdgeColor', [0.70 0.50 0.18], 'LineWidth', 1.2);
        resultNodes(i) = text(axFlow, resultPos(i,1), resultPos(i,2), '', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 7.5, 'FontWeight', 'bold', 'Color', [0.28 0.20 0.08], 'Interpreter', 'none');
    end

    sensorToConcept = [1 1; 1 4; 2 2; 2 4; 3 2; 3 3; 3 4; 4 4];
    conceptToResult = [1 1; 2 1; 3 1; 4 1; 2 2; 3 2; 4 2; 1 3; 4 3];
    arrowSensor = gobjects(size(sensorToConcept,1), 1);
    arrowConcept = gobjects(size(conceptToResult,1), 1);
    for i = 1:size(sensorToConcept,1)
        src = sensorPos(sensorToConcept(i,1), :);
        dst = conceptPos(sensorToConcept(i,2), :);
        arrowSensor(i) = quiver(axFlow, src(1)+0.08, src(2), dst(1)-src(1)-0.16, dst(2)-src(2), 0, ...
            'Color', [0.72 0.72 0.72], 'LineWidth', 1.1, 'MaxHeadSize', 0.35);
    end
    for i = 1:size(conceptToResult,1)
        src = conceptPos(conceptToResult(i,1), :);
        dst = resultPos(conceptToResult(i,2), :);
        arrowConcept(i) = quiver(axFlow, src(1)+0.09, src(2), dst(1)-src(1)-0.18, dst(2)-src(2), 0, ...
            'Color', [0.75 0.75 0.75], 'LineWidth', 1.1, 'MaxHeadSize', 0.35);
    end

    flowText = text(axFlow, 0.03, 0.05, '', 'Units', 'normalized', 'VerticalAlignment', 'bottom', ...
        'FontName', 'Courier New', 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.16 0.16 0.18], ...
        'Interpreter', 'none');

    viz = struct();
    viz.fig = fig;
    viz.cfg = cfg;
    viz.axSensor = axSensor;
    viz.sensorBar = sensorBar;
    viz.sensorLabels = sensorLabels;
    viz.axConcept = axConcept;
    viz.conceptBar = conceptBar;
    viz.conceptLabels = conceptLabels;
    viz.axTrend = axTrend;
    viz.trendProb = trendProb;
    viz.trendFeas = trendFeas;
    viz.trendCtx = trendCtx;
    viz.axFlow = axFlow;
    viz.flowTitle = flowTitle;
    viz.scenarioId = scenarioId;
    viz.hoverHeightM = scenarioCfg.hover_height_m;
    viz.summaryBox = summaryBox;
    viz.summaryText = summaryText;
    viz.sensorRects = sensorRects;
    viz.sensorNodes = sensorNodes;
    viz.conceptRects = conceptRects;
    viz.conceptNodes = conceptNodes;
    viz.resultRects = resultRects;
    viz.resultNodes = resultNodes;
    viz.arrowSensor = arrowSensor;
    viz.arrowConcept = arrowConcept;
    viz.sensorToConcept = sensorToConcept;
    viz.conceptToResult = conceptToResult;
    viz.flowText = flowText;
end


