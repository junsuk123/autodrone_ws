function monitor_autosim_parallel(sessionRoot, pollSec)
    if nargin < 1 || strlength(string(sessionRoot)) == 0
        error('Usage: monitor_autosim_parallel(''/path/to/matlab/parallel_runs/<timestamp>'', pollSec)');
    end
    if nargin < 2 || ~isfinite(pollSec)
        pollSec = 2.0;
    end

    sessionRoot = char(sessionRoot);
    fig = figure('Name', 'AutoSim Parallel Monitor', 'NumberTitle', 'off');
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    title(ax1, 'Per-worker Progress');
    ylabel(ax1, 'scenario count');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    hold(ax2, 'on');
    title(ax2, 'Aggregate Unsafe Landing Rate');
    xlabel(ax2, 'time [s]');
    ylabel(ax2, 'rate');
    ylim(ax2, [0 1]);
    grid(ax2, 'on');
    aggLine = animatedline(ax2, 'Color', [0.82 0.22 0.18], 'LineWidth', 1.8);

    t0 = tic;
    while isgraphics(fig)
        workerDirs = dir(fullfile(sessionRoot, 'output', 'data', 'worker_*'));
        workerNames = strings(0, 1);
        counts = [];
        unsafeRates = [];
        totalUnsafe = 0;
        totalValid = 0;

        for i = 1:numel(workerDirs)
            wdir = fullfile(workerDirs(i).folder, workerDirs(i).name);
            runDirs = dir(fullfile(wdir, '*'));
            runDirs = runDirs([runDirs.isdir]);
            runDirs = runDirs(~ismember({runDirs.name}, {'.', '..'}));
            if isempty(runDirs)
                continue;
            end

            [~, idxLatest] = max([runDirs.datenum]);
            latestRun = fullfile(runDirs(idxLatest).folder, runDirs(idxLatest).name);
            csvPath = fullfile(latestRun, 'autosim_dataset_latest.csv');
            if ~isfile(csvPath)
                continue;
            end

            try
                T = readtable(csvPath);
            catch
                continue;
            end

            nRows = height(T);
            nUnsafe = 0;
            nValid = 0;
            if ismember('gt_safe_to_land', T.Properties.VariableNames)
                gt = string(T.gt_safe_to_land);
                validMask = (gt == "stable") | (gt == "unsafe") | (gt == "unstable");
                unsafeMask = (gt == "unsafe") | (gt == "unstable");
                nValid = sum(validMask);
                nUnsafe = sum(unsafeMask & validMask);
            elseif ismember('label', T.Properties.VariableNames)
                gt = string(T.label);
                validMask = (gt == "stable") | (gt == "unstable");
                unsafeMask = (gt == "unstable");
                nValid = sum(validMask);
                nUnsafe = sum(unsafeMask & validMask);
            end

            wName = string(workerDirs(i).name);
            workerNames(end+1, 1) = wName; %#ok<AGROW>
            counts(end+1, 1) = nRows; %#ok<AGROW>
            if nValid > 0
                unsafeRates(end+1, 1) = nUnsafe / nValid; %#ok<AGROW>
            else
                unsafeRates(end+1, 1) = nan; %#ok<AGROW>
            end

            totalUnsafe = totalUnsafe + nUnsafe;
            totalValid = totalValid + nValid;
        end

        cla(ax1);
        if ~isempty(counts)
            yyaxis(ax1, 'left');
            b1 = bar(ax1, counts, 0.55, 'FaceColor', [0.14 0.44 0.72]); %#ok<NASGU>
            ylabel(ax1, 'scenario count');

            yyaxis(ax1, 'right');
            b2 = bar(ax1, unsafeRates, 0.30, 'FaceColor', [0.83 0.33 0.20]); %#ok<NASGU>
            ylabel(ax1, 'unsafe rate');
            ylim(ax1, [0 1]);

            xticks(ax1, 1:numel(workerNames));
            xticklabels(ax1, workerNames);
            legend(ax1, {'scenario count', 'unsafe rate'}, 'Location', 'best');
        end
        grid(ax1, 'on');

        if totalValid > 0
            addpoints(aggLine, toc(t0), totalUnsafe / totalValid);
        end
        drawnow limitrate nocallbacks;

        pause(max(0.2, pollSec));
    end
end
