function [T, sourceFiles, finalDatasetRoot] = autosimLoadAllFinalDataset(rootDir)
% Load all FinalDataset CSV files recursively.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end

finalDatasetRoot = fullfile(char(rootDir), 'FinalDataset');
T = table();
sourceFiles = strings(0, 1);

if ~isfolder(finalDatasetRoot)
    return;
end

files = dir(fullfile(finalDatasetRoot, '**', 'autosim_final_dataset_*.csv'));
if isempty(files)
    files = dir(fullfile(finalDatasetRoot, '**', '*.csv'));
end
if isempty(files)
    return;
end

[~, ord] = sort([files.datenum], 'ascend');
files = files(ord);

for i = 1:numel(files)
    p = fullfile(files(i).folder, files(i).name);
    try
        Ti = readtable(p);
    catch
        continue;
    end
    Ti.source_file = repmat(string(p), height(Ti), 1);
    if isempty(T)
        T = Ti;
    else
        shared = intersect(T.Properties.VariableNames, Ti.Properties.VariableNames, 'stable');
        T = [T(:, shared); Ti(:, shared)]; %#ok<AGROW>
    end
    sourceFiles(end+1, 1) = string(p); %#ok<AGROW>
end
end