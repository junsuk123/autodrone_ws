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
        [T, Ti] = autosimAlignDatasetTables(T, Ti);
        T = [T; Ti]; %#ok<AGROW>
    end
    sourceFiles(end+1, 1) = string(p); %#ok<AGROW>
end
end

function [A, B] = autosimAlignDatasetTables(A, B)
varsA = string(A.Properties.VariableNames);
varsB = string(B.Properties.VariableNames);
varsAll = [varsA, varsB(~ismember(varsB, varsA))];

for i = 1:numel(varsAll)
    vn = char(varsAll(i));
    hasA = ismember(vn, A.Properties.VariableNames);
    hasB = ismember(vn, B.Properties.VariableNames);
    if hasA && hasB
        [A.(vn), B.(vn)] = autosimCoerceColumnPair(A.(vn), B.(vn));
    elseif hasA
        B.(vn) = autosimMissingLike(A.(vn), height(B));
    else
        A.(vn) = autosimMissingLike(B.(vn), height(A));
    end
end

A = A(:, cellstr(varsAll));
B = B(:, cellstr(varsAll));
end

function [a, b] = autosimCoerceColumnPair(a, b)
numA = isnumeric(a) || islogical(a);
numB = isnumeric(b) || islogical(b);

if numA && numB
    a = double(a);
    b = double(b);
    return;
end

if numA || numB
    a = autosimToDouble(a);
    b = autosimToDouble(b);
    return;
end

if isstring(a) || isstring(b) || ischar(a) || ischar(b) || iscellstr(a) || iscellstr(b) || iscell(a) || iscell(b)
    a = string(a);
    b = string(b);
    return;
end

a = string(a);
b = string(b);
end

function out = autosimMissingLike(sampleCol, n)
if isnumeric(sampleCol) || islogical(sampleCol)
    out = nan(n, 1);
elseif isstring(sampleCol) || ischar(sampleCol) || iscellstr(sampleCol) || iscell(sampleCol)
    out = strings(n, 1);
else
    out = strings(n, 1);
end
end

function x = autosimToDouble(v)
if isnumeric(v) || islogical(v)
    x = double(v);
    return;
end

try
    x = str2double(string(v));
catch
    x = nan(size(v, 1), 1);
end

if isrow(x)
    x = x(:);
end
end