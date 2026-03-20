function val = autosimReadYamlScalar(filePath, key)
    val = nan;
    try
        txt = fileread(filePath);
    catch
        return;
    end

    pat = ['(?m)^\s*' regexptranslate('escape', key) '\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        return;
    end
    num = str2double(tok{1});
    if isfinite(num)
        val = num;
    end
end


