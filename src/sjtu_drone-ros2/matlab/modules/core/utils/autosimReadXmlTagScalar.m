function val = autosimReadXmlTagScalar(filePath, tagName)
    val = nan;
    try
        txt = fileread(filePath);
    catch
        return;
    end

    pat = ['<' regexptranslate('escape', tagName) '>\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*</' regexptranslate('escape', tagName) '>'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        return;
    end
    num = str2double(tok{1});
    if isfinite(num)
        val = num;
    end
end


