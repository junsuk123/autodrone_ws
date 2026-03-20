function val = autosimReadXmlAttributeScalar(filePath, tagHead, attrName)
    val = nan;
    try
        txt = fileread(filePath);
    catch
        return;
    end

    pat = [regexptranslate('escape', tagHead) '[^>]*' regexptranslate('escape', attrName) '\s*=\s*"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        return;
    end
    num = str2double(tok{1});
    if isfinite(num)
        val = num;
    end
end


