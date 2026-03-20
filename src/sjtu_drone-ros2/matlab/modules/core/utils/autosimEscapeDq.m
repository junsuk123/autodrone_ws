function s = autosimEscapeDq(x)
    s = strrep(char(string(x)), '"', '\\"');
end


