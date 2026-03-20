function autosimKillTree(pid)
    if ~isfinite(pid) || pid <= 1
        return;
    end
    cmd = sprintf('bash -i -c "pkill -9 -P %d >/dev/null 2>&1 || true; kill -9 %d >/dev/null 2>&1 || true"', round(pid), round(pid));
    system(cmd);
end


