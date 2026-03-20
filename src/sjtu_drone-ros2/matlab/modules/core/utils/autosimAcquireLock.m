function lockCleanup = autosimAcquireLock(cfg)
    lockPath = cfg.paths.lock_file;
    thisPid = feature('getpid');
    thisProc = autosimGetProcessInfo(thisPid);

    if isfile(lockPath)
        oldLock = autosimReadLockInfo(lockPath);
        if isfinite(oldLock.pid) && oldLock.pid > 1
            proc = autosimGetProcessInfo(round(oldLock.pid));
            if proc.exists
                sameProcess = true;
                if isfinite(oldLock.start_ticks) && isfinite(proc.start_ticks)
                    sameProcess = (oldLock.start_ticks == proc.start_ticks);
                end

                % Allow restart in the same MATLAB process when a stale lock remains
                % (e.g., interrupted run that skipped cleanup callback).
                if sameProcess && (round(oldLock.pid) == round(thisPid))
                    warning('[AUTOSIM] Recovering lock from same MATLAB process (pid=%d).', round(oldLock.pid));
                elseif sameProcess && autosimIsLikelyAutoSimProcess(proc.cmdline)
                    error('Another AutoSim instance is running (pid=%d). Stop it first.', round(oldLock.pid));
                else
                    warning('[AUTOSIM] Replacing stale lock (pid=%d).', round(oldLock.pid));
                end
            end
        end
    end

    fid = fopen(lockPath, 'w');
    if fid < 0
        error('Failed to create lock file: %s', lockPath);
    end
    fprintf(fid, 'pid=%d\n', round(thisPid));
    if isfinite(thisProc.start_ticks)
        fprintf(fid, 'start_ticks=%.0f\n', thisProc.start_ticks);
    else
        fprintf(fid, 'start_ticks=nan\n');
    end
    if strlength(thisProc.cmdline) > 0
        safeCmd = regexprep(char(thisProc.cmdline), '[\r\n]+', ' ');
        fprintf(fid, 'cmdline=%s\n', safeCmd);
    end
    fclose(fid);

    lockCleanup = onCleanup(@() autosimReleaseLock(lockPath));
end


