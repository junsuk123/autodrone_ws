function msgType = autosimDetectTopicType(cfg, topicName)
    msgType = "";

    detectEnabled = true;
    timeoutSec = 2.0;
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'bumper_topic_type_detect_enable')
        detectEnabled = logical(cfg.ros.bumper_topic_type_detect_enable);
    end
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'bumper_topic_type_detect_timeout_sec') && isfinite(cfg.ros.bumper_topic_type_detect_timeout_sec)
        timeoutSec = max(0.5, cfg.ros.bumper_topic_type_detect_timeout_sec);
    end
    if ~detectEnabled
        return;
    end

    setupCmd = "";
    if isfield(cfg, 'shell') && isfield(cfg.shell, 'setup_cmd')
        setupCmd = string(cfg.shell.setup_cmd);
    end

    topic = string(topicName);
    if strlength(topic) <= 0
        return;
    end

    cli = "ros2 topic info " + topic;
    if strlength(setupCmd) > 0
        fullCmd = sprintf('bash -i -c "%s && timeout %gs %s"', char(setupCmd), timeoutSec, char(cli));
    else
        fullCmd = sprintf('bash -i -c "timeout %gs %s"', timeoutSec, char(cli));
    end

    [st, out] = system(char(fullCmd));
    if st ~= 0 || strlength(string(out)) <= 0
        return;
    end

    tok = regexp(char(out), 'Type:\s*([^\s]+)', 'tokens', 'once');
    if ~isempty(tok)
        msgType = string(strtrim(tok{1}));
    end
end


