function autosimAddGeneratedMsgPath(thisDir)
    % Try common ros2genmsg outputs near this repo so custom interfaces are available
    % even when savepath cannot write to system pathdef.m.
    candidates = {
        fullfile(thisDir, 'matlab_msg_gen', 'glnxa64', 'install', 'm'), ...
        fullfile(thisDir, 'matlab_msg_gen_ros2', 'glnxa64', 'install', 'm'), ...
        fullfile(thisDir, '..', '..', '..', 'matlab_msg_ws', 'matlab_msg_gen', 'glnxa64', 'install', 'm'), ...
        fullfile(thisDir, '..', '..', '..', 'matlab_msg_ws', 'matlab_msg_gen_ros2', 'glnxa64', 'install', 'm') ...
    };

    for i = 1:numel(candidates)
        p = char(candidates{i});
        if exist(p, 'dir')
            addpath(p);
            return;
        end
    end
end


