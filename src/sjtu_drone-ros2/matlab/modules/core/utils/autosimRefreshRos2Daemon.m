function autosimRefreshRos2Daemon()
    system('bash -i -c "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; ros2 daemon stop >/dev/null 2>&1 || true; ros2 daemon start >/dev/null 2>&1 || true"');
end


