function [u, st] = autosimPidStep(err, dt, st, kp, ki, kd, iLimit, outLimit)
    if ~st.initialized
        st.prev_error = err;
        st.initialized = true;
    end

    st.integral = autosimClamp(st.integral + err * dt, -abs(iLimit), abs(iLimit));
    derr = (err - st.prev_error) / max(dt, 1e-6);
    st.prev_error = err;

    u = kp * err + ki * st.integral + kd * derr;
    u = autosimClamp(u, -abs(outLimit), abs(outLimit));
end


