function key = autosimContactArmKey(nameText)
    s = lower(char(nameText));
    key = "";
    if contains(s, 'front_left') || contains(s, 'left_front') || contains(s, 'arm_fl') || contains(s, 'fl_')
        key = "fl";
    elseif contains(s, 'front_right') || contains(s, 'right_front') || contains(s, 'arm_fr') || contains(s, 'fr_')
        key = "fr";
    elseif contains(s, 'rear_left') || contains(s, 'back_left') || contains(s, 'left_rear') || contains(s, 'arm_rl') || contains(s, 'rl_')
        key = "rl";
    elseif contains(s, 'rear_right') || contains(s, 'back_right') || contains(s, 'right_rear') || contains(s, 'arm_rr') || contains(s, 'rr_')
        key = "rr";
    end
end


