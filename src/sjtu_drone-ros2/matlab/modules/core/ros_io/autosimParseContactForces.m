function [hasContact, totalForce, fFL, fFR, fRL, fRR] = autosimParseContactForces(msg, msgTypeHint)
    hasContact = 0;
    totalForce = nan;
    fFL = nan;
    fFR = nan;
    fRL = nan;
    fRR = nan;

    if nargin < 2
        msgTypeHint = "";
    end

    parsed = false;

    [okStates, states] = autosimTryGetMessageField(msg, ["states", "States", "contacts", "Contacts", "contact_states", "contactStates"]);
    if okStates
        parsed = true;

        nState = numel(states);
        if nState <= 0
            hasContact = 0;
            totalForce = 0.0;
            fFL = 0.0;
            fFR = 0.0;
            fRL = 0.0;
            fRR = 0.0;
            return;
        end

        hasContact = 1;
        totalForce = 0.0;
        fFL = 0.0;
        fFR = 0.0;
        fRL = 0.0;
        fRR = 0.0;

        for i = 1:nState
            st = states(i);

            c1 = "";
            c2 = "";
            [okC1, c1Val] = autosimTryGetMessageField(st, ["collision1_name", "collision1Name", "collision_1_name", "name1", "collision_a"]);
            if okC1
                c1 = string(c1Val);
            end
            [okC2, c2Val] = autosimTryGetMessageField(st, ["collision2_name", "collision2Name", "collision_2_name", "name2", "collision_b"]);
            if okC2
                c2 = string(c2Val);
            end
            armKey = autosimContactArmKey(c1 + " " + c2);

            forceSum = 0.0;
            [okWrench, wrenches] = autosimTryGetMessageField(st, ["wrenches", "Wrenches", "contact_wrenches", "wrench", "forces"]);
            if okWrench
                for j = 1:numel(wrenches)
                    wj = wrenches(j);
                    [okForce, forceObj] = autosimTryGetMessageField(wj, ["force", "Force", "total", "vector"]);
                    if okForce
                        [okFx, fxVal] = autosimTryGetMessageField(forceObj, ["x", "X"]);
                        [okFy, fyVal] = autosimTryGetMessageField(forceObj, ["y", "Y"]);
                        [okFz, fzVal] = autosimTryGetMessageField(forceObj, ["z", "Z"]);
                        if okFx && okFy && okFz
                            fx = double(fxVal);
                            fy = double(fyVal);
                            fz = double(fzVal);
                            forceSum = forceSum + sqrt(fx*fx + fy*fy + fz*fz);
                        end
                    else
                        % Some custom messages flatten forces directly in each wrench entry.
                        forceSum = forceSum + autosimParseForceScalarFromCustomWrench(wj);
                    end
                end
            else
                forceSum = autosimParseForceScalarFromCustomWrench(st);
            end

            totalForce = totalForce + forceSum;
            switch armKey
                case "fl"
                    fFL = fFL + forceSum;
                case "fr"
                    fFR = fFR + forceSum;
                case "rl"
                    fRL = fRL + forceSum;
                case "rr"
                    fRR = fRR + forceSum;
            end
        end

        if totalForce <= 0
            hasContact = 0;
        end
        return;
    end

    % Flat custom message path: parse scalar force fields or numeric vector payload.
    [okTotal, totalVal] = autosimTryGetMessageField(msg, ["total_force", "totalForce", "contact_force", "impact_force", "force_total", "force", "sum_force"]);
    [okFl, flVal] = autosimTryGetMessageField(msg, ["arm_force_fl", "arm_fl_force", "fl_force", "force_fl", "front_left_force"]);
    [okFr, frVal] = autosimTryGetMessageField(msg, ["arm_force_fr", "arm_fr_force", "fr_force", "force_fr", "front_right_force"]);
    [okRl, rlVal] = autosimTryGetMessageField(msg, ["arm_force_rl", "arm_rl_force", "rl_force", "force_rl", "rear_left_force"]);
    [okRr, rrVal] = autosimTryGetMessageField(msg, ["arm_force_rr", "arm_rr_force", "rr_force", "force_rr", "rear_right_force"]);

    if okTotal || okFl || okFr || okRl || okRr
        parsed = true;
        totalForce = autosimCastFiniteOrDefault(totalVal, 0.0);
        fFL = autosimCastFiniteOrDefault(flVal, 0.0);
        fFR = autosimCastFiniteOrDefault(frVal, 0.0);
        fRL = autosimCastFiniteOrDefault(rlVal, 0.0);
        fRR = autosimCastFiniteOrDefault(rrVal, 0.0);
        if ~okTotal
            totalForce = fFL + fFR + fRL + fRR;
        end
        [okHas, hasVal] = autosimTryGetMessageField(msg, ["has_contact", "hasContact", "contact", "in_contact", "collision"]);
        if okHas
            hasContact = double(logical(hasVal));
        else
            hasContact = double(totalForce > 0.0);
        end
        return;
    end

    [okData, dataVal] = autosimTryGetMessageField(msg, ["data", "Data", "values", "forces"]);
    if okData
        dataNum = autosimToNumericVector(dataVal);
        if numel(dataNum) >= 6
            parsed = true;
            hasContact = double(dataNum(1) > 0.5);
            totalForce = max(0.0, dataNum(2));
            fFL = max(0.0, dataNum(3));
            fFR = max(0.0, dataNum(4));
            fRL = max(0.0, dataNum(5));
            fRR = max(0.0, dataNum(6));
            if totalForce <= 0
                totalForce = fFL + fFR + fRL + fRR;
            end
            if hasContact == 0
                hasContact = double(totalForce > 0.0);
            end
            return;
        elseif numel(dataNum) == 5
            parsed = true;
            totalForce = max(0.0, dataNum(1));
            fFL = max(0.0, dataNum(2));
            fFR = max(0.0, dataNum(3));
            fRL = max(0.0, dataNum(4));
            fRR = max(0.0, dataNum(5));
            hasContact = double(totalForce > 0.0);
            return;
        end
    end

    if strlength(string(msgTypeHint)) > 0 && contains(lower(string(msgTypeHint)), "contact")
        % If the topic type says contact but parser couldn't map fields, report zeros instead of NaN.
        hasContact = 0;
        totalForce = 0.0;
        fFL = 0.0;
        fFR = 0.0;
        fRL = 0.0;
        fRR = 0.0;
        return;
    end

    if parsed
        return;
    end
end


