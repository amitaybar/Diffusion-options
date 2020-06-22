if any(AllowedActionsCell{CurState}==4) % the action "down" is possible
    if randi(StochasticRatio)==StochasticRatio
        ActionChosen = 4;
    else
        ActionChosen                    =  AllowedActionsCell{CurState}(randi(length(AllowedActionsCell{CurState})));
    end
else % action down is not possible
    ActionChosen                    =  AllowedActionsCell{CurState}(randi(length(AllowedActionsCell{CurState})));
end