function [NextStateOut, ActionOut] = ChooseNextStateUsingValueFuncWithActOut(ActionsValid,CurState,KDim1,ValueFunc)
% ActionsValid =  AllowedActionsCell{CurState};
for a=1:length(ActionsValid)
    NextState(a)    = Act(ActionsValid(a),CurState,KDim1);    
    ValueFuncVec(a) = ValueFunc(NextState(a));
end
[MaxValueFunc MaxInd]        = max(ValueFuncVec);
%%  
ValueFuncVecTmp = ValueFuncVec;
ValueFuncVecTmp(MaxInd) = -inf;
if abs(max(ValueFuncVecTmp) - MaxValueFunc)< 1e-10
    NextStates          = NextState(ValueFuncVec == max(ValueFuncVecTmp));
else
%%
    NextStates          = NextState(ValueFuncVec == MaxValueFunc);
end
% Choosing randomly between states with identical value function
NextStateOut           = NextStates(randi(length(NextStates)));

ActionOut = ActionsValid(NextState==NextStateOut);