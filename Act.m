function NextState = Act(Action,CurState,KDim1)
%%
switch  Action
    case 1 % Left
        NextState = CurState-KDim1;
    case 2 % Right
        NextState = CurState+KDim1;
    case 3 % Up
        NextState = CurState-1;
    case 4 % Down
        NextState = CurState+1;
end