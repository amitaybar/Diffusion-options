function P = StateMatToP(StateMat,KStates,KDim1,KDim2)
% The function gets a state matrix and outputs P 
P = zeros(KStates,KStates); 

for CurState=1:KStates
    if StateMat(CurState)==0 
        continue;
    end

    P(CurState,CurState+1)      = StateMat(CurState+1); 
    P(CurState,CurState-1)      = StateMat(CurState-1);
    P(CurState,CurState+KDim1)  = StateMat(CurState+KDim1);
    P(CurState,CurState-KDim2)  = StateMat(CurState-KDim2);
    
end
