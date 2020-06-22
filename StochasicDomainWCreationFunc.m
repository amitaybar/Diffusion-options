function [W,D] = StochasicDomainWCreationFunc(W,StochasticRatio,KStates,StatesNoTransitionIndx,KDim1)

for i=1:KStates-1 - sum(StatesNoTransitionIndx)
    if mod(i+1,KDim1-2)==1 
        continue;
    end
    
    if W(i+1,i)==1
        switch sum(W(i+1,:))
            case 2
                W(i+1,i) = (StochasticRatio+1)/(StochasticRatio-1);
            case 3
                W(i+1,i) = (StochasticRatio+1)/(StochasticRatio-1);
            case 4
                W(i+1,i) = (StochasticRatio+3)/(StochasticRatio-1);
            otherwise
        end
                 
    end
end
D = diag(sum(W,2));