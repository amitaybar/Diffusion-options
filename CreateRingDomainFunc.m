function [StateMat,KDim1,KDim2] = CreateRingDomainFunc()
% The function creates a ring domain.

%%
KDim1           = 20; 
KDim2           = 20; 
CenterPoint     = [KDim1/2 KDim2/2];
RSmall          = 2.4;
RLarge          = 8.2;
StateMat        = ones(KDim1,KDim2);

for ii=1:KDim1
    for jj=1:KDim2
        if (ii-CenterPoint(1))^2+(jj-CenterPoint(2))^2 < RSmall^2 || ...
                (ii-CenterPoint(1))^2+(jj-CenterPoint(2))^2 > RLarge^2
            StateMat(ii,jj) = 0;
        end
    end
end
