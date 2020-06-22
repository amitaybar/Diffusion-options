function [StateMat,KDim1,KDim2] = CreateMazeDomainFunc()
% The function creates a maze domain.
%%

KDim1           = 16;
KDim2           = 16;
DoorLoc         = [];
WallRow         = [];
StateMat        = CreateStateMat(KDim1,KDim2,DoorLoc,WallRow);
%%
% Horizontal walls
StateMat(4,2:7)     = 0;
StateMat(4,10:end)  = 0;

StateMat(7,4:8)     = 0;
StateMat(7,11:end)  = 0;

StateMat(10,10:end) = 0;

StateMat(13,4:7)    = 0;
StateMat(13,10:13)  = 0;
% Vertical walls
StateMat(10:13,4)   = 0;

StateMat(4:7,7)     = 0;
StateMat(10:end,7)  = 0;

StateMat(10:13,10)  = 0;