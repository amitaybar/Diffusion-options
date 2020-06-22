function [StateMat,KDim1,KDim2] = CreateOneRoomDomainFunc()
% The function creates a one room domain.

WallRow                     = 5;
DoorLoc                     = [WallRow   6];
KDim1                       = 12;
KDim2                       = 12;
StateMat                    = CreateStateMat(KDim1,KDim2,DoorLoc,WallRow);
StateMat(WallRow,2:end-1)   = 1;