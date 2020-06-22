function [StateMat,KDim1,KDim2] = CreateFourRoomsDomainFunc()
% The function creates a 4 rooms domain.

WallRow                                     = 5;%
DoorLoc                                     = [WallRow   6];
KDim1                                       = 13;
KDim2                                       = 13;
StateMat                                    = CreateStateMat(KDim1,KDim2,DoorLoc,WallRow);
StateMat(WallRow,2:end-1)                   = 1;
WallsLocCol                                 = 7;
WallsLocRow1                                = 7;
WallsLocRow2                                = 8;
StateMat(WallsLocRow1,1:round(end/2))       = 0;
StateMat(WallsLocRow2,round(end/2)+1:end)   = 0;
StateMat(:,WallsLocCol)                     = 0;
StateMat([4 11],WallsLocCol)                = 1;
StateMat(WallsLocRow1,3)                    = 1;
StateMat(WallsLocRow2,10)                   = 1;