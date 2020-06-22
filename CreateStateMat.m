function StateMat = CreateStateMat(KDim1,KDim2,DoorLoc,WallRow)

StateMat                        = ones(KDim1,KDim2);
StateMat([1 KDim1],:)           = 0;
StateMat(:,[1 KDim2])           = 0;
StateMat(WallRow,:)             = 0;
for i=1:size(DoorLoc,1)
    StateMat(DoorLoc(i,1),DoorLoc(i,2)) = 1;
end