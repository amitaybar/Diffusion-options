function ValueFuncOptions = DiffusionOptionsCreationFromStatesListFunc(StatesList,StateMat,KStates,StatesNoTransitionIndx,KDim1,KDim2)
% The code gets a list of option goal states and outputs their value function 

%% Parameters
Gamma       = 0.9;
KTries      = 5e3;
%%
StateNumbers            = 1:KStates;
ValidStateNumbersInd    = StateMat(:);
ValidStateNumbers       = StateNumbers(ValidStateNumbersInd==1);

%% Creating a cell array of possible actions for each state
% Actions order: 1-4: Left,Right,Up,Down
AllowedActionsCell = cell(KStates,1);
for s=ValidStateNumbers
    if StateMat(s)==0  
        continue;
    end
    AllowedActionsCell{s} = [4*StateMat(s+1) 3*StateMat(s-1) 2*StateMat(s+KDim1)...
        1*StateMat(s-KDim2)];
    AllowedActionsCell{s}(AllowedActionsCell{s}==0) = [];
end
%%
for op=1:length(StatesList)  
    disp(['Option #: ' num2str(op) ' out of ' num2str(length(StatesList)) ' options']);
    %%
    OptionGoalInd           = StatesList(op);
    GoalState               = OptionGoalInd;
    %%    
    ValidStateNumbersInd    = StateMat(:);
    ValidStateNumbers       = StateNumbers(ValidStateNumbersInd==1);
    % Removing goal state from valid states to init
    ValidStateNumbersNoGoal = ValidStateNumbers;
    ValidStateNumbersNoGoal(ValidStateNumbersNoGoal==GoalState) = [];
    
    %% Derivatives Params
    KAccessStates                               = sum(~StatesNoTransitionIndx);
    Reward                                      = nan(KStates,1);
    Reward(ValidStateNumbers)                   = 0;
    Reward(GoalState)                           = 1;
    %% Initializing
    ValueFuncOptions{op}                        = nan(KStates,1);
    ValueFuncOptions{op}(ValidStateNumbers) 	= zeros(KAccessStates,1);
    ValueFuncOptions{op}(GoalState)             = Reward(GoalState);
    %%
    for i=1:KTries
        InitState   = ValidStateNumbersNoGoal(randi(length(ValidStateNumbersNoGoal)));        
        CurState    = InitState;
        while CurState~=GoalState
            ActionsValid                    =  AllowedActionsCell{CurState};
            %----performing the action----
            NextState                       = ChooseNextStateUsingValueFunc(ActionsValid,CurState,KDim1,ValueFuncOptions{op});
            ValueFuncOptions{op}(CurState)  = Reward(CurState) + Gamma*ValueFuncOptions{op}(NextState);           
            ValueFuncOptions{op}(ValueFuncOptions{op}<1e-200) = 0;            
            CurState = NextState;
        end % while
    end % i - Tries
end %op
