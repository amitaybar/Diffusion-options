%%
% The code calculates the average number of steps between every pair of states
% in the domain
%%
% close all
rng(3)
%% Numeric Params
% GoalState   = 69; % the goal state is determined below and it scans all the states
KTries      = 5e3;
Gamma       = 1;
KActions    = 4;
%% Flags
DiffusionOptionsFlag        = 1;    % 1: Diffusion options. 0:  Eigenoptions 
OnlyPrimitiveAction         = 0;    % 1: Only primitive actions, without options. 0: using options.
%%
StateNumbers                = 1:KStates;
ValidStateNumbersInd        = StateMat(:);
ValidStateNumbers           = StateNumbers(ValidStateNumbersInd==1);
%%
for st=1:length(ValidStateNumbers)
    disp(['Current state is: ' num2str(st) ' out of ' num2str(length(ValidStateNumbers)) ' states']);
    GoalState                                                   = ValidStateNumbers(st);
    ValidStateNumbersNoGoal                                     = ValidStateNumbers;
    ValidStateNumbersNoGoal(ValidStateNumbersNoGoal==GoalState) = [];
    %% Derivatives Params
    KAccessStates                                               = sum(~StatesNoTransitionIndx);
    Reward                                                      = nan(KStates,1);
    Reward(ValidStateNumbers)                                   = ones(KAccessStates,1);
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
    %% Initializing
    ValueFunc                               = nan(KStates,1);
    ValueFunc(ValidStateNumbers)            = zeros(KAccessStates,1);
    %%
    if exist('DiffusionOptionsValueFunc') % making sure it exists. Is created in Main.m
        KOptions = length(DiffusionOptionsValueFunc); 
    else
        KOptions = 15; % default  value - not used
    end
    for i=1:KTries
%         i
        InitState   = ValidStateNumbersNoGoal(randi(length(ValidStateNumbersNoGoal)));
        CurState    = InitState;
        %%
        while CurState~=GoalState
            % choosing randomly w.p 0.5 between primitive actions and options            
            IsOptionChosen = randi(2)==2;            
            if IsOptionChosen*(~OnlyPrimitiveAction) % option                
                OptionToDo                  = randi(KOptions);                
                %--Extracting the value function of the option
                if DiffusionOptionsFlag
                    OptionToDoValueFunc     = DiffusionOptionsValueFunc{OptionToDo};
                else % Eigenoption
                    OptionToDoValueFunc     = LaplacianEignOptions{OptionToDo};
                end
                %% Extrating the goal state of the option
                [~,OptionGoalState] = max(OptionToDoValueFunc); 
                while ~any(CurState == OptionGoalState) 
                    %-- moving to the next step according to the option
                    NextState       = ChooseNextStateUsingValueFunc(AllowedActionsCell{CurState},CurState,KDim1,OptionToDoValueFunc);
                    PreviouCurState = NextState;
                    if OptionToDoValueFunc(NextState)<=OptionToDoValueFunc(CurState) % we have reached a local maxima and should terminate the option
                        break;
                    end
                    %-----updating value function                   
                    ValueFunc(CurState) = Reward(NextState) + Gamma*ValueFunc(NextState);                    
                    CurState            = NextState;                    
                    if GoalState == CurState
                        break;
                    end                    
                end                
            else % Primitive action
                CurAction   =  AllowedActionsCell{CurState}(randi(length(AllowedActionsCell{CurState})));
                %----performing the action----
                NextState           = Act(CurAction,CurState,KDim1);
                PrValueRand         = 1/length(AllowedActionsCell{CurState});                               
                ValueFuncTmp        = 0;
                for action=AllowedActionsCell{CurState}
                    NextStateV      = Act(action,CurState,KDim1);
                    ValueFuncTmp    = ValueFuncTmp +PrValueRand*( Reward(CurState) + Gamma*ValueFunc(NextStateV));
                end                
                ValueFunc(CurState) = ValueFuncTmp;             
                CurState = NextState;
            end % if IsOptionChosen
        end
    end
    ValueFuncTotCell{st}    = ValueFunc;
    MeanValFunction(st)     = mean(ValueFunc,'omitnan');
end %st - loop over all goal states

disp(['Median of #steps between every two states : ' num2str(median(MeanValFunction))]);
disp(['Iqr of    #steps between every two states : ' num2str(iqr(MeanValFunction))]);
disp(['Mean      #steps between every two states : ' num2str(mean(MeanValFunction))]);
disp(['Std of    #steps between every two states : ' num2str(std(MeanValFunction))]);



