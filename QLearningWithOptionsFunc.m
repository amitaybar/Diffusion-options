function QLearningWithOptionsFunc(DiffusionOptionsValueFunc,LaplacianEignOptions,RandomOptionsValueFunc,StateMat,StatesNoTransitionIndx,FourRoomFlag,HardMazeDomainFlag,RingRoomFlag,KStates,KDim1,KDim2)
%% The code implements Q learning according to  Watking & Dayan 1992

%%
% close all
rng(3)
%% Numeric Params
%%---------Learning Parameters ---------
Gamma               = 0.9;   
Alpha               = 0.1;   
KStepsInEpisode     = 100;   
KTrials             = 100;  
KEpisodes           = 400;
%%---------Other parameters--------------
KActions            = 4;
KConfig             = 3;    
KMonteCarlo         = 30;
%% Flags
RandomOptionsFlag   = 0;    % 1: Use random options instead of primitive action. 0: use primitve actions only
%%
%%-------Parameters per domain---------
if FourRoomFlag
    StartState              = 25;    
    GoalStateDomain         = [];
    for i=0:4
        GoalStateDomain     = [ GoalStateDomain 93+13*i:93+13*i+5  ];
    end
elseif HardMazeDomainFlag
    StartState              = 18;
    GoalStateDomain         = [10*16+11:16:12*16+11 10*16+12:16:12*16+12];
elseif RingRoomFlag
    StartState              = 50;    
    GoalState               = 12*20+7;   
    GoalStateDomain         = [];
    for i=0:4
        GoalStateDomain     = [ GoalStateDomain (GoalState+i*20):((GoalState+i*20)+5)  ];
    end
end
%%
KOptions = length(DiffusionOptionsValueFunc); % The number of options

%%
StateNumbers            = 1:KStates;
ValidStateNumbersInd    = StateMat(:);
ValidStateNumbers       = StateNumbers(ValidStateNumbersInd==1);
%% Derivatives Params
KAccessStates               = sum(~StatesNoTransitionIndx);
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
for mc=1:KMonteCarlo
    %---------------------choosing random options--------------------
    if RandomOptionsFlag
        RandomOptionsSelection          = randperm(length(RandomOptionsValueFunc),KOptions);
        RandomOptionsValueFuncTrim      = RandomOptionsValueFunc(RandomOptionsSelection);
    end
    StateVisitationArrTmp           = zeros(KConfig,KStates); % array that computes the # visitation per state during learning
    for gs=1:length(GoalStateDomain)
        GoalState = GoalStateDomain(gs);
        Reward                      = nan(KStates,1);
        Reward(ValidStateNumbers)  	= zeros(KAccessStates,1);
        Reward(GoalState)           = 1;
        %% Flags
        OptionsTypeFlag             = 0;    % 2: Random Options          1: Diffusion options 0: Eigen options
        OnlyPrimitiveAction        	= 0;    % 1: Primitive actions only  0: using options.
        for confg=1:KConfig
            if confg==1
                OptionsTypeFlag         = 1;     
                OnlyPrimitiveAction  	= 0;     
            end
            if confg==2
                OptionsTypeFlag         = 0;     
                OnlyPrimitiveAction  	= 0;     
            end
            if confg==3
                OptionsTypeFlag         = 0;     
                if RandomOptionsFlag
                    OptionsTypeFlag     = 2;     
                    OnlyPrimitiveAction = 0;     
                else
                    OnlyPrimitiveAction = 1;     
                end
            end
            if confg==2 && HardMazeDomainFlag
                %  More eigenoptions only for the maze domain and for t=13
%                 KOptions = 25;
                KOptions = length(DiffusionOptionsValueFunc); % The number of options
            else
                KOptions = length(DiffusionOptionsValueFunc); % The number of options
            end

            %% Initializing
            QFunc                       = nan(KStates,KActions);
            for i=1:length(AllowedActionsCell)
                QFunc(i,AllowedActionsCell{i}) = 0;
            end
            QFunc(GoalState,AllowedActionsCell{GoalState})  = 1;            
            Steps2GoalPerEpisode                            = nan(KEpisodes,1);
            Steps2GoalPerEpisode                            = (KTrials+1)*ones(KEpisodes,1);
            %%
            for e=1:KEpisodes
                InitState       = StartState;  
                CurState       	= InitState;
                StepsCounter    = 0;
                for step = 1:KStepsInEpisode
                    if StepsCounter>KStepsInEpisode
                        break;
                    end
                    MaxQFuncIndxCurState                = find(QFunc(CurState,:)==max(QFunc(CurState,:)));  
                    IsOneMaximum                        = length(MaxQFuncIndxCurState) == 1;  
                    if IsOneMaximum   
                        ActionChosen                    = MaxQFuncIndxCurState;
                        NextState                       = Act(ActionChosen,CurState,KDim1);
                        QFunc(CurState,ActionChosen)    = (1-Alpha)*QFunc(CurState,ActionChosen) + Alpha*( Reward(CurState)+Gamma*max(QFunc(NextState,:)) );
                        CurState                        = NextState;
                    else  
                        %% choosing randomly w.p 0.5 between primitive actions and options
                        IsOptionChosen = randi(2)==2;
                        if IsOptionChosen*(~OnlyPrimitiveAction)        % option
                            OptionToDo          = randi(KOptions);
                            %--Extracting the value function of the option
                            if OptionsTypeFlag==1
                                OptionToDoValueFunc   = DiffusionOptionsValueFunc{OptionToDo};
                            elseif OptionsTypeFlag==0  
                                OptionToDoValueFunc   = LaplacianEignOptions{OptionToDo};
                            else %OptionsTypeFlag=2 % random options
                                OptionToDoValueFunc   = RandomOptionsValueFuncTrim{OptionToDo};
                            end
                            %% Extrating the goal state of the option
                            [~,OptionGoalState] = max(OptionToDoValueFunc);                             
                            while ~any(CurState==OptionGoalState)                                   
                                [NextState, ActionChosen]= ChooseNextStateUsingValueFuncWithActOut(AllowedActionsCell{CurState},CurState,KDim1,OptionToDoValueFunc);
                                StepsCounter = StepsCounter + 1;
                                if StepsCounter>KStepsInEpisode
                                    break;
                                end
                                if OptionToDoValueFunc(NextState)<=OptionToDoValueFunc(CurState) % Option terminates
                                    break;
                                end
                                %-----updating Q function
                                QFunc(CurState,ActionChosen)            = (1-Alpha)*QFunc(CurState,ActionChosen) + Alpha*( Reward(CurState)+Gamma*max(QFunc(NextState,:)) );
                                CurState                                = NextState;
                                StateVisitationArrTmp(confg,CurState)   = StateVisitationArrTmp(confg,CurState)+1;
                            end
                        else % Primitive action
                            ActionChosen                    =  AllowedActionsCell{CurState}(randi(length(AllowedActionsCell{CurState})));
                            NextState                       = Act(ActionChosen,CurState,KDim1);
                            QFunc(CurState,ActionChosen)    = (1-Alpha)*QFunc(CurState,ActionChosen) + Alpha*( Reward(CurState)+Gamma*max(QFunc(NextState,:)) );
                            if NextState==GoalState
                                StepsCounter = StepsCounter + 1;
                                break;
                            end
                            CurState                        = NextState;
                            if isempty(AllowedActionsCell{NextState}    )
                            end
                        end % if IsOptionChosen
                    end
                    StepsCounter                            = StepsCounter + 1;
                    StateVisitationArrTmp(confg,CurState)   = StateVisitationArrTmp(confg,CurState)+1;
                end % steps
                
                %% measuring performance  
                Steps2GoalCount                     = 0;
                CurStateTest                        = InitState;
                for t=1:KTrials                    
                    MaxQFuncIndxCurStateTest       	= find(QFunc(CurStateTest,:)==max(QFunc(CurStateTest,:)));
                    ActionChosenTest                = MaxQFuncIndxCurStateTest(randi(length(MaxQFuncIndxCurStateTest)));  
                    NextStateTest                   = Act(ActionChosenTest,CurStateTest,KDim1);
                    CurStateTest                    = NextStateTest;
                    Steps2GoalCount                 = Steps2GoalCount + 1;
                    if CurStateTest==GoalState
                        Steps2GoalPerEpisode(e)     = t;
                        break;
                    end
                end % trial
                %%
                
            end % episodes
            Steps2GoalPerConfgPerEpisodeTmp(confg,:)            = Steps2GoalPerEpisode;
            Steps2GoalPerConfgPerEpisodePerGoalTmp(gs,confg,:)  = Steps2GoalPerEpisode;
        end % confg - Configuration is : diffusion options, Eigenoptions, primitive actions         
    end % gs - goal state
    disp(['Monte Carlo itereation: ' num2str(mc) ' out of ' num2str(KMonteCarlo)]);
    Steps2GoalPerConfgPerEpisodeTmp2(mc,:,:)                    = Steps2GoalPerConfgPerEpisodeTmp;
    Steps2GoalPerConfgPerEpisodePerGoalTmp2(mc,:,:,:)           = Steps2GoalPerConfgPerEpisodePerGoalTmp;
    StateVisitationArrTmp2(mc,:,:)                              = StateVisitationArrTmp;
end %mc
Steps2GoalPerConfgPerEpisodeTmp                                 = squeeze(mean(Steps2GoalPerConfgPerEpisodeTmp2,1));
Steps2GoalPerConfgPerEpisodePerGoalTmp                          = squeeze(mean(Steps2GoalPerConfgPerEpisodePerGoalTmp2,1));
StateVisitationArrTmp                                           = squeeze(mean(StateVisitationArrTmp2,1));
%%
Steps2GoalPerConfgPerEpisodePerGoalMean                         = squeeze(mean(Steps2GoalPerConfgPerEpisodePerGoalTmp,1));
Steps2GoalPerConfgPerEpisodePerGoalStd                          = squeeze(std(Steps2GoalPerConfgPerEpisodePerGoalTmp,[],1));                        
%% Ploting
%----------------variables and params for plotting-----------------
XAxisEpisodes   = 1:KEpisodes;
alpha           = 0.5;
TypeOfLine = {'-','--','-.'};
Colors          =   [0         0.4470    0.7410;
                    0.8500    0.3250    0.0980;
                    0.9290    0.6940    0.1250;];
figure;hold on
for OptionType=1:3
acolor          = Colors(OptionType,:);
plot(XAxisEpisodes,Steps2GoalPerConfgPerEpisodePerGoalMean(OptionType,:),'color',acolor,'LineStyle',TypeOfLine{OptionType},'linewidth',1.5); 
end
for OptionType=1:3
acolor          = Colors(OptionType,:);
fill([XAxisEpisodes fliplr(XAxisEpisodes)],[Steps2GoalPerConfgPerEpisodePerGoalMean(OptionType,:)+ ...
      Steps2GoalPerConfgPerEpisodePerGoalStd(OptionType,:) fliplr(Steps2GoalPerConfgPerEpisodePerGoalMean(OptionType,:)-Steps2GoalPerConfgPerEpisodePerGoalStd(OptionType,:))],...
      acolor, 'FaceAlpha', alpha,'linestyle','none');  
end
set(gca,'FontSize',18)
xlabel('Episode');
ylabel('Steps to goal')
if RandomOptionsFlag
    legend('Diffusion options','Eigenoptions','Random options','location','best');
else
    legend('Diffusion options','Eigenoptions','Primitive Actions','location','best');
end
chi         = get(gca, 'Children');
chiNew(1:3) = chi(3:-1:1);
chiNew(4:6) = chi(6:-1:4);
set(gca, 'Children',chiNew);
grid off
box on
%%
%% Ploting Visitation map
load('MapBlueNew.mat');
TitleStr = {'diffusion options';'eigenoptions'};
if RandomOptionsFlag
    TitleStr(end+1) = {'random options'};
else
    TitleStr(end+1) = {'primitive actions'};
end
for c=1:KConfig
    Visitation4Plot                                 = zeros(KStates,1);
    Visitation4Plot(StateVisitationArrTmp(c,:)~=0)  = StateVisitationArrTmp(c,StateVisitationArrTmp(c,:)~=0)/max(StateVisitationArrTmp(c,StateVisitationArrTmp(c,:)~=0));
    Visitation4PlotMat                              = reshape(Visitation4Plot,KDim1,KDim2);
    
    figure;
    Im = imagesc(Visitation4PlotMat);
    set(Im,'AlphaData',Visitation4PlotMat>0)    
    title(['Normalized visitation count for ' TitleStr{c}]);
    axis off
    c                   = colorbar;
    colormap(flipud(MapBlueNew))
    c.Label.String      = 'N_0';
    pos                 = c.Position;
    c.Label.Position    = [pos(1)/2+0.2 pos(2)-.11];
    c.Label.Rotation    = 0;
    c.Label.FontSize    = 14;
    
end


