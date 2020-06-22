%%
% The main code file.
% Creates the domains, the options and tests them. 
% The domain is created according to the flags (only one of them should be '1')
%%
close all
clear
clc
rng(6) 
%% Loading colormap for the figures
load('MapBlueNew.mat');
%% Flags
% Only one flag should be '1'
OneRoomFlag                     = 0;     
FourRoomFlag                    = 1;     
RingRoomFlag                    = 0;
HardMazeDomainFlag              = 0;     
%----Stochastic flag---------------------------
IsStochasticDomainFlag          = 0;    % 1: A stochastic domain - assymetric Laplacian. 0: Deterministic domain.
%----Exploration steps between states flag-----
IsExplorStepsStatesFlag         = 0;    % 1: Performs computation of average number of steps between every pair of states. 0: Avoids computation
%% Parameters
DiffusionTime                   = 4;    % time parameter of diffusion options [f_t(s)]
StochasticRatio                 = 3;    % 1/StochasticRatio is the probability of moving down regardless of the chosen action.
%% Creating the domain
if OneRoomFlag
    [StateMat,KDim1,KDim2]      = CreateOneRoomDomainFunc();
elseif FourRoomFlag
    [StateMat,KDim1,KDim2]      = CreateFourRoomsDomainFunc();
elseif RingRoomFlag
    [StateMat,KDim1,KDim2]      = CreateRingDomainFunc();
elseif HardMazeDomainFlag
    [StateMat,KDim1,KDim2]      = CreateMazeDomainFunc();
end
KStates                         = KDim1*KDim2;
%% Building transitin matrix according to StateMat
W                               = StateMatToP(StateMat,KStates,KDim1,KDim2);
%% Calculating transition matrix (excluding wall states that are inaccessible)
WSumRows                        = sum(W,2);
StatesNoTransitionIndx          = ~WSumRows;                             
WSumRowsTrim                    = WSumRows(~StatesNoTransitionIndx);    
DTrim                           = diag(WSumRowsTrim);
WTrim                           = W;
WTrim(StatesNoTransitionIndx,:) = [];
WTrim(:,StatesNoTransitionIndx) = [];
if IsStochasticDomainFlag
    [WTrim,DTrim] = StochasicDomainWCreationFunc(WTrim,StochasticRatio,KStates,StatesNoTransitionIndx,KDim1);
end
%% For random options
AccessibleStates                                = 1:KStates;
AccessibleStates(StatesNoTransitionIndx)        = [];
%% Create options from the Laplacian
L = DTrim-WTrim;
Laplacian                                       = DTrim^-0.5*L*DTrim^-0.5; % The normalized Laplacian - N in the paper
if IsStochasticDomainFlag
    [~, ~, VMat]                                = PolDecompFunc(Laplacian);
    Laplacian                                   = real(VMat);
end
[PsiFuncs,EigenVals]                            = eig(Laplacian);
[~,IndxEigenValsSort]                           = sort(diag(EigenVals));
PsiFuncs                                        = PsiFuncs(:,IndxEigenValsSort);
EigenValsVec                                    = diag(EigenVals);
EigenVals                                       = diag(EigenValsVec(IndxEigenValsSort));

for i=2:2:2*size(PsiFuncs,1) 
    OptionsFromLTmp                             = zeros(KStates,1);
    OptionsFromLTmp(~StatesNoTransitionIndx)    = PsiFuncs(:,i/2);  
    LaplacianEignOptions{i-1}                   = OptionsFromLTmp;
    LaplacianEignOptions{i}                     = -OptionsFromLTmp;    
end
%% Computing minima of stationary distribution
if ~IsStochasticDomainFlag
    StationaryDistribution              = diag(DTrim);
    StatDis                             = zeros(KStates,1);
    StatDis(~StatesNoTransitionIndx)    = StationaryDistribution/norm(StationaryDistribution);
    StatDisMat                          = reshape(StatDis,KDim1,KDim2);
    figure;
    Im = imagesc(1:KDim1,1:KDim2,StatDisMat);
    set(Im,'AlphaData',StatDisMat>0)
%     title('Stationary distribution ');
    axis off
    colormap(MapBlueNew)
    c                   = colorbar;
    c.Label.String      = '\pi_0';    
    pos                 = c.Position;
    c.Label.Position    = [pos(1)/2+0.2 pos(2)-.11];
    c.Label.Rotation    = 0;
    c.FontSize          = 13;
    c.Label.FontSize    = 18;
end
%% Calculating Diffusion options function

for i=1:length(EigenValsVec)
    DegreeArr2Try(i)                = sum(WTrim(i,:)>0);
end
DegreeArr2Try                       = DegreeArr2Try(:);
EigenValsVec                        = diag(EigenVals);
for a=1:length(EigenValsVec)
    DiffusionAnomalyFuncTmp         = zeros(length(EigenValsVec),1);
    for i=2:length(EigenValsVec)
        DiffusionAnomalyFuncTmp     = DiffusionAnomalyFuncTmp + (1-0.5*EigenValsVec(i))^DiffusionTime*(DegreeArr2Try(a)^-0.5)*PsiFuncs(a,i)*DegreeArr2Try.^0.5.*PsiFuncs(:,i);
    end%i
    DiffusionAnomalyFunc(a)         = norm(DiffusionAnomalyFuncTmp)^2;
end %a
DiffusionOptionFunc                 = DiffusionAnomalyFunc';

%% -----------Plotting diffusion distance--------------
DiffusionOptionFuncTmp                          = zeros(KStates,1);
DiffusionOptionFuncTmp(~StatesNoTransitionIndx) = DiffusionOptionFunc;
DiffusionOptionFuncMat                          = reshape(DiffusionOptionFuncTmp,KDim1,KDim2);

figure;
Im2 = imagesc(1:KDim1,1:KDim2,(DiffusionOptionFuncMat));
set(Im2,'AlphaData',DiffusionOptionFuncMat>0)
% title(['Diffusion options for t=' num2str(DiffusionTime) ]);
axis off
c                   = colorbar;
colormap(MapBlueNew)
c.Label.String      = ['f_{' num2str(DiffusionTime) '}(s)'];
pos                 = c.Position;
c.Label.Position    = [pos(1)/2+0.2 pos(2)-.11];
c.Label.Rotation    = 0;
c.FontSize          = 13;
c.Label.FontSize    = 18;
%% Peak detection
[ListOptionsGoalStates]     = PeaksDetectionFunc(DiffusionOptionFuncMat);

%% Creating diffusion options from the list of option goal states created above
DiffusionOptionsValueFunc   = DiffusionOptionsCreationFromStatesListFunc(ListOptionsGoalStates,StateMat,KStates,StatesNoTransitionIndx,KDim1,KDim2);

%% Creating random options
if 0
    RandomOptionsValueFunc  = DiffusionOptionsCreationFromStatesListFunc(AccessibleStates,StateMat,KStates,StatesNoTransitionIndx,KDim1,KDim2);
else
    RandomOptionsValueFunc  = [];
end

%% Performing Q learning and plotting figures
if IsStochasticDomainFlag
    QLearningWithOptionsStochasticDomainFunc(DiffusionOptionsValueFunc,LaplacianEignOptions,RandomOptionsValueFunc,StochasticRatio,StateMat,StatesNoTransitionIndx,FourRoomFlag,HardMazeDomainFlag,RingRoomFlag,KStates,KDim1,KDim2);
else   
    QLearningWithOptionsFunc(DiffusionOptionsValueFunc,LaplacianEignOptions,RandomOptionsValueFunc,StateMat,StatesNoTransitionIndx,FourRoomFlag,HardMazeDomainFlag,RingRoomFlag,KStates,KDim1,KDim2);
end
%% Computation of average number of steps between every pair of states
% Results are displayed at the console
if IsExplorStepsStatesFlag
    DiffutionTimeCalcWithOptions;
end

