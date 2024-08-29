
clear all; 
close all; 
format compact;

%Defining Problem
[problem,guess]=Hypersensitive;          % Fetch the problem definition
%options= problem.settings(1,8);                  % for hp method
options= problem.settings(50);                  % for h method
[problem,guess]=AlyChan;

%Calling environment
env = UnderwaterEnvTest; 
obsInfo = env.ObsInfo; 
actInfo = env.ActInfo; 



  

% actorNetwork = [
%     sequenceInputLayer(obsInfo.Dimension, 'Name', 'state')
%     convolution2dLayer(3,32,Padding="same")
%     batchNormalizationLayer
%     reluLayer
%     maxPooling2dLayer(2,Stride=2,Padding="same")
%     %dropoutLayer(0.5, 'Name', 'dropout1')
%     % convolution2dLayer(3,32,Padding="same")
%     % batchNormalizationLayer
%     % reluLayer
%     % maxPooling1dLayer(2,Stride=2,Padding="same")
%     flattenLayer
%     lstmLayer(64, 'OutputMode', 'sequence', 'Name', 'lstm1','RecurrentWeightsInitializer','glorot')
% 
%     fullyConnectedLayer(numel(actInfo.Elements), 'Name', 'fc4', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot')  
%     softmaxLayer];   

% actorNetwork = [
%     sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state') 
%     fullyConnectedLayer(32, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu1') 
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu2')    
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu3') 
%     lstmLayer(64, 'OutputMode', 'sequence', 'Name', 'lstm1','RecurrentWeightsInitializer','glorot')
%     fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu4') 
%     dropoutLayer(0.05, 'Name', 'dropout1')
%     fullyConnectedLayer(128, 'Name', 'fc2', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu5')
%     fullyConnectedLayer(numel(actInfo.Elements), 'Name', 'fc4', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot')
%     softmaxLayer];  

% actorNetwork = [
%     sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state')  
%     lstmLayer(64, 'OutputMode', 'sequence', 'Name', 'lstm1','RecurrentWeightsInitializer','glorot')
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu1') 
% 
%     fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu4') 
%     dropoutLayer(0.05, 'Name', 'dropout1')
%     fullyConnectedLayer(128, 'Name', 'fc2', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu5')
%     fullyConnectedLayer(numel(actInfo.Elements), 'Name', 'fc4', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot')
%     softmaxLayer];  

actorNetwork = [
    sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state')  
    lstmLayer(64, 'OutputMode', 'sequence', 'Name', 'lstm1','RecurrentWeightsInitializer','glorot')
    fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    layerNormalizationLayer
    preluLayer('Name', 'relu1') 
    
    fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    layerNormalizationLayer
    preluLayer('Name', 'relu4') 
    dropoutLayer(0.5, 'Name', 'dropout1')
    fullyConnectedLayer(128, 'Name', 'fc2', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    layerNormalizationLayer
    preluLayer('Name', 'relu5')
    fullyConnectedLayer(numel(actInfo.Elements), 'Name', 'fc4', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot')
    softmaxLayer];  



obsPath = [
    sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state')
    lstmLayer(100, 'OutputMode', 'sequence', 'Name', 'lstm1','RecurrentWeightsInitializer','glorot')
    fullyConnectedLayer(100, 'Name', 'fc1', 'WeightsInitializer','glorot')
    reluLayer
    layerNormalizationLayer
    %lstmLayer(100, 'OutputMode', 'sequence', 'Name', 'lstm2','RecurrentWeightsInitializer','glorot')
    fullyConnectedLayer(100, 'Name', 'obsPathOutLyr', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot')]; 

actPath = [
    sequenceInputLayer(actInfo.Dimension(1), 'Name', 'action')
    lstmLayer(100, 'OutputMode', 'sequence', 'Name', 'act_lstm1','RecurrentWeightsInitializer','glorot')
    fullyConnectedLayer(100, 'Name', 'actPathOutLyr', 'WeightsInitializer','glorot')
    ];

% Common path
commonPath = [
    additionLayer(2,Name="add")
    reluLayer
    fullyConnectedLayer(1,Name="QValue")
    ]; 

criticNetwork2 = dlnetwork(obsPath);
%criticNetwork = addLayers(criticNetwork,obsPath);
criticNetwork2 = addLayers(criticNetwork2,actPath);
criticNetwork2 = addLayers(criticNetwork2,commonPath); 
% Connect the layers
criticNetwork2 = connectLayers(criticNetwork2, ...
    "obsPathOutLyr","add/in1");
criticNetwork2 = connectLayers(criticNetwork2, ...
    "actPathOutLyr","add/in2");  




% criticNetwork = [ 
%     sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state') 
%     convolution1dLayer(3,4,Padding="same")
%     batchNormalizationLayer
%     reluLayer
%     maxPooling1dLayer(2,Stride=2,Padding="same")  
%     convolution1dLayer(3,8,Padding="same")
%     batchNormalizationLayer
%     reluLayer
%     maxPooling1dLayer(2,Stride=2,Padding="same")
%     flattenLayer
%     lstmLayer(240, 'OutputMode', 'sequence', 'Name', 'lstm1') 
%     fullyConnectedLayer(100, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu1')
%     fullyConnectedLayer(10);
%     reluLayer;
%     fullyConnectedLayer(1)
%     ];

% criticNetwork = [ 
%     sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state') 
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu1') 
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu2')    
%     fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu3')
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu4') 
%     fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu5')
%     lstmLayer(256, 'OutputMode', 'sequence', 'Name', 'lstm1') 
%     fullyConnectedLayer(256, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu6') 
%     dropoutLayer(0.05, 'Name', 'dropout1')
%     fullyConnectedLayer(64);
%     reluLayer;
%     fullyConnectedLayer(1)
%     ];  

% criticNetwork = [ 
%     sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state') 
%     lstmLayer(256, 'OutputMode', 'sequence', 'Name', 'lstm1') 
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu1') 
%     fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu2')    
%     % fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     % layerNormalizationLayer
%     % reluLayer('Name', 'relu2') 
%     fullyConnectedLayer(256, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
%     layerNormalizationLayer
%     reluLayer('Name', 'relu6') 
%     dropoutLayer(0.05, 'Name', 'dropout1')
%     fullyConnectedLayer(64);
%     reluLayer;
%     fullyConnectedLayer(1)
%     ];  

criticNetwork = [ 
    sequenceInputLayer(obsInfo.Dimension(1), 'Name', 'state') 
    lstmLayer(256, 'OutputMode', 'sequence', 'Name', 'lstm1') 
    fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    layerNormalizationLayer
    preluLayer('Name', 'relu1') 
    fullyConnectedLayer(64, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    layerNormalizationLayer
    preluLayer('Name', 'relu2')    
    % fullyConnectedLayer(128, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    % layerNormalizationLayer
    % reluLayer('Name', 'relu2') 
    fullyConnectedLayer(256, 'Name', 'fc1', 'BiasLearnRateFactor', 0, 'WeightsInitializer','glorot') 
    layerNormalizationLayer
    preluLayer('Name', 'relu6') 
    dropoutLayer(0.5, 'Name', 'dropout1')
    fullyConnectedLayer(64);
    preluLayer;
    fullyConnectedLayer(1)
    ]; 





trainingElems.actorNet = dlnetwork(actorNetwork);  
trainingElems.targetActorNet = dlnetwork(actorNetwork); 

trainingElems.criticNet1 = dlnetwork(criticNetwork);%dlnetwork(criticNetwork);  
trainingElems.targetCriticNet1 = criticNetwork2;%dlnetwork(criticNetwork);  

trainingElems.criticNet2 = criticNetwork2;%dlnetwork(criticNetwork); 
trainingElems.targetCriticNet2 = criticNetwork2;%dlnetwork(criticNetwork);  


trainingElems.actorNet = initialize(trainingElems.actorNet); 
trainingElems.targetActorNet = initialize(trainingElems.targetActorNet); 
trainingElems.criticNet1 = initialize(trainingElems.criticNet1); 
trainingElems.criticNet2 = initialize(trainingElems.criticNet2); 
trainingElems.targetCriticNet1 = initialize(trainingElems.targetCriticNet1); 
trainingElems.targetCriticNet2 = initialize(trainingElems.targetCriticNet2);


trainingElems.actor = rlDiscreteCategoricalActor(trainingElems.actorNet, obsInfo, actInfo);%,UseDevice="gpu");
% trainingElems.actor = rlContinuousGaussianActor(trainingElems.actorNet, obsInfo, actInfo, ...
%     ActionMeanOutputNames="scale",...
%     ActionStandardDeviationOutputNames="splus",...
%     ObservationInputNames="netOin");
trainingElems.targetActor = rlDiscreteCategoricalActor(trainingElems.actorNet, obsInfo, actInfo);
trainingElems.critic1 = rlValueFunction(trainingElems.criticNet1, obsInfo);%, actInfo);
trainingElems.critic2 = rlQValueFunction(trainingElems.criticNet2, obsInfo, actInfo);
trainingElems.targetCritic1 = rlQValueFunction(trainingElems.targetCriticNet1, obsInfo, actInfo);
trainingElems.targetCritic2 = rlQValueFunction(trainingElems.targetCriticNet2, obsInfo, actInfo); 


actorOpts = rlOptimizerOptions('LearnRate', 0.001,'L2RegularizationFactor',0.001,GradientThreshold=2,GradientThresholdMethod="l2norm"); 
criticOpts = rlOptimizerOptions('LearnRate', 0.01,'L2RegularizationFactor',0.001,GradientThreshold=2,GradientThresholdMethod="l2norm"); 

trainingElems.actorOptimizer = rlOptimizer(actorOpts);
trainingElems.criticOptimizer1 = rlOptimizer(criticOpts);
trainingElems.criticOptimizer2 = rlOptimizer(criticOpts); 

trainingElems.discountFactor = 0.99;
trainingElems.targetUpdateFrequency = 2;
trainingElems.targetSmoothFactor = 0.005; 
trainingElems.actionNoise = 0.1;

% Experience buffer
trainingElems.bufferSize = 1e6;
trainingElems.miniBatchSize = 1;
%trainingElems.experienceBuffer = rlReplayMemory(trainingElems.bufferSize, trainingElems.miniBatchSize);
trainingElems.experienceBuffer = rlReplayMemory(obsInfo,actInfo); 
numOfEpisodes = 5000;  
maxStepsPerEpisode = options.maxMRiter; 

trajectoriesForLearning = 2;
trainingElems.buffLen = trajectoriesForLearning*maxStepsPerEpisode; 

trainingElems.obsBuffer = dlarray(zeros(obsInfo.Dimension(1),1,trainingElems.buffLen));
trainingElems.actBuffer = zeros([actInfo.Dimension(1) 1 maxStepsPerEpisode]);
trainingElems.rewardBuffer = dlarray(zeros(1,trainingElems.buffLen));  
trainingElems.maskBuffer = dlarray(zeros(1,trainingElems.buffLen));
returnBuffer = dlarray(zeros(1,trainingElems.buffLen));
actionSet = repmat(actInfo.Elements',1,trainingElems.buffLen); 

v = 0:(maxStepsPerEpisode-1);
p = repmat(v',1,maxStepsPerEpisode) - v;
discountWeights = tril(trainingElems.discountFactor.^p);

trainingElems.updateTarget = @updateTarget;  

trainingElems.totalReward = 0;
trainingElems.maxStepsPerEpisode = options.maxMRiter;  
trainingElems.episodeCumulativeRewardVector = [];  
episodeCumulativeReward = 0; 
trainingElems.avgWindowSize = 100;
trainingElems.episodeOffset = 0; 
trainingElems.policy = rlStochasticActorPolicy(trainingElems.actor);
%%
[trainingPlot,lineReward,lineAveReward] = hBuildFigure; 
set(trainingPlot,Visible="on");

trainingElemsobj = trainingElemHandle(trainingElems); 

actorGradFcn = dlaccelerate(@actorLossFunction); 
criticGradFcn = dlaccelerate(@criticLossFunction); 
prevCumulativeReward = 0; 
entropyWeight = 1e-3; 
% reset(trainingElemsobj.policy); 
% [trainingElemsobj.actor, trainingElemsobj.critic1, trainingElemsobj.critic2, trainingElemsobj.targetActor, trainingElemsobj.targetCritic1, trainingElemsobj.targetCritic2] = env.reset(trainingElemsobj.actor, trainingElemsobj.targetActor, ... 
%                                                          trainingElemsobj.critic1, trainingElemsobj.targetCritic1, trainingElemsobj.critic2, trainingElemsobj.targetCritic2);
learnRateDropPeriod = 5; 
learnRateDropFactor = 0.5; 
env.errorInMin = 10^-3; 
env.errorInMax = 10; 
%%
for epi = 1:numOfEpisodes 
    
    % trainingElemsobj is a struct containing necessary elements like
    % buffers, actor and critic networks and policy for training
    trainingElemsobj.episodeOffset = mod(epi-1,trajectoriesForLearning)*maxStepsPerEpisode;
    
    % Solving the problem and storing observations, rewards and actions
    [solution,MRHistory]=TrainPGdiscreteNewObs_solveMyProblem( problem,guess,options,trainingElemsobj,env);  

    episodeElements = trainingElemsobj.episodeOffset + (1:maxStepsPerEpisode);
    episodeCumulativeReward = extractdata(sum(trainingElemsobj.rewardBuffer(episodeElements))/sum(trainingElemsobj.maskBuffer(episodeElements))); 
    
    returnBuffer(episodeElements) = trainingElemsobj.rewardBuffer(episodeElements)*discountWeights./sum(trainingElemsobj.maskBuffer(episodeElements));
    if mod(epi,trajectoriesForLearning) == 0

        % Reducing entropy as training progresses
        if epi > 1
            entropyWeight = entropyWeight * (1 - 1.5 * epi / 3000);
        end 

        if entropyWeight <= 1e-5 
            entropyWeight = 1e-5; 
        end 

        % Calculating critic gradient
        criticGradient = dlfeval(criticGradFcn,...
            trainingElemsobj.critic1, trainingElemsobj,returnBuffer);
        
        % Calculating actor gradient
        actorGradient = dlfeval(actorGradFcn,...
            trainingElemsobj.actor, trainingElemsobj.critic1, trainingElemsobj,returnBuffer,entropyWeight);

        % Update the critic and actor using the computed gradients.
        [trainingElemsobj.critic1,trainingElemsobj.criticOptimizer1] = update( ...
            trainingElemsobj.criticOptimizer1, ...
            trainingElemsobj.critic1, ...
            criticGradient);

        [trainingElemsobj.actor,trainingElemsobj.actorOptimizer] = update( ...
            trainingElemsobj.actorOptimizer, ...
            trainingElemsobj.actor, ...
            actorGradient);

        % Updating the policy after updating actor
        trainingElemsobj.policy = rlStochasticActorPolicy(trainingElemsobj.actor);

        % flushing the mask and reward buffer for next set of episodes
        trainingElemsobj.maskBuffer(:) = 0;
        trainingElemsobj.rewardBuffer(:) = 0;
    end 

    % Displaying reward
    trainingElemsobj.episodeCumulativeRewardVector = cat(2,...
    trainingElemsobj.episodeCumulativeRewardVector,episodeCumulativeReward);
    movingAvgReward = movmean(trainingElemsobj.episodeCumulativeRewardVector,...
        trainingElemsobj.avgWindowSize,2); 
    if isvalid(lineReward) 
        addpoints(lineReward,epi,episodeCumulativeReward);

    end 
    if isvalid(lineAveReward)

        addpoints(lineAveReward,epi,movingAvgReward(end));  
    end
    drawnow;  

    % Resetting actor and critic (Multiple critics and actors network are
    % included to extended the algorithm. This does not affect training if
    % using only one actor and one critic
    [trainingElemsobj.actor, trainingElemsobj.critic1, trainingElemsobj.critic2, trainingElemsobj.targetActor, trainingElemsobj.targetCritic1, trainingElemsobj.targetCritic2] = env.reset(trainingElemsobj.actor, trainingElemsobj.targetActor, ... 
                                                         trainingElemsobj.critic1, trainingElemsobj.targetCritic1, trainingElemsobj.critic2, trainingElemsobj.targetCritic2);
    
    %Resetting policy for next set of episodes
    trainingElemsobj.policy = reset(trainingElemsobj.policy); 

end 
 
%%
function target = updateTarget(target, main, smoothFactor) 
    Learnables = getLearnableParameters(main);
    for i= 1:length(Learnables) 
        Learnables{i,1} = (1 - smoothFactor) .* Learnables{i,1} + smoothFactor * Learnables{i,1};
    end 
    target = setLearnableParameters(target,Learnables);
end 

function [trainingPlt, lineRewd, lineAvgRwd] = hBuildFigure()
    plotRatio = 16/9;
    trainingPlt = figure(...
                Visible="off",...
                HandleVisibility="off", ...
                NumberTitle="off",...
                Name="Mesh Refinement Custom Training");

    trainingPlt.Position(3) = ...
         plotRatio * trainingPlt.Position(4);
    
    ax = gca(trainingPlt);
    
    lineRewd = animatedline(ax);
    lineAvgRwd = animatedline(ax,Color="r",LineWidth=3);
    xlabel(ax,"Episode");
    ylabel(ax,"Reward");
    legend(ax,"Cumulative Reward","Average Reward", ...
           Location="northwest")
    title(ax,"Training Progress");
end

function actorGradient = actorLossFunction(actor, critic, trainingElems, returnBuffer, entropyWeight)
    
    % Evaluating the value function using the critic network
    valueEstimates = evaluate(critic, {trainingElems.obsBuffer}, UseForward=true);
    valueEstimates = valueEstimates{1};
    valueEstimates = squeeze(valueEstimates);
    
    % Computing the advantage as the difference between the returns and the
    % value estimates. The mask buffer ensures that valid only valid
    % return values are considered
    advantage = (returnBuffer - valueEstimates) .* trainingElems.maskBuffer; 

    % Normalizing the advantage
    advantageMean = mean(advantage, 'all');
    advantageStd = std(advantage, 0, 'all');
    advantage = (advantage - advantageMean) / (advantageStd + eps);
    
    % Evaluating the action probabilities using the actor network
    out = evaluate(actor, {trainingElems.obsBuffer}, UseForward=true);
    
    actionProbabilities = out{1};

    % Clipping any action probability values less than a small eps value
    actionProbabilities(actionProbabilities < eps) = eps;
    actionProbabilities = squeeze(actionProbabilities);
    
    % Computing the log of the action probabilities
    actionLogProbabilities = log(actionProbabilities);
    actionLogProbabilities = squeeze(actionLogProbabilities);
    
    % Computing the policy gradient loss 
    advantage = reshape(advantage, [1, size(actionLogProbabilities,2), size(actionLogProbabilities,3)]);
    policyGradientLoss = squeeze(-sum(advantage .* actionLogProbabilities, [1,2]));
    policyGradientLoss = policyGradientLoss(policyGradientLoss ~= 0);
    policyGradientLoss = mean(policyGradientLoss); 

    %Computing the entropy loss for better exploration
    entropyLoss = -sum(actionProbabilities .* actionLogProbabilities,[1,2]); 
    entropyLoss = entropyLoss(entropyLoss ~= 0);
    entropyLoss = mean(entropyLoss); 

    % Computing the total loss as the sum of policy gradient loss and
    % entropy loss and dividing by the total number of steps taken
    totalLoss = (policyGradientLoss + entropyWeight * entropyLoss) / sum(trainingElems.maskBuffer);

    % Computing the gradient of loss with respect to the actor learnable parameters
    actorGradient = dlgradient(totalLoss, actor.Learnables); 
end



function criticGradient = criticLossFunction(critic, trainingElems, returnBuffer)
    
    % Evaluating the value function using the critic network
    valueEstimates = evaluate(critic, {trainingElems.obsBuffer}, UseForward=true);
    valueEstimates = valueEstimates{1};
    valueEstimates = squeeze(valueEstimates);
    
    % Computing the mean squared error (mse) loss 
    advantage = (returnBuffer - valueEstimates) .* trainingElems.maskBuffer; 
    tempAdvantage = mean(advantage);
    mseLoss = sum(tempAdvantage.^2,'all');

    % Computing the gradient of loss with respect to the critic learnable parameters
    criticGradient = dlgradient(mseLoss, critic.Learnables); 
end 

function gradients = thresholdGlobalL2Norm(gradients,gradientThreshold)

globalL2Norm = 0;
for i = 1:numel(gradients)
    globalL2Norm = globalL2Norm + sum(gradients{i}(:).^2);
end
globalL2Norm = sqrt(globalL2Norm);

if globalL2Norm > gradientThreshold
    normScale = gradientThreshold / globalL2Norm;
    for i = 1:numel(gradients)
        gradients{i} = gradients{i} * normScale;
    end
end 
end 

function entropyWeightOut = calcEntropyWeight(entropyWeight, rewardTrend) 
    
    adjustmentFactor = 1 + 0.5 * abs(rewardTrend);

    if rewardTrend > 0 
        entropyWeightOut = entropyWeight/adjustmentFactor ; 
    elseif rewardTrend < 0 
        entropyWeightOut = entropyWeight * adjustmentFactor; 
    else 
        entropyWeightOut = entropyWeight;
    end 

    entropyWeightOut = max(min(entropyWeightOut, 0.1), 1e-5);
end