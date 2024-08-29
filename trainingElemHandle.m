classdef trainingElemHandle < handle
    properties
actorNet, targetActorNet, criticNet1, targetCriticNet1, criticNet2, targetCriticNet2, actor, targetActor, critic1, critic2, targetCritic1, targetCritic2, actorOptimizer, criticOptimizer1, criticOptimizer2, discountFactor, targetUpdateFrequency, targetSmoothFactor, actionNoise, bufferSize, miniBatchSize, experienceBuffer, buffLen, obsBuffer, actBuffer, rewardBuffer, maskBuffer, updateTarget, totalReward, maxStepsPerEpisode, episodeCumulativeRewardVector, avgWindowSize, episodeOffset, targetActionBuffer, nextObsBuffer, policy, oldActor,rewardSeqBuffer   
    end
    methods
        % You can add methods here if needed
        function obj = trainingElemHandle(s)
            if nargin > 0
                % Initialize the handle object with values from the struct
                props = fieldnames(s);
                for i = 1:length(props)
                    obj.(props{i}) = s.(props{i});
                end
            end
        end
    end
end