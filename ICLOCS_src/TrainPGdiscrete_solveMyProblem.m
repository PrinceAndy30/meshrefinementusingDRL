function [ varargout ] = TrainPGdiscrete_solveMyProblem( varargin )
%solveMyProblem - main file for solving NLPs
%
% Syntax:  [ varargout ] = solveMyProblem( problem,guess,options )
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk


%------------- BEGIN CODE --------------
if nargin==3
    problem=varargin{1};
    guess=varargin{2};
    options=varargin{3};
elseif nargin==5
    problem=varargin{1};
    guess=varargin{2};
    options=varargin{3};
    trainingElems=varargin{4}; 
    env=varargin{5};
end
clearIntermVariables;

problem_org=problem;
%criticGradFcn = dlaccelerate(@criticLossFunction); 
% actorGradFcn = dlaccelerate(@actorLossFunction);
    switch options.meshstrategy   
        case{'mesh_refinement'}
            if strcmp(options.transcription,'integral_res_min')
                error('Mesh refinement does not currectly supported the integrated residual minimization method. Please use a fixed mesh for integrated residual minimization, or use direct collocation method with mesh refinement.')
            end
            options.constraintErrorTol_org=problem.constraintErrorTol;
            errorHistory=cell(1,1);
            ConstraintErrorHistory=cell(1,1);
            timeHistory=zeros(1,1);
            iterHistory=zeros(1,1);
            solutionHistory=cell(1,1);
            problemHistory=cell(1,1);
            statusHistory=cell(1,1);
            resErrorHistory=cell(1,1);
            MRiterCheck=false(1);
            
            runCondition=1;
            i=1; imax=options.maxMRiter;
            i_reg=1;
            j = -1;
%             problem_iter=problem;
            while runCondition
                if i>imax+1 
                    runCondition = 0; 
                    break 
                end
                problem_iter=problem;
                if strcmp(options.transcription,'direct_collocation_intres_reg') 
                        if isfield(problem_org.states,'resNormCusWeight')
                            problem_iter.states.resNormCusWeight=problem_iter.states.resNormCusWeight*(1/2/options.resCostWeight(i_reg));
                        else
                            problem_iter.states.resNormCusWeight=1/2/options.resCostWeight(i_reg);
                        end
                        if isfield(problem_org.constraints,'resNormCusWeight_eq')
                            problem_iter.constraints.resNormCusWeight_eq=problem_iter.constraints.resNormCusWeight_eq*(1/2/options.resCostWeight(i_reg));
                        else
                            problem_iter.constraints.resNormCusWeight_eq=(1/2/options.resCostWeight(i_reg));
                        end
                        if i_reg< length(options.resCostWeight)
                            i_reg=i_reg+1;
                        end
                else
                    if isfield(options,'resCostWeight')
                        options=rmfield(options,'resCostWeight');
                    end
                end


                if isfield(options,'ECH') && options.ECH.enabled
                    if i~=1
                        [ problem_iter,guess,options ] = selectAppliedConstraint( problem_iter, guess, options, data, solutionHistory, i );
                    end
                end
                
                
                problemHistory{i}=problem_iter;
                
                if i==1 && exist('OCP_in','var') == 1
                    infoNLP=OCP_in.infoNLP;
                    data=OCP_in.data;
                    options=OCP_in.options;
                    data=checkDynamics( infoNLP.z0,data );
                else
                    [infoNLP,data,options]=transcribeOCP(problem_iter,guess,options); % Format for NLP solver
                end
                
                if i==1 && (nargout==3 || nargout==4)
                    OCP_ini.data=data;
                    OCP_ini.infoNLP=infoNLP;
                    OCP_ini.options=options;
                end
                
                OCP_MR.data=data;
                OCP_MR.infoNLP=infoNLP;
                OCP_MR.options=options;
                
                if isfield(options,'regstrategy') && strcmp(options.regstrategy,'simultaneous')
                    if isfield(data.data.penalty,'i') && isfield(data.data.penalty,'values')
                        idx_penalty=i;
                        idx_penalty(idx_penalty>length(data.data.penalty.values))=length(data.data.penalty.values);
                        data.data.penalty.i=idx_penalty;
                    else
                        error('Regularization Parameters Not Properly Configured!')
                    end
                end

                [solution,status,data] = solveNLP(infoNLP,data);      % Solve the NLP
                
                try
                    [solution]=runPostSolveTasks(problem,solution,options,data); % Output solutions 
 
                    % Storing size of initial mesh
                    if i == 1 
                        sz = length(solution.T_error')-1; 
                        compTime = 0;
                    end
                    if i == 1 && trainingElems.episodeOffset == 0
                        
                        trainingElems.obsBuffer = dlarray(zeros(env.ObsInfo.Dimension(1),1,sz,trainingElems.buffLen));
                        trainingElems.actBuffer = zeros([env.ActInfo.Dimension(1) 1 sz trainingElems.buffLen]); 
                        
                    end
                    ErrorMatrix = zeros(sz,1); 
                    RErrorMatrix = zeros(sz,1);
                    
                    % Getting observations
                    ErrorMatrix = max(solution.Error,[],2);  
                    RErrorMatrix = max(solution.ErrorRelative,[],2); 

                    % Ensuring observations are constant length throughout
                    % all episodes
                    if i~=1 
                        ErrorMatrix = errorHandle(ErrorMatrix,increTrack,i); 
                        RErrorMatrix = errorHandle(RErrorMatrix,increTrack,i);   
                    end   

                    L_ErrorMatrix = log(ErrorMatrix+10^-40); 
                    L_RErrorMatrix = log(RErrorMatrix+10^-40);
                    
                    R_ErrorMatrix = rescale(L_ErrorMatrix,-1,1,'InputMin',-92,'InputMax',5.3);
                    R_RErrorMatrix = rescale(L_RErrorMatrix,-1,1,'InputMin',-92,'InputMax',5.3);

                    if i == 1 
                        env.Observation = [R_ErrorMatrix';R_RErrorMatrix']; 
                        env.Observation = reshape(env.Observation, [env.ObsInfo.Dimension(1), 1, sz]); 
                    end 
                    obs = env.Observation;
                    
                    
                    if i ~= 1 

                        nextObser = reshape([R_ErrorMatrix';R_RErrorMatrix'],[env.ObsInfo.Dimension(1), 1, sz]);  
                        
                        % Calculating Reward
                        compTime = compTime + solution.computation_time;
                        env.Reward = env.getReward(solution.MaxAbsError, solution.MaxConstVioError,options.ipopt.tol,compTime); 

                        % Storing experiences in respective buffers
                        j = trainingElems.episodeOffset + (i-1);
                        trainingElems.obsBuffer(:,:,:,j) = obs;
                        trainingElems.actBuffer(:,:,:,j) = action{1};
                        trainingElems.rewardBuffer(j) = env.Reward;%*10^-5; 
                        trainingElems.maskBuffer(:,j) = 1;
 
                        obs = nextObser; 
                        env.Observation = obs;

                    end 
                    %trainingElems.episodeCumulativeReward = sum(trainingElems.rewBuffer);
                    maxAbsError=max(abs(solution.Error)); 
                    maxAbsConstraintError=max(solution.ConstraintError);
                    if isfield(options.print,'residual_error') && options.print.residual_error
                        resError=solution.residuals.r;
                        resErrorHistory{i,1}=resError;
                    end
                    errorHistory{i,1}=maxAbsError;
                    ConstraintErrorHistory{i,1}=maxAbsConstraintError;
                    timeHistory(i)=solution.computation_time;
                    solutionHistory{i,1}=solution;
                    statusHistory{i,1}=status;
                    if isfield(status,'iter')
                        iterHistory(i)=status.iter;
                    end
                    


                    switch options.errortype
                    case{'local_abs'}
                        runCondition_MR=(any(maxAbsError>problem.states.xErrorTol_local) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax;
                        if ~runCondition_MR && (strcmp(options.resultRep,'res_min_final_manual') || strcmp(options.resultRep,'res_min_final_default'))
                            data.options.resultRep='res_min';
                            [solution]=runPostSolveTasks(problem,solution,options,data);         % Output solutions
                            maxAbsError=max(abs(solution.Error));
                            maxAbsConstraintError=max(solution.ConstraintError);
                            errorHistory{i,1}=maxAbsError;
                            ConstraintErrorHistory{i,1}=maxAbsConstraintError;
                            timeHistory(i)=solution.computation_time;
                            solutionHistory{i,1}=solution;
                            statusHistory{i,1}=status;
                            if isfield(options.print,'residual_error') && options.print.residual_error
                                resError=solution.residuals.r;
                                resErrorHistory{i,1}=resError;
                            end
                            if isfield(status,'iter')
                                iterHistory(i)=status.iter;
                            end
                        end
                        if i>1
                              MRiterCheck(i)=(any((min(cell2mat(errorHistory(1:i-1)))-errorHistory{i})./min(cell2mat(errorHistory(1:i-1)))<0) || all(0<(min(cell2mat(errorHistory(1:i-1)))-errorHistory{i})./min(cell2mat(errorHistory(1:i-1)))<0.05)) && (any((min(cell2mat(ConstraintErrorHistory(1:i-1)))-ConstraintErrorHistory{i})./min(cell2mat(ConstraintErrorHistory(1:i-1)))<0) || all(0<(min(cell2mat(ConstraintErrorHistory(1:i-1)))-ConstraintErrorHistory{i})./min(cell2mat(ConstraintErrorHistory(1:i-1)))<0.05));
                        end
                        % if runCondition_MR && i>3 && all(MRiterCheck(i-2:i)) 
                        %     if isfield(options,'DisableMRConvergenceCheck') && options.DisableMRConvergenceCheck
                        %     else
                        %         waitAnswer=1;
                        %         while waitAnswer
                        %             disp('Possible slow convergence or diverging mesh refinement iterations, Please selection your options')
                        %             disp('1. Continue with mesh refinement')
                        %             disp('2. Switch to integrated residuals regulated direct collocation method')
                        %             disp('3. Terminate now')
                        %             keepMR = input('\n', 's');
                        %             if strcmp(keepMR, '1')
                        %                 waitAnswer=0;
                        %             elseif strcmp(keepMR, '2')
                        %                 options.transcription='direct_collocation_intres_reg';
                        %                 options.resCostWeight = str2num(input('Please provide one or a sequence of weights for regularization, e.g. [1e-3 1e-6] \n', 's'));
                        %                 % options.ipopt.hessian_approximation='limited-memory';
                        %                 waitAnswer=0;
                        %             elseif strcmp(keepMR, '3')
                        %                 runCondition_MR=false(1); 
                        %                 waitAnswer=0;
                        %             else
                        %                 disp('Answer not recognized, please enter again!')
                        %             end
                        %         end
                        %     end
                        % end

                    case{'int_res'}
                        runCondition_MR=(any(resError*0.99>problem.states.xErrorTol_integral'.^2) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax;
                        if ~any(maxAbsError>problem.states.xErrorTol_local) && ~any(maxAbsConstraintError>problem.constraintErrorTol) && runCondition_MR
                            solution.Error=solution.Error./max(solution.Error).*problem.states.xErrorTol_local.*resError'./problem.states.xErrorTol_integral;
                        end

                        if i>1
                              MRiterCheck(i)=(any((min(cell2mat(resErrorHistory(1:i-1)))-resErrorHistory{i})./min(cell2mat(resErrorHistory(1:i-1)))<0) || all(0<(min(cell2mat(resErrorHistory(1:i-1)))-resErrorHistory{i})./min(cell2mat(resErrorHistory(1:i-1)))<0.05)) && (any((min(cell2mat(ConstraintErrorHistory(1:i-1)))-ConstraintErrorHistory{i})./min(cell2mat(ConstraintErrorHistory(1:i-1)))<0) || all(0<(min(cell2mat(ConstraintErrorHistory(1:i-1)))-ConstraintErrorHistory{i})./min(cell2mat(ConstraintErrorHistory(1:i-1)))<0.05));
                        end
                        if runCondition_MR && i>5 && all(MRiterCheck(i-4:i))
                            if isfield(options,'DisableMRConvergenceCheck') && options.DisableMRConvergenceCheck
                            else
                                waitAnswer=1;
                                while waitAnswer
                                    keepMR = input('Possible slow convergence or diverging mesh refinement iterations, continue to refine the mesh? (Yes/No) \n', 's');
                                    if strcmp(keepMR, 'Yes')
                                        waitAnswer=0;
                                    elseif strcmp(keepMR, 'No')
                                        runCondition_MR=false(1); 
                                        waitAnswer=0;
                                    else
                                        disp('Answer not recognized, please enter again!')
                                    end
                                end
                            end
                        end

                    case{'both'}
                        runCondition_local=(any(maxAbsError>problem.states.xErrorTol_local) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax;
                        runCondition_integral=(any(resError*0.99>problem.states.xErrorTol_integral'.^2) || any(maxAbsError>problem.states.xErrorTol_local) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax;
                        runCondition_MR=runCondition_local || runCondition_integral;
                        if runCondition_integral && ~runCondition_local && (strcmp(options.resultRep,'res_min_final_manual') || strcmp(options.resultRep,'res_min_final_default'))
                            data.options.resultRep='res_min';
                            [solution]=runPostSolveTasks(problem,solution,options,data);         % Output solutions
                            maxAbsError=max(abs(solution.Error));
                            maxAbsConstraintError=max(solution.ConstraintError);
                            errorHistory{i,1}=maxAbsError;
                            ConstraintErrorHistory{i,1}=maxAbsConstraintError;
                            timeHistory(i)=solution.computation_time;
                            solutionHistory{i,1}=solution;
                            statusHistory{i,1}=status;
                            if isfield(options.print,'residual_error') && options.print.residual_error
                                resError=solution.residuals.r;
                                resErrorHistory{i,1}=resError;
                            end
                            if isfield(status,'iter')
                                iterHistory(i)=status.iter;
                            end

                           runCondition_MR=(any(resError*0.99>problem.states.xErrorTol_integral'.^2) || any(maxAbsError>problem.states.xErrorTol_local) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax;
                        end

                        if i>1
                              MRiterCheck(i)=((any((min(cell2mat(errorHistory(1:i-1)))-errorHistory{i})./min(cell2mat(errorHistory(1:i-1)))<0) || all(0<(min(cell2mat(errorHistory(1:i-1)))-errorHistory{i})./min(cell2mat(errorHistory(1:i-1)))<0.05)) && (any((min(cell2mat(resErrorHistory(1:i-1)))-resErrorHistory{i})./min(cell2mat(resErrorHistory(1:i-1)))<0) || all(0<(min(cell2mat(resErrorHistory(1:i-1)))-resErrorHistory{i})./min(cell2mat(resErrorHistory(1:i-1)))<0.05))) && (any((min(cell2mat(ConstraintErrorHistory(1:i-1)))-ConstraintErrorHistory{i})./min(cell2mat(ConstraintErrorHistory(1:i-1)))<0) || all(0<(min(cell2mat(ConstraintErrorHistory(1:i-1)))-ConstraintErrorHistory{i})./min(cell2mat(ConstraintErrorHistory(1:i-1)))<0.05));
                        end
                        if runCondition_MR && i>5 && all(MRiterCheck(i-4:i))
                            if isfield(options,'DisableMRConvergenceCheck') && options.DisableMRConvergenceCheck
                            else
                                waitAnswer=1;
                                while waitAnswer
                                    keepMR = input('Possible slow convergence or diverging mesh refinement iterations, continue to refine the mesh? (Yes/No) \n', 's');
                                    if strcmp(keepMR, 'Yes')
                                        waitAnswer=0;
                                    elseif strcmp(keepMR, 'No')
                                        runCondition_MR=false(1); 
                                        waitAnswer=0;
                                    else
                                        disp('Answer not recognized, please enter again!')
                                    end
                                end
                            end
                        end
                    end

                    runCondition=runCondition_MR;
                    if runCondition_MR
                        if ~isfield(options,'resCostWeight') || i_reg == length(options.resCostWeight)

                                if options.tau==0
                                    % Generating intial mesh
                                    options.tau=diff(linspace(0,1,sz+1))'; 
                                    options.tau(:, 1) = options.tau(:, 1) / sum(options.tau(:, 1));
                                    init_tau = options.tau; 
                                    increTrack = zeros(size(init_tau));
                                end 
                                % Getting discrete action
                                action = getAction(trainingElems.policy,{env.Observation});
                                %Post-processing action to generate mesh
                                [options.tau, increTrack] = actionHandle(action{1}, init_tau, increTrack,R_ErrorMatrix); 
                            
                            Mold=length(options.tau);
                            options.ipopt.max_iter=options.ipopt.max_iter*(length(options.tau))/(Mold); 
                            [ options, guess] = doWarmStart( options, guess, solution, data );

                        
                        end
                    else
                        if isfield(options,'regstrategy') && strcmp(options.regstrategy,'simultaneous') && data.data.penalty.i<length(data.data.penalty.values)
                            [ options, guess] = doWarmStart( options, guess, solution, data );
                            runCondition=1;
                        end
                    end
                    i=i+1;
                catch e
                    fprintf(1,'There was an error! The message was:\n%s \n',e.message);
                    error('Error encountered when post-processing the solution. Please ensure the NLP solve has been terminated successfully, and the error tolerances have been correctly configured');
                end

            end 
            % if j~=-1
            %     trainingElems.rewardBuffer(j) = env.Reward; 
            % end
            if isfield(data.data,'penalty') && strcmp(data.options.regstrategy,'MR_priority')
                if isfield(data.data.penalty,'i') && isfield(data.data.penalty,'values')
                    for j=1:length(data.data.penalty.values)
                        data.data.penalty.i=j;
                        [solution,status,data]=solveSingleNLP_DirectCollocation(infoNLP,data);
                        
                        try
                            [solution]=runPostSolveTasks(problem,solution,options,data);         % Output solutions

                            maxAbsError=max(abs(solution.Error));
                            maxAbsConstraintError=max(solution.ConstraintError);
                            errorHistory{i,1}=maxAbsError;
                            ConstraintErrorHistory{i,1}=maxAbsConstraintError;
                            timeHistory(i)=solution.computation_time;
                            solutionHistory{i,1}=solution;
                            statusHistory{i,1}=status;
                            if isfield(options.print,'residual_error') && options.print.residual_error
                                resError=solution.residuals.r;
                                resErrorHistory{i,1}=resError;
                            end
                            if isfield(status,'iter')
                                iterHistory(i)=status.iter;
                            end

                            data.multipliers.lambda=solution.multipliers.lambdaNLP;
                            infoNLP.z0=solution.z;
                            i=i+1;
                        catch e
                             fprintf(1,'There was an error! The message was:\n%s \n',e.message);
                            error('Error encountered when post-processing the solution. Please ensure the NLP solve has been terminated successfully, and the error tolerances have been correctly configured');
                        end
                    end
                else
                    error('Regularization Parameters Not Properly Configured!')
                end
            end

            MeshRefinementHistory.errorHistory=errorHistory;
            MeshRefinementHistory.timeHistory=timeHistory;
            MeshRefinementHistory.iterHistory=iterHistory;
            MeshRefinementHistory.ConstraintErrorHistory=ConstraintErrorHistory;
            if isfield(options.print,'residual_error') && options.print.residual_error
                MeshRefinementHistory.resErrorHistory=resErrorHistory;
            end
            MeshRefinementHistory.statusHistory=statusHistory;
            MeshRefinementHistory.solutionHistory=solutionHistory;
            if isfield(options,'ECH') && options.ECH.enabled
                MeshRefinementHistory.problemHistory=problemHistory;
            end
            varargout{1}=solution;
            varargout{2}=MeshRefinementHistory;
            if nargout==3 || nargout==4
                varargout{3}=OCP_MR;
            end
            if nargout==4
                varargout{4}=OCP_ini;
            end
        otherwise
            error('Unknown Meshing Strategy Selected!')
    end
end 

function [rawActionCell, normalizedAction] = customNormalizeAction(actor, observation)
    % Get action from the actor network
    rawActionCell = getAction(actor, observation); 
    rawAction = double(rawActionCell{1});
    rawAction = abs(rawAction);
    rawAction = squeeze(rawAction); 
    minValue = min(rawAction);
    rawAction = rawAction - minValue;
    
    
    scalingFactor = 1.2;
    rawAction = rawAction * scalingFactor;
    %rawAction = rawAction(1:end-1); 
    % Normalize the action at each timestep
    for t = 1:size(rawAction, 2)
        %actionSum = sum(rawAction(:, t)); % Sum of actions at timestep t
        %rawAction(:, t) = rawAction(:, t) / actionSum; % Normalize to ensure sum is 1 
        rawAction(:, t) = softmax(rawAction(:, t)); 
        rawAction(rawAction(:, t) < 0.009, t) = 0.009;

        % Re-normalize to ensure the sum is 1
        rawAction(:, t) = rawAction(:, t) / sum(rawAction(:, t));
    end
    
    % Reshape back to original size if necessary 
    %rawAction(rawAction<0.009) = 0.009;
    normalizedAction = reshape(rawAction, size(rawAction));
end 

function [params] = convertToDlarray(input) 
    params = cell(size(input));
    for i = 1:length(input) 
        temp = input(i);
        params{i} = dlarray(temp{:}); 
    end
end 

function [criticLoss,criticGradient] = criticLossFunction(critic, obsBatch, actBatch, target) 
        % criticValue = getValue(critic, {obsBatch}, {actBatch});
        % if length(target) > length(criticValue) 
        %     numZeros = length(target) - length(criticValue); 
        % end 
        % criticValue_padded = [dlarray(criticValue), zeros(1,numZeros)];  
        % target = dlarray(target);
        %criticValue = getValue(critic, {obsBatch}, {actBatch},UseForward=true);  
        criticValue = evaluate(critic, {obsBatch, actBatch});
        %criticLearnables = convertToDlarray(getLearnableParameters(critic));
        %criticValue2 = double(getValue(critic2, {obsBatch}, {actBatch}));
        if length(target) > length(criticValue{:}) 
             numZeros = length(target) - length(criticValue{:}); 
        else 
             numZeros = 0;
         end 
        %criticValue_padded = [criticValue, zeros(1,numZeros)]; 
        % %criticValue2_padded = [criticValue2, zeros(1,numZeros)]; 
        % allValues = [target(:); criticValue_padded(:)];%; criticValue2_padded(:)];
        % minVal = min(allValues);
        % maxVal = max(allValues);
        % % 
        % normalizedTarget = (target - minVal) / (maxVal - minVal);
        % normalizedCriticValues = (criticValue_padded - minVal) / (maxVal - minVal);
       %normalizedCriticValues2 = (criticValue2_padded - minVal) / (maxVal - minVal);
        criticLoss = mean(([criticValue{:},zeros(1,numZeros)] - target).^2); %mse([criticValue{:},zeros(1,numZeros)],target,'DataFormat','CT');
        % criticLearnables = convertToDlarray(getLearnableParameters(critic)); 
        criticGradient = dlgradient(criticLoss, critic.Learnables); 
        %criticGradient = thresholdGlobalL2Norm(criticGradient,1);
end 

function minCriticGradient = minCriticLossFunction(minCritic, action) 
        minCriticGradient = dlgradient(minCritic, action);
end 

function actionGradient = actionGradFunction(action, learnables)   
         actionGradient = dlgradient(action, learnables); 
end

% function actorGradient = actorLossFunction(actor, critic1,critic2,obsBatch,env,actBatchCellArray,actBatch) 
%             %actorLoss = mean(minCriticGradient.*actionGradient);  
%             % Actions = getAction(actor, {obsBatch});  
%             % actorActions = env.convertArray(double(Actions{:})); 
%             % %actorActions = convertToDlarray(actorActions);  
%             % actBatch = zeros(env.ActInfo.Dimension(1),1,length(env.Observation)); 
%             % actBatchCellArray = actorActions{:};
% 
%             for i_Act = 1:env.ActInfo.Dimension(1) 
%                 actBatch(i_Act, 1, :) = actBatchCellArray(i_Act);
%             end
%             criticValue1 = getValue(critic1, {obsBatch}, {actBatch});
%             criticValue2 = getValue(critic2, {obsBatch}, {actBatch});
%             % allValues = [criticValue1(:); criticValue2(:)];
%             % minVal = min(allValues);
%             % maxVal = max(allValues);
%             % 
%             % 
%             % normalizedCriticValues1 = (criticValue1 - minVal) / (maxVal - minVal);
%             % normalizedCriticValues2 = (criticValue2 - minVal) / (maxVal - minVal);
%             criticValue = min(criticValue1,criticValue2); 
%             actorLoss = mean(criticValue);  
%             %actorLoss2 = mean(actBatchCellArray);
%             actorGradient = dlgradient(actorLoss, actBatchCellArray);  
% end 

function actorGradient = actorLossFunction(critic1,critic2, actor,observation)

% Evaluate actions from current observations. Set the UseForward name-value
% pair to true to support cases where the actor has layers that define a
% forward pass different than prediction (e.g. batch normalization or
% dropout layers).
%action = getAction(actor,{observation},UseForward=true);
action = evaluate(actor,observation,UseForward=true);
% Compute: q = Q(s,a) 
q1 = evaluate(critic1, {observation{1}, action{1}},UseForward=true); 
q2 = evaluate(critic2, {observation{1}, action{1}},UseForward=true);
%q1 = getValue(critic1,{observation},action); 
%q2 = getValue(critic2,{observation},action);
q = min(q1{1}, q2{1}); 

% Compute: actorLoss = -sum(q)/N to maximize q
actorLoss = -sum(q,"all")/numel(q);

% Compute: d(-sum(q)/N)/dActorParams
actorGradient = dlgradient(actorLoss,actor.Learnables);
end

function newArray = arrayHandle(array) 
    temp = zeros([size(array)]); 
    for i = 1:length(array) 
        if(mod(i,2) == 0) 
            temp(i) = array(i) + array(i-1); 
        end 
    end
    newArray = temp(temp ~= 0);
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

function [tau, newIncreTrack] = actionHandle(inputAction, init_tau, increTrack,error) 
    inputAction = squeeze(inputAction); 
    
    % Tracking previous actions to get total discrete action
    newIncreTrack = increTrack + inputAction;
    newIncreTrack(newIncreTrack > 120) = 120;  
    newIncreTrack(newIncreTrack < 0) = 0; 
    sumTrack = sum(newIncreTrack); 
    tau = zeros(sumTrack + numel(init_tau),1); 
    offset = 1;
    
    % Generating normalised mesh from discrete actions
    for i = 1:length(inputAction) 
        tau(offset:offset+newIncreTrack(i)) = init_tau(i)/(newIncreTrack(i)+1); 
        offset = offset+newIncreTrack(i)+1;
    end 
end 
    
function errorArray = errorHandle(inputArray, increTrack,iter) 
    errorArray = zeros(length(increTrack),1);
    offset = 1; 
    w = 2; 

    % Calcualting the offsets and taking the max values between the ranges
    % such that length of observation is always constant
    for i = 1:length(increTrack) 
        errorArray(i) = max(inputArray(offset:offset+(w*increTrack(i))+1)); 
        offset = offset+(w*increTrack(i))+1+1;
    end 
    
end 
