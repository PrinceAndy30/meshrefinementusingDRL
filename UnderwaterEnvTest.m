classdef UnderwaterEnvTest < rl.env.MATLABEnvironment
    
    %% Properties (set properties' attributes accordingly)
    properties
        ObsInfo 
        ActInfo 
    end
    
    properties
        
        Observation; 
        Reward; 
        isDone = false; 
        errorInMin 
        errorInMax
    end
    

    %% Necessary Methods
    methods              

        function this = UnderwaterEnvTest()
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([2 1]);
            ObservationInfo.Name = 'Error Observations';
            ObservationInfo.Description = 'MaxAbsError, MeanRelError';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([1 1]);
            ActionInfo.Name = 'tau'; 
            ActionInfo.LowerLimit = 0; 
            ActionInfo.UpperLimit = 1;  
            % ActionInfo = rlFiniteSetSpec([0 2 4 6 8 10 20]); % [0 1 2 3 4 5 6]
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo); 
            this.ObsInfo = ObservationInfo; 
            this.ActInfo = ActionInfo;

        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            % signal that the environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
        
        
        function [obsInfo, actInfo] = getObs_Act_Info(this) 

            obsInfo = this.ObsInfo; 
            actInfo = this.ActInfo;
            
        end 
        % Reset environment to initial state and output initial observation
        function varargout = reset(this, varargin) 

                actor=varargin{1};
                targetActor=varargin{2};
                critic1=varargin{3}; 
                targetCritic1=varargin{4};
                critic2=varargin{5}; 
                targetCritic2=varargin{6};

            actor_out = resetState(actor); 
            critic1_out = resetState(critic1); 

                critic2_out = resetState(critic2); 
                targetActor_out = resetState(targetActor);
                targetCritic1_out = resetState(targetCritic1); 
                targetCritic2_out = resetState(targetCritic2);


            varargout{1} = actor_out; 
            varargout{2} = critic1_out; 

                varargout{3} = critic2_out;  
                varargout{4} = targetActor_out; 
                varargout{5} = targetCritic1_out; 
                varargout{6} = targetCritic2_out; 
            this.Observation = [];

            
            % Signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end   
        
        % Different reward functions
        % function Reward = getReward(this, MaxAbsError, MaxConstVioError,eTol,cETol,compTime,action) 
        %     dVal = [0,2,4,6,8,10,20];
        %     % log_error = log(error); 
        %     minLog = min(MaxAbsError); 
        %     maxLog = max(MaxAbsError); 
        %     errorScale = (MaxAbsError - minLog) ./ (maxLog - minLog) * 20; 
        %     discreteAction = zeros(size(MaxAbsError)); 
        %     for i = 1:length(MaxAbsError) 
        %         [~,errorIndex] = min(abs(dVal - errorScale(i)));  
        %         discreteAction(i) = dVal(errorIndex);
        %     end 
        %     action = squeeze(action);
        %     discreteAction = squeeze(discreteAction);
        %     exponents = exp(-0.5.*(abs(action - discreteAction)./4).^2);
        %     R1 = 0.5 * sum(exponents)/numel(exponents); 
        %     MaxError = log(max(MaxConstVioError)); 
        %     tol = log(eTol(1));
        %     sr = 1.5; 
        %     sl = 2;
        %     Reward = exp(-0.5*(((MaxError < tol)*(abs(MaxError - tol)/sl)^2) + ((MaxError >= tol)*(abs(MaxError - tol)/sr)^2))) + R1
        % end 

        function Reward = getReward(this, MaxAbsError, MaxConstVioError,eTol,cETol,compTime,action) 
            MaxError = log(max(MaxAbsError)); 
            tol = log(eTol(1));
            %Reward = exp(-0.5*(abs(MaxError - tol)/2)^2) 
            sr = 1.5; 
            sl = 2;
            Reward = exp(-0.5*(((MaxError < tol)*(abs(MaxError - tol)/sl)^2) + ((MaxError >= tol)*(abs(MaxError - tol)/sr)^2)))
        end 
        % function Reward = getReward(this, MaxAbsError, MaxConstVioError,eTol,cETol,compTime,action) 
        %     P_ErrorMatrix = (MaxAbsError - eTol)./eTol; 
        %     constVals = numel(MaxConstVioError);
        %     cETolVals = numel(cETol); 
        %     cETol = [cETol zeros(1,constVals-cETolVals)];
        %     P_RErrorMatrix = (MaxConstVioError - cETol)./cETol;  
        %     val = abs(log(max(eTol)));
        %     errorIdx = find(P_ErrorMatrix < 0); 
        %     rErrorIdx = find(P_RErrorMatrix < 0);
        %     L_ErrorMatrix = log(abs(P_ErrorMatrix)+10^-40); 
        %     L_RErrorMatrix = log(abs(P_RErrorMatrix)+10^-40); 
        %     L_ErrorMatrix(errorIdx) = -1*-L_ErrorMatrix(errorIdx); 
        %     L_RErrorMatrix(rErrorIdx) = -1*-L_RErrorMatrix(rErrorIdx);
        %     R_ErrorMatrix = rescale(L_ErrorMatrix,-1,1,'InputMin',-val,'InputMax',val);%,5.3);%ErrorMatrix*10^7;%rescale(ErrorMatrix,-1,1,'InputMin',env.errorInMin,'InputMax',env.errorInMax);%'InputMin',10^-7,'InputMax',10^-3);%"InputMin",10^-4,"InputMax",5);%robustScaleAndMinMax(ErrorMatrix);%rescale(ErrorMatrix,-10,10,"InputMin",min(ErrorMatrix),"InputMax",max(ErrorMatrix));%"InputMin",10^-12,"InputMax",10^-4); 
        %     R_RErrorMatrix = rescale(L_RErrorMatrix,-1,1,'InputMin',-val,'InputMax',val);%,5.3);%robustScaleAndMinMax(RErrorMatrix);%rescale(RErrorMatrix,-1,1,'InputMin',10^-7,'InputMax',10^-3);%"InputMin",10^-4,"InputMax",5);%robustScaleAndMinMax(RErrorMatrix);%rescale(RErrorMatrix,-10,10,"InputMin",min(RErrorMatrix),"InputMax",max(RErrorMatrix));%"InputMin",10^-12,"InputMax",10^-4);
        % 
        %     weight = 20;
        %         if compTime <= 1.5 
        %             RTime = 60*exp(-0.5*compTime); 
        %         else 
        %             RTime = -30; 
        %         end 
        %         if max(R_ErrorMatrix) < 0 
        %             R_error = 80; 
        %         else 
        %             R_error = 80 * exp(-2*max(R_ErrorMatrix)); 
        %         end 
        %         if max(R_RErrorMatrix) < 0 
        %             R_Cerror = 10; 
        %         else 
        %             R_Cerror = 10 * exp(-10*max(R_RErrorMatrix)); 
        %         end 
        %         Reward =  R_error + R_Cerror + RTime%+ (-tanh(compTime-2)*25);%20 %-0.01.*(1+exp(4.5.*(compTime-0.1)));%(75./(1+exp(6.5*(compTime-0.001))));
        %         if var(action) == 0 
        %             Reward = Reward - 20 
        %         end
        % end 

        % function Reward = getReward(this, MaxAbsError, MaxConstVioError,t,compTime) 
        %         %Reward = -max(MaxAbsError)-max(MaxConstVioError);  
        %         std = 8;
        %         weight = 2;%5;
        %         %term = 1*(-log(max(MaxAbsError)) -
        %         %log(max(MaxConstVioError))) - weight*computationTime;  
        %         % if max(MaxConstVioError) ~= 0 && max(MaxAbsError) ~= 0 && max(MaxAbsError) < 1
        %         %     Reward = 1*(-log(max(MaxAbsError)) - log(max(MaxConstVioError))) - weight*computationTime - cost; %exp(-0.5 * (abs(term - 20) / std)^2); 
        %         % else 
        %         %     Reward = -max(MaxAbsError) - max(MaxConstVioError) - weight*computationTime - cost; 
        %         % end  
        %         if compTime <= 2 
        %             RTime = 100*exp(-0.5*compTime); 
        %         else 
        %             RTime = -30; 
        %         end 
        %         % 
        %         % if max(MaxAbsError) < 10^-6 
        %         %     R_AbsError = 0; 
        %         % else
        %         %     R_AbsError = weight*log(1/(abs(max(MaxAbsError) - t) + 10^-12)); 
        %         % end 
        %         % 
        %         % if max(MaxConstVioError) < 10^-6 
        %         %     R_ConstError = 0; 
        %         % else
        %         %     R_ConstError = log(1/(abs(max(MaxConstVioError) - t) + 10^-12));
        %         % end
        %         % 
        %         % Reward = R_AbsError + R_ConstError + RTime
        %         Reward = weight*log(1/(abs(max(MaxAbsError) - t) + 10^-12)) + log(1/(abs(max(MaxConstVioError) - t) + 10^-12)) + RTime %+ (-tanh(compTime-2)*25);%20 %-0.01.*(1+exp(4.5.*(compTime-0.1)));%(75./(1+exp(6.5*(compTime-0.001))));
        %         %Reward = 1*log(1/(max(MaxAbsError) - t + 10^-5)) + log(1/(max(MaxConstVioError) - t + 10^-5)) + (-tanh(compTime-1.5)*60);
        % end 


        function B = convertCellArray(this,A,obs)
            % Converts a obsx1xn cell array to a 3x1 cell array
            % A - Input obsx1xn cell array
            % B - Output obsx1 cell array with concatenated elements along the third dimension
        
            % Get the size of the third dimension
            [~, ~, n] = size(A);
        
            % Initialize the resulting 3x1 cell array
            B = cell(obs, 1);
        
            % Concatenate elements along the third dimension
            for i = 1:obs
                concatenatedElements = [];
                for j = 1:n
                    concatenatedElements = [concatenatedElements, A{i, 1, j}];
                end
                B{i, 1} = concatenatedElements;
            end
        end 

        function B = convertArray(this,A)
            % Converts a 3x1x99 array to a 3x1 cell array
            
            % Check if the input array has the expected dimensions
            [rows, cols, depth] = size(A);
            % Initialize the resulting 3x1 cell array
            B = cell(rows, 1);
            
            % Extract and concatenate elements along the third dimension
            for i = 1:rows
                concatenatedElements = reshape(A(i, 1, :), [1, depth]);
                B{i} = concatenatedElements;
            end
        end
    end
end
