
clear all; 
close all; 
format compact;

[problem,guess]=UnderwaterVehicle;          % Fetch the problem definition
options= problem.settings(50);                  % Get options and solver settings 
%options.tau=[0.2,0.2,0.2,0.2,0.2]'; 
env = UnderwaterEnvTest; 
obsInfo = env.ObsInfo; 
actInfo = env.ActInfo; 
obj = load('trainingElems_AC_PG_MultilatestRewardPerStepVarNewObsAlyChan(TEMP l2 and droput increase)220824.mat');
DeployElems.policy = obj.trainingElemsobj.policy; 
DeployElems.policy.UseMaxLikelihoodAction = true; 
% [solution,MRHistory]=DeployDiscrete_solveMyProblem( problem,guess,options,DeployElems,env);
[solution,MRHistory]=DeployDiscreteNewObs_solveMyProblem( problem,guess,options,DeployElems,env);
[ tv, xv, uv ] = simulateSolution( problem, solution, 'ode113');

%% figure
tt=solution.T;
x1=speval(solution,'X',1,tt);
x2=speval(solution,'X',2,tt); 
x3=speval(solution,'X',3,tt);
x4=speval(solution,'X',4,tt); 
x5=speval(solution,'X',5,tt);
x6=speval(solution,'X',6,tt); 
x7=speval(solution,'X',7,tt);
x8=speval(solution,'X',8,tt); 
x9=speval(solution,'X',9,tt);
x10=speval(solution,'X',10,tt); 

u1=speval(solution,'U',1,tt); 
u2=speval(solution,'U',2,tt); 
u3=speval(solution,'U',3,tt);
u4=speval(solution,'U',4,tt);


figure
hold on
plot(tt,x1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(1), problem.states.xl(1)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(1), problem.states.xu(1)],'r-' )
xlabel('Time [s]')
ylabel('y1')
grid on

figure
hold on
plot(tt,x2,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(2), problem.states.xl(2)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(2), problem.states.xu(2)],'r-' )
xlabel('Time [s]')
ylabel('y2')
grid on

figure
hold on
plot(tt,x1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(3), problem.states.xl(3)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(3), problem.states.xu(3)],'r-' )
xlabel('Time [s]')
ylabel('y3')
grid on

figure
hold on
plot(tt,x2,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(4), problem.states.xl(4)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(4), problem.states.xu(4)],'r-' )
xlabel('Time [s]')
ylabel('y4')
grid on 

figure
hold on
plot(tt,x1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(5), problem.states.xl(5)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(5), problem.states.xu(5)],'r-' )
xlabel('Time [s]')
ylabel('y5')
grid on

figure
hold on
plot(tt,x2,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(6), problem.states.xl(6)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(6), problem.states.xu(6)],'r-' )
xlabel('Time [s]')
ylabel('y6')
grid on 

figure
hold on
plot(tt,x1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(7), problem.states.xl(7)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(7), problem.states.xu(7)],'r-' )
xlabel('Time [s]')
ylabel('y7')
grid on

figure
hold on
plot(tt,x2,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(8), problem.states.xl(8)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(8), problem.states.xu(8)],'r-' )
xlabel('Time [s]')
ylabel('y8')
grid on 

figure
hold on
plot(tt,x1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(9), problem.states.xl(9)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(9), problem.states.xu(9)],'r-' )
xlabel('Time [s]')
ylabel('y9')
grid on

figure
hold on
plot(tt,x2,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.states.xl(10), problem.states.xl(10)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.states.xu(10), problem.states.xu(10)],'r-' )
xlabel('Time [s]')
ylabel('y10')
grid on

figure
hold on
plot(tt,u1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(1), problem.inputs.ul(1)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(1), problem.inputs.uu(1)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('u1')

figure
hold on
plot(tt,u1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(2), problem.inputs.ul(2)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(2), problem.inputs.uu(2)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('u1') 

figure
hold on
plot(tt,u1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(3), problem.inputs.ul(3)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(3), problem.inputs.uu(3)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('u1') 

figure
hold on
plot(tt,u1,'b-' ,'LineWidth',2)
plot([solution.T(1,1); solution.tf],[problem.inputs.ul(4), problem.inputs.ul(4)],'r-' )
plot([solution.T(1,1); solution.tf],[problem.inputs.uu(4), problem.inputs.uu(4)],'r-' )
xlim([0 solution.tf])
xlabel('Time [s]')
grid on
ylabel('u4') 

function target = updateTarget(target, main, smoothFactor)
    target.Learnables.Value = (1 - smoothFactor) * target.Learnables.Value + smoothFactor * main.Learnables.Value;
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
