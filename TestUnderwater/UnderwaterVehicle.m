function [problem,guess] = UnderwaterVehicle 


InternalDynamics=@underwater_Dynamics_Internal;
SimDynamics=@underwater_Dynamics_Sim;
% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@settings_UnderwaterVehicle; 


%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=1.2;     
problem.time.tf_max=1.2; 
guess.tf=10000*60;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[]; 

% Initial conditions for system.
x10=0; x20=0; x30=0.2; x40=pi/2; x50=0.1; x60=(-pi/4); 
x70=1; x80=0; x90=0.5; x100=0.1;
problem.states.x0=[x10 x20 x30 x40 x50 x60 x70 x80 x90 x100];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[x10 x20 x30 x40 x50 x60 x70 x80 x90 x100]; 
problem.states.x0u=[x10 x20 x30 x40 x50 x60 x70 x80 x90 x100]; 

% State bounds. xl=< x <=xu
%x1_min=-inf; x2_min=-inf;
%x1_max=inf; x2_max=inf;
x1_min = 0; x2_min = 0; x3_min = 0; x4_min = (pi/2 - 0.02); x5_min = 0; x6_min = -pi/4; 
x7_min = 0; x8_min = 0; x9_min = 0; x_10_min = 0;

x1_max=1; x2_max=0.5; x3_max=0.2; x4_max=(pi/2 + 0.02); x5_max=0.1; x6_max=0; 
x7_max=1; x8_max=0; x9_max=0.5; x_10_max=0.1;
problem.states.xl= [-inf -inf -inf x4_min -inf -inf -inf -inf -inf -inf]; %[x1_min x2_min x3_min x4_min x5_min x6_min x7_min x8_min x9_min x_10_min];
problem.states.xu= [inf inf inf x4_max inf inf inf inf inf inf]; %[x1_max x2_max x3_max x4_max x5_max x6_max x7_max x8_max x9_max x_10_max]; 

% State error bounds
problem.states.xErrorTol_local=[1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6];
problem.states.xErrorTol_integral=[1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6];


% State constraint error bounds
problem.states.xConstraintTol=[1e-4 1e-4 1e-4 1e-4 1e-4 1e-4 1e-4 1e-4 1e-4 1e-4];

% Terminal state bounds. xfl=< xf <=xfu
x1f=1; x2f=0.5; x3f=0; x4f=pi/2; x5f=0; x6f=0; 
x7f=0; x8f=0; x9f=0; x10f=0;
problem.states.xfl=[x1f x2f x3f x4f x5f x6f x7f x8f x9f x10f];
problem.states.xfu=[x1f x2f x3f x4f x5f x6f x7f x8f x9f x10f];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[x10 x1f];
guess.states(:,2)=[x20 x2f]; 
guess.states(:,3)=[x30 x3f]; 
guess.states(:,4)=[x40 x4f]; 
guess.states(:,5)=[x50 x5f]; 
guess.states(:,6)=[x60 x6f]; 
guess.states(:,7)=[x70 x7f]; 
guess.states(:,8)=[x80 x8f]; 
guess.states(:,9)=[x90 x9f]; 
guess.states(:,10)=[x100 x10f]; 

problem.inputs.N=0;       
      
% Input bounds
u1l= -15; u1u=15; 
u2l= -15; u2u=15; 
u3l= -15; u3u=15; 
u4l= -15; u4u=15; 

problem.inputs.ul=[u1l u2l u3l u4l]; 
problem.inputs.uu=[u1u u2u u3u u4u];

% Bounds on the control action
problem.inputs.u0l=[u1l u2l u3l u4l];
problem.inputs.u0u=[u1u u2u u3u u4u]; 

problem.inputs.ufl=[u1l u2l u3l u4l];
problem.inputs.ufu=[u1u u2u u3u u4u];

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1 0.1 0.1 0.1];

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[u1l u1u]; 
guess.inputs(:,2)=[u1l u1u]; 
guess.inputs(:,3)=[u1l u1u];
guess.inputs(:,4)=[u1l u1u]; 

% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

problem.constraints.gl=[];
problem.constraints.gu=[];
problem.constraints.gTol_neq=[];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];
problem.constraints.bTol=[];  


% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------



stageCost = u(:,1).*u(:,1)+u(:,2).*u(:,2)+u(:,3).*u(:,3)+u(:,4).*u(:,4);

%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost=0;

%------------- END OF CODE --------------


function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1}; 

bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.discretization,'hpLGR')) || (strcmp(options.discretization,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc,diff(t_segment)];
        end
    end
end
