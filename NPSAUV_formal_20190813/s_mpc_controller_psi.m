%原文件在sfuntmpl.m中
function [sys,x0,str,ts,simStateCompliance] = s_MPC_controller(t,x,u,flag)

switch flag,


  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    u=0;

  case 1,
    sys=mdlDerivatives(t,x,u);


  case 2,
    sys=mdlUpdate(t,x,u);


  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);


  case 9,
    sys=mdlTerminate(t,x,u);


  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

global ts_psi;
sizes = simsizes;

sizes.NumContStates  = 0;%连续状态变量的个数
sizes.NumDiscStates  = 3;%离散状态变量个数
sizes.NumOutputs     = 5;%输出变量的个数
sizes.NumInputs      = 5;%输入变量的个数
sizes.DirFeedthrough = 1;%不管
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0 0 0];%状态变量初始的值
str = [];%不管

%
% initialize the array of sample times
%
ts  = [ts_psi 0];%ts=[采样周期 偏移量],设为[0 0]则为连续系统

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)%计算连续状态的微分，即dx/dt=Ax+Bu
sys = [];
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)%计算离散状态的下一状态，即x(k+1)=Ax(k)+Bu
sys = x;

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)%计算输出，即状态方程中y=Cx+Du的部分
%x=[v r psi set]
    global A_psi;
    global B_psi;
    global A_psi_n;
    global B_psi_n;
    global predict_horizon_psi;
    global control_horizon_psi;
    global d_deltar_limit;
    global weight_psi;
    global deltar_limit;
    global u0;
    global u_threhold;
    global u_min;
    persistent s_predict_mpc;
    persistent delta_pre;
    if isempty(delta_pre)
        delta_pre=0;
    end
    
    s=[u(1);u(2);u(3);delta_pre];%s=[v r psi]

    if abs(u0-u(4))>u_threhold
        u0=u(4);
        if u(4)<u_min
            u0=u_min;
        end
        linear_npsauv_psi;
        generate_A_psi_n_B_psi_n;
    end
    set=u(5);
    if isempty(s_predict_mpc)
        s_predict_mpc=s;
        
    end
    weight=weight_psi;

            
        ref=zeros(predict_horizon_psi,1);

        for i=1:predict_horizon_psi
            ref(i,1)=set;
        end

        H=B_psi_n'*B_psi_n;
        g=-(ref-A_psi_n*s)'*B_psi_n;%x为状态变量，ref 为参考输出
        lb=zeros(control_horizon_psi,1);
        ub=zeros(control_horizon_psi,1);
        for i=1:control_horizon_psi
            lb(i,1)=-d_deltar_limit*pi/180;
            ub(i,1)=d_deltar_limit*pi/180;
        end
      
        sum=zeros(control_horizon_psi,control_horizon_psi);
        for i=1:control_horizon_psi
            for j=1:i
                sum(i,j)=1;
            end
        end
        sum=[sum;-sum];
        b_1=(deltar_limit*pi/180-delta_pre)*ones(control_horizon_psi,1);
        b_2=(deltar_limit*pi/180+delta_pre)*ones(control_horizon_psi,1);
        b=[b_1;b_2];
        phi_min=quadprog(weight*H+(1-weight)*eye(control_horizon_psi),weight*g,sum,b,[],[],lb,ub);
        y=phi_min(1);
        delta_pre=delta_pre+y;
        s_predict_mpc=A_psi*s+B_psi*y;
sys =[delta_pre;delta_pre;0;0;0];%;[0 0 1]*(s-s_predict_mpc)];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)%初始化ts=[-2 0]时，才会调用此函数，
%用于离散系统需要变步长时，采样时间的设置，其他情况不用管
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)%结束时需要干点什么

sys = [];

% end mdlTerminate