%原文件在sfuntmpl.m中
function [sys,x0,str,ts,simStateCompliance] = s_npsauv_model(t,x,u,flag,x_init)

switch flag,


  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(x_init);
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


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(x_init)


sizes = simsizes;

sizes.NumContStates  = 12;%连续状态变量的个数
sizes.NumDiscStates  = 0;%离散状态变量个数
sizes.NumOutputs     = 12;%输出变量的个数
sizes.NumInputs      = 6;%输入变量的个数
sizes.DirFeedthrough = 1;%不管
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
%u v w p q r x y z phi theta psi
x0  = x_init;%[0.1 0 0 0 0 0 0 0 0 0 0 0];%状态变量初始的值
str = [];%不管

%
% initialize the array of sample times
%
ts  = [0 0];%ts=[采样周期 偏移量],设为[0 0]则为连续系统

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



sys = npsauv(x,u);


% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)%计算离散状态的下一状态，即x(k+1)=Ax(k)+Bu
sys=[];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)%计算输出，即状态方程中y=Cx+Du的部分

sys = x;
while x(12)>2*pi
    x(12)=x(12)-2*pi;
end
while x(12)<-2*pi
    x(12)=x(12)+2*pi;
end

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

