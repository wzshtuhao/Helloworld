%ԭ�ļ���sfuntmpl.m��
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


sizes = simsizes;

sizes.NumContStates  = 0;%����״̬�����ĸ���
sizes.NumDiscStates  = 3;%��ɢ״̬��������
sizes.NumOutputs     = 1;%��������ĸ���
sizes.NumInputs      = 1;%��������ĸ���
sizes.DirFeedthrough = 1;%����
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0 0 0 ];%״̬������ʼ��ֵ
str = [];%����

%
% initialize the array of sample times
%
ts  = [1 0];%ts=[�������� ƫ����],��Ϊ[0 0]��Ϊ����ϵͳ

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
function sys=mdlDerivatives(t,x,u)%��������״̬��΢�֣���dx/dt=Ax+Bu
sys = [];
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)%������ɢ״̬����һ״̬����x(k+1)=Ax(k)+Bu
sys = x;

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)%�����������״̬������y=Cx+Du�Ĳ���

persistent integral;
if isempty(integral)
    integral=0;
end
integral=integral+u;
if integral>5
    integral=5;
elseif integral<-5
    integral=-5;
end
sys=5000*u+300*integral;


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
function sys=mdlGetTimeOfNextVarHit(t,x,u)%��ʼ��ts=[-2 0]ʱ���Ż���ô˺�����
%������ɢϵͳ��Ҫ�䲽��ʱ������ʱ������ã�����������ù�
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)%����ʱ��Ҫ�ɵ�ʲô

sys = [];

% end mdlTerminate