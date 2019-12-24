%ԭ�ļ���sfuntmpl.m��
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

sizes.NumContStates  = 12;%����״̬�����ĸ���
sizes.NumDiscStates  = 0;%��ɢ״̬��������
sizes.NumOutputs     = 12;%��������ĸ���
sizes.NumInputs      = 6;%��������ĸ���
sizes.DirFeedthrough = 1;%����
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
%u v w p q r x y z phi theta psi
x0  = x_init;%[0.1 0 0 0 0 0 0 0 0 0 0 0];%״̬������ʼ��ֵ
str = [];%����

%
% initialize the array of sample times
%
ts  = [0 0];%ts=[�������� ƫ����],��Ϊ[0 0]��Ϊ����ϵͳ

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



sys = npsauv(x,u);


% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)%������ɢ״̬����һ״̬����x(k+1)=Ax(k)+Bu
sys=[];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)%�����������״̬������y=Cx+Du�Ĳ���

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

