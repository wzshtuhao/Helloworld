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

global ts_psi;
sizes = simsizes;

sizes.NumContStates  = 0;%����״̬�����ĸ���
sizes.NumDiscStates  = 3;%��ɢ״̬��������
sizes.NumOutputs     = 5;%��������ĸ���
sizes.NumInputs      = 5;%��������ĸ���
sizes.DirFeedthrough = 1;%����
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0 0 0];%״̬������ʼ��ֵ
str = [];%����

%
% initialize the array of sample times
%
ts  = [ts_psi 0];%ts=[�������� ƫ����],��Ϊ[0 0]��Ϊ����ϵͳ

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
        g=-(ref-A_psi_n*s)'*B_psi_n;%xΪ״̬������ref Ϊ�ο����
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