global predict_horizon_psi;
global control_horizon_psi;
global  which_line_psi;
global u0;
global  u_threhold;
global  u_min;
global  ts_psi;
global  deltar_limit;
global  d_deltar_limit;
global  weight_psi;
global A_psi;
global B_psi;
global A_psi_n;
global B_psi_n;

    rou=1000;
    L=5.3;
    m=5454.54;
    xG=0;
    Y_v=-0.1;
    Y_r=3.0e-02;
    Y_deltar=2.7e-02;
    N_v=-7.4e-03;
    N_r=-1.6e-02;
    N_deltar=-1.3e-02;
    Y_vdot=-5.5e-02;
    Y_rdot=1.2e-03;
    N_vdot=1.2e-3;
    N_rdot=-3.4e-3;
    Yv=0.5*rou*L^2*Y_v;
    Yr=0.5*rou*L^3*Y_r;
    Ydeltar=0.5*rou*L^2*u0^2*Y_deltar;
    Nv=0.5*rou*L^3*N_v;
    Nr=0.5*rou*L^4*N_r;
    Ndeltar=0.5*rou*L^3*u0^2*N_deltar;
    Yvdot=0.5*rou*L^3*Y_vdot;
    Yrdot=0.5*rou*L^4*Y_rdot;
    Nvdot=0.5*rou*L^4*N_vdot;
    Nrdot=0.5*rou*L^5*N_rdot;
    Iz=13587;

    temp=inv([m-Yvdot m*xG-Yrdot;m*xG-Nvdot Iz-Nrdot])*[Yv Yr-m*u0     Ydeltar;Nv Nr-m*xG*u0    Ndeltar];
    A_psi=[temp(1,1) temp(1,2) 0;
           temp(2,1) temp(2,2) 0;
           0         1         0
          ];
      B_psi=[temp(1,3);temp(2,3);0];

    [A_psi B_psi]=c2d(A_psi,B_psi,ts_psi);
    A_psi=[A_psi B_psi;[0 0 0 1]];
    B_psi=[B_psi;1];
%%%x(t+1)=A_psi*x(t)+B_psi*u(t)%%%
%%%x=[v;r;psi;d_deltar],u=d_deltar%%%
