global A_psi;
global B_psi;
global A_psi_n;
global B_psi_n;
global predict_horizon_psi;
global control_horizon_psi;
global which_line_psi;
    A_psi_n=which_line_psi*A_psi;
    for i=2:1:predict_horizon_psi
        A_psi_n=[A_psi_n;which_line_psi*A_psi^i];
    end
    temp=zeros(1,control_horizon_psi);
    temp2=B_psi;
    temp(1)=which_line_psi*temp2;
    B_psi_n=temp;
    for i=2:1:predict_horizon_psi
        temp2=A_psi*temp2;
        temp=[which_line_psi*temp2 temp(1:end-1)];
        B_psi_n=[B_psi_n;temp];
    end

