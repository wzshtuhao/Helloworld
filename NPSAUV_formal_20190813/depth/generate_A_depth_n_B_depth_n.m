global predict_horizon_depth;
global control_horizon_depth;
global which_line_depth;
global A_depth_n;
global B_depth_n;
global B_depth;
global A_depth;
A_depth_n=which_line_depth*A_depth;
for i=2:1:predict_horizon_depth
    A_depth_n=[A_depth_n;which_line_depth*A_depth^i];
end
temp=zeros(1,control_horizon_depth);
temp2=B_depth;
temp(1)=which_line_depth*temp2;
B_depth_n=temp;
for i=2:1:predict_horizon_depth
    temp2=A_depth*temp2;
    temp=[which_line_depth*temp2 temp(1:end-1)];
    B_depth_n=[B_depth_n;temp];
end