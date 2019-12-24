global ts_depth;
global u0;
global A_depth;
global B_depth;
global predict_horizon_depth;
global control_horizon_depth;
global which_line_depth;
global A_depth_n;
global B_depth_n;
global weight_depth;
global d_deltas_limit;
global deltas_limit;
ts_depth=1;
which_line_depth=[0 0 0 1 0];
u0=0.1;%只是为了保证初始的时候不出错
predict_horizon_depth=50;
control_horizon_depth=5;
weight_depth=0.7;
deltas_limit=20;%in degree
d_deltas_limit=20;%in degree