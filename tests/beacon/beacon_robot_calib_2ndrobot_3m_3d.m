%% Load beacon data calibration "robot_calib_2nd_robot_3m_3d" (see beacon.xls)

clear all
load beacon_robot_calib_2ndrobot_3m_3d;
x = data(1:end,2);   % (size/period) x 100000 (counts/counts)
y = data(1:end,1);   % distance(cm)
x_eval_ini = 4040;   % evaluated distance min (in counts)
x_eval_end = 744;    % evaluated distance max (in counts)
x_eval_delta = 8;    % x delta increment evaluation

