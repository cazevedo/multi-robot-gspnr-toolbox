clc
clear

toolbox_top_level = pwd;
cd ..;
toolbox_location = pwd;
addpath(genpath(pwd));
cd multi-robot-gspnr-toolbox
mkdir logs/execution/
open("GettingStarted.mlx")