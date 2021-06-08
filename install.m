clc
clear

toolbox_top_level = pwd;
cd ..;
toolbox_location = pwd;
addpath(genpath('multi-robot-gspnr-toolbox'));
cd multi-robot-gspnr-toolbox
open("GettingStarted.mlx")