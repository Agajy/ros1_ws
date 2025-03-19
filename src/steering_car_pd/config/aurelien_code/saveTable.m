clc; clear;
load("Marconi_computed_path.mat")
theta = zeros(721, 1);
data_matrix = [x, y, theta];
writematrix(data_matrix, 'output.xlsx');