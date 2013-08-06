%% Load ground truth data and visualize it in the images:
% 
% Software License Agreement (BSD License)
% 
% Copyright (c) 2013, Matteo Munaro [matteo.munaro@dei.unipd.it]
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
%  * Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
%  * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in the
%      documentation and/or other materials provided with the distribution.
%  * Neither the name of the Southwest Research Institute, nor the names
%      of its contributors may be used to endorse or promote products derived
%      from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% If you use this code, please cite:
%   M. Munaro, F. Basso and E. Menegatti. Tracking people within groups with RGB-D data. 
%   In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
%
clc; clear all; close all;

%% Parameters:
code_dir = pwd;
dataset_dir = '/home/matteo/Scrivania/kinect_combined/';
gt_filename = [code_dir '/../../Results/SwRI_Dataset_Kinect_gt.mat'];

%% Load ground truth and visualize:
load(gt_filename);

cd(dataset_dir)
for i = 1:size(gt_timestamps,1)
    imname = dir([gt_timestamps{i,1} '*image.jpg']);
    I = imread(imname.name);
    imshow(I); 
    hold on;
    
    j = 1;
    while (j <= size(gt_test,1)) && (gt_test(j,3,i) > 0)
        rectangle('Position', gt_test(j,:,i), 'LineWidth', 2, 'EdgeColor', 'b');
        j = j+1;
    end
    
    pause(0.1);
    hold off;
end
