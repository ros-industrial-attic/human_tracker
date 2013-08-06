%% Read ground truth data and save them to a Matlab file:
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

clc; clear all; close all;

%% Parameters:
code_dir = pwd;
dataset_dir = '/home/matteo/Scrivania/kinect_combined/';
results_filename = [code_dir '/../../Results/SwRI_Dataset_Kinect_gt.mat'];

%% Reading and writing:
cd(dataset_dir)
gt_files = dir('*image.tru');

gt_test = zeros(20,4,size(gt_files,1));
gt_timestamps = cell(size(gt_files,1),1);
max_people_per_image = 0;
for i = 1:size(gt_files,1)
    gt_timestamps(i) = {gt_files(i).name(1:13)};
    bboxes = read_tru_file(gt_files(i).name);
    if size(bboxes,1) > 0
        gt_test(1:size(bboxes,1),:,i) = bboxes;
        max_people_per_image = max(max_people_per_image,size(bboxes,1));
    end
end
gt_test = gt_test(1:max_people_per_image,:,:);

%% Save results:
save(results_filename, 'gt_test', 'gt_timestamps');
