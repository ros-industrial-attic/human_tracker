%% Evaluation pipeline for computing tracking results %%
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
code_dir = pwd;
addpath(code_dir);

%% Parameters:
% Select tracking results written to csv file:
[filename, pathname] = uigetfile({'*.csv'},'File Selector');  

dataset_dir = '/home/matteo/Scrivania/kinect_combined/';
gt_filename = 'SwRI_Dataset_Kinect_gt.mat';

imm_ext = '*.jpg';
frame_h = 480;
frame_w = 640;

%% Read tracking results:
results_filename = [pathname filename];
filedir = pathname;
output_name = [results_filename(1:end-4) '.mat'];

read_tracking_results;

%% Load data and select only tracking results for which ground truth is available:
load(gt_filename);
new_tracking_results = zeros(20,4,size(gt_test,3));
max_number_tracked_people = 0;
k = 1;
for i = 1:size(gt_test,3)
    gt_time = gt_timestamps{i};
    res_time = tracking_timestamps{k};
    image_time = image_timestamps{k};
    while (str2double(image_time) < str2double(gt_time))  % iterate until the current gt frame is found
        k = k + 1;
        if (k > size(tracking_timestamps,1))
            break;
        end
        res_time = tracking_timestamps{k};
        image_time = image_timestamps{k};
    end
    if (strcmp(res_time, gt_time))   % if there are some tracked people for that frame
        new_tracking_results(1:size(tracking_results,1),:,i) = tracking_results(:,2:5,k);
    end
end
detection_results = new_tracking_results(1:size(tracking_results,1),:,:);

disp('Tracking results read.');

%% Evaluating data:
% Evaluation for detectors (no matters of tracks IDs)
[TAR, FAR, TRR, FRR, false_pos, num_detection, perc_fp, fppf, miss, num_people, perc_miss, mpf, false_pos_flag] = detector_evaluation(gt_test,detection_results);
disp(['FRR = ' num2str(100*FRR) '%']);
disp(['fppf = ' num2str(fppf)]);
disp(['N_people = ' num2str(num_people)]);

disp('Evaluation complete.');

%% Save false positives flag:
save([output_name(1:end-4) '_detection_evaluation_results_03.mat']);
% save([output_name(1:end-4) '_false_pos_03.mat'],'false_pos_flag');
