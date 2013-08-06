function [TAR, FAR, TRR, FRR, false_pos, num_detection, perc_fp, fppf, miss, num_people, perc_miss, mpf, false_pos_flag] = detector_evaluation(ground_truth,detection_results)
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
% This is a function for evaluting object detectors. It computes:
%  the number of false positives (false_pos)
%  the number of true positives (num_detection)
%  the false positives percentage (perc_fp)
%  the number of false positives per frame (fppf)
%  the number of miss (miss)
%  the total number of people instances in the ground truth (num_people)
%  the miss percentage (perc_miss)
%  the number of miss per frame (mpf)
%  the false acceptance rate (FAR)
%  the false rejection rate (FRR)
%  the true acceptance rate (TAR)
%  the true rejection rate (TRR)
%  a false positive mask which points at false positives (false_pos_flag)
%
%  version 3: with Munkres algorithm to associate detections and ground
%  truth instances in a globally optimal way
%
% If you use this code, please cite:
%   M. Munaro, F. Basso and E. Menegatti. Tracking people within groups with RGB-D data. 
%   In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
%

threshold = 0.3;       % threshold for bounding box test with PASCAL rule (intersection over union)
false_pos_flag = false(size(detection_results,1),size(detection_results,3));  % num_tracks x num_frames: it contains true if a track is a false positive at the current frame

T = size(ground_truth,3);

% Detection results:
people = detection_results;     

%% Main loop (for computing false positives and misses):
num_detection = 0;
num_people = 0;
false_pos = 0;
miss = 0;
mean_overlap = 0;
for t = 1:T
    % Munkres assignment gt-detections:
    costMatrix = createCostMatrix(ground_truth(:,:,t), people(:,:,t), threshold);
    [assignment, cost] = munkresOptimalAssignment(costMatrix);  
    
    % Variables update:
    num_people = num_people + size(costMatrix,1);
    for i = 1:length(assignment)
        if assignment(i) > 0
            rect_gt = ground_truth(i,:,t);
            gt_area = rect_gt(3)*rect_gt(4);              % bounding box area for ground truth 
            rect_class = detection_results(assignment(i),1:4,t);
            class_area = rect_class(3)*rect_class(4);     % bounding box area for detection result
            overlap = rectint(rect_class,rect_gt);
            union_area = class_area + gt_area - overlap;  % area of bounding boxes union     
            mean_overlap = mean_overlap + overlap/union_area;
            num_detection = num_detection + 1;
        else
            miss = miss + 1;
        end
    end
    if isempty(assignment)
       miss = miss + size(ground_truth(ground_truth(:,3,t)>0,:,t),1);
    end
    for i = 1:size(costMatrix,2)
        if ~(sum(assignment == i) > 0)
            if isfinite(people(i,1,t))
                num_detection = num_detection +1;
                false_pos = false_pos + 1;
                false_pos_flag(i,t) = true;
            end
        end
    end
    
%     disp(costMatrix);
%     disp(assignment);
    
end

mpf = miss/T;                                       % miss per frame 
if num_people == 0
    perc_miss = 0;
else
    perc_miss = 100*miss/num_people;                % miss percentage: 100*#miss/#people
end

fppf = false_pos/T;                                 % false positives per frame 
if num_detection == 0
    perc_fp = 0;
else
    perc_fp = 100*false_pos/num_people;             % false positives percentage: 100*#f.p./#people
end

%% Compute TAR, FAR, TRR, FRR:

N_tp = num_detection - false_pos;                   % number of true positives
N_tn = T;                                           % 1 true negative is considered for every frame

% TAR: True Acceptance Rate�
TAR = N_tp/(N_tp+miss);

% FAR: False Acceptance Rate
FAR = false_pos/(false_pos+N_tn);

% TRR: True Rejection Rate: N_tn/(false_pos+N_tn)
TRR = 1-FAR;

% FRR: False Rejection Rate: miss/(N_tp+miss)
FRR = 1-TAR;

function costMatrix = createCostMatrix(ground_truth, people, threshold)
% create cost matrix between ground truth entries and results entries 
ground_truth = ground_truth(ground_truth(:,3)>0,:); % remove empty entries
people = people(people(:,3)>0,:);                   % remove empty entries
costMatrix = inf(size(ground_truth,1),size(people,1));
for i = 1:size(ground_truth,1)
    rect_gt = ground_truth(i,:);
    gt_area = rect_gt(3)*rect_gt(4);        % area of ground truth bounding box                               
    for j = 1:size(people,1)
        if isfinite(people(j,1));
            rect_class = people(j,:);
            class_area = rect_class(3)*rect_class(4);     % area of detection bounding box 
            overlap = rectint(rect_class,rect_gt);
            union_area = class_area + gt_area - overlap;  % area of union of bounding boxes
            if overlap/union_area >= threshold            % if j passes the test, the complementary of the result is inserted in costMatrix
                costMatrix(i,j) = 1 - overlap/union_area;
            end 
        end
    end
end

