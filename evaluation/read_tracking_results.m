%% Read human_tracker results and save them as mat file %%
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

%% Remove first row from results file
file_to_read = [results_filename(1:end-4) '_b.csv'];
if (~exist(file_to_read,'file'))
    fid = fopen(results_filename);
    fout = fopen(file_to_read, 'w');
    line = fgetl(fid);
    while ~feof(fid)
        line =fgetl(fid);
        fprintf(fout, line);
        fprintf(fout, '\r\n');
    end
    fclose(fid);
    fclose(fout);
end

%% Load tracking results file:
data = csvread(file_to_read);

%% Write results to matrix:
cd(dataset_dir)
images = dir('*image.jpg');

tracking_results = zeros(20,5,size(images,1));
tracking_timestamps = cell(size(images,1),1);
image_timestamps = cell(size(images,1),1);
max_number_tracked_people = 0;
k = 1;
for i =1:size(images,1)
   if (mod(i,1000) == 0)
       disp(i);
   end
    
   image_time = images(i).name(1:13);
   image_timestamps(i) = {image_time};
%    imshow(imread(images(i).name));
%    hold on;
  
   if (k > size(data,1))
       break;
   end
   results_time = floor(data(k,1)/(10^6));
   while (results_time < str2double(image_time))
       k = k + 1;
       if (k > size(data,1))
           break;
       end
       results_time = floor(data(k,1)/(10^6));
   end
   row = 1;
   while (strcmp(num2str(results_time), image_time))        
%         rectangle('Position',data(k,17:20), 'LineWidth',2, 'EdgeColor','r');
        tracking_results(row, :, i) = [data(k,2) data(k,17:20)];
        row = row+1;
        k = k+1;
        if (k > size(data,1))
           break;
        end
        results_time = floor(data(k,1)/(10^6));
   end
   if (row > 1) % if at least one person has been found
       tracking_timestamps(i) = {image_time};
       max_number_tracked_people = max(max_number_tracked_people, row - 1);
   end
   
%    pause(0.0001);
%    hold off
end
tracking_results = tracking_results(1:max_number_tracked_people,:,:);

%% Saving:
save([results_filename(1:end-4) '.mat'], 'tracking_results', 'image_timestamps', 'tracking_timestamps');
