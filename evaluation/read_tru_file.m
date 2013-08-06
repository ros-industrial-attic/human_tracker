function bboxes = read_tru_file(filename)
% read ground truth file (.tru) and output the persons' bounding boxes
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

bboxes = [];

fid = fopen(filename);
values = textscan(fid, '%s', 'Delimiter', ':');
if ~isempty(values{1,1})
    values = values{1,1}(2:2:end,1);
    i = 1;
    k = 1;
    bboxes = zeros(20,4);
    while ((i*6) <= size(values,1))
       if (str2double(values((i-1)*6+5)) > 0) && (strcmp(values((i-1)*6+6),'false'))
          curr_rect = str2double(values((i-1)*6+1:(i-1)*6+4)');
          bboxes(k,:) = [curr_rect(1:2) (curr_rect(3:4) - curr_rect(1:2) + 1)];
          k = k+1;
       end
       i = i+1;
    end
    if (k > 1)  % if at least one person is present
        bboxes = bboxes(1:k-1,:);
    else
        bboxes = [];
    end
end
    
fclose(fid);
