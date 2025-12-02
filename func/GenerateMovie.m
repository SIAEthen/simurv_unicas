function GenerateMovie(flag)
%
% Generate movie or numbered frames according to the flag
%
% GenerateMovie(flag)
%
% input:
%       flag   dim 1       ==1 uses avifile command from 
%                              matlab and show animation (slow)
%                          ==0 generate frames/frame%0d.jpg files
%                              in hidden figures
%                              to be used with other sw (fast, default)
%
% output:
%       look under frames/ directory
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

if nargin==0
    flag = 0;
end

clc

% ---------------------------------------------------
% load the data
% ---------------------------------------------------
cd ../output
data_name = uigetfile('*.mat', 'select data to play');
if (data_name==0)
    fprintf('\n GenerateMovie.m: no file selected\n');
    cd ../func
    return;
end
eval(['load ' data_name]);
cd ../func

if Ts==0.001;
    my_dec = 100;
else
    my_dec = 10;
end

% ---------------------------------------------------
% some info depending on the input
% ---------------------------------------------------
fprintf('\n\n GenerateMovie \n')
if flag==1
    fprintf('\n now creating avifile for the selected data in frames/simurv_movie.avi')
    fprintf('\n for a faster run try with other flag')
    FileName  = 'frames/simurv_movie.avi';
    FrameRate = 10; 
    myvideo = avifile(FileName,'fps', FrameRate, 'quality', 100, 'compression', 'None');
else
    delete frames/frame*.jpg
    fprintf('\n now creating frames for the selected data in frames/*frame%%04d.jpg')
    fprintf('\n number of points: %d', npti)
    fprintf('\n decimation      : %d', my_dec)
    fprintf('\n number of frames: %d', ceil(npti/my_dec))
    fprintf('\n')
end

% ---------------------------------------------------
% main loop
% ---------------------------------------------------
t_start_sim = clock();
if (flag==1)
    hf = figure;
    set(hf,'Position',[50 30 1000 750]);
    hold on
end
k = 1;
for i = 1:my_dec:npti
    if (flag==0)
        hf = figure('Visible','off','Position',[50 30 1000 750]);
        hold on
    end
    
    DrawSnap(eta(:,i),[DH(:,1:3) q(:,i)],PARAM,2.1);
    %plot3(eta_ee1_d(1,1:i),eta_ee1_d(2,1:i),eta_ee1_d(3,1:i),'g')
    plot3(target(1,1:i),target(2,1:i),target(3,1:i),'g')
    plot3(eta_ee1(1,1:i),eta_ee1(2,1:i),eta_ee1(3,1:i),'k')
    if (exist('obj_pos','var')==1)
        %DrawCube(obj_pos,obj_size);
        DrawCube(target(:,i),obj_size);
    end
    axis tight;
    if (flag==1)
        myvideo = addframe(myvideo, getframe(gcf));
        clf(hf);
    else
        drawnow;
        %saveas(gcf,sprintf('frames/frame%04d',k),'jpg');
        %saveas(hf,sprintf('frames/frame%04d',k),'jpg');
        print('-djpeg', sprintf('frames/frame%04d',k));
        hold off
        clf(hf);
    end
    k = k + 1;
    EstimateEndSim(t_start_sim, k, ceil(npti/my_dec));
end
close(hf);
if (flag==1)
    myvideo = close(myvideo);
else
    fprintf('\n to record a video under Linux, from shell:');
    fprintf('\n avconv -y -r 25 -i frame%%04d.jpg -s 640x480 -b 5000k foo.avi');
end

fprintf('\n');


