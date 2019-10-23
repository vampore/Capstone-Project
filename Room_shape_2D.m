%if assume room dimension is 6 5 3
%then here consider floor (with dimension 6 5)

%set origin at speaker while finding IS, then transform all ISs to global
%coordinate
%this version is to form a room

clear all
close all
clc

%% Generate impulse response
c = 343;                    % Sound velocity (m/s)
fs = 96000;                 % Sample frequency (samples/s)
room_dim = [6 5 4];                % Room dimensions [x y z] (m)
beta = 1;                 % Reverberation time (s)
n = 2^12;                   % Number of samples (original 2^12)
mtype = 'omnidirectional';  % Type of microphone
order = -1;                 % -1 equals maximum reflection order!
dim = 2;                    % Room dimension
orientation = 0;            % Microphone orientation (rad)
hp_filter = 1;              % Enable high-pass filter
srd=0.5; %distance from source to receivers %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para
dist_move=0.5; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para
srd_t=srd;
swd=srd+0.3; %distance from sensor to wall
initial_source_loc = [2 1 2];  % Source position [x y z] (m)
result_source_loc=[];
result_im_source=[];
result_slope=[];
result_intercept=[];
iii=1;
kk=0;
plot_source_loc=[];
line_idx=1;
cc=0;
ccc=0;
cccc=0;
kkkk=0;

for ii=1:500
    srd_s(ii)=srd;
    iii_s(ii)=iii;
    if ii==1
        source_loc=initial_source_loc;
    end
    %if 50 consecutive times, robot run on the same line, it needs to break by random walk
    if size(plot_source_loc,1)>50
        robot_loc=plot_source_loc(end-10:end,1:2); %temporary
        robot_line=0;
        if abs(robot_loc(1,1)-robot_loc(end,1))<=0.001
            for i=1:size(robot_loc,1)
                if abs(robot_loc(1,1)-robot_loc(i,1))<=0.001
                    robot_line=robot_line+1;
                end
            end
        elseif abs(robot_loc(1,2)-robot_loc(end,2))<=0.001
            for i=1:size(robot_loc,1)
                if abs(robot_loc(1,2)-robot_loc(i,2))<=0.001
                    robot_line=robot_line+1;
                end
            end
        else
            for i=2:size(robot_loc,1)-1
                if ((robot_loc(i,2)-robot_loc(i-1,2))/(robot_loc(i,1)-robot_loc(i-1,1)))-((robot_loc(i+1,2)-robot_loc(i,2))/(robot_loc(i+1,1)-robot_loc(i,1)))<=0.001
                    robot_line=robot_line+1;
                end 
            end
        end
        if robot_line>=(size(robot_loc,1)-2)
            source_loc=global_source_loc; %should change to previous location rather than initial one
            angle_rnd=-pi+2*pi*rand;
            source_loc(1)=source_loc(1)+0.5*cos(angle_rnd);
            if source_loc(1)<swd %keep robot far from wall at least srd = 0.5 (assume robot platform has "leg" to avoid robot to crash sensor to the wall)
                source_loc(1)=swd;
            elseif source_loc(1)>room_dim(1)-swd
                source_loc(1)=room_dim(1)-swd;
            end
            source_loc(2)=source_loc(2)+0.5*sin(angle_rnd);
            if source_loc(2)<swd
                source_loc(2)=swd;
            elseif source_loc(2)>room_dim(2)-swd
                source_loc(2)=room_dim(2)-swd;
            end
            result_source_loc=[];
            result_im_source=[];
            result_slope=[];
            result_intercept=[];
            srd=srd_t; 
            iii=1;
        end
    end
    temp_source_loc=source_loc;
    ISs_at_one_stop=[];
    for jj=1:37
        source_loc=temp_source_loc;
        alfa=(jj-1)*pi/18;
        mic_loc = [source_loc(1)-srd*cos(alfa) source_loc(2)-srd*sin(alfa) 2; source_loc(1)+srd*sin(alfa) source_loc(2)-srd*cos(alfa) 2; source_loc(1)+srd*cos(alfa) source_loc(2)+srd*sin(alfa) 2; source_loc(1)-srd*sin(alfa) source_loc(2)+srd*cos(alfa) 2]; 
        h = rir_generator(c, fs, mic_loc, source_loc, room_dim, beta, n, mtype, order, dim, orientation, hp_filter);
        %% set origin at speaker
        mic_loc=mic_loc-repmat(source_loc,4,1);
        global_source_loc=source_loc;
        source_loc=[0,0];
        %% compute direct distances
        clear T_direct
        for i=1:size(mic_loc,1)
            T_direct(i)=pdist([mic_loc(i,1:2);source_loc(1:2)]);
        end
        directDistances=T_direct;
        %% compute image source distances
        clear h_t
        for i=1:size(mic_loc,1)
            h_t(i,:)=h(i,:)/max(abs(h(i,:)));
        end
        %using peakPick.m
        clear maxvec
        thres=0.05; %0.2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para
        for i=1:size(mic_loc,1)
            maxvec{i}=peakPick(abs(h_t(i,:)),thres);
            maxvec{i}(1,:)=[];
        end
        %distance from images to mics
        clear T
        T = cell(size(mic_loc,1), 1);
        for i=1:size(mic_loc,1)
            T{i}=(maxvec{i}(:,1)*c/fs)'; %T{i} is distances from all images to mic i
            T{i}=T{i}(T{i}<10);
        end
        imageDistances=T;
        %compute square distance
        clear imageDistances2
        mic_loc2=mic_loc.^2;
        T_direct2=T_direct.^2;
        for i=1:size(mic_loc,1)
            imageDistances2{i}=T{i}.^2;
        end
        %% find source locs
        %%%%%%%%%%%%%%%%%%%%%%%%%%% find real sound source
        real_source=source_loc;
        %%%%%%%%%%%%%%%%%%%%%%%%% find image sources
        %number of peaks in RIR
        n1=size(imageDistances2{1},2);
        n2=size(imageDistances2{2},2);
        n3=size(imageDistances2{3},2);
        n4=size(imageDistances2{4},2);
        %by mics [1,2,3]
        A=[-2*(mic_loc(1,1)-mic_loc(2,1)),-2*(mic_loc(1,2)-mic_loc(2,2)); ...
            -2*(mic_loc(2,1)-mic_loc(3,1)),-2*(mic_loc(2,2)-mic_loc(3,2))];
        m1=repmat(imageDistances2{1}',1,size(imageDistances2{2},2))-repmat(imageDistances2{2},size(imageDistances2{1},2),1);
        m2=repmat(imageDistances2{2}',1,size(imageDistances2{3},2))-repmat(imageDistances2{3},size(imageDistances2{2},2),1);
        m1=m1'; l1=repmat(m1(:),1,n3)';
        l1=l1(:)-mic_loc2(1,1)+mic_loc2(2,1)-mic_loc2(1,2)+mic_loc2(2,2);
        m2=m2'; l2=repmat(m2(:),1,n1);
        l2=l2(:)-mic_loc2(2,1)+mic_loc2(3,1)-mic_loc2(2,2)+mic_loc2(3,2);
        b=[l1';l2'];
        bb=A\b;
        im_sources_1=bb';
        %by mics [1,2,4]
        A=[-2*(mic_loc(1,1)-mic_loc(2,1)),-2*(mic_loc(1,2)-mic_loc(2,2)); ...
            -2*(mic_loc(2,1)-mic_loc(4,1)),-2*(mic_loc(2,2)-mic_loc(4,2))];
        m1=repmat(imageDistances2{1}',1,size(imageDistances2{2},2))-repmat(imageDistances2{2},size(imageDistances2{1},2),1);
        m2=repmat(imageDistances2{2}',1,size(imageDistances2{4},2))-repmat(imageDistances2{4},size(imageDistances2{2},2),1);
        m1=m1'; l1=repmat(m1(:),1,n4)';
        l1=l1(:)-mic_loc2(1,1)+mic_loc2(2,1)-mic_loc2(1,2)+mic_loc2(2,2);
        m2=m2'; l2=repmat(m2(:),1,n1)';
        l2=l2(:)-mic_loc2(2,1)+mic_loc2(4,1)-mic_loc2(2,2)+mic_loc2(4,2);
        b=[l1';l2'];
        bb=A\b;
        im_sources_2=bb';
        %by mics [1,3,4]
        A=[-2*(mic_loc(1,1)-mic_loc(3,1)),-2*(mic_loc(1,2)-mic_loc(3,2)); ...
            -2*(mic_loc(3,1)-mic_loc(4,1)),-2*(mic_loc(3,2)-mic_loc(4,2))];
        m1=repmat(imageDistances2{1}',1,size(imageDistances2{3},2))-repmat(imageDistances2{3},size(imageDistances2{1},2),1);
        m2=repmat(imageDistances2{3}',1,size(imageDistances2{4},2))-repmat(imageDistances2{4},size(imageDistances2{3},2),1);
        m1=m1'; l1=repmat(m1(:),1,n4)';
        l1=l1(:)-mic_loc2(1,1)+mic_loc2(3,1)-mic_loc2(1,2)+mic_loc2(3,2);
        m2=m2'; l2=repmat(m2(:),1,n1)'; 
        l2=l2(:)-mic_loc2(3,1)+mic_loc2(4,1)-mic_loc2(3,2)+mic_loc2(4,2);
        b=[l1';l2'];
        bb=A\b;
        im_sources_3=bb';
        %% find image sources close to real sources
        N=10000;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para
        if size(im_sources_1,1)<=N
            im_sources_1t=im_sources_1;
        else
            d1=sqrt((real_source(1,1)-im_sources_1(:,1)).^2+(real_source(1,2)-im_sources_1(:,2)).^2);
            [~,ind]=sort(d1);
            im_sources_1t=im_sources_1(ind(1:N),:);
        end
        if size(im_sources_2,1)<=N
            im_sources_2t=im_sources_2;
        else
            d2=sqrt((real_source(1,1)-im_sources_2(:,1)).^2+(real_source(1,2)-im_sources_2(:,2)).^2);
            [~,ind]=sort(d2);
            im_sources_2t=im_sources_2(ind(1:N),:);
        end
        if size(im_sources_3,1)<=N
            im_sources_3t=im_sources_3;
        else
            d3=sqrt((real_source(1,1)-im_sources_3(:,1)).^2+(real_source(1,2)-im_sources_3(:,2)).^2);
            [~,ind]=sort(d3);
            im_sources_3t=im_sources_3(ind(1:N),:);
        end
        %%%%%%%%%%% find common image sources
        im_sources=[];
        for i=1:length(im_sources_1t(:,1))
            d=sqrt((im_sources_1t(i,1)-im_sources_2t(:,1)).^2+(im_sources_1t(i,2)-im_sources_2t(:,2)).^2);
            if min(d)<0.1 %0.1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para
                clear d
                d=sqrt((im_sources_1t(i,1)-im_sources_3t(:,1)).^2+(im_sources_1t(i,2)-im_sources_3t(:,2)).^2);
                if min(d)<0.1 %0.1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para
                    im_sources=[im_sources;im_sources_1t(i,:)];
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%
        if isempty(im_sources)
            continue
        end
        %%%%%%%%%%%%%%%%%%%%%
        %% compute distance from image sources to real sources
        d=sqrt((real_source(1,1)-im_sources(:,1)).^2+(real_source(1,2)-im_sources(:,2)).^2);
        [v,~]=sortrows([d,im_sources],1);
        im_sources=v(:,2:3);
        %% find possible IS
        for k=1:size(mic_loc,1)
            first_order_im_sources=im_sources(1,:);
            mic=mic_loc(k,:);
            for i=1:size(im_sources,1)
                im_source_i=im_sources(i,:);
                A=[real_source(1,1)-im_source_i(1,1),real_source(1,2)-im_source_i(1,2); ...
                    1/(mic(1,1)-im_source_i(1,1)),-1/(mic(1,2)-im_source_i(1,2))];
                b=[0.5*(real_source(1,1)^2-im_source_i(1,1)^2+real_source(1,2)^2-im_source_i(1,2)^2); ...
                    mic(1,1)/(mic(1,1)-im_source_i(1,1))-mic(1,2)/(mic(1,2)-im_source_i(1,2))];
                %find reflective point
                RP=A\b;
                %check whether real source and RP are in the same side of plane
                count=0;
                for j=1:size(first_order_im_sources,1)
                    first_order_im_source_j=first_order_im_sources(j,:);
                    a=[real_source(1,1)-first_order_im_source_j(1,1),real_source(1,2)-first_order_im_source_j(1,2)];
                    check_s=a*(real_source(1:2)')-0.5*(real_source(1,1)^2-first_order_im_source_j(1,1)^2+real_source(1,2)^2-first_order_im_source_j(1,2)^2);
                    check_RP=a*RP-0.5*(real_source(1,1)^2-first_order_im_source_j(1,1)^2+real_source(1,2)^2-first_order_im_source_j(1,2)^2);
                    if check_s*check_RP>0
                        count=count+1;
                    end
                end
                if count==size(first_order_im_sources,1)
                    first_order_im_sources=[first_order_im_sources;im_source_i];
                end
            end
            if k==1
                first_order_im_sources_final=first_order_im_sources;
            else
                first_order_im_sources_final=intersect(first_order_im_sources_final,first_order_im_sources,'rows');
            end
        end
        ISs_at_one_stop=[ISs_at_one_stop;first_order_im_sources_final];
    end
    plot_source_loc=[plot_source_loc;global_source_loc];
    if isempty(ISs_at_one_stop) %when none of IS found in a stop, then robot move random
        source_loc=global_source_loc; %should change to previous location rather than initial one
        angle_rnd=-pi+2*pi*rand;
        source_loc(1)=source_loc(1)+0.5*cos(angle_rnd);
        if source_loc(1)<swd %keep robot far from wall at least srd = 0.5 (assume robot platform has "leg" to avoid robot to crash sensor to the wall)
            source_loc(1)=swd;
        elseif source_loc(1)>room_dim(1)-swd
            source_loc(1)=room_dim(1)-swd;
        end
        source_loc(2)=source_loc(2)+0.5*sin(angle_rnd);
        if source_loc(2)<swd
            source_loc(2)=swd;
        elseif source_loc(2)>room_dim(2)-swd
            source_loc(2)=room_dim(2)-swd;
        end
        result_source_loc=[];
        result_im_source=[];
        result_slope=[];
        result_intercept=[];
        %clear lines_s
        srd=srd_t; 
        iii=1;        
        continue
    end
    ISs_at_one_stop(:,1)=ISs_at_one_stop(:,1)+global_source_loc(1);
    ISs_at_one_stop(:,2)=ISs_at_one_stop(:,2)+global_source_loc(2);
    %ISs{ii}=ISs_at_one_stop;
    %% clustering IS, where IS close to each other are grouped in a cluster
    i=1;
    clear clusters
    while size(ISs_at_one_stop,1)
        d=sqrt((ISs_at_one_stop(1,1)-ISs_at_one_stop(2:end,1)).^2+(ISs_at_one_stop(1,2)-ISs_at_one_stop(2:end,2)).^2);
        tmp=ISs_at_one_stop(2:end,:);
        clusters{i}=[ISs_at_one_stop(1,:);tmp(d<0.1,:)]; %0.1 is assumed max dist among ISs in one group
        tmp(d<0.1,:)=[];
        ISs_at_one_stop=tmp;
        i=i+1;
    end
    %% find biggest cluster, which will be used to decide a path for robot
    idx=0;
    for i=1:size(clusters,2)
        if size(clusters{i},1)>idx
            idx=size(clusters{i},1);
            cluster_for_move=clusters{i};
            del_i=i;
        end
    end
    clusters(del_i)=[];
    %% store unused clusters and possible corresponding lines (walls)
    %store_clusters=[store_clusters clusters];
    if ~isempty(clusters)
        clear store_slope store_intercept
        for i=1:size(clusters,2)
            avg_im_source=mean(clusters{i},1); %average all im_sources in cluster 
            midpoint=[(avg_im_source(1)+global_source_loc(1))/2,(avg_im_source(2)+global_source_loc(2))/2];
            store_slope(i)=-((avg_im_source(1)-global_source_loc(1))/(avg_im_source(2)-global_source_loc(2)));
            store_intercept(i)=midpoint(2)-store_slope(i)*midpoint(1);
        end
        lines_s{ii}=[store_slope' store_intercept']; %lines found in previous steps but not used
    else
        lines_s{ii}=[];
    end
    %% find a path for robot to move (idea is to move robot in parallel with the wall that is wanted to verify if it is a real wall, where robot needs to move at least to 3 stops in parallel paths with that wall)
    avg_im_source=mean(cluster_for_move,1); %average all im_sources in cluster for move to compute 1 perpendicular line
    midpoint=[(avg_im_source(1)+global_source_loc(1))/2,(avg_im_source(2)+global_source_loc(2))/2]; %midpoint between real source and im source
    if (avg_im_source(2)-global_source_loc(2))==0
        global_source_loc(2)=global_source_loc(2)+0.00001; %to avoid Inf in avg_slope
    end
    avg_slope=-((avg_im_source(1)-global_source_loc(1))/(avg_im_source(2)-global_source_loc(2))); %slope of the line (wall)
    intercept_move=global_source_loc(2)-avg_slope*global_source_loc(1); %incept of the path where robot move
    sol=roots([1+avg_slope^2, -2*global_source_loc(1)+2*avg_slope*(intercept_move-global_source_loc(2)), global_source_loc(1)^2+(intercept_move-global_source_loc(2))^2-dist_move^2]); %there are two locations robot should move to
    if iii==1
        if ~exist('p_lines_s')
            i=1;
        elseif exist('p_lines_s')
            for i=1:size(sol,1)
                x_new=real(sol(i)); %check to see which one should be chosen
                y_new=avg_slope*x_new+intercept_move;
                dist_sum(i)=0; %sum of distances from new source loc to all other possible and found walls 
                for jj=1:size(p_lines_s,1)
                    check_old=p_lines_s(jj,1)*global_source_loc(1)+p_lines_s(jj,2)-global_source_loc(2);
                    check_new=p_lines_s(jj,1)*x_new+p_lines_s(jj,2)-y_new;
                    distpl=abs(p_lines_s(jj,1)*x_new+p_lines_s(jj,2)-y_new)/sqrt(p_lines_s(jj,1)^2+1); %distance from point to line
                    if check_old*check_new<0
                        dist_sum(i)=0;
                        break
                    else
                        dist_sum(i)=dist_sum(i)+distpl;
                    end
                end
            end
            if dist_sum(1)>dist_sum(2)
                i=1;
            else
                i=2;
            end
        end
        x_new=real(sol(i)); 
        y_new=avg_slope*x_new+intercept_move;
    else
        if (global_source_loc(1)-result_source_loc(iii-1,1)==0) && (global_source_loc(2)-result_source_loc(iii-1,2)==0)
            source_loc=global_source_loc;
            result_source_loc(iii-1,:)=[];
            result_im_source(iii-1,:)=[];
            result_slope(iii-1)=[];
            result_intercept(iii-1)=[];
            iii=iii-1;
            ccc=ccc+1;
            continue
        end
        x_new=global_source_loc(1)+dist_move*(global_source_loc(1)-result_source_loc(iii-1,1))/sqrt((global_source_loc(1)-result_source_loc(iii-1,1))^2+(global_source_loc(2)-result_source_loc(iii-1,2))^2);
        y_new=global_source_loc(2)+dist_move*(global_source_loc(2)-result_source_loc(iii-1,2))/sqrt((global_source_loc(1)-result_source_loc(iii-1,1))^2+(global_source_loc(2)-result_source_loc(iii-1,2))^2);
    end
    source_loc=[x_new, y_new, 2];
    if source_loc(1)<swd %keep robot far from wall at least srd = 0.5 (assume robot platform has "leg" to avoid robot to crash sensor to the wall)
        source_loc(1)=swd;
    elseif source_loc(1)>room_dim(1)-swd
        source_loc(1)=room_dim(1)-swd;
    end
    if source_loc(2)<swd
        source_loc(2)=swd;
    elseif source_loc(2)>room_dim(2)-swd
        source_loc(2)=room_dim(2)-swd;
    end
    %% result lines
    result_source_loc=[result_source_loc;global_source_loc];
    result_im_source=[result_im_source;avg_im_source];
    result_slope=[result_slope;avg_slope]; %slope of the wall
    result_intercept=[result_intercept;midpoint(2)-avg_slope*midpoint(1)]; %intercept of the wall
    slope_s(ii)=avg_slope;
    inter_s(ii)=midpoint(2)-avg_slope*midpoint(1);
    p_lines_s(ii,:)=[avg_slope midpoint(2)-avg_slope*midpoint(1)]; %potential lines found every move
    if iii>1 %compare if the newly obtained wall close to previous walls (if we find 3 close walls, we stop and then form one real wall)
        if abs(atan(result_slope(iii-1))-atan(avg_slope))<pi/2
            thres_tan=abs(atan(result_slope(iii-1))-atan(avg_slope));
        else
            thres_tan=pi-abs(atan(result_slope(iii-1))-atan(avg_slope));
        end
        %%%%thres_intercept=abs(result_intercept(iii-1)-result_intercept(iii));
        %if thres_tan<0.05, check if two lines close even intercept is
        %different (use (0,0) to check if perpendicular vectors from both lines
        %to (0,0) is same direction or not)
        intercept_check=0;
        O_p_x=(initial_source_loc(1)+result_slope(iii-1)*(initial_source_loc(2)-result_intercept(iii-1)))/(1+(result_slope(iii-1))^2); %x of perpendicular point
        O_p_y=(result_slope(iii-1)*initial_source_loc(1)+(result_slope(iii-1))^2*initial_source_loc(2)+result_intercept(iii-1))/(1+(result_slope(iii-1))^2); %y of perpendicular point
        p_v_1=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line
        
        O_p_x=(initial_source_loc(1)+result_slope(iii)*(initial_source_loc(2)-result_intercept(iii)))/(1+(result_slope(iii))^2); %x of perpendicular point
        O_p_y=(result_slope(iii)*initial_source_loc(1)+(result_slope(iii))^2*initial_source_loc(2)+result_intercept(iii))/(1+(result_slope(iii))^2); %y of perpendicular point
        p_v_2=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

        if (p_v_1(1)*p_v_2(1)+p_v_1(2)*p_v_2(2))/(sqrt(p_v_1*p_v_1')*sqrt(p_v_2*p_v_2'))<0 %cos of angle between two perpendicular vectors
            intercept_check=1;
        else
            distpl1=abs(result_slope(iii-1)*initial_source_loc(1)+result_intercept(iii-1)-initial_source_loc(2))/sqrt(result_slope(iii-1)^2+1); %distance from initial_source_loc to line 1
            distpl2=abs(result_slope(iii)*initial_source_loc(1)+result_intercept(iii)-initial_source_loc(2))/sqrt(result_slope(iii)^2+1); %distance from initial_source_loc to line 1
            if abs(distpl1-distpl2)>0.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para thres_tan (tan of slope)
               intercept_check=1;
            end
        end
        if iii==2
            if (thres_tan>0.05 || intercept_check==1) && srd>0.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para thres_tan (tan of slope)
                source_loc=global_source_loc; %should change to previous location rather than initial one
                srd=srd-0.1; %reduce robot arm (source to receiver)
                result_source_loc(end,:)=[];
                result_im_source(end,:)=[];
                result_slope(end)=[];
                result_intercept(end)=[];
                intercept_check=0;
                iii=1;
                cc=cc+1;
            elseif (thres_tan>0.05 || intercept_check==1) && srd<=0.2 %move to random location
                source_loc=global_source_loc; %should change to previous location rather than initial one
                angle_rnd=-pi+2*pi*rand;
                source_loc(1)=source_loc(1)+0.5*cos(angle_rnd);
                if source_loc(1)<swd %keep robot far from wall at least srd = 0.5 (assume robot platform has "leg" to avoid robot to crash sensor to the wall)
                    source_loc(1)=swd;
                elseif source_loc(1)>room_dim(1)-swd
                    source_loc(1)=room_dim(1)-swd;
                end
                source_loc(2)=source_loc(2)+0.5*sin(angle_rnd);
                if source_loc(2)<swd
                    source_loc(2)=swd;
                elseif source_loc(2)>room_dim(2)-swd
                    source_loc(2)=room_dim(2)-swd;
                end
                result_source_loc(end,:)=[];
                result_im_source(end,:)=[];
                result_slope(end)=[];
                result_intercept(end)=[];
                intercept_check=0;
                srd=srd_t; 
                iii=1;
                ccc=ccc+1;
            end
        end
        if iii==3
            iii=2;
            if (thres_tan>0.05 || intercept_check==1) && srd>0.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para thres_tan (tan of slope)
                source_loc=global_source_loc; %should change to previous location rather than initial one
                srd=srd-0.1; %reduce robot arm (source to receiver)
                result_source_loc(end,:)=[];
                result_im_source(end,:)=[];
                result_slope(end)=[];
                result_intercept(end)=[];
                intercept_check=0;
                kk=kk+1;
                cccc=cccc+1;
            elseif (thres_tan>0.05 || intercept_check==1) && srd<=0.2 %move to random location
                source_loc=global_source_loc; %should change to previous location rather than initial one
                angle_rnd=-pi+2*pi*rand;
                source_loc(1)=source_loc(1)+0.5*cos(angle_rnd);
                if source_loc(1)<swd %keep robot far from wall at least srd = 0.5 (assume robot platform has "leg" to avoid robot to crash sensor to the wall)
                    source_loc(1)=swd;
                elseif source_loc(1)>room_dim(1)-swd
                    source_loc(1)=room_dim(1)-swd;
                end
                source_loc(2)=source_loc(2)+0.5*sin(angle_rnd);
                if source_loc(2)<swd
                    source_loc(2)=swd;
                elseif source_loc(2)>room_dim(2)-swd
                    source_loc(2)=room_dim(2)-swd;
                end
                result_source_loc(end,:)=[];
                result_im_source(end,:)=[];
                result_slope(end)=[];
                result_intercept(end)=[];
                intercept_check=0;
                srd=srd_t; 
                kkk=1;
            end
        end
    end
    iii=iii+1;
    %% whenever one line found
    if size(result_slope,1)==3 %check and save found wall line here
        srd=srd_t;
%         break
        found_lines(line_idx,:)=[mean(result_slope) mean(result_intercept)];
%         r_s{line_idx}=result_slope;
%         r_i{line_idx}=result_intercept;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %check just found line with previous found lines to remove overlap
        %lines
        for i=1:size(found_lines,1)-1
            if abs(atan(found_lines(line_idx,1))-atan(found_lines(i,1)))<pi/2
                compare_slope=abs(atan(found_lines(line_idx,1))-atan(found_lines(i,1)));
            else
                compare_slope=pi-abs(atan(found_lines(line_idx,1))-atan(found_lines(i,1)));
            end
            %compare_intercept=abs(found_lines(line_idx,2)-found_lines(i,2));
            if (compare_slope<0.05) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%set parameters
                O_p_x=(initial_source_loc(1)+found_lines(i,1)*(initial_source_loc(2)-found_lines(i,2)))/(1+(found_lines(i,1))^2); %x of perpendicular point
                O_p_y=(found_lines(i,1)*initial_source_loc(1)+(found_lines(i,1))^2*initial_source_loc(2)+found_lines(i,2))/(1+(found_lines(i,1))^2); %y of perpendicular point
                p_v_1=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

                O_p_x=(initial_source_loc(1)+found_lines(end,1)*(initial_source_loc(2)-found_lines(end,2)))/(1+(found_lines(end,1))^2); %x of perpendicular point
                O_p_y=(found_lines(end,1)*initial_source_loc(1)+(found_lines(end,1))^2*initial_source_loc(2)+found_lines(end,2))/(1+(found_lines(end,1))^2); %y of perpendicular point
                p_v_2=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

                if (p_v_1(1)*p_v_2(1)+p_v_1(2)*p_v_2(2))/(sqrt(p_v_1*p_v_1')*sqrt(p_v_2*p_v_2'))<0.98 %cos of angle between two perpendicular vectors, if this angle > 11 deg, then two lines are not one
                    continue
                else
                    distpl1=abs(found_lines(i,1)*initial_source_loc(1)+found_lines(i,2)-initial_source_loc(2))/sqrt(found_lines(i,1)^2+1); %distance from initial_source_loc to line 1
                    distpl2=abs(found_lines(end,1)*initial_source_loc(1)+found_lines(end,2)-initial_source_loc(2))/sqrt(found_lines(end,1)^2+1); %distance from initial_source_loc to line 1
                    if abs(distpl1-distpl2)>0.1
                        continue
                    else
                        found_lines(line_idx,:)=[]; %This line was already found!
                        line_idx=line_idx-1;
                        break
                    end
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%check if we find enough lines and form a room
        %%find intersections
        clear intersection_x intersection_y
        if exist('found_lines')
            if size(found_lines,1)>2
                for i=1:size(found_lines,1)
                    for j=1:size(found_lines,1)
                        intersection_x(i,j)=(found_lines(j,2)-found_lines(i,2))/(found_lines(i,1)-found_lines(j,1));
                        intersection_y(i,j)=found_lines(j,1)*intersection_x(i,j)+found_lines(j,2);
                    end
                end
                intersection_points=[intersection_x(:) intersection_y(:)];
                intersection_points(isnan(intersection_points(:,1)),:)=[]; %remove NaN 
                intersection_points(isnan(intersection_points(:,2)),:)=[]; %remove NaN 
                %assume that the room is not too big, dimensions are unknown but
                %limited by a number, e.g. 100m
                intersection_points(abs(intersection_points(:,1))>100,:)=[]; %when dimension>100m
                intersection_points(abs(intersection_points(:,2))>100,:)=[]; %when dimension>100m
                %remove overlapped or close points (not done
                for i=1:size(intersection_points,1)-1
                    for j=i+1:size(intersection_points,1)
                        if pdist([intersection_points(i,:);intersection_points(j,:)])<0.1
                            intersection_points(j,:)=[Inf Inf];
                        end
                    end
                end
                intersection_points(abs(intersection_points(:,1))>100,:)=[]; %when dimension>100m
                intersection_points(abs(intersection_points(:,2))>100,:)=[]; %when dimension>100m
                %check if a found line has two intersection points
                if ~isempty(intersection_points)
                    points_on_found_lines=0;
                    for i=1:size(found_lines,1)
                        clear distpl_c
                        for j=1:size(intersection_points,1)
                            distpl_c(j)=abs(found_lines(i,1)*intersection_points(j,1)+found_lines(i,2)-intersection_points(j,2))/sqrt(found_lines(i,1)^2+1); %distance from an intersection point to a found line
                        end
                        if numel(find(distpl_c<0.01))==2 %make sure there are two intersection points on a found line
                            points_on_found_lines=points_on_found_lines+1;
                        end
                    end
                    if points_on_found_lines==size(found_lines,1)
                        break
                    end
                end
           end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %if no room found, keep finding line
        line_idx=line_idx+1;
        result_source_loc=[];
        result_im_source=[];
        result_slope=[];
        result_intercept=[];
        iii=1;
        kk=0;
        %compare lines confirmed and lines found previous step but not used
        %yet
        if isempty(lines_s{end})
            continue
        else
            clear c_idx
            for i=1:size(clusters,2)
                c_idx(i)=size(clusters{i},1);
            end
            [~,sort_c_idx]=sort(c_idx,'descend'); %rank clusters, then use biggest cluster first, corresponding to lines_s
%             lines_s_t=lines_s; %temporary
%             lines_s_t{end+1}=found_lines; %add found lines in for checking
            for no_lines_s=1:size(lines_s{end},1)
                break_no_lines_s=0;
                for no_line=1:size(found_lines,1)
                    if abs(atan(found_lines(no_line,1))-atan(lines_s{end}(sort_c_idx(no_lines_s),1)))<pi/2
                        compare_slope=abs(atan(found_lines(no_line,1))-atan(lines_s{end}(sort_c_idx(no_lines_s),1)));
                    else
                        compare_slope=pi-abs(atan(found_lines(no_line,1))-atan(lines_s{end}(sort_c_idx(no_lines_s),1)));
                    end
                    if (compare_slope<0.05) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%set parameters
                        O_p_x=(initial_source_loc(1)+found_lines(no_line,1)*(initial_source_loc(2)-found_lines(no_line,2)))/(1+(found_lines(no_line,1))^2); %x of perpendicular point
                        O_p_y=(found_lines(no_line,1)*initial_source_loc(1)+(found_lines(no_line,1))^2*initial_source_loc(2)+found_lines(no_line,2))/(1+(found_lines(no_line,1))^2); %y of perpendicular point
                        p_v_1=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

                        O_p_x=(initial_source_loc(1)+lines_s{end}(sort_c_idx(no_lines_s),1)*(initial_source_loc(2)-lines_s{end}(sort_c_idx(no_lines_s),2)))/(1+(lines_s{end}(sort_c_idx(no_lines_s),1))^2); %x of perpendicular point
                        O_p_y=(lines_s{end}(sort_c_idx(no_lines_s),1)*initial_source_loc(1)+(lines_s{end}(sort_c_idx(no_lines_s),1))^2*initial_source_loc(2)+lines_s{end}(sort_c_idx(no_lines_s),2))/(1+(lines_s{end}(sort_c_idx(no_lines_s),1))^2); %y of perpendicular point
                        p_v_2=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

                        if (p_v_1(1)*p_v_2(1)+p_v_1(2)*p_v_2(2))/(sqrt(p_v_1*p_v_1')*sqrt(p_v_2*p_v_2'))>0.98 %cos of angle between two perpendicular vectors, if this angle > 11 deg, then two lines are not one
                            distpl1=abs(found_lines(no_line,1)*initial_source_loc(1)+found_lines(no_line,2)-initial_source_loc(2))/sqrt(found_lines(no_line,1)^2+1); %distance from initial_source_loc to line 1
                            distpl2=abs(lines_s{end}(sort_c_idx(no_lines_s),1)*initial_source_loc(1)+lines_s{end}(sort_c_idx(no_lines_s),2)-initial_source_loc(2))/sqrt(lines_s{end}(sort_c_idx(no_lines_s),1)^2+1); %distance from initial_source_loc to line 1
                            if abs(distpl1-distpl2)<0.1
                                disp('This line was already found!') %This line was already found!
                                break
                            end
                        end
                    end
                    if no_line==size(found_lines,1) %if a line in lines_s{end} has not been confirmed yet, can be used for next step
                        cluster_for_move=clusters{sort_c_idx(no_lines_s)};%using other clusters in the immediately previous step to find path for robot
                        avg_im_source=mean(cluster_for_move,1); %average all im_sources in cluster for move to compute 1 perpendicular line
                        midpoint=[(avg_im_source(1)+global_source_loc(1))/2,(avg_im_source(2)+global_source_loc(2))/2]; %midpoint between real source and im source
                        if (avg_im_source(2)-global_source_loc(2))==0
                            global_source_loc(2)=global_source_loc(2)+0.00001; %to avoid Inf in avg_slope
                        end
                        avg_slope=-((avg_im_source(1)-global_source_loc(1))/(avg_im_source(2)-global_source_loc(2))); %slope of the line (wall)
                        intercept_move=global_source_loc(2)-avg_slope*global_source_loc(1); %incept of the path where robot move
                        sol=roots([1+avg_slope^2, -2*global_source_loc(1)+2*avg_slope*(intercept_move-global_source_loc(2)), global_source_loc(1)^2+(intercept_move-global_source_loc(2))^2-dist_move^2]); %there are two locations robot should move to
                        for i=1:size(sol,1)
                            x_new=real(sol(i)); %check to see which one should be chosen
                            y_new=avg_slope*x_new+intercept_move;
                            dist_sum(i)=0; %sum of distances from new source loc to all other possible and found walls 
                            for j=1:size(found_lines,1)
                                check_old=found_lines(j,1)*global_source_loc(1)+found_lines(j,2)-global_source_loc(2);
                                check_new=found_lines(j,1)*x_new+found_lines(j,2)-y_new;
                                distpl=abs(found_lines(j,1)*x_new+found_lines(j,2)-y_new)/sqrt(found_lines(j,1)^2+1); %distance from point to line
                                if check_old*check_new<0
                                    dist_sum(i)=0;
                                    break
                                else
                                    dist_sum(i)=dist_sum(i)+distpl;
                                end
                            end
                        end
                        if dist_sum(1)>dist_sum(2)
                            i=1;
                        else
                            i=2;
                        end
                        x_new=real(sol(i)); 
                        y_new=avg_slope*x_new+intercept_move;
                        source_loc=[x_new, y_new, 2];
                        if source_loc(1)<swd %keep robot far from wall at least srd = 0.5 (assume robot platform has "leg" to avoid robot to crash sensor to the wall)
                            source_loc(1)=swd;
                        elseif source_loc(1)>room_dim(1)-swd
                            source_loc(1)=room_dim(1)-swd;
                        end
                        if source_loc(2)<swd
                            source_loc(2)=swd;
                        elseif source_loc(2)>room_dim(2)-swd
                            source_loc(2)=room_dim(2)-swd;
                        end
                        result_source_loc=[result_source_loc;global_source_loc];
                        result_im_source=[result_im_source;avg_im_source];
                        result_slope=[result_slope;avg_slope]; %slope of the wall
                        result_intercept=[result_intercept;midpoint(2)-avg_slope*midpoint(1)]; %intercept of the wall
                        break_no_lines_s=1;
                        iii=2;
                    end
                end
                if break_no_lines_s==1
                    break
                end
            end
        end
    end
    %% 
    if pdist([source_loc(1:2);global_source_loc(1:2)])~=0 && iii==3 && kk==0
        srd=0.5;
        kkkk=kkkk+1;
    end
    clc
end

figure; plot(plot_source_loc(:,1),plot_source_loc(:,2),'*')
hold on
line(plot_source_loc(:,1),plot_source_loc(:,2))
axis([0 6 0 5])
title('How the robot moves to reconstruct the room')

figure; plot(intersection_points(:,1),intersection_points(:,2),'*')
title('Four corners of the room')

%plot_source_loc=[plot_source_loc srd_s' iii_s' slope_s' inter_s'];