    a = input("Insert x image source ");
    b = input("Insert y image source ");
    IS = [a b];
    previousSource = newSource;
    sourcePlot = [sourcePlot ; newSource];
    %% Find Possible Stops
    midPoint=[(IS(1)+newSource(1))/2,(IS(2)+newSource(2))/2];
    if (IS(2)-previousSource(2))==0
        previousSource(2)=previousSource(2)+0.00001; %to avoid Inf in slope
    end
    slope=-((IS(1)-previousSource(1))/(IS(2)-previousSource(2)));
    interceptMove=previousSource(2)-slope*previousSource(1);
    %two possible next stop
    sol=roots([1+slope^2, -2*previousSource(1)+2*slope*(interceptMove-previousSource(2)), previousSource(1)^2+(interceptMove-previousSource(2))^2-dist_move^2]);
    %% decide which next stop to take
    if stops==1
        if isempty('potentialLines')
            i=1;
        elseif ~isempty('potentialLines')
            for i=1:size(sol,1)
                x_new=real(sol(i)); %check to see which one should be chosen
                y_new=slope*x_new+interceptMove;
                dist_sum(i)=0; %sum of distances from new source loc to all other possible and found walls 
                for jj=1:size(potentialLines,1)
                    check_old=potentialLines(jj,1)*previousSource(1)+potentialLines(jj,2)-previousSource(2);
                    check_new=potentialLines(jj,1)*x_new+potentialLines(jj,2)-y_new;
                    distpl=abs(potentialLines(jj,1)*x_new+potentialLines(jj,2)-y_new)/sqrt(potentialLines(jj,1)^2+1); %distance from point to line
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
        y_new=slope*x_new+interceptMove;
        else
            if (previousSource(1)-resultSource(stops-1,1)==0) && (previousSource(2)-resultSource(stops-1,2)==0)
                newSource=previousSource;
                resultSource(stops-1,:)=[];
                resultIS(stops-1,:)=[];
                resultSlope(stops-1)=[];
                resultIntercept(stops-1)=[];
                stops=stops-1;
            end
            x_new=previousSource(1)+dist_move*(previousSource(1)-resultSource(stops-1,1))/sqrt((previousSource(1)-resultSource(stops-1,1))^2+(previousSource(2)-resultSource(stops-1,2))^2);
            y_new=previousSource(2)+dist_move*(previousSource(2)-resultSource(stops-1,2))/sqrt((previousSource(1)-resultSource(stops-1,1))^2+(previousSource(2)-resultSource(stops-1,2))^2);
    end
    newSource=[x_new, y_new];
    %% Check the validity of the new walls
    resultSource=[resultSource;previousSource];
    resultIS=[resultIS;IS];
    resultSlope=[resultSlope;slope]; %slope of the wall
    resultIntercept=[resultIntercept;midPoint(2)-slope*midPoint(1)]; %intercept of the wall
    potentialLines =[slope midPoint(2)-slope*midPoint(1)]; %potential lines found every move
    if stops>1 %compare if the newly obtained wall close to previous walls (if we find 3 close walls, we stop and then form one real wall)
        if abs(atan(resultSlope(stops-1))-atan(slope))<pi/2
            thres_tan=abs(atan(resultSlope(stops-1))-atan(slope));
        else
            thres_tan=pi-abs(atan(resultSlope(stops-1))-atan(slope));
        end
        %%%%thres_intercept=abs(resultIntercept(stops-1)-resultIntercept(stops));
        %if thres_tan<0.05, check if two lines close even intercept is
        %different (use (0,0) to check if perpendicular vectors from both lines
        %to (0,0) is same direction or not)
        intercept_check=0;
        O_p_x=(initialLoc(1)+resultSlope(stops-1)*(initialLoc(2)-resultIntercept(stops-1)))/(1+(resultSlope(stops-1))^2); %x of perpendicular point
        O_p_y=(resultSlope(stops-1)*initialLoc(1)+(resultSlope(stops-1))^2*initialLoc(2)+resultIntercept(stops-1))/(1+(resultSlope(stops-1))^2); %y of perpendicular point
        p_v_1=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

        O_p_x=(initialLoc(1)+resultSlope(stops)*(initialLoc(2)-resultIntercept(stops)))/(1+(resultSlope(stops))^2); %x of perpendicular point
        O_p_y=(resultSlope(stops)*initialLoc(1)+(resultSlope(stops))^2*initialLoc(2)+resultIntercept(stops))/(1+(resultSlope(stops))^2); %y of perpendicular point
        p_v_2=[O_p_x,O_p_y]; %perpendicular vector from (0,0) to the line

        if (p_v_1(1)*p_v_2(1)+p_v_1(2)*p_v_2(2))/(sqrt(p_v_1*p_v_1')*sqrt(p_v_2*p_v_2'))<0 %cos of angle between two perpendicular vectors
            intercept_check=1;
        else
            distpl1=abs(resultSlope(stops-1)*initialLoc(1)+resultIntercept(stops-1)-initialLoc(2))/sqrt(resultSlope(stops-1)^2+1); %distance from initialLocto line 1
            distpl2=abs(resultSlope(stops)*initialLoc(1)+resultIntercept(stops)-initialLoc(2))/sqrt(resultSlope(stops)^2+1); %distance from initialLocto line 1
            if abs(distpl1-distpl2)>0.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para thres_tan (tan of slope)
               intercept_check=1;
            end
        end
        if stops==2
            if (thres_tan>0.05 || intercept_check==1) && srd>0.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para thres_tan (tan of slope)
                disp('Need a good image source for second stop');
                newSource=previousSource; %should change to previous location rather than initial one
                srd=srd-0.1; %reduce robot arm (source to receiver)
                resultSource(end,:)=[];
                resultIS(end,:)=[];
                resultSlope(end)=[];
                resultIntercept(end)=[];
                intercept_check=0;
                stops=1;
            elseif (thres_tan>0.05 || intercept_check==1) && srd<=0.3 %move to random location
                disp('Cannot find a good image source at the second stop, everything restarts');
                newSource=previousSource; %should change to previous location rather than initial one
                angle_rnd=-pi+2*pi*rand;
                newSource(1)=newSource(1)+0.5*cos(angle_rnd);
                newSource(2)=newSource(2)+0.5*sin(angle_rnd);
                resultSource=[];
                resultIS=[];
                resultSlope=[];
                resultIntercept=[];
                potentialLines = [];
                intercept_check=0;
                srd = 0.5; 
                stops=0;
            end
        end
        if stops==3
            stops=2;            
            if (thres_tan>0.05 || intercept_check==1) && srd>0.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% set para thres_tan (tan of slope)
                newSource=previousSource; %should change to previous location rather than initial one
                srd=srd-0.1; %reduce robot arm (source to receiver)
                disp('Need a good image source at this last stop');
                resultSource(end,:)=[];
                resultIS(end,:)=[];
                resultSlope(end)=[];
                resultIntercept(end)=[];
                intercept_check=0;
            elseif (thres_tan>0.05 || intercept_check==1) && srd<=0.3 %move to random location
                newSource=previousSource; %should change to previous location rather than initial one
                angle_rnd=-pi+2*pi*rand;
                disp('Cannot find a correct image source, it will restart');
                newSource(1)=newSource(1)+0.5*cos(angle_rnd);           
                newSource(2)=newSource(2)+0.5*sin(angle_rnd);           
                resultSource=[];
                resultIS=[];
                resultSlope=[];
                resultIntercept=[];
                potentialLines = [];
                intercept_check=0;
                srd=0.5;
                stops=0;
            end
        end
    end
    stops=stops+1;
    %% Check if any wall can be confirmed
    if size(resultSlope,1)==3
        foundLines = [foundLines ; mean(resultSlope) mean(resultIntercept)];
        disp('A real wall is formed');
        try
            resultSource=[];
            resultIS=[];
            resultSlope=[];
            resultIntercept=[];
            potentialLines = foundLines;
            stops = 1;
            angle_rnd=-pi+2*pi*rand;
            disp(angle_rnd);
            newSource(1)=newSource(1)+0.5*cos(angle_rnd);           
            newSource(2)=newSource(2)+0.5*sin(angle_rnd);            
        catch
            disp('Cannot clear the container');
        end
    end
    if size(foundLines,1)>2
        for i=1:size(foundLines,1)
            for j=1:size(foundLines,1)
                intersection_x(i,j)=(foundLines(j,2)-foundLines(i,2))/(foundLines(i,1)-foundLines(j,1));
                intersection_y(i,j)=foundLines(j,1)*intersection_x(i,j)+foundLines(j,2);
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
            points_on_foundLines=0;
            for i=1:size(foundLines,1)
                clear distpl_c
                for j=1:size(intersection_points,1)
                    distpl_c(j)=abs(foundLines(i,1)*intersection_points(j,1)+foundLines(i,2)-intersection_points(j,2))/sqrt(foundLines(i,1)^2+1); %distance from an intersection point to a found line
                end
                if numel(find(distpl_c<0.01))==2 %make sure there are two intersection points on a found line
                    points_on_foundLines=points_on_foundLines+1;
                end
            end
            if points_on_foundLines==size(foundLines,1)
                disp('Show something');
            end
        end
    end