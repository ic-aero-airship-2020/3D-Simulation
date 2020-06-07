classdef Visualise2DMap < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties(Nontunable)
        robotRadius = 1.5;    % Robot radius [m]
        mapName = '';       % Map
    end
    
    properties(Nontunable)
        mapXDim = 100;
        mapYDim = 100;
        mapResolution = 10;
    end

    % Pre-computed constants
    properties(Access = private)
        map;                % Occupancy grid
        fig;                % Figure window
        ax;                 % Axes for plotting
        RobotHandle;        % Handle to robot body marker or circle
        OrientationHandle;  % Handle to robot orientation line
        xoffset;
        yoffset;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            % Create figure
            FigureName = 'Robot Visualization';
            FigureTag = 'RobotVisualization';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag);
            end
            obj.ax = axes('parent',obj.fig);   
            hold(obj.ax,'on');
            
            % creating empty binaryoccupancy map
            obj.map = binaryOccupancyMap(obj.mapXDim,obj.mapYDim,obj.mapResolution);
            if ~isempty(obj.map)
                show(obj.map,'Parent',obj.ax);
            end
            
            % Offset the robot to start in the middle
            obj.xoffset = obj.mapXDim/2;
            obj.yoffset = obj.mapYDim/2;
            
            % Initialize robot plot
            obj.OrientationHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);
            if obj.robotRadius > 0
                % Finite size robot
                [x,y] = internal.circlePoints(0,0,obj.robotRadius,17);
                obj.RobotHandle = plot(obj.ax,x+obj.xoffset,y+obj.yoffset,'b','LineWidth',1.5);
            else
                % Point robot
                obj.RobotHandle = plot(obj.ax,0+obj.xoffset,0+obj.yoffset,'bo', ...
                    'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
            end
            
            
            % Final setup
            title(obj.ax,'Robot Visualization');
            hold(obj.ax,'off'); 
            axis equal  
        end

        function stepImpl(obj,posetrans,poserot,sensor1,sensor2,sensor3,sensor4,sensor5,sensor6)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            x = posetrans(1)+obj.xoffset;
            y = -1*posetrans(3)+obj.yoffset;
            theta = poserot(2);
            
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/40;
            if obj.robotRadius > 0
                % Finite radius case
                [xc,yc] = internal.circlePoints(x,y,obj.robotRadius,17);
                set(obj.RobotHandle,'xdata',xc,'ydata',yc);
                len = max(lineLength,2*obj.robotRadius); % Plot orientation based on radius unless it's too small
                xp = [x, x+(len*cos(theta))];
                yp = [y, y+(len*sin(theta))];
                set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            else
                % Point robot case
                xp = [x, x+(lineLength*cos(theta))];
                yp = [y, y+(lineLength*sin(theta))];
                set(obj.RobotHandle,'xdata',x,'ydata',y);
                set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            end
            
            
            % Update scan map
            maxrange = 2.0;
    
            offset1 =   60  /180*pi;
            offset2 = - 60  /180*pi;
            offset3 =   180 /180*pi;
            offset4 =   120 /180*pi;
            offset5 =   0   /180*pi;
            offset6 = - 120 /180*pi;
            
            pose1 = [x,y,theta+offset1];
            pose2 = [x,y,theta+offset2];
            pose3 = [x,y,theta+offset3];
            pose4 = [x,y,theta+offset4];
            pose5 = [x,y,theta+offset5];
            pose6 = [x,y,theta+offset6];
            
            function addScans(map,pose,sensor,maxrange)
                % Helper funcion to add scan to the map
                xidx = 1;
                yidx = 3;

                if size(sensor,1)>0
                    xavg = mean(sensor(:,xidx));
                    yavg = mean(sensor(:,yidx));
                    scan = lidarScan([xavg,yavg]);
%                     scan = lidarScan([sensor(:,xidx),sensor(:,yidx)]);
                    insertRay(map,pose,scan,maxrange);
                end
            end
            
%             addScans(obj.map,pose1,sensor1,maxrange);
%             addScans(obj.map,pose1,sensor2,maxrange);
%             addScans(obj.map,pose1,sensor3,maxrange);
%             addScans(obj.map,pose1,sensor4,maxrange);
            addScans(obj.map,pose1,sensor5,maxrange);
%             addScans(obj.map,pose1,sensor6,maxrange);
            
            
            show(obj.map,'Parent',obj.ax,'FastUpdate',1)
            % Update the figure
            drawnow('limitrate')
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    
    
    methods (Static, Access = protected)
        % Do not show "Simulate using" option
        function flag = showSimulateUsingImpl
            flag = false;
        end
        % Always run in interpreted mode
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end
    end
end
