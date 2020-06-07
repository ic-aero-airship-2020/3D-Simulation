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
    
    properties(Nontunable, Logical)
        showLocalLidar = false;      % Show local lidar
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
        localsesorfig;
        localsensorax;
        sensorDataHandle;
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
            
            
            if obj.showLocalLidar
                % Create figure
                FigureName = 'Sensor Visualization';
                FigureTag = 'SensorVisualization';
                existingFigures = findobj('type','figure','tag',FigureTag);
                if ~isempty(existingFigures)
                    obj.localsesorfig = figure(existingFigures(1)); % bring figure to the front
                    clf;
                else
                    obj.localsesorfig = figure('Name',FigureName,'tag',FigureTag);
                end
                obj.localsensorax = axes('parent',obj.localsesorfig);   
                hold(obj.localsensorax,'on');

                % Plotting center
                obj.sensorDataHandle = image(obj.localsensorax,uint8(zeros(3,3)));
                cmap = [0 0 0; 1 1 1];
                colormap(cmap)
                
                % Final setup
                title(obj.localsensorax,'Sensor Visualization (Front this direction, upwards)');
                hold(obj.localsensorax,'off'); 
            end
        end

        function stepImpl(obj,posetrans,poserot,sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,isActive1,isActive2,isActive3,isActive4,isActive5,isActive6)
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
    
            offset1 =   120 /180*pi;
            offset2 = - 120 /180*pi;
            offset3 =   0 /180*pi;
            offset4 = - 120  /180*pi;
            offset5 =   0   /180*pi;
            offset6 =   120 /180*pi;
            
            pose1 = [x,y,theta+offset1];
            pose2 = [x,y,theta+offset2];
            pose3 = [x,y,theta+offset3];
            pose4 = [x,y,theta+offset4];
            pose5 = [x,y,theta+offset5];
            pose6 = [x,y,theta+offset6];
            
            function addScans(map,pose,sensor,maxrange,isActive)
                % Helper funcion to add scan to the map
                xidx = 3;
                yidx = 1;

                if isActive
                    scan = lidarScan([-sensor(xidx),sensor(yidx)]);
                    insertRay(map,pose,scan,maxrange);
                    
                end
            end
            
            addScans(obj.map,pose1,sensor1,maxrange,isActive1);
            addScans(obj.map,pose2,sensor2,maxrange,isActive2);
            addScans(obj.map,pose3,sensor3,maxrange,isActive3);
            addScans(obj.map,pose4,sensor4,maxrange,isActive4);
            addScans(obj.map,pose5,sensor5,maxrange,isActive5);
            addScans(obj.map,pose6,sensor6,maxrange,isActive6);
            
            show(obj.map,'Parent',obj.ax,'FastUpdate',1)
            % Update the figure
            drawnow('limitrate')
            
            if obj.showLocalLidar
                data = [isActive4,isActive3,isActive6;
                        0,0,0;
                        isActive1,isActive5,isActive2];
                set(obj.sensorDataHandle,'cdata',uint8(data));
            end
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
