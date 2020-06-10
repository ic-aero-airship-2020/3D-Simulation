classdef Visualise2DMap < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties(Nontunable)
        robotRadius = 1.5;          % Airship radius [m]
        mapName = '';               % Map Name
    end
    
    properties(Nontunable, Logical)
        showLocalLidar = false;     % Show local lidar
        showFlownPath = false;      % Show flown path
    end
    properties
        minTurningRadius = 0.5;     % Minimum turning radius [m]
        validationDistance = 0.05;  % Validation distance [m]
    end
    properties(Nontunable)
        mapXDim = 100;              % Map dimesions x direction
        mapYDim = 100;              % Map dimesions y direction
        mapResolution = 10;         % Map resolution
    end
    
    % Pre-computed constants
    properties(Access = private)
        map;                % Occupancy grid
        fig;                % Figure window
        ax;                 % Axes for plotting
        RobotHandle;        % Handle to robot body marker or circle
        OrientationHandle;  % Handle to robot orientation line
        flownPathHandle;
        flownPositions;
        plannedPathHandle;
        endPointHandle;
        localsesorfig;
        localsensorax;
        sensorDataHandle;
        xoffset;
        yoffset;
        stateValidator;
        ss;
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
            initx = 0+obj.xoffset;
            inity = 0+obj.yoffset;
            
            obj.OrientationHandle = plot(obj.ax,initx,inity,'r','LineWidth',1.5);
            if obj.robotRadius > 0
                % Finite size robot
                [x,y] = circlePoints(initx,inity,obj.robotRadius,17);
                obj.RobotHandle = plot(obj.ax,x,y,'b','LineWidth',1.5);
            else
                % Point robot
                obj.RobotHandle = plot(obj.ax,initx,inity,'bo', ...
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
                        
            % Setting up path planner plotting
            hold(obj.ax,'on');
            obj.flownPathHandle = plot(obj.ax,initx,inity,'g-');
            obj.endPointHandle = plot(obj.ax,initx,inity,'mx','LineWidth',1.5);
            obj.plannedPathHandle = plot(obj.ax,initx,inity,'r-');
            obj.flownPositions = [initx,inity];

            % Defining state space of the vehicle
            bounds = [[0 obj.mapXDim]; [0 obj.mapYDim]; [-pi pi]];
            obj.ss = stateSpaceDubins(bounds);
            obj.ss.MinTurningRadius = obj.minTurningRadius;

            % map is inflated to account for the width of the airship
            obj.stateValidator = validatorOccupancyMap(obj.ss); 
            % Defining validation distance
            obj.stateValidator.ValidationDistance = obj.validationDistance; % m 

        end

        function nextPose = stepImpl(obj,posetrans,poserot,sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,isActive1,isActive2,isActive3,isActive4,isActive5,isActive6,goalPose)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            x = posetrans(1)+obj.xoffset;
            y = -1*posetrans(3)+obj.yoffset;
            theta = poserot(2);
            position = [x;y;theta];
            
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/40;
            if obj.robotRadius > 0
                % Finite radius case
                [xc,yc] = circlePoints(x,y,obj.robotRadius,17);
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
            
            if obj.showFlownPath
                obj.flownPositions = [obj.flownPositions;[x,y]];
                set(obj.flownPathHandle,'xdata',obj.flownPositions(:,1),'ydata',obj.flownPositions(:,2));
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
            
            if sum(isnan(goalPose)) == 0
                if obj.robotRadius > 0
                    mapInflated = copy(obj.map);
                    inflate(mapInflated, obj.robotRadius);
                    obj.stateValidator.Map = mapInflated;
                else
                    obj.stateValidator.Map = obj.map;
                end
                % Defining validation distance
                obj.stateValidator.ValidationDistance =  obj.validationDistance; % m 
                planner = plannerRRTStar(obj.ss, obj.stateValidator);
                planner.MaxConnectionDistance = 2.0;
                planner.MaxIterations = 30000;

                planner.GoalReachedFcn = @CheckIfGoalReached;
                [pthObj, solnInfo] = plan(planner, position', goalPose');
                
                if ~solnInfo.IsPathFound
                    disp('E Stop Implemented');
                    nextX = 0;
                    nextY = 0;
                    nextPsi = 1.57;
                else
                    nextX = pthObj.States(2,1);
                    nextY = pthObj.States(2,2);
                    nextPsi = pthObj.States(2,3);
                    set(obj.plannedPathHandle,'xdata',pthObj.States(:,1),'ydata',pthObj.States(:,2));
                end
                
                
            else
                nextX = nan;
                nextY = nan;
                nextPsi = nan;
            end
            nextPose = [nextX,nextY,nextPsi];
        end
        
        % More methods needed for the Simulink block to inherit its output
        % sizes from the scan angle parameter provided.
        function sz = getOutputSizeImpl(~)
            sz = [1,3];
        end
        
        function fx = isOutputFixedSizeImpl(~)
           fx = false;
        end
        
        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function cp = isOutputComplexImpl(~)
            cp = false;
        end
        
        
%         function flag = isInactivePropertyImpl(obj,prop)
%              flag = false;
%              switch prop
%                  case 'endGoal'
%                      flag = ~obj.planFlightPath;
%              end
%         end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    
    
%     methods (Access = public)
%         % Attaches all properties associated with a LidarSensor object
%         function attachFlightPath(obj,flightPath)
%             obj.planFlightPath = true;
%             obj.path = flightPath.endGoal;
%             
%         end
%         
%     end
    
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
