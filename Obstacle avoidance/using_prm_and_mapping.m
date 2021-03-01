load exampleMaps.mat
%Creating a map
%refMap = binaryOccupancyMap(simpleMap,1);

refMap = binaryOccupancyMap(30,30,30);
x = [5; 11; 13; 14; 15];
y = [5; 11; 13; 20; 11];
% add occupancy in map with objects/robots with known positions/poses
setOccupancy(refMap, [x y], ones(5,1))
%increase size of the objects by changing radius
inflate(refMap, 0.5)
refFigure = figure('Name','SimpleMap');
show(refMap);

%create empty map with same dimensions
%[mapdimx,mapdimy] = size(simpleMap);
map = binaryOccupancyMap(30,30,30);
inflate(map, 0.5)
mapFigure = figure('Name','Unknown Map');
show(map);

diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);

%simulating sensor
sensor = rangeSensor;
sensor.Range = [0,10];

%path = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
%path plannng using PRM and path finder
path=planning([4 6],[25 20], refMap);
figure(refFigure);
hold on
plot(path(:,1),path(:,2), 'o-');
hold off

controller.Waypoints = path;

initPose = [path(1,1) path(1,2), pi/2];
goal = [path(end,1) path(end,2)]';
poses(:,1) = initPose';

exampleHelperDiffDriveCtrl(diffDrive,controller,initPose,goal,refMap,map,refFigure,mapFigure,sensor)

function waypoints = planning(startPos, endPos, mapMatrix)

persistent waypointsInternal
persistent prevStartPos
persistent prevEndPos

% Set constant parameters
maxNumNodes = 100;

% Initialize persistent variables that are used downstream
if isempty(prevStartPos)
    prevStartPos = startPos;
end

if isempty(prevEndPos)
    prevEndPos = endPos;
end

isNewPath = (~isequal(prevStartPos, startPos) || ~isequal(prevEndPos, endPos));

if isempty(waypointsInternal) || isNewPath
    % Two conditions in which waypoints must be set: they are not yet
    % initialized, or they are initialized, but the current start and end
    % do not match those used to generate the previous set of waypoints
    [waypointsInternal] = planPath(mapMatrix, maxNumNodes, startPos, endPos);
end

% Update the outputs
waypoints = waypointsInternal;

% Update the persistent variables that indicate previous state
prevStartPos = startPos;
prevEndPos = endPos;

%% Helper Function

function wayPoints = planPath(mapMatrix, maxNodes, startPos, endPos)
    
    % Create a probabilistic roadmap
    map = binaryOccupancyMap(mapMatrix);
    prm = mobileRobotPRM(map, maxNodes);
    show(prm)
    % Find a path between two points
    xy = findpath(prm, startPos, endPos);
    wayPoints = repmat(endPos, maxNodes, 1);
    
    % Number of nofindpathdes that are actually used
    numNodes = length(xy);
    wayPoints(1:numNodes,:) = xy;    

end

end