% Caricamento del robot
robot = loadrobot("robotisOpenManipulator");

% Visualizzazione del robot
figure;
axes = show(robot);
axes.CameraPositionMode = 'auto';

% Definizione cubo e scatola
cubeSize = 0.03; 
boxSize = 0.07;

% Definizione punto iniziale e finale traiettoria
startPoint = [0.2, -0.2, 0.02 + cubeSize/2];
endPoint = [0.2, 0.2, 0.02 + cubeSize/2];

% Waypoints per la traiettoria
wayPoints = [startPoint; 
             0.2, -0.2, 0.1;   
             0.2, 0, 0.15;     
             0.2, 0.2, 0.1;    
             endPoint];       
hold on

% Tracciamento della traiettoria
plot3(wayPoints(:, 1), wayPoints(:, 2), wayPoints(:, 3), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
trajectory = cscvn(wayPoints');
numWaypoints = 100;
pointsOnTrajectory = fnplt(trajectory, numWaypoints);
plot3(pointsOnTrajectory(1,:), pointsOnTrajectory(2,:), pointsOnTrajectory(3,:), 'r', 'LineWidth', 2);

% Tappeto sul piano XY
carpetWidth = cubeSize * 1.5; 
carpetLength = 0.3; 
carpetHeight = 0.02; 
carpetPosition = [startPoint(1), startPoint(2), carpetHeight];

carpetVertices = [
    carpetPosition(1) - carpetLength/2, carpetPosition(2) - carpetWidth/2, carpetPosition(3);
    carpetPosition(1) + carpetLength/2, carpetPosition(2) - carpetWidth/2, carpetPosition(3);
    carpetPosition(1) + carpetLength/2, carpetPosition(2) + carpetWidth/2, carpetPosition(3);
    carpetPosition(1) - carpetLength/2, carpetPosition(2) + carpetWidth/2, carpetPosition(3);
];

carpetFaces = [
    1, 2, 3, 4
];

carpetColor = [0, 0, 0];

patch('Vertices', carpetVertices, 'Faces', carpetFaces, 'FaceColor', carpetColor, 'EdgeColor', 'none');

% Definizione del muro
wallWidth = 0.1;
wallHeight = 0.1; 
wallDepth = 0.02;
wallPosition = [0.2, 0, wallHeight/2]; 

wallVertices = [
    wallPosition(1) - wallWidth/2, wallPosition(2) - wallDepth/2, wallPosition(3) - wallHeight/2;
    wallPosition(1) + wallWidth/2, wallPosition(2) - wallDepth/2, wallPosition(3) - wallHeight/2;
    wallPosition(1) + wallWidth/2, wallPosition(2) + wallDepth/2, wallPosition(3) - wallHeight/2;
    wallPosition(1) - wallWidth/2, wallPosition(2) + wallDepth/2, wallPosition(3) - wallHeight/2;
    wallPosition(1) - wallWidth/2, wallPosition(2) - wallDepth/2, wallPosition(3) + wallHeight/2;
    wallPosition(1) + wallWidth/2, wallPosition(2) - wallDepth/2, wallPosition(3) + wallHeight/2;
    wallPosition(1) + wallWidth/2, wallPosition(2) + wallDepth/2, wallPosition(3) + wallHeight/2;
    wallPosition(1) - wallWidth/2, wallPosition(2) + wallDepth/2, wallPosition(3) + wallHeight/2;
];

wallFaces = [
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8;
    1, 2, 3, 4;
    5, 6, 7, 8
];

wallColor = [0.2, 0.2, 0.8];

patch('Vertices', wallVertices, 'Faces', wallFaces, 'FaceColor', wallColor, 'EdgeColor', 'none');

% Scatola nel punto finale
boxPosition = endPoint;
boxVertices = [
    boxPosition(1) - boxSize/2, boxPosition(2) - boxSize/2, boxPosition(3) - boxSize/2;
    boxPosition(1) + boxSize/2, boxPosition(2) - boxSize/2, boxPosition(3) - boxSize/2;
    boxPosition(1) + boxSize/2, boxPosition(2) + boxSize/2, boxPosition(3) - boxSize/2;
    boxPosition(1) - boxSize/2, boxPosition(2) + boxSize/2, boxPosition(3) - boxSize/2;
    boxPosition(1) - boxSize/2, boxPosition(2) - boxSize/2, boxPosition(3) + boxSize/2;
    boxPosition(1) + boxSize/2, boxPosition(2) - boxSize/2, boxPosition(3) + boxSize/2;
    boxPosition(1) + boxSize/2, boxPosition(2) + boxSize/2, boxPosition(3) + boxSize/2;
    boxPosition(1) - boxSize/2, boxPosition(2) + boxSize/2, boxPosition(3) + boxSize/2;
];

boxFaces = [
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8;
    1, 2, 3, 4;
    5, 6, 7, 8
];

boxColor = [0.8, 0.2, 0.2]; % Colore rosso
boxAlpha = 0.5; % Valore di trasparenza

patch('Vertices', boxVertices, 'Faces', boxFaces, 'FaceColor', boxColor, 'EdgeColor', 'none', 'FaceAlpha', boxAlpha);


% Visualizzazione risultato
title('Visualizzazione Braccio in Movimento')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);

% Cinematica inversa
eeOffset = 0.12;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint, trvec2tform([eeOffset, 0, 0]));
addBody(robot, eeBody, 'link5');

% Creazione dell'oggetto per la cinematica inversa
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0.1, 0.1, 0, 1, 1, 1];

numTotalPoints = 30;
eePositions = ppval(trajectory, linspace(0, trajectory.breaks(end), numTotalPoints));

% Movimento del braccio
initialGuess = homeConfiguration(robot);
for idx = 1:size(eePositions, 2)
    tform = trvec2tform(eePositions(:, idx)');
    configSoln = ik('end_effector', tform, weights, initialGuess);
    show(robot, configSoln, 'PreservePlot', false, 'Frames', 'off');
    cubePosition = eePositions(:, idx)' - [0, 0, cubeSize/2];
    drawCube(cubePosition, cubeSize);
    checkCollision = checkCollisionWithWall(robot, configSoln, wallVertices);
    if checkCollision
        disp("Collisione con il muro, stop.");
        break;
    end
    pause(0.1);
    initialGuess = configSoln;
end

hold off;

function drawCube(position, size)
cubeVertices = [
position + [-size/2, -size/2, 0];
position + [size/2, -size/2, 0];
position + [size/2, size/2, 0];
position + [-size/2, size/2, 0];
position + [-size/2, -size/2, size];
position + [size/2, -size/2, size];
position + [size/2, size/2, size];
position + [-size/2, size/2, size]
];

cubeFaces = [
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8;
    1, 2, 3, 4;
    5, 6, 7, 8
];

cubeColors = [
    0.8, 0.8, 0.8;
    0.8, 0.8, 0.8;
    0.8, 0.8, 0.8;
    0.8, 0.8, 0.8;
    0.8, 0.8, 0.8;
    0.8, 0.8, 0.8
];

patch('Vertices', cubeVertices, 'Faces', cubeFaces, 'FaceVertexCData', cubeColors, 'FaceColor', 'flat', 'EdgeColor', 'k', 'FaceAlpha', 0.5);

end

function collision = checkCollisionWithWall(robot, configSoln, wallVertices)
eePose = getTransform(robot, configSoln, 'end_effector');
eePosition = eePose(1:3, 4);
collision = inpolygon(eePosition(1), eePosition(2), wallVertices(:, 1), wallVertices(:, 2));
end
