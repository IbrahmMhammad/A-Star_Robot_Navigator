
function robot_simulation
    % Define the grid size and the obstacles
    gridSize = [10, 10];
    obstacleCoords = [2, 2; 2, 3; 2, 4]; % Obstacles defined as x, y
    startCoord = [1, 1]; % Start defined as x, y
    goalCoord = [8, 9]; % Goal defined as x, y
    
    % Convert the obstacle coordinates to linear indices
    obstacles = sub2ind(gridSize, obstacleCoords(:,2), obstacleCoords(:,1));
    
    % Perform A* search
    [path, numExpanded] = astar(gridSize, obstacles, startCoord, goalCoord);
    
    % Visualize the results
    map = zeros(gridSize);
    map(obstacles) = 1; % Mark obstacles
    pathInd = sub2ind(gridSize, path(:,2), path(:,1));
    map(pathInd) = 2; % Mark the path
    
    imagesc(map);
    colormap([1 1 1; 0 0 0; 1 0 0]); % White for empty, Black for obstacles, Red for path
    title(sprintf('Path found with A* (Nodes expanded: %d)', numExpanded));
end

function [path, numExpanded] = astar(gridSize, obstacles, startCoord, goalCoord)
    % A* path planning
    
    % Convert coordinates to linear indices
    startInd = sub2ind(gridSize, startCoord(2), startCoord(1));
    goalInd = sub2ind(gridSize, goalCoord(2), goalCoord(1));
    
    % Initialize open and closed lists
    openList = startInd;
    cameFrom = zeros(1, prod(gridSize)); % Track the path
    gScore = inf(1, prod(gridSize));
    gScore(startInd) = 0;
    fScore = inf(1, prod(gridSize));
    fScore(startInd) = heuristic(startCoord, goalCoord);
    
    numExpanded = 0;
    
    while ~isempty(openList)
        % Find the node in the open list with the lowest f score
        [~, currentIndex] = min(fScore(openList));
        currentInd = openList(currentIndex);
        
        % Check if we've reached the goal
        if currentInd == goalInd
            path = reconstruct_path(cameFrom, currentInd, gridSize);
            return;
        end
        
        % Move current node from open to closed list
        openList(currentIndex) = [];
        numExpanded = numExpanded + 1;
        
        % Get neighbors
        neighbors = get_neighbors(currentInd, gridSize, obstacles);
        
        for neighborInd = neighbors
            % Tentative g score
            tentativeGScore = gScore(currentInd) + 1;
            
            if tentativeGScore < gScore(neighborInd)
                % This path to neighbor is better than any previous one
                cameFrom(neighborInd) = currentInd;
                gScore(neighborInd) = tentativeGScore;
                
                % Correct handling of ind2sub for heuristic calculation
                [neighborRow, neighborCol] = ind2sub(gridSize, neighborInd);
                neighborCoord = [neighborCol, neighborRow];
                
                fScore(neighborInd) = gScore(neighborInd) + heuristic(neighborCoord, goalCoord);
                
                if ~ismember(neighborInd, openList)
                    openList = [openList neighborInd]; % Add to open list
                end
            end
        end
    end
    
    % If we get here, no path was found
    path = [];
    numExpanded = -1;
end

function h = heuristic(nodeCoord, goalCoord)
    % Manhattan distance
    h = abs(nodeCoord(1) - goalCoord(1)) + abs(nodeCoord(2) - goalCoord(2));
end

function path = reconstruct_path(cameFrom, currentInd, gridSize)
    % Reconstruct the path from goal to start
    path = [];
    while currentInd ~= 0
        [i, j] = ind2sub(gridSize, currentInd);
        path = [j, i; path]; % Prepend to path with correct order
        currentInd = cameFrom(currentInd);
    end
end

function neighbors = get_neighbors(ind, gridSize, obstacles)
    % Get valid neighbors for the current node
    [i, j] = ind2sub(gridSize, ind);
    neighborOffsets = [-1, 0; 1, 0; 0, -1; 0, 1]; % 4-connected grid
    neighbors = [];
    for k = 1:size(neighborOffsets, 1)
        ni = i + neighborOffsets(k, 1);
        nj = j + neighborOffsets(k, 2);
        if ni > 0 && nj > 0 && ni <= gridSize(1) && nj <= gridSize(2)
            neighborInd = sub2ind(gridSize, ni, nj);
            if ~ismember(neighborInd, obstacles)
                neighbors = [neighbors, neighborInd];
            end
        end
    end
end
