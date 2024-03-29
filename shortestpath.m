function [path] = shortestpath(map, start, goal)

% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal) 
%
%   [PATH] = DIJKSTRA(...) returns the path 

    % Initializes necessary variables and get information about the map
    nodenumber = map.nodenumber; % Total no. of nodes
    leftbound = map.boundary(1, 1:2);
    blockflag = map.blockflag;
    resolution = map.resolution; % Map resolution
    xy_res = resolution(1);
    m = map.segment; % Divide x dimension into mx parts and divide y dimension into my parts
    mx = m(1);
    my = m(2);
    [row, ~] = size(nodenumber);
    

    % assign cell numbers to start and goal
    sourcecell = worldtocell(leftbound,resolution,start);
    sourcecellnum = celltonumber(m,sourcecell);

    goalcell = worldtocell(leftbound,resolution,goal);
    goalcellnum = celltonumber(m,goalcell);

    % pre establish variables
    visited = blockflag; % all at 0 except blocks are marked as visited
    distance = [];
    parent = [];
    for j = 1:row
        distance(j) = Inf;
        parent(j) = 0;
    end

    distance(sourcecellnum) = 0; % distance of start node is 0
    %while visited(goalcellnum) == 0
    for i = 1:(row-1)
        temp = [];
        for k = 1:row
            if visited(k) == 0 % if unvisited
                temp = [temp distance(k)];
            else
                temp = [temp Inf];
            end
        end
    %while visited(goalcellnum) == 0
        [t,u] = min(temp);
        visited(u) = 1;

        for v = 1:row % for v = u + 1, u - 1, u + mx, u - mx
            if visited(v) == 0
                if v == u+1 && mod(u,mx) ~= mx-1 %right
                    newDistance = distance(u) + 1;
                    distance(v) = newDistance;
                    parent(v) = u;
                end
                if v == u-1 && mod(u,mx) ~= 0%left
                    newDistance = distance(u) + 1;
                    distance(v) = newDistance;
                    parent(v) = u;
                end
                if v == u+mx && u < row - mx %up
                    newDistance = distance(u) + 1;
                    distance(v) = newDistance;
                    parent(v) = u;
                end
                if v == u-mx && u > mx %down
                    newDistance = distance(u) + 1;
                    distance(v) = newDistance;
                    parent(v) = u;
                end
                temp(v) = distance(v);
            end
        end
    end

    cellpath = [];
    if parent(goalcellnum) ~= 0
        t = goalcellnum;
        cellpath = [goalcellnum];
        while t ~= sourcecellnum
            p = parent(t);
            cellpath = [p cellpath];
            t = p;
        end
    end

    cellpath_coord = [];
    for j = 1:length(cellpath)
        cellpath_coord = numbertocell(map,cellpath(j));
        coord = celltoworld(leftbound,resolution,cellpath_coord);
        path(j,1) = coord(1);
        path(j,2) = coord(2);
        path(j,3) = j;
    end



end
