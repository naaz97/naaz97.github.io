---
title: Robotics:Computational Motion Planning:Study Notes - Week 1
tags: 
- Robotics
- Motion Planning
---

### Grassfire Algrithm

### Dijkstra Algorithm

### A* Algorithm
### Quiz
1. If you use the Grassfire or breadth first search procedure to plan a path through a grid from a node A to a node B, then you use the same procedure to plan a path from node B to node A, will the two paths have the same length?\
**Yes**\
2.If you use the Grassfire or breadth first search procedure to plan a path through a grid from a node A to a node B, then you use the same procedure to plan a path from node B to node A, are the two paths guaranteed to be the same except in opposite directions?\
**No**\
3.If you use the grassfire algorithm to plan a path through a series of grids with increasing dimension, 2 dimensional, 3 dimensional, 4 dimensional etc. The amount of computational effort required increases
with the dimension of the problem.\
**exponentially**\
4.Generally speaking, which procedure would take less time to find a solution to a typical path planning problem on a discrete grid or graph?\
**A***


### Assignment: Dijkstra Algorithm
```matlab
% ********************************************************************* 
% YOUR CODE BETWEEN THESE LINES OF STARS

% Visit each neighbor of the current node and update the map, distances
% and parent tables appropriately.
offsets = [-1 0; 1 0; 0 -1; 0 1]; % Left, Right, Top, Bottom

for k = 1:size(offsets, 1)
    neighbor = [i, j] + offsets(k,:);
    if any(neighbor < 1) || any(neighbor > [nrows, ncols])
        continue;
    end
    nextNode = sub2ind(size(map), neighbor(1), neighbor(2));
    if ((map(nextNode) ~= 2 && map(nextNode) ~= 3 && map(nextNode) ~= 5 && min_dist + 1 < distanceFromStart(nextNode)))
        map(nextNode) = 4;
        distanceFromStart(nextNode) = min_dist + 1;
        parent(nextNode) = current;
    end
end
numExpanded = numExpanded + 1;
%*********************************************************************
```


### Assignment: Dijkstra Algorithm
```matlab
% *********************************************************************
    % ALL YOUR CODE BETWEEN THESE LINES OF STARS
    % Visit all of the neighbors around the current node and update the
    % entries in the map, f, g and parent arrays
    %
    % Visit each neighbor of the current node and update the map, distances, and parent tables appropriately.
    offsets = [-1 0; 1 0; 0 -1; 0 1]; % Left, Right, Top, Bottom
    for k = 1:size(offsets, 1)
        neighbor = [i, j] + offsets(k,:);
        if any(neighbor < 1) || any(neighbor > [nrows, ncols])
            continue;
        end
        nextNode = sub2ind(size(map), neighbor(1), neighbor(2));
        if map(nextNode) ~= 2 && map(nextNode) ~= 3 && map(nextNode) ~= 5
            tentative_g = g(current) + 1;
            if tentative_g < g(nextNode)
                parent(nextNode) = current;
                g(nextNode) = tentative_g;
                f(nextNode) = tentative_g + H(nextNode);
                map(nextNode) = 4; % mark neighbor as on list
            end
        end
    end
    numExpanded = numExpanded + 1;
```