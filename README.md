Pathfinding Algorithms in Unity
----------------------------------

<!-- Make sure to remove all comments -->

This is a mini-project in collaboration with Khaoula Ait Soussi for CSC 4301: Intoduction to Artifical Intelligence.

## Overview
In this project, we adapted the code written by Sebastian Lague (https://github.com/SebLague/Pathfinding) by adding the following items:
- a different A* implementation using a different heuristic
- implementations of the following pathfinding algorithms: Depth First Search (DFS), Breadth First Search (BFS), and Uniform Cost Search (UCS).
At the level of Unity, we also experimented with different configurations of the positions of the seeker and the target in a way that would allow for a richer discussion of the trade-offs among these 5 pathfinding implementations.

In this personal report, I summarize the strengths and weaknesses that I noticed for each algorithm with respect to particular configurations. As measures of performance, we chose:
- the time elapsed for each algorithm in milliseconds as an accurate measure for time especially with larger grids
- the number of expanded nodes (i.e., nodes whose neighbors are visited) as another measure of time efficiency
- the maximum fringe (data structure containing nodes we might visit in the future) size reached at each algorithm as a measure of memory usage

A note on memory profiling: due to time constraints, we were not able to find an easy way to profile the memory usage of a specific function in C#. Unity provides some memory profiling utilities but they would likely consider the grid as well, which could preclude us from seeing accurate results. Since the fringe is the only factor that scales memory usage, then its maximum size is sure to give a tangible comparison of the algorithms' space complexities.

## Discussion

### Grid setup

#### Dimensions
To better differentiate between the algorithms' performances, we chose a large enough grid size of 200x200. All results shown below were ran with these dimensions.

#### Obstacles
Throughout the experiments, we did not change the obstacles, rather we changed the positions of the seeker and target nodes. Note: seeker is in white and target is in red in the screenshots.

### Algorithms

#### A*
The A* implementation by Sebastian Lague that we chose is from Episode 4 (https://github.com/SebLague/Pathfinding/tree/master/Episode%2004%20-%20heap), where a heap implementation of the priority queue is used to optimize the time required to get the least cost neighbor.

#### Alternative A*
A* may be the best pathfinding algorithm we've seen, but "your algorithm is only as good as your heuristic". We tweaked the heuristic for neighbor selection to reflect this principle. Pretty much any other heuristic we could propose would either decrease performance or leave it unchanged. We redefined a node's f-cost for this purpose such that:
```
fCost-alt(node) = min{ hCost(node), gCost(node) }
```
What does this mean in practice? Remember the A* implementation uses a min heap, so a lower fCost means a higher priority. Here, the highest priority neighbor could either have a small hCost (if hCost(node) is the minimum) or a small gCost. The former case is great: small hCost means we're striving closer to the target. The latter is bad: steering away from the target and going backwards towards the start. Hence, the effect of this alternative heuristic is an arbitrary switching between going in the right and wrong directions, leading to a deteriorated time and space complexity. Completeness, however, is still guaranteed, because the algorithm only terminates when the target is reached (even if it takes you tons of time and effort to get there).

#### DFS
We implemented an iterative DFS that uses a stack as the fringe. A standard recursive implementation would use excessively too much memory on large grids, and hence the benchmarking/performance comparison would not be accurate or fair.  
Below, there is a brief discussion on the effect of the order of neighbor enumeration and the relative positions of seeker and target on the performance of DFS in our project.

#### BFS
Of course, BFS works like a flood, expanding equally in all directions until the target is reached. We can reasonably expect BFS to perform best when the path between the seeker and target is unobstructed. We will take a closer look at this assumption <!-- --> below.

#### UCS
As per the project requirements, our UCS implementation assumes the same cost in all directions (horizontal, vertical, and diagonal moves all cost 10). Before implementing uniform cost in all directions, we noticed UCS would generally take vertical or horizontal moves. After introducing this change, we noticed a greater tendency to go in diagonals, which is likely to increase performance especially if seeker and target are at a diagonal from each other.  
Small note on implementation: Because UCS uses a priority queue as a fringe, we re-used the heap class provided by Sebastian Lague; but because each node is accompanied with a priority, we defined a class `NodePriority`.

### Performance per configuration
<!-- Talk about time and space complexities here and try to map that to measures, if possible -->
The characteristics and results of each configuration are detailed in tables below. You can find corresponding screenshots for each configuration in /Plane Configurations. Here, I only include the most relevant screenshots.
#### Configuration 1

| Config 1 | Time | Expanded | Max Fringe |
|----------|------|----------|------------|
| A*       | 5    | 94       | 194        |
| Alt A*   | 36   | 1298     | 182        |
| DFS      | 11   | 7645     | 21632      |
| BFS      | 13   | 6462     | 581        |
| UCS      | 35   | 6545     | 757        |

This configuration (and the next 2) positions seeker and target unobstructed from each other, showing the default behavior of each algorithm.  
A* is the most efficient by all measures. It is difficult to order the other algorithms because of the differing metrics.  
Again, seeker is white, target is red.  
- DFS may be the second fastest, but it consumes the most memory. It plans to possibly explore more than half the grid!

<img width="1440" alt="DFS1" src="https://user-images.githubusercontent.com/58636465/153768875-317bdd53-a7df-43f7-b36d-15054d1788e7.png">

- We would expect expanded nodes to be closely correlated with time elapsed, but interestingly Alternative A* takes more time than DFS and BFS in spite of expanding far less nodes. Along the same vein, Alternative A* and UCS take about the same time to execute, but Alternative A* expands about a fifth of the nodes expanded by UCS.

#### Configuration 2

| Config 2 | Time | Expanded | Max Fringe |
|----------|------|----------|------------|
| A*       | 0    | 15       | 64         |
| Alt A*   | 11   | 205      | 77         |
| DFS      | 1    | 15       | 92         |
| BFS      | 6    | 588      | 271        |
| UCS      | 6    | 555      | 348        |

Again, A* performs the best on all fronts. Beyond that, this configuration (and the next) highlights the relationship between the performance and DFS and the order in which `Grid.GetNeighbors()` gets the current node's neigbors. DFS performs better than the 3 other algorithms and is even comparable to A*, because it starts by exploring the neighbor in the most direct direction (bottom left). It just so happens that this neighbor was the best to start with because target is broadly at the bottom left of seeker.  
<img width="1440" alt="DFS2" src="https://user-images.githubusercontent.com/58636465/153768893-3dd807aa-bdc2-49ae-8303-54a7c9b970a1.png">

This relationship between DFS performance and the neighbor it expands first can be leveraged to optimize the algorithm. From the coordinates of seeker and target, we can determine the best neighbor to start expanding and thus slightly modify the logic of `Grid.GetNeighbors()`. However, this optimization is obviously not guaranteed to find the target more efficiently -- in fact it might even be detrimental if there were obstacles.  

#### Configuration 3

| Config 3 | Time | Expanded | Max Fringe |
|----------|------|----------|------------|
| A*       | 1    | 15       | 64         |
| Alt A*   | 10   | 186      | 81         |
| DFS      | 8    | 5079     | 14543      |
| BFS      | 4    | 565      | 328        |
| UCS      | 35   | 650      | 402        |

In this configuration, it is almost as if the relative positions of seeker and target from configuration 2 is inverted. DFS performs rather poorly (largest fringe and greatest number of expanded nodes).  
Notice that UCS (in yellow) allows itself to go diagonally because we arranged uniform cost in all directions.  
<img width="1440" alt="All3" src="https://user-images.githubusercontent.com/58636465/153771252-2449bde9-0bdf-4951-96aa-6a5333fb57f6.png">

#### Configuration 4

| Config 4 | Time | Expanded | Max Fringe |
|----------|------|----------|------------|
| A*       | 59   | 1632     | 273        |
| Alt A*   | 84   | 3305     | 253        |
| DFS      | 9    | 7388     | 20962      |
| BFS      | 16   | 7464     | 536        |
| UCS      | 38   | 7480     | 611        |

<img width="1440" alt="All4" src="https://user-images.githubusercontent.com/58636465/153771305-c93331ef-4802-4917-a5b4-f1c9482a3f96.png">


This configuration puts 3 obstacles in the shortest path between seeker and target. Here, we notice something suprising: A* takes noticeably more time to execute than the other algorithms. Even upon multiple re-runs, the result is always that A* takes about 50-something ms, much more than we've seen before. At the same time, A* expands less nodes than DFS, BFS, and UCS. This is surprising because (a) A* should always be the fastest, given a good heuristic, and (b) expanded nodes should be a good measure of time complexity.  
My best guess is that with more obstacles between seeker and target, there is greater overhead associated with maintaining the heap in A*. This overhead may decrease with larger input, where we may see the performance of the other algorithms decrease further.

### Conclusion
Based on these remarks, I conclude that A* is generally the most efficient pathfinding algorithm unless the path between seeker and target is significantly obstructed. The Alternative A* that we chose performs consistently poorly as expected due to the inconsistency of the alternative heuristic. The performance of DFS, among other things, is related to the order in which the neighbors are listed, such that if the algorithm starts out going in depth in the wrong direction, it will end up creating an unnecessarily long path. BFS is generally consistent and depends largely on the absolute distance between seeker and target because it operates like a flood and is bound to expand way more nodes than needed anyway.

