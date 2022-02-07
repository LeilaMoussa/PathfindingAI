using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;


public class Pathfinding : MonoBehaviour {

    public Transform seeker, target;
    
    Grid grid;

    void Awake() {
        grid = GetComponent<Grid>();
    }

    void Update() {
		BFS(seeker.position,target.position);
		DFS(seeker.position,target.position);
        AStar(seeker.position,target.position);      
        AStar(seeker.position,target.position, "1");  //This calls AStar with an additional argument to run the alternative heuristic function
        UCS(seeker.position,target.position);
    }

    void AStar(Vector3 startPos, Vector3 targetPos, String algoIndex = "0") {
    	Stopwatch stopwatch = new Stopwatch();
    	stopwatch.Start();

        Environment.SetEnvironmentVariable("AStarAlt", algoIndex);

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0) {
            Node currentNode = openSet.RemoveFirst();
            closedSet.Add(currentNode);
            if (currentNode == targetNode) {
            	stopwatch.Stop();
            	if(algoIndex.Equals("0")){
                	print("AStar Time Elapsed: " + stopwatch.ElapsedMilliseconds);
                    RetracePath(startNode,targetNode, 1);
                }
                else{
                	print("AStar alternative Time Elapsed: " + stopwatch.ElapsedMilliseconds);
                    RetracePath(startNode,targetNode, 2);
                }
                return;
            }

            foreach (Node neighbour in grid.GetNeighbours(currentNode)) {
                if (!neighbour.walkable || closedSet.Contains(neighbour)) {
                    continue;
                }

                int newMovementCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
                if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
                    neighbour.gCost = newMovementCostToNeighbour;
                    neighbour.hCost = GetDistance(neighbour, targetNode);
                    neighbour.parent = currentNode;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                    else {
                        openSet.UpdateItem(neighbour);
                    }
                }
            }
        }
        
    }

    void DFS(Vector3 startPos, Vector3 targetPos) {
        // This is an iterative DFS. A recursive one would require too much memory
        // on a large grid. It's only fair to compare the algorithms based on their 
        // best performance.

        Stopwatch stopwatch = new Stopwatch();
    	stopwatch.Start();

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        
        HashSet<Node> visited = new HashSet<Node>();
            
        Stack<Node> stack = new Stack<Node>();
        stack.Push(startNode);

        while (stack.Count > 0) {
            Node currentNode = stack.Pop();
            
            if (currentNode == targetNode) {
            	stopwatch.Stop();
            	print("DFS Time Elapsed: " + stopwatch.ElapsedMilliseconds);
                RetracePath(startNode, targetNode, 3);
                return;
            }

            if (visited.Contains(currentNode))
                continue;

            visited.Add(currentNode);

            foreach(Node neighbour in grid.GetNeighbours(currentNode)){
                if (!neighbour.walkable || visited.Contains(neighbour)) {
                    continue;
                }
                if (!visited.Contains(neighbour)) {
                    neighbour.parent = currentNode;
                    stack.Push(neighbour);
                }
            }
        }
    }

    void BFS(Vector3 startPos, Vector3 targetPos) {
    	Stopwatch stopwatch = new Stopwatch();
    	stopwatch.Start();
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        
        
        HashSet<Node> visited = new HashSet<Node>();
        
        Queue<Node> queue = new Queue<Node>();
        queue.Enqueue(startNode);

        while (queue.Count > 0) {
            Node currentNode = queue.Dequeue();
            
            if (currentNode == targetNode) {
            	stopwatch.Stop();
            	print("BFS Time Elapsed: " + stopwatch.ElapsedMilliseconds);
                RetracePath(startNode,targetNode, 4);
                return;
            }

            if (visited.Contains(currentNode))
                continue;

            visited.Add(currentNode);

            foreach(Node neighbour in grid.GetNeighbours(currentNode)){
                if (!neighbour.walkable) {
                    continue;
                }
                if (!visited.Contains(neighbour)) {
                   neighbour.parent = currentNode;
                   queue.Enqueue(neighbour);
                }
            }
        }
    }

    void UCS(Vector3 startPos, Vector3 targetPos) {
    	Stopwatch stopwatch = new Stopwatch();
    	stopwatch.Start();
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        Heap<NodePriority> queue = new Heap<NodePriority>(grid.MaxSize);

        NodePriority start = new NodePriority(startNode, 0);
     
        queue.Add(start);
     
        HashSet<Node> visited = new HashSet<Node>();
     
        while (queue.Count > 0) {
            NodePriority p = queue.RemoveFirst();

            if (p.node == targetNode) {
            	stopwatch.Stop();
            	print("UCS Time Elapsed: " + stopwatch.ElapsedMilliseconds);
                RetracePath(startNode, targetNode, 5);
                return;
            }

			if (visited.Contains(p.node))
                continue;

            visited.Add(p.node);

            foreach (Node neighbour in grid.GetNeighbours(p.node)) {
                if (!neighbour.walkable || visited.Contains(neighbour)) {
                    continue;
                }
                NodePriority n = new NodePriority(neighbour, (p.priority + GetDistance(p.node, neighbour)));
                queue.Add(n);
                neighbour.parent = p.node;
            }
        }
    }

    void RetracePath(Node startNode, Node endNode, int algoIndex) {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode) {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        if(algoIndex == 1){
            grid.AStarpath = path;
        }
        else if(algoIndex == 2){
            grid.AStarAltpath = path;
        }
        else if(algoIndex == 3){
            grid.DFSpath = path;
        }
        else if(algoIndex == 4){
            grid.BFSpath = path;
        }
        else{
            grid.UCSpath = path;
        }
    }

    int GetDistance(Node nodeA, Node nodeB) {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14*dstY + 10* (dstX-dstY);
        return 14*dstX + 10 * (dstY-dstX);
    }
}