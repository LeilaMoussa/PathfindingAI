using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class Pathfinding : MonoBehaviour {

    public Transform seeker, target;
    
    Grid grid;

    void Awake() {
        grid = GetComponent<Grid>();
    }

    void Update() {
        AStar(seeker.position,target.position);      
        //AStar(seeker.position,target.position, 1);  //This calls AStar with an additional argument to run the alternative heuristic function
        DFS(seeker.position,target.position);
        BFS(seeker.position,target.position);
        //UCS(seeker.position,target.position);
    }

    void AStar(Vector3 startPos, Vector3 targetPos) {

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0) {
            Node currentNode = openSet.RemoveFirst();
            closedSet.Add(currentNode);
            if (currentNode == targetNode) {
                RetracePath(startNode,targetNode, 1);
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
                        //openSet.UpdateItem(neighbour);
                    }
                }
            }
        }
    }

    void DFS(Vector3 startPos, Vector3 targetPos) {
        // This is an iterative DFS. A recursive one would require too much memory
        // on a large grid. It's only fair to compare the algorithms based on their 
        // best performance.
        
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        
        HashSet<Node> visited = new HashSet<Node>();
            
        Stack<Node> stack = new Stack<Node>();
        stack.Push(startNode);

        while (stack.Count > 0) {
            Node currentNode = stack.Pop();
            
            if (currentNode == targetNode) {
                RetracePath(startNode, targetNode, 3);
                return;
            }

            if (visited.Contains(currentNode))
                continue;

            visited.Add(currentNode);

            foreach(Node neighbor in grid.GetNeighbours(currentNode)){
                if (!neighbor.walkable || visited.Contains(neighbor)) {
                    continue;
                }
                if (!visited.Contains(neighbor)) {
                    neighbor.parent = currentNode;
                    stack.Push(neighbor);
                }
            }
        }
    }

    void BFS(Vector3 startPos, Vector3 targetPos) {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        
        HashSet<Node> visited = new HashSet<Node>();
        
        Queue<Node> queue = new Queue<Node>();
        queue.Enqueue(startNode);

        while (queue.Count > 0) {
            Node currentNode = queue.Dequeue();
            
            if (currentNode == targetNode) {
                RetracePath(startNode,targetNode, 4);
                return;
            }

            if (visited.Contains(currentNode))
                continue;

            visited.Add(currentNode);

            foreach(Node neighbor in grid.GetNeighbours(currentNode)){
                if (!neighbor.walkable || visited.Contains(neighbor)) {
                    continue;
                }
                if (!visited.Contains(neighbor)) {
                   neighbor.parent = currentNode;
                   queue.Enqueue(neighbor);
                }
            }
        }
    }

    /*void UCS(Vector3 startPos, Vector3 targetPos) {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
     
        List<(int, Node)> queue = new List<(int, Node)>();
     
        queue.Add((0, startNode));
     
        Dictionary<Node, int> visited = new Dictionary<Node,int>();
     
        while (queue.Count != 0) {
            (int, Node) q = queue[0];
            (int, Node) p = (-q.Item1,q.Item2);

            queue.RemoveAt(0);

            if (p.Item2 == targetNode) {
                RetracePath(startNode, targetNode, 5);
                return;
            }

            if (!visited.ContainsKey(p.Item2)) {
                foreach (Node neighbour in grid.GetNeighbours(p.Item2)) {
                    queue.Add((
                        (p.Item1 + GetDistance(p.Item2, neighbour, 1))*-1,
                        neighbour
                        )
                    );
                    neighbour.parent = p.Item2;
                }
                visited.Add(p.Item2, 1);
            }
        }
    }*/

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
            //grid.AStarAltpath = path;
        }
        else if(algoIndex == 3){
            grid.DFSpath = path;
        }
        else if(algoIndex == 4){
            grid.BFSpath = path;
        }
        else{
            //grid.UCSpath = path;
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