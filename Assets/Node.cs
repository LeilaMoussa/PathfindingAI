using System;
using UnityEngine;
using System.Collections;

public class Node : IHeapItem<Node> {
    
    public bool walkable;
    public Vector3 worldPosition;
    public int gridX;
    public int gridY;

    public int gCost;
    public int hCost;
    public Node parent;
    int heapIndex;
    
    public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY) {
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }

    public int fCost {
        get {
            String isAlternative = Environment.GetEnvironmentVariable("AStarAlt");
            if(isAlternative.Equals("0"))
                return gCost + hCost;
            //The following is the alternative A* heuristic which returns the minumum value among hCost and gCost
            if(gCost > hCost)
                return hCost;
            return gCost;
        }
    }

    public int HeapIndex {
        get {
            return heapIndex;
        }
        set {
            heapIndex = value;
        }
    }
    
    public int CompareTo(Node nodeToCompare) {
        int compare = fCost.CompareTo(nodeToCompare.fCost);
        if (compare == 0) {
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }
        return -compare;
    }
}