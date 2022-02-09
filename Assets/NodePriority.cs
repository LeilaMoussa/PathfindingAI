using System;
using UnityEngine;
using System.Collections;

public class NodePriority : IHeapItem<NodePriority>{
    
    public bool walkable;
    public Vector3 worldPosition;
    public int gridX;
    public int gridY;
    public Node parent;
    public int priority;
    public Node node;
    int heapIndex;
    
    public NodePriority(Node _n, int _priority) {
        node = _n;
        priority = _priority;
    }
    
    public int HeapIndex {
        get {
            return heapIndex;
        }
        set {
            heapIndex = value;
        }
    }

    public int CompareTo(NodePriority nodeToCompare) {
        int compare = priority.CompareTo(nodeToCompare.priority);
        return -compare;
    }
}