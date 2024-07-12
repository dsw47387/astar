using System.Collections.Generic;
using UnityEngine;

public class Node
{
    public Vector3 position;
    public bool isWalkable;
    public Node parent;
    public float gCost;
    public float hCost;
    public float fCost => gCost + hCost;

    public Node(Vector3 position, bool isWalkable)
    {
        this.position = position;
        this.isWalkable = isWalkable;
    }
}

public class AStarPath : MonoBehaviour
{
    public GameObject targetObject; // The target GameObject
    public LayerMask obstacleLayer; // Layer mask to identify obstacles

    private List<Node> _openList;
    private HashSet<Node> _closedList;
    private Node[,,] _nodes;
    private Vector3 _originPosition;
    private int _gridWidth;
    private int _gridHeight;
    private int _gridDepth;

    void Start()
    {
        _originPosition = transform.position;
        Vector3 startPosition = transform.position;
        Vector3 endPosition = targetObject.transform.position;

        CalculateGridDimensions(startPosition, endPosition);
        InitializeNodes();

        List<Node> path = FindPath(startPosition, endPosition);
        if (path != null)
        {
            foreach (Node node in path)
            {
                // Instantiate(pathPrefab, new Vector3(node.position.x, node.position.y, node.position.z), Quaternion.identity);
                Debug.Log(node.position);
            }
        }
        else
        {
            Debug.Log("No path found!");
        }
    }

    void CalculateGridDimensions(Vector3 start, Vector3 end)
    {
        _gridWidth = Mathf.CeilToInt(Mathf.Abs(end.x - start.x)) + 1;
        _gridHeight = Mathf.CeilToInt(Mathf.Abs(end.y - start.y)) + 1;
        _gridDepth = Mathf.CeilToInt(Mathf.Abs(end.z - start.z)) + 1;
    }

    void InitializeNodes()
    {
        _nodes = new Node[_gridWidth, _gridHeight, _gridDepth];

        for (int x = 0; x < _gridWidth; x++)
        {
            for (int y = 0; y < _gridHeight; y++)
            {
                for (int z = 0; z < _gridDepth; z++)
                {
                    Vector3 position = _originPosition + new Vector3(x, y, z);
                    bool isWalkable = IsWalkable(position);
                    _nodes[x, y, z] = new Node(position, isWalkable);
                }
            }
        }
    }

    bool IsWalkable(Vector3 position)
    {
        // Use a raycast to check if there is an obstacle at the given position
        if (Physics.Raycast(position + Vector3.up * 0.5f, Vector3.down, 1f, obstacleLayer))
        {
            return false;
        }
        return true;
    }

    List<Node> FindPath(Vector3 start, Vector3 end)
    {
        // Convert start and end positions to grid coordinates relative to _originPosition
        Vector3Int startGridPos = WorldToGridPosition(start);
        Vector3Int endGridPos = WorldToGridPosition(end);

        Node startNode = _nodes[startGridPos.x, startGridPos.y, startGridPos.z];
        Node endNode = _nodes[endGridPos.x, endGridPos.y, endGridPos.z];

        _openList = new List<Node> { startNode };
        _closedList = new HashSet<Node>();

        while (_openList.Count > 0)
        {
            Node currentNode = GetLowestFCostNode(_openList);
            if (currentNode == endNode)
            {
                return RetracePath(startNode, endNode);
            }

            _openList.Remove(currentNode);
            _closedList.Add(currentNode);

            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (!neighbor.isWalkable || _closedList.Contains(neighbor))
                {
                    continue;
                }

                float newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (!(newMovementCostToNeighbor < neighbor.gCost) && _openList.Contains(neighbor)) continue;

                neighbor.gCost = newMovementCostToNeighbor;
                neighbor.hCost = GetDistance(neighbor, endNode);
                neighbor.parent = currentNode;

                if (!_openList.Contains(neighbor))
                {
                    _openList.Add(neighbor);
                }
            }
        }

        return null;
    }

    Node GetLowestFCostNode(List<Node> nodeList)
    {
        Node lowestFCostNode = nodeList[0];
        foreach (Node node in nodeList)
        {
            if (node.fCost < lowestFCostNode.fCost)
            {
                lowestFCostNode = node;
            }
        }
        return lowestFCostNode;
    }

    List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        Vector3[] directions = {
            new Vector3(0, 1, 0),
            new Vector3(1, 0, 0),
            new Vector3(0, -1, 0),
            new Vector3(-1, 0, 0),
            new Vector3(0, 0, 1),
            new Vector3(0, 0, -1)
        };

        foreach (Vector3 direction in directions)
        {
            Vector3 neighborPosition = node.position + direction;
            Vector3Int neighborIndex = WorldToGridPosition(neighborPosition);

            if (IsPositionInBounds(neighborIndex))
            {
                neighbors.Add(_nodes[neighborIndex.x, neighborIndex.y, neighborIndex.z]);
            }
        }

        return neighbors;
    }

    float GetDistance(Node nodeA, Node nodeB)
    {
        float dstX = Mathf.Abs(nodeA.position.x - nodeB.position.x);
        float dstY = Mathf.Abs(nodeA.position.y - nodeB.position.y);
        float dstZ = Mathf.Abs(nodeA.position.z - nodeB.position.z);
        return dstX + dstY + dstZ;
    }

    List<Node> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }

        path.Reverse();
        return path;
    }

    Vector3Int WorldToGridPosition(Vector3 worldPosition)
    {
        Vector3 relativePosition = worldPosition - _originPosition;
        return Vector3Int.RoundToInt(relativePosition);
    }

    bool IsPositionInBounds(Vector3Int position)
    {
        return position.x >= 0 && position.x < _gridWidth &&
               position.y >= 0 && position.y < _gridHeight &&
               position.z >= 0 && position.z < _gridDepth;
    }
}