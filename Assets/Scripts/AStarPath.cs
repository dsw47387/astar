using System.Collections;
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
    public GameObject targetObject;
    public GameObject playerObject;
    public LayerMask obstacleLayer;
    public GameObject openNodeMarker;
    public GameObject closedNodeMarker;

    private List<Node> _openList;
    private HashSet<Node> _closedList;
    private Node[,,] _nodes;
    private Vector3 _originPosition;
    private int _gridWidth;
    private int _gridHeight;
    private int _gridDepth;
    private bool _isFindingPath;
    private Node _startNode;
    private Node _endNode;
    private Node _currentNode;
    private List<Node> _path;

    void Start()
    {
        _originPosition = playerObject.transform.position;
        Vector3 startPosition = _originPosition;
        Vector3 endPosition = targetObject.transform.position;

        CalculateGridDimensions(startPosition, endPosition);
        InitializeNodes();

        _startNode = _nodes[WorldToGridPosition(startPosition).x, WorldToGridPosition(startPosition).y, WorldToGridPosition(startPosition).z];
        _endNode = _nodes[WorldToGridPosition(endPosition).x, WorldToGridPosition(endPosition).y, WorldToGridPosition(endPosition).z];

        _openList = new List<Node> { _startNode };
        _closedList = new HashSet<Node>();

        _isFindingPath = true;
        StartCoroutine(FindPathCoroutine());
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
                    Vector3 gridPosition = _originPosition + new Vector3(x, y, z);
                    _nodes[x, y, z] = new Node(gridPosition, IsWalkable(gridPosition));
                }
            }
        }
    }

    bool IsWalkable(Vector3 gridPosition)
    {
        return !Physics.Raycast(gridPosition + Vector3.up * 2f, Vector3.down, 2f, obstacleLayer);
    }

    IEnumerator FindPathCoroutine()
    {
        while (_openList.Count > 0 && _isFindingPath)
        {
            _currentNode = GetLowestFCostNode(_openList);
            if (_currentNode == _endNode)
            {
                _isFindingPath = false;
                _path = RetracePath(_startNode, _endNode);
                Debug.Log("Path found!");
                StartCoroutine(MovePlayerAlongPath());
                yield break;
            }

            _openList.Remove(_currentNode);
            _closedList.Add(_currentNode);
            Instantiate(closedNodeMarker, _currentNode.position, Quaternion.identity);

            foreach (Node neighbor in GetNeighbors(_currentNode))
            {
                if (!neighbor.isWalkable || _closedList.Contains(neighbor))
                {
                    continue;
                }

                float newMovementCostToNeighbor = _currentNode.gCost + GetDistance(_currentNode, neighbor);
                if (!(newMovementCostToNeighbor < neighbor.gCost) && _openList.Contains(neighbor)) continue;

                neighbor.gCost = newMovementCostToNeighbor;
                neighbor.hCost = GetDistance(neighbor, _endNode);
                neighbor.parent = _currentNode;

                if (!_openList.Contains(neighbor))
                {
                    _openList.Add(neighbor);
                    Instantiate(openNodeMarker, neighbor.position, Quaternion.identity);
                }
            }

            yield return new WaitForSeconds(0.05f);
        }
        Debug.Log("No path found!");
    }

    IEnumerator MovePlayerAlongPath()
    {
        foreach (Node node in _path)
        {
            Vector3 startPosition = playerObject.transform.position;
            Vector3 endPosition = node.position;
            float elapsedTime = 0;
            float duration = .1f;

            while (elapsedTime < duration)
            {
                playerObject.transform.position = Vector3.Lerp(startPosition, endPosition, elapsedTime / duration);
                elapsedTime += Time.deltaTime;
                yield return null;
            }

            playerObject.transform.position = endPosition;
            yield return new WaitForSeconds(.1f);
        }
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
