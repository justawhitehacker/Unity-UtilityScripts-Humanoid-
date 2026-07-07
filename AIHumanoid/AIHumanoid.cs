using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;
using System.Collections;

public enum HeuristicType
{
    Static,
    Flexible,
    Dynamic
}

public class HAGridNode
{
    public float gCost;
    public float hCost;
    
    public float fCost => gCost + hCost; // Relaying on A* formula: f(n) = g(n) + h(n)

    public Vector3 WorldPosition;

    public float GridX;
    public float GridY;
    public float GridZ;

    public HAGridNode Parent; 
}

public class HAGridCalculation
{
    private Vector3 __startPosition;
    private Vector3 __targetPosition;
    private HeuristicType __heuType;

    public HAGridCalculation(Vector3 __startPosition, Vector3 __targetPosition, HeuristicType __heuType)
    {
        this.__startPosition = __startPosition;
        this.__targetPosition = __targetPosition;
        this.__heuType = __heuType;
    }

    private float Heuristic(float n)
    {
        switch (__heuType)
        {
            case HeuristicType.Static:
            {
                float __x = __startPosition.x - __targetPosition.x;
                float __y = __startPosition.y - __targetPosition.y;
                float __z = __startPosition.z - __targetPosition.z;

                return Mathf.Abs(__x) + Mathf.Abs(__y) + Mathf.Abs(__z);
            }
            
            case HeuristicType.Flexible:
            {
                float __x = __startPosition.x - __targetPosition.x;
                float __y = __startPosition.y - __targetPosition.y;
                float __z = __startPosition.z - __targetPosition.z;

                return Mathf.Sqrt(__x*__x + __y*__y + __z*__z);        
            }

            case HeuristicType.Dynamic:
            {
                float __x = __startPosition.x - __targetPosition.x;
                float __y = __startPosition.y - __targetPosition.y;
                float __z = __startPosition.z - __targetPosition.z;

                float __dx = Mathf.Abs(__x);
                float __dy = Mathf.Abs(__y);
                float __dz = Mathf.Abs(__z);

                float __d3 = Mathf.Sqrt(3);
                float __d2 = Mathf.Sqrt(2);
                float __d1 = 1.0f;

                float __max = Mathf.Max(__dx, __dy, __dz);
                float __min = Mathf.Min(__dx, __dy, __dz);
                float __mid = __dx + __dy + __dz - __min - __max;

                return (__d3 - __d2) * __min + (__d2 - __d1) * __mid + __d1 * Mathf.Min(__dx, __dy, __dz);
            }
        }

        return 0;
    } 

    
}

public class AIHumanoidAgent : MonoBehaviour
{
    // private struct DijkstraIdentity
    // {
    //     public int Index;    
    //     public Vector3 Position;
    // }

    private NavMeshPath navMeshPath;

    [SerializeField] private Humanoid humanoid;
    [SerializeField] private HumanoidMotor humanoidMotor;
    
    [Header("Agent Chrono")]
    [SerializeField] private bool isActive = true;
    [SerializeField] private float updateInterval = 0.35f;
    [SerializeField] private int sleepDurationThread = 3;
    [SerializeField] private float updateDistanceThreshold = 3.0f;
    [SerializeField] private float agentChunkLength = 1000.0f;

    [Header("Details")]
    [SerializeField] private LayerMask areaMask;
    [SerializeField] private int allocateSize = 32;

    private Dictionary<Vector3, int> _distance;
    private Dictionary<Vector3, Vector3> _previous;
    private Dictionary<Vector3, bool> _visited;

    private List<Vector3Int> _marked;

    private Vector3[] _calculatedWaypoints;

    private Vector3 _target;
    private bool _calculationRequested;

    private Vector3Int WorldToGrid(Vector3 __position, float __chunkSize)
    {
        return new Vector3Int(
            Mathf.RoundToInt(__position.x / __chunkSize),
            Mathf.RoundToInt(__position.y / __chunkSize),
            Mathf.RoundToInt(__position.z / __chunkSize)
        );
    }

    private List<Vector3Int> MarkChunks3DDDA(Vector3 __st, Vector3 __trg, float __chunkSize)
    {
        Vector3Int __start = WorldToGrid(__st, __chunkSize);
        Vector3Int __target = WorldToGrid(__trg, __chunkSize);

        _marked.Clear();
        _marked.Add(__start);

        if (__start == __target)
            return _marked;

        Vector3Int __dir = __target - __start;

        Vector3Int __step = new Vector3Int(
            (__dir.x > 0) ? 1 : ((__dir.x < 0) ? -1 : 0),
            (__dir.y > 0) ? 1 : ((__dir.y < 0) ? -1 : 0),
            (__dir.z > 0) ? 1 : ((__dir.z < 0) ? -1 : 0)
        );

        float __inf = float.PositiveInfinity;
        Vector3 __tdelta = new Vector3(
            Mathf.Approximately(__dir.x, 0.0f) ? __inf : Mathf.Abs(__chunkSize / __dir.x),
            Mathf.Approximately(__dir.y, 0.0f) ? __inf : Mathf.Abs(__chunkSize / __dir.y),
            Mathf.Approximately(__dir.z, 0.0f) ? __inf : Mathf.Abs(__chunkSize / __dir.z)
        );

        Vector3 __tmax;


        return _marked;
    }

    private List<Vector3Int> MarkChunks(Vector3 __st, Vector3 __trg, float __chunkSize)
    {
        Vector3Int __start = WorldToGrid(__st, __chunkSize);
        Vector3Int __target = WorldToGrid(__trg, __chunkSize);

        _marked.Clear();

        Vector3Int __dist = __target - __start;
        int __steps = Mathf.Max(
            Mathf.Abs(__dist.x),
            Mathf.Abs(__dist.y),
            Mathf.Abs(__dist.z)
        );

        if (__steps == 0)
        {
            _marked.Add(__start);
            return _marked;
        }

        for (int __i = 0; __i <= __steps; __i++)
        {
            float __t = __i / (float) __steps;

            Vector3 __mark = Vector3.Lerp(
                __start,
                __target,
                __t
            );

            Vector3Int __chunk = new Vector3Int(
                Mathf.RoundToInt(__mark.x),
                Mathf.RoundToInt(__mark.y),
                Mathf.RoundToInt(__mark.z)
            );

            if (!_marked.Contains(__chunk))
                _marked.Add(__chunk);
        }

        return _marked;
    }

    private int BuildPath(Vector3 __start, Vector3 __target, Vector3[] __pathVectors)
    {
        if (__start.sqrMagnitude < 0.01f || __target.sqrMagnitude < 0.01f)
            return 0;

        List<Vector3Int> __temp = MarkChunks(__start, __target, agentChunkLength);
        if (__temp.Count < 2)
        {
            __temp.Clear();
            return 0;
        }
        
        Vector3 __ref_lastChunkPos = __start;

        float __distance = 0.0f;
        for (int __i = 0; __i < __temp.Count; __i++)
        {
            Vector3 __chunkPos = __temp[__i];

            Vector3 __diff = __chunkPos - __ref_lastChunkPos;
            float __total = __diff.sqrMagnitude;

            __distance += __total;
            __ref_lastChunkPos = __chunkPos;
        }

        if (NavMesh.SamplePosition(__start, out NavMeshHit __info, __distance, NavMesh.AllAreas))
        {
            if (NavMesh.CalculatePath(__start, __info.position, NavMesh.AllAreas, navMeshPath))
            {
                int __count = navMeshPath.GetCornersNonAlloc(__pathVectors);

                return __count;
            }
        }

        return 0;
    }

    private IEnumerator CalculatePath()
    {
        while (isActive)
        {
            yield return new WaitForSeconds(sleepDurationThread);

            if (_calculationRequested) continue;

            Vector3 __currentPos;
            Vector3 __targetPos;
           
            __currentPos = transform.position;
            __targetPos = _target;

            bool __readyToGenerate = (_target - transform.position).sqrMagnitude 
                >= updateDistanceThreshold * updateDistanceThreshold;

            if (!__readyToGenerate) continue;

            _calculationRequested = true;

            if (NavMesh.CalculatePath(__currentPos, __targetPos, NavMesh.AllAreas, navMeshPath))
            {
                _distance.Clear();
                _previous.Clear();
                _visited.Clear();

                int __cornersNonAlloc = navMeshPath.GetCornersNonAlloc(_calculatedWaypoints);

                for (int __i = 0; __i < __cornersNonAlloc; __i++)
                {
                    Vector3 __waypoint = _calculatedWaypoints[__i];

                    _distance[__waypoint] = int.MaxValue;
                     _previous[__waypoint] = Vector3.zero;
                    _visited[__waypoint] = false;
                }

                _distance[__currentPos] = 0;
                _previous[__currentPos] = Vector3.zero;
                _visited[__currentPos] = false;
            }

            _calculationRequested = false;

        }
    }


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private void Start()
    {
        navMeshPath = new NavMeshPath();

        _distance = new();
        _previous = new();
        _visited = new();

        _marked = new();

        _calculatedWaypoints = new Vector3[allocateSize];
    }

    private void LateUpdate()
    {
        StartCoroutine(CalculatePath());
    }
}
