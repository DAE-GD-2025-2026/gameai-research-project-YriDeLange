using System.Collections.Generic;
using UnityEngine;

namespace FlowFieldResearch
{
    public class FlowFieldAgent : MonoBehaviour
    {
        private static readonly List<FlowFieldAgent> Agents = new List<FlowFieldAgent>();

        private static readonly Dictionary<Vector2Int, List<FlowFieldAgent>> Buckets =
            new Dictionary<Vector2Int, List<FlowFieldAgent>>();
        private static int _hashFrame = -1;
        private static float _cellSize = 1f;

        [SerializeField] private FlowFieldController controller;
        [SerializeField] private float maxSpeed = 4f;
        [SerializeField] private float separationRadius = 0.8f;
        [SerializeField, Range(0f, 3f)] private float separationWeight = 1.2f;

        [Tooltip("Higher = snappier response, lower = floatier. ~6-10 is a good range.")]
        [SerializeField] private float velocitySmoothing = 8f;

        private Vector2 _snapshot;

        private Vector2 _velocity;

        private void Awake()
        {
            if (controller == null)
                controller = FindAnyObjectByType<FlowFieldController>();
        }

        private void OnEnable() => Agents.Add(this);
        private void OnDisable() => Agents.Remove(this);

        private void Update()
        {
            if (controller == null) return;

            Vector2 position = transform.position;
            if (controller.HasReachedGoal(position))
            {
                _velocity = Vector2.Lerp(_velocity, Vector2.zero, SmoothingFactor());
                return;
            }

            EnsureSpatialHash();

            Vector2 flow = controller.GetFlowDirection(position);
            Vector2 separation = ComputeSeparation(position) * separationWeight;

            Vector2 steer = flow + separation;

            Vector2 target = Vector2.ClampMagnitude(steer, maxSpeed);

            _velocity = Vector2.Lerp(_velocity, target, SmoothingFactor());

            if (_velocity.sqrMagnitude < 0.0001f) return;

            Vector2 desired = position + _velocity * Time.deltaTime;

            if (controller.IsWalkable(desired))
            {
                transform.position = desired;
            }
            else
            {
                Vector2 slideX = new Vector2(desired.x, position.y);
                Vector2 slideY = new Vector2(position.x, desired.y);
                if (controller.IsWalkable(slideX)) transform.position = slideX;
                else if (controller.IsWalkable(slideY)) transform.position = slideY;
            }
        }

        private float SmoothingFactor() => 1f - Mathf.Exp(-velocitySmoothing * Time.deltaTime);

        private static void EnsureSpatialHash()
        {
            if (Time.frameCount == _hashFrame) return;
            _hashFrame = Time.frameCount;

            float maxRadius = 0f;
            foreach (FlowFieldAgent a in Agents)
                if (a.separationRadius > maxRadius) maxRadius = a.separationRadius;
            _cellSize = Mathf.Max(maxRadius, 0.01f);

            foreach (List<FlowFieldAgent> bucket in Buckets.Values)
                bucket.Clear();

            foreach (FlowFieldAgent a in Agents)
            {
                a._snapshot = a.transform.position;
                Vector2Int key = BucketKey(a._snapshot);
                if (!Buckets.TryGetValue(key, out List<FlowFieldAgent> list))
                {
                    list = new List<FlowFieldAgent>();
                    Buckets[key] = list;
                }
                list.Add(a);
            }
        }

        private static Vector2Int BucketKey(Vector2 pos) => new Vector2Int(
            Mathf.FloorToInt(pos.x / _cellSize),
            Mathf.FloorToInt(pos.y / _cellSize));

        private Vector2 ComputeSeparation(Vector2 position)
        {
            Vector2 push = Vector2.zero;
            int count = 0;

            Vector2Int center = BucketKey(position);
            float radiusSqr = separationRadius * separationRadius;

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (!Buckets.TryGetValue(center + new Vector2Int(dx, dy),
                            out List<FlowFieldAgent> bucket))
                        continue;

                    foreach (FlowFieldAgent other in bucket)
                    {
                        if (other == this) continue;

                        Vector2 offset = position - other._snapshot;
                        float distSqr = offset.sqrMagnitude;
                        if (distSqr > 0f && distSqr < radiusSqr)
                        {
                            push += offset / distSqr;
                            count++;
                        }
                    }
                }

            return count == 0 ? Vector2.zero : push / count;
        }
    }
}