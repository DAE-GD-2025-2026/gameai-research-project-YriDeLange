using System.Collections.Generic;
using UnityEngine;

namespace FlowFieldResearch
{
    /// <summary>
    /// Follows the flow field: each frame it samples the flow vector for the cell it
    /// stands in (the "many-to-one" part — no per-agent search) and layers a local
    /// separation force on top so a crowd spreads out instead of stacking on one cell.
    ///
    /// Separation neighbours come from a shared spatial hash rebuilt once per frame, so
    /// the crowd cost is ~O(N) for roughly uniform density instead of the O(N^2)
    /// all-pairs scan — which matters precisely because flow fields exist for big crowds.
    ///
    /// Movement uses a CLAMPED, SMOOTHED velocity rather than normalising (flow +
    /// separation) to a fixed speed. Normalising forces full speed even when the steering
    /// is tiny or self-cancelling, which makes agents oscillate ("bounce") around their
    /// equilibrium in dense crowds. Clamping lets weak steering produce slow, gentle
    /// motion; smoothing removes the remaining high-frequency jitter.
    /// </summary>
    public class FlowFieldAgent : MonoBehaviour
    {
        private static readonly List<FlowFieldAgent> Agents = new List<FlowFieldAgent>();

        // Spatial hash over agent positions, shared by every agent and rebuilt at most
        // once per frame from a consistent start-of-frame snapshot.
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

        // Position captured when the hash was last built; used for both bucketing and
        // distance, so separation is independent of agent update order within a frame.
        private Vector2 _snapshot;

        // Current velocity, eased toward the target each frame (kills bounce/jitter).
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
                // Ease to a stop at the goal instead of freezing mid-step.
                _velocity = Vector2.Lerp(_velocity, Vector2.zero, SmoothingFactor());
                return;
            }

            EnsureSpatialHash();

            Vector2 flow = controller.GetFlowDirection(position);
            Vector2 separation = ComputeSeparation(position) * separationWeight;

            Vector2 steer = flow + separation;

            // Clamp to maxSpeed (a ceiling), do NOT normalise to it (a setpoint). Weak or
            // self-cancelling steering now yields slow, gentle motion near equilibrium.
            Vector2 target = Vector2.ClampMagnitude(steer, maxSpeed);

            // Frame-rate-independent easing toward the target velocity.
            _velocity = Vector2.Lerp(_velocity, target, SmoothingFactor());

            if (_velocity.sqrMagnitude < 0.0001f) return;

            Vector2 desired = position + _velocity * Time.deltaTime;

            // Flow already steers around walls, but a strong separation push can aim a
            // step into one. Refuse a move that lands in a wall; try sliding along each
            // axis so the agent grazes the wall instead of stopping dead against it.
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
                // else fully boxed in this frame: hold position.
            }
        }

        // exp-based smoothing factor: independent of frame rate, unlike a raw Lerp(t).
        private float SmoothingFactor() => 1f - Mathf.Exp(-velocitySmoothing * Time.deltaTime);

        // First agent to update in a frame rebuilds the shared hash; the rest reuse it.
        private static void EnsureSpatialHash()
        {
            if (Time.frameCount == _hashFrame) return;
            _hashFrame = Time.frameCount;

            // Bucket size must be at least the largest separation radius, so every
            // neighbour within range lands in the 3x3 block around the querying agent.
            float maxRadius = 0f;
            foreach (FlowFieldAgent a in Agents)
                if (a.separationRadius > maxRadius) maxRadius = a.separationRadius;
            _cellSize = Mathf.Max(maxRadius, 0.01f);

            foreach (List<FlowFieldAgent> bucket in Buckets.Values)
                bucket.Clear(); // empty the lists but keep them, to avoid per-frame GC

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
                            // offset / (distance * distance) == offset / distSqr, but with no
                            // sqrt: the same inverse-distance push the all-pairs version used.
                            push += offset / distSqr;
                            count++;
                        }
                    }
                }

            return count == 0 ? Vector2.zero : push / count;
        }
    }
}