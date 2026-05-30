using System.Collections.Generic;
using UnityEngine;

namespace FlowFieldResearch
{
    /// <summary>
    /// Follows the flow field: each frame it samples the flow vector for the cell it
    /// stands in (the "many-to-one" part — no per-agent search) and layers a local
    /// separation force on top so a crowd spreads out instead of stacking on one cell.
    /// </summary>
    public class FlowFieldAgent : MonoBehaviour
    {
        // Static registry so agents can see each other for separation without
        // every agent needing a collider + physics query.
        private static readonly List<FlowFieldAgent> Agents = new List<FlowFieldAgent>();

        [SerializeField] private FlowFieldController controller;
        [SerializeField] private float maxSpeed = 4f;
        [SerializeField] private float separationRadius = 0.8f;
        [SerializeField, Range(0f, 3f)] private float separationWeight = 1.2f;

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
            if (controller.HasReachedGoal(position)) return;

            Vector2 flow = controller.GetFlowDirection(position);
            Vector2 separation = ComputeSeparation(position) * separationWeight;

            Vector2 steer = flow + separation;
            if (steer.sqrMagnitude < 0.0001f) return;

            Vector2 velocity = steer.normalized * maxSpeed;
            transform.position = position + velocity * Time.deltaTime;
        }

        private Vector2 ComputeSeparation(Vector2 position)
        {
            Vector2 push = Vector2.zero;
            int count = 0;

            foreach (FlowFieldAgent other in Agents)
            {
                if (other == this) continue;

                Vector2 offset = position - (Vector2)other.transform.position;
                float distance = offset.magnitude;
                if (distance > 0f && distance < separationRadius)
                {
                    // Weight by inverse square: the closer the neighbour, the harder the push.
                    push += offset / (distance * distance);
                    count++;
                }
            }

            return count == 0 ? Vector2.zero : push / count;
        }
    }
}