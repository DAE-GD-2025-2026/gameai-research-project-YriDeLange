using UnityEngine;
using UnityEngine.InputSystem;

namespace FlowFieldResearch
{
    /// <summary>
    /// Scene-side owner of the <see cref="FlowField"/>. Bakes the grid + cost field
    /// once on Start, then re-runs only the integration and flow passes whenever the
    /// goal moves (left-click by default). Agents read it through GetFlowDirection().
    /// </summary>
    public class FlowFieldController : MonoBehaviour
    {
        [Header("Grid")]
        [SerializeField] private Vector2Int gridSize = new Vector2Int(20, 20);
        [SerializeField] private float cellDiameter = 1f;
        [Tooltip("Use this object's position as the grid's bottom-left corner.")]
        [SerializeField] private bool useTransformAsOrigin = true;
        [SerializeField] private Vector2 origin = Vector2.zero;

        [Header("Cost baking")]
        [SerializeField] private LayerMask obstacleMask;
        [SerializeField] private LayerMask roughTerrainMask;
        [SerializeField, Range(2, 254)] private byte roughTerrainCost = 8;

        [Header("Goal")]
        [Tooltip("Optional transform used as the goal at start. If empty, the grid centre is used.")]
        [SerializeField] private Transform initialGoal;
        [SerializeField] private bool setGoalOnClick = true;

        public FlowField Field { get; private set; }

        private void Start()
        {
            Vector2 gridOrigin = useTransformAsOrigin ? (Vector2)transform.position : origin;

            Field = new FlowField(gridOrigin, gridSize, cellDiameter);
            Field.CreateGrid();
            Field.CreateCostField(obstacleMask, roughTerrainMask, roughTerrainCost);

            Vector2 goal = initialGoal != null
                ? (Vector2)initialGoal.position
                : Field.Cells[gridSize.x / 2, gridSize.y / 2].WorldPosition;

            Rebuild(goal);
        }

        private void Update()
        {
            if (!setGoalOnClick || Mouse.current == null) return;
            if (!Mouse.current.leftButton.wasPressedThisFrame) return;
            if (Camera.main == null) return;

            Vector2 screen = Mouse.current.position.ReadValue();
            Vector3 world = Camera.main.ScreenToWorldPoint(
                new Vector3(screen.x, screen.y, -Camera.main.transform.position.z));
            Rebuild(world);
        }

        /// <summary>Re-run the integration + flow passes for a new goal position.</summary>
        public void Rebuild(Vector2 goalWorldPosition)
        {
            if (Field == null) return;

            FlowFieldCell destination = Field.GetCellFromWorldPosition(goalWorldPosition);

            // A wall is never a valid goal: ignore the click and keep the current field.
            if (destination.Cost == FlowFieldCell.ImpassableCost) return;

            Field.CreateIntegrationField(destination);
            Field.CreateFlowField();
        }

        /// <summary>Flow direction at a world position (zero outside the grid or on a wall).</summary>
        public Vector2 GetFlowDirection(Vector2 worldPosition)
        {
            return Field == null
                ? Vector2.zero
                : Field.GetCellFromWorldPosition(worldPosition).FlowDirection;
        }

        public bool HasReachedGoal(Vector2 worldPosition)
        {
            if (Field?.Destination == null) return false;
            return Field.GetCellFromWorldPosition(worldPosition) == Field.Destination;
        }

        /// <summary>True if the world position maps to a non-wall cell.</summary>
        public bool IsWalkable(Vector2 worldPosition)
        {
            if (Field == null) return false;
            return Field.GetCellFromWorldPosition(worldPosition).Cost != FlowFieldCell.ImpassableCost;
        }
    }
}