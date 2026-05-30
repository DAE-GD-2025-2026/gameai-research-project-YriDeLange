using UnityEngine;

namespace FlowFieldResearch
{
    /// <summary>
    /// Draws the grid, walls and flow arrows as gizmos. This is the "field-with-arrows
    /// overlay" the README result graphic refers to. Gizmos appear while in Play mode
    /// (the field is baked on Start). Toggle the layers from the inspector.
    /// </summary>
    [RequireComponent(typeof(FlowFieldController))]
    public class FlowFieldDebugView : MonoBehaviour
    {
        [SerializeField] private bool drawGrid = true;
        [SerializeField] private bool drawWalls = true;
        [SerializeField] private bool drawFlow = true;

        private FlowFieldController _controller;

        private void OnDrawGizmos()
        {
            if (_controller == null) _controller = GetComponent<FlowFieldController>();

            FlowField field = _controller != null ? _controller.Field : null;
            if (field?.Cells == null) return;

            float d = field.CellDiameter;

            foreach (FlowFieldCell cell in field.Cells)
            {
                Vector3 pos = cell.WorldPosition;

                if (drawGrid)
                {
                    Gizmos.color = new Color(1f, 1f, 1f, 0.12f);
                    Gizmos.DrawWireCube(pos, new Vector3(d, d, 0f));
                }

                if (drawWalls && cell.Cost == FlowFieldCell.ImpassableCost)
                {
                    Gizmos.color = new Color(1f, 0.1f, 0.1f, 0.35f);
                    Gizmos.DrawCube(pos, new Vector3(d, d, 0f));
                }

                if (drawFlow && cell.FlowDirection != Vector2.zero)
                {
                    Gizmos.color = Color.cyan;
                    Vector3 tip = pos + (Vector3)cell.FlowDirection * (d * 0.4f);
                    Gizmos.DrawLine(pos, tip);
                    Gizmos.DrawSphere(tip, d * 0.06f);
                }
            }
        }
    }
}