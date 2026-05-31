using UnityEngine;
using UnityEngine.Rendering;

namespace FlowFieldResearch
{
    [RequireComponent(typeof(FlowFieldController))]
    public class FlowFieldRuntimeView : MonoBehaviour
    {
        [SerializeField] private bool drawGrid = true;
        [SerializeField] private bool drawWalls = true;
        [SerializeField] private bool drawFlow = true;

        [Header("Colours")]
        [SerializeField] private Color gridColor = new Color(1f, 1f, 1f, 0.12f);
        [SerializeField] private Color wallColor = new Color(1f, 0.1f, 0.1f, 0.35f);
        [SerializeField] private Color flowColor = Color.cyan;

        private FlowFieldController _controller;
        private Material _lineMaterial;

        private void Awake() => _controller = GetComponent<FlowFieldController>();

        private void OnEnable() => RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
        private void OnDisable() => RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;

        private void OnDestroy()
        {
            if (_lineMaterial != null)
                Destroy(_lineMaterial);
        }

        private void OnEndCameraRendering(ScriptableRenderContext context, Camera camera)
        {
            if (_controller == null) _controller = GetComponent<FlowFieldController>();

            FlowField field = _controller != null ? _controller.Field : null;
            if (field?.Cells == null) return;

            EnsureMaterial();
            _lineMaterial.SetPass(0);

            GL.PushMatrix();
            GL.modelview = camera.worldToCameraMatrix;
            GL.LoadProjectionMatrix(camera.projectionMatrix);

            if (drawWalls) DrawWalls(field);
            if (drawGrid) DrawGrid(field);
            if (drawFlow) DrawFlow(field);

            GL.PopMatrix();
        }

        private void DrawGrid(FlowField field)
        {
            float d = field.CellDiameter;
            Vector2 min = field.Origin;
            Vector2 max = field.Origin + (Vector2)field.GridSize * d;

            GL.Begin(GL.LINES);
            GL.Color(gridColor);

            for (int x = 0; x <= field.GridSize.x; x++)
            {
                float wx = min.x + x * d;
                GL.Vertex3(wx, min.y, 0f);
                GL.Vertex3(wx, max.y, 0f);
            }
            for (int y = 0; y <= field.GridSize.y; y++)
            {
                float wy = min.y + y * d;
                GL.Vertex3(min.x, wy, 0f);
                GL.Vertex3(max.x, wy, 0f);
            }

            GL.End();
        }

        private void DrawWalls(FlowField field)
        {
            float h = field.CellDiameter * 0.5f;

            GL.Begin(GL.QUADS);
            GL.Color(wallColor);

            foreach (FlowFieldCell cell in field.Cells)
            {
                if (cell.Cost != FlowFieldCell.ImpassableCost) continue;

                Vector2 c = cell.WorldPosition;
                GL.Vertex3(c.x - h, c.y - h, 0f);
                GL.Vertex3(c.x - h, c.y + h, 0f);
                GL.Vertex3(c.x + h, c.y + h, 0f);
                GL.Vertex3(c.x + h, c.y - h, 0f);
            }

            GL.End();
        }

        private void DrawFlow(FlowField field)
        {
            float d = field.CellDiameter;
            float shaft = d * 0.4f;
            float head = d * 0.18f;

            GL.Begin(GL.LINES);
            GL.Color(flowColor);

            foreach (FlowFieldCell cell in field.Cells)
            {
                Vector2 dir = cell.FlowDirection;
                if (dir == Vector2.zero) continue;

                Vector2 start = cell.WorldPosition;
                Vector2 tip = start + dir * shaft;

                GL.Vertex3(start.x, start.y, 0f);
                GL.Vertex3(tip.x, tip.y, 0f);

                Vector2 back = -dir;
                Vector2 perp = new Vector2(-dir.y, dir.x);
                Vector2 barbA = tip + (back + perp).normalized * head;
                Vector2 barbB = tip + (back - perp).normalized * head;

                GL.Vertex3(tip.x, tip.y, 0f);
                GL.Vertex3(barbA.x, barbA.y, 0f);
                GL.Vertex3(tip.x, tip.y, 0f);
                GL.Vertex3(barbB.x, barbB.y, 0f);
            }

            GL.End();
        }

        private void EnsureMaterial()
        {
            if (_lineMaterial != null) return;

            Shader shader = Shader.Find("Hidden/Internal-Colored");
            _lineMaterial = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
            _lineMaterial.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
            _lineMaterial.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
            _lineMaterial.SetInt("_Cull", (int)CullMode.Off);
            _lineMaterial.SetInt("_ZWrite", 0);
            _lineMaterial.SetInt("_ZTest", (int)CompareFunction.Always);
        }
    }
}