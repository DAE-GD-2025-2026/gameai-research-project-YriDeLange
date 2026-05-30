using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

namespace FlowFieldResearch
{
    /// <summary>
    /// Pure algorithm + data. It only touches the scene through Physics2D when
    /// baking the cost field, so the three passes can be read (and tested) on
    /// their own, independent of how the grid is wired into a MonoBehaviour.
    ///
    /// Pipeline:
    ///   1. CreateGrid            - lay out the cells
    ///   2. CreateCostField       - mark walls / rough terrain (baked once)
    ///   3. CreateIntegrationField- Dijkstra flood-fill outward from the goal
    ///   4. CreateFlowField       - each cell points at its cheapest neighbour
    /// Only steps 3 and 4 need re-running when the goal moves.
    /// </summary>
    public class FlowField
    {
        public FlowFieldCell[,] Cells { get; private set; }
        public Vector2Int GridSize { get; }
        public float CellDiameter { get; }
        public Vector2 Origin { get; } // world position of the grid's bottom-left corner
        public FlowFieldCell Destination { get; private set; }

        public FlowField(Vector2 origin, Vector2Int gridSize, float cellDiameter)
        {
            Origin = origin;
            GridSize = gridSize;
            CellDiameter = cellDiameter;
        }

        public void CreateGrid()
        {
            Cells = new FlowFieldCell[GridSize.x, GridSize.y];
            for (int x = 0; x < GridSize.x; x++)
            {
                for (int y = 0; y < GridSize.y; y++)
                {
                    Vector2 worldPos = Origin + new Vector2(
                        (x + 0.5f) * CellDiameter,
                        (y + 0.5f) * CellDiameter);
                    Cells[x, y] = new FlowFieldCell(new Vector2Int(x, y), worldPos);
                }
            }
        }

        /// <summary>
        /// Stamp each cell's traversal cost from the level geometry. A box overlap
        /// against the obstacle mask marks walls; an optional rough-terrain mask
        /// raises (but does not block) the cost so the field flows around it.
        /// </summary>
        public void CreateCostField(LayerMask obstacleMask, LayerMask roughMask, byte roughCost)
        {
            // Slightly under-size the test box so we don't pick up colliders that
            // only just clip into the neighbouring cell.
            Vector2 boxSize = Vector2.one * CellDiameter * 0.9f;

            foreach (FlowFieldCell cell in Cells)
            {
                if (Physics2D.OverlapBox(cell.WorldPosition, boxSize, 0f, obstacleMask) != null)
                {
                    cell.Cost = FlowFieldCell.ImpassableCost;
                }
                else if (Physics2D.OverlapBox(cell.WorldPosition, boxSize, 0f, roughMask) != null)
                {
                    cell.Cost = roughCost;
                }
            }
        }

        /// <summary>
        /// Dijkstra flood-fill from the goal using a real priority queue: the cell
        /// with the lowest integration cost is always expanded next. Every reachable
        /// cell ends up storing the cheapest total cost to get there. 4-connectivity
        /// so the field can never "leak" diagonally through a wall corner.
        /// </summary>
        public void CreateIntegrationField(FlowFieldCell destination)
        {
            Destination = destination;

            foreach (FlowFieldCell cell in Cells)
                cell.BestCost = ushort.MaxValue;

            // Only seed the integration value. Do NOT touch destination.Cost: that
            // traversal cost belongs to the baked-once cost field.
            destination.BestCost = 0;

            var open = new MinHeap();
            open.Push(destination, 0);

            while (open.Count > 0)
            {
                open.Pop(out FlowFieldCell current, out ushort priority);

                // Lazy deletion: a cheaper route to `current` was queued after this
                // entry, so this one is stale — skip it.
                if (priority > current.BestCost)
                    continue;

                foreach (FlowFieldCell neighbour in GetNeighbours(current.GridIndex, GridDirection.CardinalDirections))
                {
                    if (neighbour.Cost == FlowFieldCell.ImpassableCost)
                        continue;

                    // Computed as int so the sum can't wrap a ushort mid-calculation.
                    int tentative = current.BestCost + neighbour.Cost;
                    if (tentative < neighbour.BestCost)
                    {
                        neighbour.BestCost = (ushort)tentative;
                        open.Push(neighbour, neighbour.BestCost);
                    }
                }
            }
        }

        /// <summary>
        /// For each passable cell, point at the 8-neighbour with the lowest
        /// integration value. Diagonals that would cut a wall corner are rejected.
        /// </summary>
        public void CreateFlowField()
        {
            foreach (FlowFieldCell cell in Cells)
            {
                if (cell.Cost == FlowFieldCell.ImpassableCost)
                {
                    cell.FlowDirection = Vector2.zero;
                    continue;
                }

                ushort bestCost = cell.BestCost;
                FlowFieldCell bestNeighbour = null;

                foreach (FlowFieldCell neighbour in GetNeighbours(cell.GridIndex, GridDirection.AllDirections))
                {
                    if (neighbour.BestCost >= bestCost)
                        continue;

                    // Reject a diagonal unless both shared orthogonal cells are open,
                    // otherwise agents would clip through the corner of a wall.
                    Vector2Int delta = neighbour.GridIndex - cell.GridIndex;
                    if (delta.x != 0 && delta.y != 0 &&
                        (!IsPassable(cell.GridIndex + new Vector2Int(delta.x, 0)) ||
                         !IsPassable(cell.GridIndex + new Vector2Int(0, delta.y))))
                    {
                        continue;
                    }

                    bestCost = neighbour.BestCost;
                    bestNeighbour = neighbour;
                }

                cell.FlowDirection = bestNeighbour == null
                    ? Vector2.zero // goal cell (or fully walled in): no downhill neighbour
                    : (bestNeighbour.WorldPosition - cell.WorldPosition).normalized;
            }
        }

        public FlowFieldCell GetCellFromWorldPosition(Vector2 worldPosition)
        {
            Vector2 local = worldPosition - Origin;
            int x = Mathf.Clamp(Mathf.FloorToInt(local.x / CellDiameter), 0, GridSize.x - 1);
            int y = Mathf.Clamp(Mathf.FloorToInt(local.y / CellDiameter), 0, GridSize.y - 1);
            return Cells[x, y];
        }

        private bool InBounds(Vector2Int index) =>
            index.x >= 0 && index.x < GridSize.x &&
            index.y >= 0 && index.y < GridSize.y;

        private bool IsPassable(Vector2Int index) =>
            InBounds(index) && Cells[index.x, index.y].Cost != FlowFieldCell.ImpassableCost;

        private IEnumerable<FlowFieldCell> GetNeighbours(Vector2Int index, IReadOnlyList<Vector2Int> directions)
        {
            foreach (Vector2Int dir in directions)
            {
                Vector2Int n = index + dir;
                if (InBounds(n))
                    yield return Cells[n.x, n.y];
            }
        }

        /// <summary>
        /// Tiny binary min-heap keyed on integration cost. Specialised to
        /// (cell, cost) so the Dijkstra pass has a real priority queue without
        /// depending on System.Collections.Generic.PriorityQueue, which isn't
        /// guaranteed across Unity's API-compatibility levels.
        /// </summary>
        private sealed class MinHeap
        {
            private readonly List<(FlowFieldCell cell, ushort cost)> _items =
                new List<(FlowFieldCell, ushort)>();

            public int Count => _items.Count;

            public void Push(FlowFieldCell cell, ushort cost)
            {
                _items.Add((cell, cost));

                // Sift up.
                int i = _items.Count - 1;
                while (i > 0)
                {
                    int parent = (i - 1) / 2;
                    if (_items[parent].cost <= _items[i].cost) break;
                    (_items[parent], _items[i]) = (_items[i], _items[parent]);
                    i = parent;
                }
            }

            public void Pop(out FlowFieldCell cell, out ushort cost)
            {
                cell = _items[0].cell;
                cost = _items[0].cost;

                // Move the last item to the root, then sift down.
                int last = _items.Count - 1;
                _items[0] = _items[last];
                _items.RemoveAt(last);

                int n = _items.Count;
                int i = 0;
                while (true)
                {
                    int left = 2 * i + 1;
                    int right = 2 * i + 2;
                    int smallest = i;

                    if (left < n && _items[left].cost < _items[smallest].cost) smallest = left;
                    if (right < n && _items[right].cost < _items[smallest].cost) smallest = right;
                    if (smallest == i) break;

                    (_items[smallest], _items[i]) = (_items[i], _items[smallest]);
                    i = smallest;
                }
            }
        }
    }
}