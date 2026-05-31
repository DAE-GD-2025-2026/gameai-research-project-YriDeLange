using System.Collections.Generic;
using UnityEngine;

namespace FlowFieldResearch
{
    public class FlowField
    {
        public FlowFieldCell[,] Cells { get; private set; }
        public Vector2Int GridSize { get; }
        public float CellDiameter { get; }
        public Vector2 Origin { get; }
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
        public void CreateCostField(LayerMask obstacleMask, LayerMask roughMask, byte roughCost)
        {
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
        public void CreateIntegrationField(FlowFieldCell destination)
        {
            Destination = destination;

            foreach (FlowFieldCell cell in Cells)
                cell.BestCost = ushort.MaxValue;

            destination.BestCost = 0;

            var open = new MinHeap();
            open.Push(destination, 0);

            while (open.Count > 0)
            {
                open.Pop(out FlowFieldCell current, out ushort priority);

                if (priority > current.BestCost)
                    continue;

                foreach (FlowFieldCell neighbour in GetNeighbours(current.GridIndex, GridDirection.CardinalDirections))
                {
                    if (neighbour.Cost == FlowFieldCell.ImpassableCost)
                        continue;

                    int tentative = current.BestCost + neighbour.Cost;
                    if (tentative < neighbour.BestCost)
                    {
                        neighbour.BestCost = (ushort)tentative;
                        open.Push(neighbour, neighbour.BestCost);
                    }
                }
            }
        }

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
                    ? Vector2.zero
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

        private sealed class MinHeap
        {
            private readonly List<(FlowFieldCell cell, ushort cost)> _items =
                new List<(FlowFieldCell, ushort)>();

            public int Count => _items.Count;

            public void Push(FlowFieldCell cell, ushort cost)
            {
                _items.Add((cell, cost));

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