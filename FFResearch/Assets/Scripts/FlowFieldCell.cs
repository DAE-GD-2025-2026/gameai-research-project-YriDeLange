using UnityEngine;

namespace FlowFieldResearch
{
    /// <summary>
    /// A single grid cell. Holds the three values that make up the algorithm:
    /// the raw traversal <see cref="Cost"/>, the accumulated <see cref="BestCost"/>
    /// produced by the integration pass, and the resulting <see cref="FlowDirection"/>.
    /// </summary>
    public class FlowFieldCell
    {
        /// <summary>Cost value that marks a cell as a wall / unreachable.</summary>
        public const byte ImpassableCost = byte.MaxValue; // 255

        public Vector2Int GridIndex { get; }
        public Vector2 WorldPosition { get; }

        /// <summary>Traversal cost: 1 = open ground, higher = slower, 255 = wall.</summary>
        public byte Cost;

        /// <summary>Cheapest total cost to reach the goal (the integration field value).</summary>
        public ushort BestCost;

        /// <summary>Unit vector pointing "downhill" toward the goal.</summary>
        public Vector2 FlowDirection;

        public FlowFieldCell(Vector2Int gridIndex, Vector2 worldPosition)
        {
            GridIndex = gridIndex;
            WorldPosition = worldPosition;
            Cost = 1;
            BestCost = ushort.MaxValue;
            FlowDirection = Vector2.zero;
        }
    }
}