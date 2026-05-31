using UnityEngine;

namespace FlowFieldResearch
{
    public class FlowFieldCell
    {
        public const byte ImpassableCost = byte.MaxValue; // 255

        public Vector2Int GridIndex { get; }
        public Vector2 WorldPosition { get; }

        public byte Cost;

        public ushort BestCost;

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