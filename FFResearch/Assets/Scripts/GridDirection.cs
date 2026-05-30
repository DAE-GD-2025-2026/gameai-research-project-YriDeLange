using System.Collections.Generic;
using UnityEngine;

namespace FlowFieldResearch
{
    /// <summary>
    /// Pre-computed neighbour offsets. Keeping these in tables (instead of nested
    /// if/else over dx/dy) is what lets the integration and flow passes stay short:
    /// the integration pass walks the 4 cardinals (no diagonal corner-cutting),
    /// the flow pass picks from all 8.
    /// </summary>
    public static class GridDirection
    {
        public static readonly IReadOnlyList<Vector2Int> CardinalDirections = new[]
        {
            new Vector2Int( 0,  1), // up
            new Vector2Int( 1,  0), // right
            new Vector2Int( 0, -1), // down
            new Vector2Int(-1,  0), // left
        };

        public static readonly IReadOnlyList<Vector2Int> AllDirections = new[]
        {
            new Vector2Int( 0,  1),
            new Vector2Int( 1,  1),
            new Vector2Int( 1,  0),
            new Vector2Int( 1, -1),
            new Vector2Int( 0, -1),
            new Vector2Int(-1, -1),
            new Vector2Int(-1,  0),
            new Vector2Int(-1,  1),
        };
    }
}