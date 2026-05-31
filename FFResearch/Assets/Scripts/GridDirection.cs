using System.Collections.Generic;
using UnityEngine;

namespace FlowFieldResearch
{
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