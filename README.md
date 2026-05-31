# Flow Field Pathfinding

A self-contained implementation of grid-based **flow field pathfinding** in Unity,
written for the Gameplay / AI Programming research exam. It computes a single shared
direction field over a grid and lets any number of agents follow it toward a common
goal, routing around walls and around (but not necessarily through) costly terrain.

![Flow field result: a 20×20 grid with cyan flow arrows in every passable cell, a red impassable wall the arrows avoid, an orange rough-terrain patch whose interior arrows point back out toward the cheapest exit, and a white goal marker in the top-right corner.](Images/FlowFieldResult.png)

---

## Introduction

Most pathfinding is *one-to-one*: an algorithm such as A\* or Dijkstra computes a
separate path for a single agent from where it is to where it is going. If 500 agents
all chase the same target, that is 500 searches, recomputed every time the situation
changes. A **flow field** inverts the problem. Instead of a path *per agent*, it computes
one **direction field** over the whole grid pointing toward a *shared* goal, and every
agent simply reads the cell it stands in to know which way to move. This is a
*many-to-one* technique: the expensive work is done once and amortised across the entire
crowd, so adding the 501st agent costs only one extra cheap lookup per frame. This makes
flow fields well suited to large crowds converging on the same target — RTS armies,
tower-defense creeps, zombie swarms, evacuations — where a per-agent search would be
prohibitively expensive [Emerson 2013].

A useful mental model is water. To get every raindrop in a valley to one drain, you do
not trace a route for each drop; you shape the terrain into a bowl with the drain at the
bottom and let gravity do the rest. A flow field *is* that bowl, plus an arrow in every
cell pointing downhill toward the goal. The same "distance map → follow it downhill" idea
underlies tower-defense pathfinding and Dijkstra maps more generally [Patel].

---

## Design / implementation

The system is a three-stage field-building pipeline plus an agent layer that follows
the result. The terminology (cost field → integration field → flow field) follows
Emerson's "flow field tiles" formulation [Emerson 2013].

### Project structure

| File | Responsibility |
|------|----------------|
| `FlowField.cs` | Pure algorithm + data: the grid and the three passes. No `MonoBehaviour` dependency except a `Physics2D` query when baking cost. |
| `FlowFieldCell.cs` | One grid cell: traversal `Cost`, integrated `BestCost`, resulting `FlowDirection`. |
| `GridDirection.cs` | Pre-computed neighbour-offset tables (4 cardinal, 8 all). |
| `FlowFieldController.cs` | Scene owner: builds the field, re-bakes on goal change, exposes lookups to agents. |
| `FlowFieldAgent.cs` | Follows the field; adds local separation so a crowd spreads out. |
| `FlowFieldDebugView.cs` / `FlowFieldRuntimeView.cs` | Gizmo and runtime (GL) overlays for the grid, walls and arrows. |

### 1. The grid and cost field

`CreateGrid` lays out a `GridSize.x × GridSize.y` array of cells (20×20 here), each cell's
world centre at `Origin + ((x + 0.5), (y + 0.5)) × CellDiameter`.

`CreateCostField` stamps a per-cell traversal cost from the level geometry, baked **once**
because it does not depend on the goal. Each cell's centre is tested with a
`Physics2D.OverlapBox` (sized at 90% of a cell so a collider that merely clips a neighbour
is not picked up) against two layer masks: the obstacle mask marks a cell impassable, and
an optional rough-terrain mask raises its cost without blocking it. Costs are stored in a
`byte`, with `255` reserved as the impassable sentinel; the inspector caps the
rough-terrain cost at `2..254`, so a rough value can never accidentally collide with the
wall sentinel.

### 2. The integration field (Dijkstra flood from the goal)

`CreateIntegrationField` computes, for every reachable cell, the **cheapest total cost to
reach the goal**. Starting from the goal (value `0`), it floods outward, accumulating
each step's entry cost.

The important design choice is that this is a true **Dijkstra** flood, not a
breadth-first one [Dijkstra 1959; Patel]. The difference matters as soon as terrain costs
vary. Breadth-first search expands by *number of steps*; Dijkstra expands by *lowest
accumulated cost so far*. With a rough-terrain cost of, say, 8, a route that is fewer
steps but crosses the rough patch can be more expensive than a longer detour over cheap
ground — and a breadth-first flood, which finalises each cell on first visit, would lock
in the wrong (expensive) value. Dijkstra always expands the cheapest open cell next, so
the cheap detour is finalised before the expensive shortcut can overwrite it. To get that
ordering without depending on `System.Collections.Generic.PriorityQueue` (which is not
guaranteed across Unity's API-compatibility levels), the implementation uses a small
hand-written binary **min-heap** keyed on integration cost, with *lazy deletion*: a stale
queue entry (one whose stored priority is worse than the cell's current best) is simply
skipped when popped.

Two further details: the relaxation sum is computed as an `int`
(`current.BestCost + neighbour.Cost`) so it cannot wrap the `ushort` mid-calculation, and
the flood walks only the **4 cardinal** neighbours. Cardinal-only connectivity is
deliberate — it prevents cost from "leaking" diagonally through the corner of a wall,
which would later let an agent clip that corner.

Because the integration field is the *exact* global Dijkstra cost-to-goal, it has a useful
property the artificial-potential-field family famously lacks: **no local minima**. Every
reachable non-goal cell provably has at least one strictly-lower-cost neighbour (the next
cell on its own shortest route), so "downhill" always exists and always leads to the goal
— agents cannot get permanently stuck in a dead-end pocket [Treuille et al. 2006].

### 3. The flow field

`CreateFlowField` collapses the integration bowl into directions. For each passable cell
it scans all **8** neighbours, picks the one with the lowest `BestCost`, and stores a unit
vector pointing at it. Reading from 8 neighbours (while integrating over 4) lets the
resulting arrows point diagonally, so agents move in smooth diagonals instead of a blocky
staircase. A diagonal candidate is rejected unless **both** shared orthogonal cells are
open, which is the corner-cut guard that stops an agent being routed through the corner of
a wall. The accumulation convention is internally consistent: an agent at cell C choosing
neighbour N pays N's entry cost plus N's own remaining cost, which sums to exactly
`BestCost[N]`, so minimising `BestCost[N]` minimises true remaining travel. Walls and the
goal get a zero direction (no downhill neighbour).

The 4-cardinal / 8-all split is encoded as two pre-computed offset tables in
`GridDirection`, which keeps both passes branch-free and short.

### Re-baking strategy

`FlowFieldController` builds the grid and cost field once in `Start`, then re-runs **only**
the integration and flow passes when the goal moves (left-click by default). This is the
core amortisation: the cost field is goal-independent, so rebaking it on every goal change
would be wasted work.

### The agent layer

A flow field tells an agent *which way* to go but says nothing about the other agents, so
`FlowFieldAgent` layers a local **separation** behaviour on top — the classic
combination of global guidance with local steering [Reynolds 1999; Treuille et al. 2006].
Each frame an agent samples the flow vector for its current cell, adds a weighted
separation push away from nearby agents (weighted by inverse-square distance, so the
closer the neighbour the harder the push), and steps.

A note on combining the two forces. An early version *normalised* `flow + separation` to a
fixed `maxSpeed`, which forced every agent to move full-speed even when the steering was
tiny or self-cancelling. In dense crowds this produced visible oscillation — agents shoved
apart by separation, pulled back by flow, then re-normalised to full speed in the opposite
direction, bouncing around an equilibrium they could never settle into. The fix is to treat
`maxSpeed` as a *ceiling* rather than a *setpoint*: the steering vector is **clamped** to
`maxSpeed` (so weak steering yields slow, gentle motion near equilibrium) and the agent
**eases** toward that target velocity with frame-rate-independent smoothing
(`1 - exp(-k·dt)`), which removes the remaining high-frequency jitter. This is the standard
"flow as desired velocity, separation as a clamped steering force" formulation
[Reynolds 1999].

Two further implementation notes worth highlighting:

- **Separation neighbours come from a shared spatial hash**, rebuilt at most once per frame
  from a start-of-frame position snapshot. Each agent only inspects the 3×3 block of
  buckets around itself rather than scanning every other agent, so the crowd cost is
  near-linear for roughly uniform density instead of the O(N²) of an all-pairs scan —
  which matters precisely because flow fields exist for large crowds. Taking distances from
  a single snapshot also makes separation independent of script update order.
- **A walkability guard** refuses a step that would land in a wall cell and tries sliding
  along each axis instead, so a strong separation push in a dense crowd cannot shove an
  agent's centre onto a wall (where it would lose all flow guidance).

---

## Result

The screenshot above is a 20×20 field with the goal in the top-right corner, an impassable
wall (red), and a rough-terrain patch (orange). You can read the algorithm's correctness
directly off it:

- **The wall is avoided.** No arrow points into a red cell; the columns flanking the wall
  run parallel to it and funnel agents up and over the end. This is the cardinal
  integration plus the corner-cut guard working together.
- **Rough terrain is genuinely costed, not just coloured.** Inside the orange patch the
  arrows *diverge* — the right edge points out toward the goal, but the left and bottom
  cells point back *out* toward the nearest cheap exit rather than plowing straight across.
  If the patch were ordinary cost-1 ground, every orange cell would point uniformly toward
  the goal like its blue neighbours. The divergence is the inflated integration cost inside
  the patch making "leave cheaply" the downhill direction — the clearest single sign that
  variable cost is integrated correctly.
- **Both features interact.** The vertical corridor of up-arrows immediately right of the
  wall does not point at the goal; it funnels traffic up the cheap gap *between* the wall
  and the rough patch before peeling toward the goal — wall avoidance and rough-terrain
  avoidance combining in one route.
- **No dead cells.** Every passable cell has exactly one arrow; the only cells without a
  direction are the wall and the goal itself, which is the no-local-minima guarantee
  holding in practice.

On performance, the field is built once per goal; thereafter any number of agents read
their cell's vector in O(1) per frame, and the separation layer stays near-linear thanks
to the spatial hash. This is the many-to-one payoff: the cost of the field does not grow
with the size of the crowd.

---

## Conclusion

What works well: variable-cost terrain is handled *correctly* because the integration pass
is a real Dijkstra flood rather than a breadth-first one; the field is provably free of
local minima; and the separation of "field" (pure algorithm) from "agent" (behaviour)
keeps the code testable and the re-bake cost minimal.

**Where flow fields are the right tool:** many agents sharing few goals that change
infrequently — RTS armies, tower-defense waves, swarms, evacuations.

**Where they are not:** a handful of agents each heading to a *different* goal, where you
would rebuild the whole field constantly and a per-agent A\* is cheaper; and very large
maps, where a single full-grid field becomes expensive in memory and integration time. The
production answer to the latter is to tile the map and connect tiles with portals,
computing only the tiles a route passes through (Emerson's "flow field tiles", often paired
with hierarchical A\*) [Emerson 2013].

**Honest limitations of *this* implementation**, kept as named caveats rather than hidden:

- *Diagonal cost inconsistency.* Integration accumulates over cardinals only, so a
  diagonally reachable cell's `BestCost` reflects a Manhattan-style path, while the flow
  pass then sends the agent across the true √2 diagonal. The *direction* is correct; the
  field merely overestimates cost slightly for diagonal cells. Invisible in play, but worth
  stating.
- *`ushort` ceiling.* `BestCost` is a `ushort` with `ushort.MaxValue` as the "unreached"
  sentinel. On a large map with stacked rough terrain a genuine accumulated cost could
  approach 65535; a non-issue at 20×20, but a documented bound.
- *Cell-resolution wall guard.* The guard keeps an agent's *centre* out of wall cells, so a
  sprite can still visually overlap a wall by up to its own radius. True circle-vs-wall
  resolution would push the agent out by penetration depth.
- *Off-grid clamping.* `GetCellFromWorldPosition` clamps out-of-bounds samples to the
  nearest edge cell rather than returning "none", so an agent walking off the open edge of
  the grid is not stopped unless an explicit bounds check is added.
- *Separation worst case.* The spatial hash is near-linear for uniform density but degrades
  toward O(N²) if every agent piles into one bucket — the degenerate case the separation
  force is itself working to prevent.

**Possible extensions:** bilinearly interpolate the flow vectors of nearby cells when an
agent samples the field, for smooth any-angle movement instead of 45°-snapped paths;
chunked / hierarchical flow fields for large maps [Emerson 2013]; a struct-of-arrays cell
layout for cache-friendliness; and partial re-integration for dynamic obstacles (a D\*-style
update of only the affected region).

---

## References

1. **Emerson, E. (2013).** *Crowd Pathfinding and Steering Using Flow Field Tiles.* In
   S. Rabin (Ed.), *Game AI Pro: Collected Wisdom of Game AI Professionals* (Chapter 23,
   pp. 307–316). CRC Press. Free PDF:
   <https://www.gameaipro.com/GameAIPro/GameAIPro_Chapter23_Crowd_Pathfinding_and_Steering_Using_Flow_Field_Tiles.pdf>
   (Republished in *Game AI Pro 360: Guide to Movement and Pathfinding*, CRC Press, 2019,
   pp. 67–76, DOI: 10.1201/9780429055096-7.) — the canonical games-industry description of
   the cost field / integration field / flow field pipeline and of tiling for large maps.

2. **Dijkstra, E. W. (1959).** *A Note on Two Problems in Connexion with Graphs.*
   *Numerische Mathematik, 1*, 269–271. DOI: 10.1007/BF01386390. — the shortest-path
   algorithm the integration pass implements.

3. **Patel, A. — Red Blob Games.** *Introduction to A\** (covers Breadth-First Search,
   Dijkstra's Algorithm, distance maps and flow fields):
   <https://www.redblobgames.com/pathfinding/a-star/introduction.html>; and
   *Tower Defense pathfinding* (Dijkstra maps / flow-field pathfinding worked example):
   <https://www.redblobgames.com/pathfinding/tower-defense/> — accessible, interactive
   explanations of the BFS-vs-Dijkstra distinction and flow-field construction.

4. **Reynolds, C. W. (1999).** *Steering Behaviors For Autonomous Characters.* In
   *Proceedings of the Game Developers Conference 1999* (pp. 763–782). Miller Freeman Game
   Group. Free article: <https://www.red3d.com/cwr/steer/> — the basis for the seek and
   separation steering used in the agent layer.

5. **Treuille, A., Cooper, S., & Popović, Z. (2006).** *Continuum Crowds.* *ACM
   Transactions on Graphics (TOG), 25*(3), 1160–1168. DOI: 10.1145/1141911.1142008. — the
   academic lineage for combining a global guidance field with local crowd avoidance, and
   the source of the "dynamic potential field with no local minima" framing.