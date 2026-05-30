# Flow Fields

Research topic for the Gameplay/AI Programming exam — a self-contained implementation of grid-based flow field pathfinding in Unity.

## Introduction

Flow fields are a *many-to-one* pathfinding technique: instead of computing a path per agent, a single direction field is computed once over the whole grid toward a shared goal, and any number of agents read the cell they stand in to know which way to move. This makes them well suited to large crowds converging on the same target (zombie swarms, RTS armies, evacuations), where running a separate search per agent would be prohibitively expensive.

> _(Result graphic goes here — screenshot of the field-with-arrows overlay.)_

## Design / implementation

The algorithm is a three-stage pipeline over a grid:

1. **Cost field** — per-cell traversal cost (open ground = 1, walls = impassable). Baked once from the level.
2. **Integration field** — Dijkstra flood-fill outward from the goal; each cell stores the cheapest total cost to reach the goal.
3. **Flow field** — each cell points toward the neighbour with the lowest integration value ("downhill" in the cost bowl).

Agents sample the flow vector in their current cell, seek along it, and layer a local separation/avoidance behaviour on top to keep the crowd from piling up.

_(Details of the Unity implementation, grid setup, and neighbour handling to be filled in.)_

## Result

_(Describe and show the outcome — visualisation, agent behaviour, integration-pass timing, etc.)_

## Conclusion

_(Summary of what worked, what the technique is good and bad at, possible extensions such as chunked flow fields or interpolation.)_

## References

_(To be added — e.g. Emerson's "Game AI Pro" flow field chapter, the Continuum Crowds paper, Red Blob Games.)_
