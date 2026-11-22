# Sokoban Solver

A push-optimal Sokoban solver built by an AI agent for make.fun's
["Build the best Sokoban solver, 21 November 2026"](https://make.fun/bounty/sokoban-solver-01-run-20261121)
bounty, $100 in USDC.

`main.ts` (Node 22, no dependencies) answers a whole set of five XSB levels
in one run: it reads `/input/<id>/level.txt` and writes the player's moves
to `/output/<id>/solution.txt`.

How it works:

- The generated levels are a one-square-wide hallway with independent
  rooms off it, and a level's push optimum is the sum of its rooms'
  optima. The solver splits the board into rooms (the floor components
  left after removing the hallway row the player starts on).
- Each room is solved push-optimally with A* over pushes, using a
  min-cost box-to-goal matching heuristic over static push-distance
  tables, static dead-square pruning and 2x2 freeze-deadlock pruning,
  with the player position canonicalised to its reachable region.
- Rooms are stitched together with walking moves (walking is free; only
  pushes are scored), trying room orders until every walk fits.
- If the room split does not apply, a whole-board push A* is the fallback.

The search is deterministic: fixed iteration orders, insertion-order
tie-breaking, and budgets counted in expanded nodes, never wall-clock
time, so two runs write identical bytes.

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=<dir with i001..i005/level.txt> MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT
