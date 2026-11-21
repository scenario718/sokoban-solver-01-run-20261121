# Sokoban Solver — push-optimal, room by room

[![make.fun bounty](https://img.shields.io/badge/make.fun-bounty-blue)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the [make.fun bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121): one Node 22 program (`main.ts`) answers 5 generated levels, writing each level's player moves to `solution.txt` with as few pushes as possible.

## Approach

A generated level is a hallway plus 1–4 rooms (3–4 boxes each). A box pushed out of a room's door into the hallway can never come back, so each room is an independent problem and the level's push-optimal count is the sum of the rooms' optima. Each room is solved push-optimally with A*:

- **State:** the boxes' squares plus the player's reachable region (exact bitmask).
- **Estimate:** the cheapest assignment of boxes to goals over shortest-path distances — admissible, so A* returns the true push-optimum.
- **Deadlock pruning:** live squares (reverse pull-reachability from goals) plus frozen 2×2 blocks of walls/boxes.
- **Determinism:** all budgets are counts of expanded states, never the wall clock; no randomness — two runs write the same bytes.

Fallbacks (whole-level A*, then greedy best-first) cover levels that do not decompose into rooms. Every solution is replayed and verified before it is written.

## Verification (local, kit's own checker)

- **Practice set: 1,000,000 / 1,000,000** — all 5 instances push-optimal (13, 16, 29, 47, 60 pushes = P*), in ~1.3 s total.
- **Robustness: 70/70 instances optimal** on 14 freshly generated sets from the kit's own generator (each set < 1 s, far under the 60 s wall).
- **Determinism:** two runs byte-identical.
- **House baseline (plain-astar 1.0.0) on the same sets: 200,000** — it solves only the 1-room level and gives up on the rest. This entry beats it by 800,000 points.

## Run

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```
