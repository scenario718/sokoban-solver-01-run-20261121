// Sokoban push-optimal solver — my own implementation (A* over pushes, per room).
//
// Idea: a level is a hallway plus 1-4 rooms (3-4 boxes each). A box pushed out of a
// room's door into the hallway can never come back, so each room is an independent
// problem and the level's push-optimal count is the sum of the rooms' optima. Each
// room is solved push-optimally with A*: a state is the boxes' squares plus the
// player's reachable region; the estimate is the cheapest assignment of boxes to
// goals over shortest-path distances (admissible: one push moves one box one square).
// Deadlocks pruned: squares a box can never be pulled back from (live squares) and
// frozen 2x2 blocks of walls/boxes. All budgets are counts of expanded states, never
// the wall clock, so two runs write the same bytes. No randomness anywhere.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** Per-room A* expansion cap; greedy fallback cap; whole-level cap. State counts, not time. */
const ASTAR_CAP = 200_000;
const GREEDY_CAP = 100_000;
const WHOLE_CAP = 300_000;
/** Total expansions across the whole set before rooms drop to greedy only. */
const GLOBAL_CAP = 800_000;

const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals: goals.sort((a, b) => a - b), boxes: boxes.sort((a, b) => a - b), player };
}

/** Squares a box can be pulled back from onto some goal (reverse reachability from goals). */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (y >= 0 && y < wall.length && !wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Shortest walking distances from each goal over open floor (boxes ignored), per goal. */
function goalDistances(level: Level): Int32Array[] {
  const { w, wall, goals } = level;
  const out: Int32Array[] = [];
  for (const g of goals) {
    const dist = new Int32Array(wall.length).fill(-1);
    const queue = [g];
    dist[g] = 0;
    for (let qi = 0; qi < queue.length; qi++) {
      const p = queue[qi]!;
      for (const [dx, dy] of DIRS) {
        const q = p + dy * w + dx;
        if (q >= 0 && q < wall.length && !wall[q] && dist[q] < 0) {
          dist[q] = dist[p]! + 1;
          queue.push(q);
        }
      }
    }
    out.push(dist);
  }
  return out;
}

/** Cheapest assignment of boxes to goals (brute-force permutations; <= 4 boxes/goals here). */
function assignmentCost(boxes: number[], dist: Int32Array[]): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (dist.length < n) return 1e9;
  const perm: number[] = [];
  for (let i = 0; i < n; i++) perm.push(i);
  let best = 1e9;
  const rec = (k: number, acc: number): void => {
    if (acc >= best) return;
    if (k === n) {
      best = acc;
      return;
    }
    for (let j = k; j < n; j++) {
      [perm[k], perm[j]] = [perm[j]!, perm[k]!];
      const d = dist[perm[k]!]![boxes[k]!]!;
      if (d >= 0) rec(k + 1, acc + d);
      [perm[k], perm[j]] = [perm[j]!, perm[k]!];
    }
  };
  rec(0, 0);
  return best;
}

/** Whether the box pushed to `t` sits in a frozen 2x2 block of walls/boxes with a box off its goal. */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (a < 0 || d >= wall.length) continue;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** The player's reachable region from `from` (walls/boxes block), as a bitmask plus its squares. */
function regionMask(level: Level, occupied: Uint8Array, from: number): bigint {
  const { w, wall } = level;
  let mask = 0n;
  const stack = [from];
  mask |= 1n << BigInt(from);
  while (stack.length > 0) {
    const p = stack.pop()!;
    for (const [dx, dy] of DIRS) {
      const q = p + dy * w + dx;
      if (q < 0 || q >= wall.length || wall[q] || occupied[q]) continue;
      const bit = 1n << BigInt(q);
      if ((mask & bit) === 0n) {
        mask |= bit;
        stack.push(q);
      }
    }
  }
  return mask;
}

/** Shortest walk from `from` to `to` around walls and boxes, lower LURD letters (u first on ties). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string | null {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how: string[] = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (q < 0 || q >= wall.length || wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return null;
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

interface SearchResult {
  pushes: number;
  moves: string;
}

/**
 * Push-optimal (A*) or greedy best-first search of one (sub-)level. A state is the sorted
 * boxes plus the player's reachable region. `mode` 'astar' is optimal for an admissible
 * estimate; 'greedy' takes the first solved state it meets (valid, maybe not optimal).
 * Returns null when the state cap is hit or no solution exists. Deterministic: fixed caps,
 * ties broken by insertion order, no clock, no randomness.
 */
function search(level: Level, cap: number, mode: 'astar' | 'greedy', budget: { used: number }): SearchResult | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  const dist = goalDistances(level);
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (level.boxes.some((b) => !live[b])) return null;

  const occupied = new Uint8Array(size);
  const hCache = new Map<string, number>();
  const hOf = (boxes: number[]): number => {
    const k = boxes.join(',');
    let v = hCache.get(k);
    if (v === undefined) {
      v = assignmentCost(boxes, dist);
      hCache.set(k, v);
    }
    return v;
  };

  // States: boxes, player square (where the push that made it left the player), parent,
  // the push (from-square, direction) that made it, pushes so far. Key: boxes + region mask.
  const sBoxes: number[][] = [level.boxes.slice()];
  const sPlayer: number[] = [level.player];
  const sParent: number[] = [-1];
  const sPushFrom: number[] = [-1];
  const sPushDir: number[] = [-1];
  const sG: number[] = [0];
  const sH: number[] = [hOf(level.boxes)];
  const sKey: string[] = [];
  const best = new Map<string, number>();
  for (const b of level.boxes) occupied[b] = 1;
  const startMask = regionMask(level, occupied, level.player);
  for (const b of level.boxes) occupied[b] = 0;
  sKey.push(level.boxes.join(',') + '|' + startMask.toString(36));
  best.set(sKey[0]!, 0);

  // Binary heap on (f, h, seq): f = g + h for A*, f = h for greedy.
  const heap: [number, number, number, number][] = [];
  const prio = (s: number): number => (mode === 'astar' ? sG[s]! + sH[s]! : sH[s]!);
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  let seq = 0;
  heapPush([prio(0), sH[0]!, seq++, 0]);

  let expanded = 0;
  let found = -1;
  while (heap.length > 0 && found < 0) {
    const [, , , s] = heapPop();
    const boxes = sBoxes[s]!;
    const g = sG[s]!;
    // Skip a heap entry superseded by a better path to the same state.
    if (best.get(sKey[s]!) !== s) continue;
    // A goal state is optimal only when popped (A*), not when first generated.
    if (solved(boxes)) {
      found = s;
      break;
    }
    for (const b of boxes) occupied[b] = 1;
    const mask = regionMask(level, occupied, sPlayer[s]!);
    if (expanded >= cap || budget.used >= GLOBAL_CAP) {
      for (const b of boxes) occupied[b] = 0;
      return null;
    }
    expanded++;
    budget.used++;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di]!;
        const d = dy * w + dx;
        const pushFrom = b - d;
        const t = b + d;
        if (t < 0 || t >= size || wall[t] || occupied[t] || !live[t]) continue;
        if (((mask >> BigInt(pushFrom)) & 1n) === 0n) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let next: number[] | null = null;
        let nextKey = '';
        let nextMask = 0n;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((a, bb) => a - bb);
          nextMask = regionMask(level, occupied, b);
          nextKey = next.join(',') + '|' + nextMask.toString(36);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || next === null) continue;
        const prevBest = best.get(nextKey);
        if (prevBest !== undefined && sG[prevBest]! <= g + 1) continue;
        const h = hOf(next);
        if (h >= 1e9) continue;
        const idx = sBoxes.length;
        best.set(nextKey, idx);
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(s);
        sPushFrom.push(b);
        sPushDir.push(di);
        sG.push(g + 1);
        sH.push(h);
        sKey.push(nextKey);
        heapPush([mode === 'astar' ? g + 1 + h : h, h, seq++, idx]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  // Reconstruct the pushes, then expand each into a walk + push.
  const path: number[] = [];
  for (let s = found; s > 0; s = sParent[s]!) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes.slice();
  for (const s of path) {
    const from = sPushFrom[s]!;
    const di = sPushDir[s]!;
    const [dx, dy, , letter] = DIRS[di]!;
    const d = dy * w + dx;
    const to = from + d;
    for (const b of boxes) occupied[b] = 1;
    const walk = walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    if (walk === null) return null;
    moves += walk + letter;
    player = from;
    const at = boxes.indexOf(from);
    boxes[at] = to;
    boxes.sort((a, bb) => a - bb);
  }
  return { pushes: path.length, moves };
}

/** The square the player stands on after `moves` (case aside) from `from` on a board `w` wide. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway: a box there
 * reaches no goal) out of the floor; each remaining part holding a box is a room.
 * Null when some part holds more/fewer boxes than goals, or a box starts in the hallway.
 */
function roomsOf(level: Level): { squares: number[]; boxes: number[]; goals: number[]; done: boolean }[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: { squares: number[]; boxes: number[]; goals: number[]; done: boolean }[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (q >= 0 && q < wall.length && !wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (goals.some((g) => part[g]! < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Replay `moves` on `level`: true when every step is legal and every box ends on a goal. */
function replay(level: Level, moves: string): { solved: boolean; pushes: number } {
  const { w, wall, goals } = level;
  const goal = new Uint8Array(wall.length);
  for (const g of goals) goal[g] = 1;
  const box = new Uint8Array(wall.length);
  for (const b of level.boxes) box[b] = 1;
  let player = level.player;
  let pushes = 0;
  for (const ch of moves.toLowerCase()) {
    const [dx, dy] = DIRS.find(([, , l]) => l === ch)!;
    const d = dy * w + dx;
    const next = player + d;
    if (next < 0 || next >= wall.length || wall[next]) return { solved: false, pushes };
    if (box[next]) {
      const beyond = next + d;
      if (beyond < 0 || beyond >= wall.length || wall[beyond] || box[beyond]) return { solved: false, pushes };
      box[next] = 0;
      box[beyond] = 1;
      pushes++;
    }
    player = next;
  }
  for (let i = 0; i < box.length; i++) if (box[i] && !goal[i]) return { solved: false, pushes };
  return { solved: true, pushes };
}

/** Solve one level: room by room when it decomposes, else whole-level A*, then greedy. */
function solve(text: string): string {
  const level = parseLevel(text);
  const goal = new Uint8Array(level.wall.length);
  for (const g of level.goals) goal[g] = 1;
  if (level.boxes.every((b) => goal[b])) return '';
  const budget = { used: 0 };

  const solveWhole = (): string | null => {
    const r = search(level, WHOLE_CAP, 'astar', budget) ?? search(level, GREEDY_CAP, 'greedy', budget);
    return r === null ? null : r.moves;
  };

  const rooms = roomsOf(level);
  if (rooms !== null) {
    let player = level.player;
    let moves = '';
    let ok = true;
    for (const room of rooms) {
      const subWall = level.wall.slice();
      for (const other of rooms) {
        if (other === room) continue;
        for (const b of other.done ? other.goals : other.boxes) subWall[b] = 1;
      }
      const sub: Level = {
        w: level.w,
        h: level.h,
        wall: subWall,
        goals: room.goals.slice().sort((a, b) => a - b),
        boxes: room.boxes.slice().sort((a, b) => a - b),
        player,
      };
      const solved =
        search(sub, ASTAR_CAP, 'astar', budget) ?? search(sub, GREEDY_CAP, 'greedy', budget);
      if (solved === null) {
        ok = false;
        break;
      }
      moves += solved.moves;
      player = endOf(level.w, player, solved.moves);
      room.done = true;
    }
    if (ok) {
      const check = replay(level, moves);
      if (check.solved) return moves;
    }
    const whole = solveWhole();
    if (whole !== null && replay(level, whole).solved) return whole;
    return '';
  }
  const whole = solveWhole();
  if (whole !== null && replay(level, whole).solved) return whole;
  return '';
}

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
