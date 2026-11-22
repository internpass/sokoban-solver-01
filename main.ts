// Sokoban solver for make.fun's sokoban-solver-01.
//
// Strategy: the generated levels are a one-wide hallway with independent
// rooms off it, and a level's push optimum is the sum of its rooms' optima.
// So: split the board into rooms (the floor components left after removing
// the hallway row the player starts on), solve each room push-optimally
// with A* over pushes, then stitch the rooms together with walking moves
// (walking is free; only pushes are scored). If the split does not apply,
// fall back to a whole-board push A*.
//
// Deterministic by construction: fixed iteration orders, a binary heap with
// insertion-order tie-breaking, and search budgets counted in expanded
// nodes, never in wall-clock time. Two runs write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions in a fixed order: right, left, down, up.
const DX = [1, -1, 0, 0];
const DY = [0, 0, 1, -1];
const DC = ['r', 'l', 'd', 'u'];
const INF = 1e9;

// ---------------------------------------------------------------- parsing

function parseLevel(text) {
  const rows = text.replace(/\r/g, '').split('\n');
  while (rows.length && rows[rows.length - 1] === '') rows.pop();
  const H = rows.length;
  let W = 0;
  for (const r of rows) W = Math.max(W, r.length);
  const wall = new Uint8Array(W * H).fill(1);
  const goal = new Uint8Array(W * H);
  const boxes = [];
  let player = -1;
  let playerRow = -1;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const ch = x < rows[y].length ? rows[y][x] : '#';
      const c = y * W + x;
      if (ch === '#') continue;
      wall[c] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[c] = 1;
      if (ch === '$' || ch === '*') boxes.push(c);
      if (ch === '@' || ch === '+') { player = c; playerRow = y; }
    }
  }
  boxes.sort((a, b) => a - b);
  const goals = [];
  for (let c = 0; c < W * H; c++) if (goal[c]) goals.push(c);
  return { W, H, wall, goal, goals, boxes, player, playerRow };
}

// Flood fill over cells where pass(c) holds; returns a Uint8Array mark and
// the list of reached cells. Deterministic order (stack, fixed dir order).
function flood(W, H, start, pass) {
  const seen = new Uint8Array(W * H);
  const cells = [];
  if (start < 0 || !pass(start)) return { seen, cells };
  const stack = [start];
  seen[start] = 1;
  while (stack.length) {
    const c = stack.pop();
    cells.push(c);
    const x = c % W, y = (c / W) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d], ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const n = ny * W + nx;
      if (!seen[n] && pass(n)) { seen[n] = 1; stack.push(n); }
    }
  }
  return { seen, cells };
}

// Shortest walking path (list of direction indices) from `from` to `to`
// over cells where pass(c) holds, or null. BFS, fixed direction order.
function walkPath(W, H, from, to, pass) {
  if (from === to) return [];
  const prev = new Int32Array(W * H).fill(-2);
  const pdir = new Int8Array(W * H);
  const q = [from];
  prev[from] = -1;
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    const x = c % W, y = (c / W) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d], ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const n = ny * W + nx;
      if (prev[n] !== -2 || !pass(n)) continue;
      prev[n] = c; pdir[n] = d;
      if (n === to) {
        const path = [];
        let cur = to;
        while (cur !== from) { path.push(pdir[cur]); cur = prev[cur]; }
        path.reverse();
        return path;
      }
      q.push(n);
    }
  }
  return null;
}

// ------------------------------------------------------- push-space tables

// For one goal, the minimum pushes to bring a box from any cell to the goal,
// ignoring other boxes. Reverse BFS in push space: a box at cell c could
// have come from c-d when c-d is a box cell and the stand cell c-2d is a
// player cell. boxCells/playerCells are Uint8Array membership marks.
function pushDistances(W, H, goalCell, boxCells, playerCells) {
  const dist = new Int32Array(W * H).fill(INF);
  dist[goalCell] = 0;
  const q = [goalCell];
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    const x = c % W, y = (c / W) | 0;
    for (let d = 0; d < 4; d++) {
      const px = x - DX[d], py = y - DY[d];
      const sx = x - 2 * DX[d], sy = y - 2 * DY[d];
      if (px < 0 || py < 0 || px >= W || py >= H) continue;
      if (sx < 0 || sy < 0 || sx >= W || sy >= H) continue;
      const p = py * W + px, s = sy * W + sx;
      if (!boxCells[p] || !playerCells[s]) continue;
      if (dist[p] === INF) { dist[p] = dist[c] + 1; q.push(p); }
    }
  }
  return dist;
}

// Min-cost matching of boxes to goals with the given per-goal distance
// tables (DP over subsets; at most a handful of boxes). INF if impossible.
function matchCost(boxes, distTables) {
  const n = boxes.length;
  if (n === 0) return 0;
  const full = (1 << n) - 1;
  let dp = new Float64Array(1 << n).fill(INF);
  dp[0] = 0;
  for (let mask = 0; mask <= full; mask++) {
    let i = 0;
    for (let m = mask; m; m >>= 1) i += m & 1;
    if (i >= n || dp[mask] === INF) continue;
    for (let j = 0; j < n; j++) {
      if (mask & (1 << j)) continue;
      const c = distTables[j][boxes[i]];
      if (c >= INF) continue;
      const nm = mask | (1 << j);
      if (dp[mask] + c < dp[nm]) dp[nm] = dp[mask] + c;
    }
  }
  return dp[full];
}

// ------------------------------------------------------------ A* over pushes

// A binary heap of nodes ordered by (f, h, insertion counter).
function heapPush(heap, node) {
  heap.push(node);
  let i = heap.length - 1;
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (less(heap[i], heap[p])) { const t = heap[i]; heap[i] = heap[p]; heap[p] = t; i = p; }
    else break;
  }
}
function heapPop(heap) {
  const top = heap[0];
  const last = heap.pop();
  if (heap.length) {
    heap[0] = last;
    let i = 0;
    for (;;) {
      const l = 2 * i + 1, r = l + 1;
      let m = i;
      if (l < heap.length && less(heap[l], heap[m])) m = l;
      if (r < heap.length && less(heap[r], heap[m])) m = r;
      if (m === i) break;
      const t = heap[i]; heap[i] = heap[m]; heap[m] = t; i = m;
    }
  }
  return top;
}
function less(a, b) {
  if (a.f !== b.f) return a.f < b.f;
  if (a.h !== b.h) return a.h < b.h;
  return a.n < b.n;
}

// Solve one push-space problem. boxCells/playerCells: membership marks.
// startBoxes: sorted array; playerStart: a player cell. Returns the list of
// pushes {box, dir} in order, or null when unsolved within the node budget.
function solvePushSpace(W, H, goal, boxCells, playerCells, startBoxes, playerStart, nodeCap) {
  const N = W * H;
  const goalCells = [];
  for (let c = 0; c < N; c++) if (goal[c] && boxCells[c]) goalCells.push(c);
  if (goalCells.length !== startBoxes.length) return null;
  const distTables = goalCells.map((g) => pushDistances(W, H, g, boxCells, playerCells));
  // Dead squares: a box cell that reaches no goal.
  const dead = new Uint8Array(N);
  for (let c = 0; c < N; c++) {
    if (!boxCells[c] || goal[c]) continue;
    let ok = false;
    for (const t of distTables) if (t[c] < INF) { ok = true; break; }
    if (!ok) dead[c] = 1;
  }
  for (const b of startBoxes) if (dead[b]) return null;
  const h0 = matchCost(startBoxes, distTables);
  if (h0 >= INF) return null;

  const boxSet = new Set(startBoxes);
  const reach0 = flood(W, H, playerStart, (c) => playerCells[c] && !boxSet.has(c));
  let canon0 = Infinity;
  for (const c of reach0.cells) if (c < canon0) canon0 = c;
  if (canon0 === Infinity) return null;

  const keyOf = (boxes, canon) => boxes.join(',') + '|' + canon;
  const startKey = keyOf(startBoxes, canon0);
  const nodes = new Map();
  nodes.set(startKey, { boxes: startBoxes, player: playerStart, prev: null, push: null, g: 0 });
  const closed = new Map();
  closed.set(startKey, 0);
  const heap = [];
  let counter = 0;
  heapPush(heap, { key: startKey, f: h0, h: h0, n: counter++ });
  let expanded = 0;

  while (heap.length) {
    const cur = heapPop(heap);
    const node = nodes.get(cur.key);
    if (!node) continue;
    if (closed.get(cur.key) < node.g) continue;
    let done = true;
    for (const b of node.boxes) if (!goal[b]) { done = false; break; }
    if (done) {
      const pushes = [];
      let nd = node;
      while (nd.prev) { pushes.push(nd.push); nd = nodes.get(nd.prev); }
      pushes.reverse();
      return pushes;
    }
    if (++expanded > nodeCap) return null;
    const bset = new Set(node.boxes);
    const reach = flood(W, H, node.player, (c) => playerCells[c] && !bset.has(c));
    for (const b of node.boxes) {
      const bx = b % W, by = (b / W) | 0;
      for (let d = 0; d < 4; d++) {
        const stand = (by - DY[d]) * W + (bx - DX[d]);
        const dest = (by + DY[d]) * W + (bx + DX[d]);
        if (stand < 0 || stand >= N || dest < 0 || dest >= N) continue;
        if (!reach.seen[stand]) continue;
        if (!boxCells[dest] || bset.has(dest) || dead[dest]) continue;
        const nb = node.boxes.slice();
        nb[nb.indexOf(b)] = dest;
        nb.sort((a, z) => a - z);
        // 2x2 freeze: a 2x2 of boxes/walls holding a box off its goal.
        if (frozen(W, H, nb, bset, dest, boxCells, goal)) continue;
        const h = matchCost(nb, distTables);
        if (h >= INF) continue;
        const g = node.g + 1;
        const nbSet = new Set(nb);
        const nreach = flood(W, H, b, (c) => playerCells[c] && !nbSet.has(c));
        let canon = Infinity;
        for (const c of nreach.cells) if (c < canon) canon = c;
        if (canon === Infinity) continue;
        const key = keyOf(nb, canon);
        if (closed.has(key) && closed.get(key) <= g) continue;
        closed.set(key, g);
        nodes.set(key, { boxes: nb, player: b, prev: cur.key, push: { box: b, dir: d }, g });
        heapPush(heap, { key, f: g + h, h, n: counter++ });
      }
    }
  }
  return null;
}

// True when the box just moved to `dest` completes a 2x2 block of boxes or
// non-box cells in which some box is not on a goal (a frozen deadlock).
function frozen(W, H, boxes, _oldSet, dest, boxCells, goal) {
  const set = new Set(boxes);
  const dx = dest % W, dy = (dest / W) | 0;
  for (const ox of [-1, 0]) {
    for (const oy of [-1, 0]) {
      const x0 = dx + ox, y0 = dy + oy;
      if (x0 < 0 || y0 < 0 || x0 + 1 >= W || y0 + 1 >= H) continue;
      let allSolid = true;
      let badBox = false;
      for (const cx of [x0, x0 + 1]) {
        for (const cy of [y0, y0 + 1]) {
          const c = cy * W + cx;
          if (set.has(c)) { if (!goal[c]) badBox = true; }
          else if (boxCells[c]) allSolid = false;
        }
      }
      if (allSolid && badBox) return true;
    }
  }
  return false;
}

// Turn a push list into LURD moves, tracking the player from playerStart.
// passCells: where the player may walk (a membership predicate).
function pushesToMoves(W, H, pushes, startBoxes, playerStart, passCells) {
  const boxes = new Set(startBoxes);
  let player = playerStart;
  let out = '';
  for (const p of pushes) {
    const bx = p.box % W, by = (p.box / W) | 0;
    const stand = (by - DY[p.dir]) * W + (bx - DX[p.dir]);
    const path = walkPath(W, H, player, stand, (c) => passCells(c) && !boxes.has(c));
    if (path === null) return null;
    for (const d of path) out += DC[d];
    out += DC[p.dir];
    boxes.delete(p.box);
    boxes.add((by + DY[p.dir]) * W + (bx + DX[p.dir]));
    player = p.box;
  }
  return { moves: out, boxes, player };
}

// ------------------------------------------------------------- level solving

const ROOM_NODE_CAP = 600000;
const BOARD_NODE_CAP = 3000000;

function solveLevel(L) {
  const { W, H, wall, goal, goals, boxes, player, playerRow } = L;
  const N = W * H;
  if (boxes.length === 0) return '';
  if (boxes.length !== goals.length) return solveWholeBoard(L);

  // Hallway: the floor cells of the player's row.
  const hallway = new Uint8Array(N);
  for (let x = 0; x < W; x++) {
    const c = playerRow * W + x;
    if (!wall[c]) hallway[c] = 1;
  }
  // Rooms: floor components once the hallway cells are removed.
  const compId = new Int32Array(N).fill(-1);
  const comps = [];
  for (let c = 0; c < N; c++) {
    if (wall[c] || hallway[c] || compId[c] >= 0) continue;
    const f = flood(W, H, c, (n) => !wall[n] && !hallway[n] && compId[n] < 0);
    const id = comps.length;
    for (const cell of f.cells) compId[cell] = id;
    comps.push(f.cells);
  }
  const rooms = [];
  let transit = new Uint8Array(N);
  for (let id = 0; id < comps.length; id++) {
    const cells = comps[id];
    const cBoxes = boxes.filter((b) => compId[b] === id);
    const cGoals = goals.filter((g) => compId[g] === id);
    if (cBoxes.length === 0 && cGoals.length === 0) {
      for (const cell of cells) transit[cell] = 1;
      continue;
    }
    if (cBoxes.length === 0 || cBoxes.length !== cGoals.length) return solveWholeBoard(L);
    // Entry cells: hallway cells next to this component.
    const entries = [];
    const cellSet = new Set(cells);
    for (const cell of cells) {
      const x = cell % W, y = (cell / W) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d], ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        const n = ny * W + nx;
        if (hallway[n] && !entries.includes(n)) entries.push(n);
      }
    }
    if (entries.length === 0) return solveWholeBoard(L);
    entries.sort((a, b) => a - b);
    const boxCells = new Uint8Array(N);
    for (const cell of cells) boxCells[cell] = 1;
    const playerCells = new Uint8Array(N);
    for (const cell of cells) playerCells[cell] = 1;
    for (const e of entries) playerCells[e] = 1;
    const roomGoal = new Uint8Array(N);
    for (const g of cGoals) roomGoal[g] = 1;
    rooms.push({
      cells, cellSet, boxes: cBoxes, goals: cGoals, entries,
      boxCells, playerCells, goal: roomGoal,
    });
  }
  // Every box and goal must sit in some room.
  if (rooms.reduce((s, r) => s + r.boxes.length, 0) !== boxes.length) return solveWholeBoard(L);

  // Solve each room on its own, from its first entry cell.
  for (const r of rooms) {
    const pushes = solvePushSpace(W, H, r.goal, r.boxCells, r.playerCells, r.boxes, r.entries[0], ROOM_NODE_CAP);
    if (pushes === null) return solveWholeBoard(L);
    const conv = pushesToMoves(W, H, pushes, r.boxes, r.entries[0], (c) => r.playerCells[c] === 1);
    if (conv === null) return solveWholeBoard(L);
    r.moves = conv.moves;
    r.endPlayer = conv.player;
  }

  // Order the rooms so the player can always walk to the next entry:
  // try permutations (at most 4 rooms), the natural order first.
  const order = rooms.map((_, i) => i);
  const perms = permutations(order);
  for (const perm of perms) {
    const attempt = assemble(L, rooms, perm, transit);
    if (attempt !== null) return attempt;
  }
  return solveWholeBoard(L);
}

// Walk the player room to room, emitting walking moves plus each room's own
// move string. Returns the full move string, or null when some walk fails.
function assemble(L, rooms, perm, transit) {
  const { W, H, wall } = L;
  const boxesNow = new Set(L.boxes);
  let player = L.player;
  let out = '';
  const solved = new Set();
  for (const ri of perm) {
    const r = rooms[ri];
    const entry = r.entries[0];
    const path = walkPath(W, H, player, entry, (c) => !wall[c] && !boxesNow.has(c));
    if (path === null) return null;
    for (const d of path) out += DC[d];
    out += r.moves;
    // Apply the room's final box positions (its goals) and end player cell.
    for (const b of r.boxes) boxesNow.delete(b);
    for (const g of r.goals) boxesNow.add(g);
    player = r.endPlayer;
    solved.add(ri);
  }
  return out;
}

function permutations(arr) {
  if (arr.length <= 1) return [arr.slice()];
  const out = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = arr.slice(0, i).concat(arr.slice(i + 1));
    for (const p of permutations(rest)) out.push([arr[i]].concat(p));
  }
  return out;
}

// Whole-board push A* (fallback when the room split does not apply).
function solveWholeBoard(L) {
  const { W, H, wall, goal, boxes, player } = L;
  const N = W * H;
  const cells = new Uint8Array(N);
  for (let c = 0; c < N; c++) if (!wall[c]) cells[c] = 1;
  const pushes = solvePushSpace(W, H, goal, cells, cells, boxes, player, BOARD_NODE_CAP);
  if (pushes === null) return '';
  const conv = pushesToMoves(W, H, pushes, boxes, player, (c) => !wall[c]);
  return conv === null ? '' : conv.moves;
}

// --------------------------------------------------------------------- main

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solveLevel(parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8')));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
