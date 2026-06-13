# graph-astar

A Rust implementation of the A\* (A-star) shortest-path algorithm for weighted graphs. Given a graph with non-negative edge weights and an admissible heuristic, A\* finds the optimal (minimum-cost) path from a start node to a goal node. This implementation uses a binary heap priority queue with lazy deletion for efficiency.

## Why It Matters

Pathfinding is one of the most fundamental problems in computer science. A\* is the gold standard: it is **optimal** (always finds the shortest path if the heuristic is admissible) and **optimal in efficiency** (no algorithm with the same heuristic explores fewer nodes). It powers GPS navigation, game AI, network routing, robot motion planning, and puzzle solving. Dijkstra's algorithm is the special case of A\* with h(n) = 0 for all n.

The key insight: A\* is a **best-first search** that ranks nodes by `f(n) = g(n) + h(n)` where `g(n)` is the exact cost from start to n and `h(n)` is the estimated cost from n to goal. The heuristic guides the search toward the goal, dramatically reducing the search space compared to uniform-cost Dijkstra.

## How It Works

### Algorithm

```
OPEN  = priority queue ordered by f(n) = g(n) + h(n)
g[start] = 0
push (h[start], start) onto OPEN

while OPEN is not empty:
    pop node u with minimum f(u)
    if u == goal: return g[goal]         ← optimal cost found
    for each neighbor (v, w) of u:
        tentative_g = g[u] + w
        if tentative_g < g[v]:
            g[v] = tentative_g
            push (g[v] + h[v], v) onto OPEN  ← lazy deletion
return failure                            ← goal unreachable
```

### Admissibility and Optimality

A heuristic h is **admissible** if it never overestimates the true cost:

```
h(n) ≤ h*(n)   ∀n
```

where h*(n) is the true shortest distance from n to goal. With an admissible heuristic, A\* is guaranteed to find the optimal path.

A heuristic is **consistent** (monotone) if:

```
h(n) ≤ c(n, n') + h(n')   for every edge (n, n')
```

Consistency implies admissibility. With a consistent heuristic, each node is expanded at most once (no re-expansion needed). Common consistent heuristics:

| Domain | Heuristic | Formula |
|--------|-----------|---------|
| Grid (4-connected) | Manhattan | h = |x₁−x₂| + |y₁−y₂| |
| Grid (8-connected) | Chebyshev | h = max(|x₁−x₂|, |y₁−y₂|) |
| Euclidean space | Euclidean | h = √(Δx² + Δy²) |
| General | Dijkstra | h = 0 |

### Time Complexity

| Component | Complexity |
|-----------|------------|
| Priority queue operations | O(E log V) total (each edge may push) |
| Relaxation checks | O(E) |
| **Total** | **O((V + E) log V)** |

where V = number of vertices, E = number of edges. With a perfect heuristic (h = h\*), A\* explores only nodes on the optimal path: O(L) where L = path length. With h = 0, it degenerates to Dijkstra's O((V + E) log V).

### Space Complexity

O(V) for the g-score array and priority queue.

### Implementation: Lazy Deletion

This implementation uses **lazy deletion**: instead of decreasing keys in the heap (which requires an indexed priority queue), it pushes duplicate entries and skips stale ones. When a node is popped, its current f-value may differ from g[u] + h[u] if a shorter path was found after the entry was pushed. The algorithm simply processes the most recent g-score. This trades slightly more heap operations for simpler code.

## Quick Start

```rust
// Graph: adjacency list with (neighbor, weight) pairs
let graph: Vec<Vec<(usize, u64)>> = vec![
    vec![(1, 1), (2, 4)],  // Node 0 → Node 1 (cost 1), Node 2 (cost 4)
    vec![(2, 2)],           // Node 1 → Node 2 (cost 2)
    vec![],                 // Node 2 (goal)
];

// Heuristic: estimated distance to goal (node 2)
let heuristic: Vec<u64> = vec![3, 2, 0];

let cost = astar(&graph, 0, 2, &heuristic);
assert_eq!(cost, Some(3)); // 0 → 1 → 2: cost 1 + 2 = 3
```

## API

### `astar(graph, start, goal, heuristic)`

| Parameter | Type | Description |
|-----------|------|-------------|
| `graph` | `&Vec<Vec<(usize, u64)>>` | Adjacency list: graph[u] = [(v, weight)] |
| `start` | `usize` | Source vertex index |
| `goal` | `usize` | Target vertex index |
| `heuristic` | `&Vec<u64>` | h[v] = estimated cost from v to goal |
| **Returns** | `Option<u64>` | `Some(cost)` if path exists, `None` otherwise |

**Precondition**: All edge weights must be non-negative. Heuristic must be admissible for optimality.

## Architecture Notes

A\* is a **γ** (search strategy) applied to a **graph** (**η**) to produce a **path** (**C**). The heuristic h is what makes A\* more than Dijkstra: it injects domain knowledge into the search, biasing exploration toward the goal. The quality of C depends on the quality of h — a perfect heuristic gives O(L) performance, a zero heuristic gives O(E log V). This is the γ + η = C principle in action: the algorithm (γ) and the problem structure (η) together determine the cost of the solution (C). Better heuristics → less search → faster answers.

### Lazy Deletion Tradeoff

With lazy deletion, the heap may contain up to O(E) entries (one per edge relaxation). This is asymptotically the same as indexed priority queue approaches. The constant factor is slightly worse due to stale entries, but the implementation is simpler and cache-friendly.

## References

- **A\* original paper**: Hart, P. E., Nilsson, N. J., & Raphael, B. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics* 4.2 (1968): 100–107.
- **Admissibility and consistency**: Russell, S., & Norvig, P. *Artificial Intelligence: A Modern Approach.* 4th ed., Pearson, 2020. Chapter 3.
- **Lazy deletion analysis**: Edelkamp, S., & Schrödl, S. *Heuristic Search: Theory and Applications.* Morgan Kaufmann, 2012.
- **Dijkstra's algorithm (h=0 special case)**: Dijkstra, E. W. "A note on two problems in connexion with graphs." *Numerische Mathematik* 1 (1959): 269–271.

## License

MIT
