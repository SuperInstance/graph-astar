# A* Pathfinding Algorithm

**A Rust implementation of the A\* (A-star) shortest-path algorithm** using a binary heap priority queue with heuristic-guided exploration, finding optimal paths in weighted graphs.

## Why It Matters

A* is the most widely used pathfinding algorithm in production systems: GPS navigation (Google Maps), video game AI (Unity NavMesh), network routing, and robot motion planning (ROS). It combines the exactness of Dijkstra's algorithm with the speed of greedy best-first search by using a heuristic function `h(n)` that estimates the distance to the goal. With an **admissible** heuristic (never overestimates), A* guarantees the optimal shortest path while exploring far fewer nodes than Dijkstra. With a **consistent** heuristic, each node is settled at most once, giving **O((V+E) log V)** performance.

## How It Works

The algorithm maintains a priority queue (min-heap via `BinaryHeap<Reverse<(f, node)>>`) ordered by `f(n) = g(n) + h(n)`, where `g(n)` is the known cost from start to `n` and `h(n)` is the estimated cost from `n` to the goal. At each step, it pops the node with lowest `f` and relaxes its outgoing edges. If a neighbor's tentative `g` value is lower than its current value, the neighbor is pushed onto the heap.

The key insight: with an admissible heuristic, the first time the goal is popped from the heap, its `g` value is the optimal path cost. The heuristic prunes exploration by prioritizing nodes that appear to lead toward the goal — a good heuristic can reduce the explored area from the entire graph (Dijkstra) to a narrow corridor.

The graph is represented as an adjacency list: `Vec<Vec<(usize, u64)>>` where each entry is `(neighbor, weight)`.

## Quick Start

```rust
// The crate is a binary — run it to see A* in action:
// cargo run

// Graph: 0 --1--> 1 --2--> 2
//              0 --4--> 2
// Heuristic: [3, 2, 0]
// A* finds: 0 → 1 → 2 with cost 3 (not 0 → 2 with cost 4)
```

## API

The implementation is in `main.rs` as a reference. Key function:

| Function | Complexity | Description |
|---|---|---|
| `astar(graph, start, goal, heuristic)` | **O((V+E) log V)** | Find shortest path cost with admissible heuristic |

## Architecture Notes

Part of the SuperInstance graph algorithms collection. Companion crates: `graph-bfs`, `graph-dfs`, `graph-dijkstra`, `graph-bellman-ford`, `graph-coloring`. See the [Architecture Guide](https://github.com/SuperInstance/SuperInstance/blob/main/ARCHITECTURE.md).

## License

MIT
