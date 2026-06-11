use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

fn astar(
    graph: &Vec<Vec<(usize, u64)>>,
    start: usize,
    goal: usize,
    heuristic: &Vec<u64>,
) -> Option<u64> {
    let n = graph.len();
    let mut g_score = vec![u64::MAX; n];
    let mut heap = BinaryHeap::new();
    g_score[start] = 0;
    heap.push(Reverse((heuristic[start], start)));
    while let Some(Reverse((_, u))) = heap.pop() {
        if u == goal { return Some(g_score[u]); }
        for &(v, w) in &graph[u] {
            let ng = g_score[u] + w;
            if ng < g_score[v] {
                g_score[v] = ng;
                heap.push(Reverse((ng + heuristic[v], v)));
            }
        }
    }
    None
}

fn main() {
    let graph = vec![vec![(1, 1), (2, 4)], vec![(2, 2)], vec![]];
    let heuristic = vec![3, 2, 0];
    if let Some(cost) = astar(&graph, 0, 2, &heuristic) {
        println!("A* cost: {}", cost);
    }
}
