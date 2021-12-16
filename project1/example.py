from data import runtests
from math import inf
from collections import deque


class ResNet:
    def __init__(self, V, E, max_flow):
        V += 1

        self.connections = [set() for _ in range(V)]
        self.edge_losses = dict()
        self.current_flow = dict()

        self.connections[0].add(1)
        self.edge_losses[(0, 1)] = [0] * max_flow
        self.current_flow[(0, 1)] = 0
        self.current_flow[(1, 0)] = 0

        for (u, v), losses in E:
            self.connections[u].add(v)
            self.connections[v].add(u)
            self.edge_losses[(u, v)] = losses
            self.current_flow[(u, v)] = 0
            self.current_flow[(v, u)] = 0

    def __len__(self):
        return len(self.connections)

    def get_losses(self):
        losses = 0
        for (u, v), flow in self.current_flow.items():
            assert flow == -self.current_flow[(v, u)]
            if flow > 0:
                losses += self.edge_losses[(u, v)][flow - 1]
        return losses

    def residual_flows_and_costs(self, u):
        for v in self.connections[u]:
            flow = self.current_flow[(u, v)]
            losses = self.edge_losses.get((u, v), None)

            if flow == 0 and losses is not None:
                capacity = len(losses)
                yield v, capacity - flow, losses[0]
            elif 0 < flow < len(losses):
                capacity = len(losses)
                old_w = losses[flow - 1]
                new_w = losses[flow]
                delta_w = new_w - old_w
                yield v, capacity - flow, delta_w
            elif flow < -1:
                losses = self.edge_losses[(v, u)]
                old_w = losses[-flow - 1]
                new_w = losses[-flow - 2]
                delta_w = new_w - old_w
                yield v, -flow, delta_w
            elif flow < 0:
                losses = self.edge_losses[(v, u)]
                yield v, -flow, -losses[0]

    def cost_edges(self):
        for u in range(len(self)):
            for v, flow, w in self.residual_flows_and_costs(u):
                yield u, v, w

    def residual_flows(self, u):
        for v, flow, w in self.residual_flows_and_costs(u):
            yield v, flow


def backtrack_path(parents, s):
    if parents[s] is None:
        return [s]
    return backtrack_path(parents, parents[s]) + [s]


def bellman_ford(E, n, s):
    parent = [None] * n
    d = [inf] * n
    d[s] = 0

    def relax(u, v, w):
        if d[u] + w < d[v]:
            d[v] = d[u] + w
            parent[v] = u

    for i in range(n - 1):
        for (u, v, w) in E:
            relax(u, v, w)

    for (u, v, w) in E:
        if d[u] + w < d[v]:
            d[v] = d[u] + v
            parent[v] = u
            C = v
            for _ in range(n):
                C = parent[C]
            cycle = []
            v = C
            while True:
                cycle.append(v)
                if v == C and len(cycle) > 1:
                    break
                v = parent[v]
            cycle.reverse()
            return cycle

    return None


def find_negative_cycle(G, s):
    return bellman_ford(list(G.cost_edges()), len(G), s)


def bfs(G, s, t):
    V = len(G)
    visited = [False] * V
    parent = [None] * V
    Q = deque()

    visited[s] = True
    Q.append(s)

    while len(Q) > 0:
        u = Q.popleft()
        for v, w in G.residual_flows(u):
            if w > 0 and not visited[v]:
                visited[v] = True
                parent[v] = u
                Q.append(v)

    if parent[t] is None:
        return None

    return backtrack_path(parent, t)


def find_path_flow(G, path):
    flow = inf
    u = path[0]
    for v in path[1:]:
        flow = min(flow, len(G.edge_losses[(u, v)]) - G.current_flow[(u, v)])
        u = v
    return flow


def run_flow(G, path, flow):
    u = path[0]
    for v in path[1:]:
        G.current_flow[(u, v)] += flow
        G.current_flow[(v, u)] -= flow
        u = v


def ford_fulkerson(G, s, t):
    max_flow = 0

    while True:
        path = bfs(G, s, t)
        if path is None:
            break
        flow = find_path_flow(G, path)
        run_flow(G, path, flow)
        max_flow += flow

    return max_flow


def find_max_flow(G, s, t):
    return ford_fulkerson(G, s, t)


def min_cost_max_flow(G, s, t):
    find_max_flow(G, s, t)

    while True:
        cycle = find_negative_cycle(G, s + 1)
        if cycle is None:
            break
        run_flow(G, cycle, 1)

    return G.get_losses()


def my_solve(V, k, edges):
    G = ResNet(V, edges, k)
    return min_cost_max_flow(G, 0, len(G) - 1)


runtests(my_solve)
