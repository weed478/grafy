from dimacs import loadDirectedWeightedGraph, readSolution
from collections import deque
from math import inf
import os


def to_dict(V, E):
    G = [dict() for _ in range(V)]

    for (u, v, w) in E:
        G[u - 1][v - 1] = w

    return G


def init_residual(G):
    for u in range(len(G)):
        for v in G[u]:
            if u not in G[v]:
                G[v][u] = 0


def bfs(G, s, t):
    V = len(G)
    visited = [False] * V
    parent = [None] * V
    Q = deque()

    visited[s] = True
    parent[s] = s
    Q.append(s)

    while len(Q) > 0:
        u = Q.popleft()
        for (v, w) in G[u].items():
            if w > 0 and not visited[v]:
                visited[v] = True
                parent[v] = u
                Q.append(v)

    def get_path(u):
        if parent[u] == u:
            return [u]
        return get_path(parent[u]) + [u]

    if parent[t] is None:
        return None

    return get_path(t)


def find_path_flow(G, path):
    flow = inf
    u = path[0]
    for v in path[1:]:
        flow = min(flow, G[u][v])
        u = v
    return flow


def find_max_flow(G, s=None, t=None):
    find_path = bfs
    init_residual(G)

    if s is None:
        s = 0
    if t is None:
        t = V - 1

    max_flow = 0

    while True:
        path = find_path(G, s, t)
        if path is None:
            break
        flow = find_path_flow(G, path)
        u = path[0]
        for v in path[1:]:
            G[u][v] -= flow
            G[v][u] += flow
            u = v
        max_flow += flow

    return max_flow


if __name__ == "__main__":
    for graph in os.listdir("./flow"):
        graph_path = "./flow/" + graph
        V, E = loadDirectedWeightedGraph(graph_path)
        sol = int(readSolution(graph_path))
        G = to_dict(V, E)
        max_flow = find_max_flow(G)
        if sol != max_flow:
            print("ERR", graph_path, "expected", sol, "got", max_flow)
        else:
            print("OK", graph_path)
