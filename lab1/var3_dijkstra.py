from queue import PriorityQueue
from math import inf


def to_adj(V, E):
    G = [[] for _ in range(V)]

    for (u, v, w) in E:
        G[u - 1].append((v - 1, w))
        G[v - 1].append((u - 1, w))

    return G


def find_max_min_edge(V, E):
    adj = to_adj(V, E)

    Q = PriorityQueue()
    d = [-inf] * V
    visited = [False] * V
    d[0] = inf

    for u in range(V):
        Q.put((-d[u], u))

    while not Q.empty():
        _, u = Q.get()
        if visited[u]:
            continue
        visited[u] = True

        for (v, w_uv) in adj[u]:
            if min(d[u], w_uv) > d[v]:
                d[v] = min(d[u], w_uv)
                Q.put((-d[v], v))

    return d[1]
