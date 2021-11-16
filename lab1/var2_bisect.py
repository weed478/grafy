from collections import deque

def toAdj(V, E):
    G = [[] for _ in range(V)]

    for (u, v, w) in E:
        G[u - 1].append((v - 1, w))
        G[v - 1].append((u - 1, w))

    return G

def findMaxMinEdge(V, E):
    adj = toAdj(V, E)
    minW, maxW = min(map(lambda e: e[2], E)), max(map(lambda e: e[2], E))

    def hasPathWith(minW):
        visited = [False] * V
        Q = deque()
        Q.append(0)

        while len(Q) > 0:
            u = Q.popleft()
            for (v, _) in filter(lambda uv: uv[1] >= minW, adj[u]):
                if not visited[v]:
                    visited[v] = True
                    Q.append(v)

        return visited[1]

    L = minW
    R = maxW

    if (hasPathWith(R)):
        return R

    if not hasPathWith(L):
        return None

    while R - L > 1:
        M = (R + L) // 2
        if hasPathWith(M):
            L = M
        else:
            R = M

    return L
