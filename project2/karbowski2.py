from data import runtests
from time import time
from random import shuffle


def maximal_matching(E):
    matching = []
    used = set()
    for u, v in E:
        if u not in used and v not in used:
            matching.append((u, v))
            used.add(u)
            used.add(v)
    return matching


def is_perfect_matching(M, V):
    return 2 * len(M) == V


def xd_matching(V, E):
    stop = time() + 0.9
    while time() < stop:
        shuffle(E)
        M = maximal_matching(E)
        if is_perfect_matching(M, V):
            return M
    return []


def perfect_matching(V, E):
    # import networkx as nx
    # G = nx.Graph()
    # G.add_edges_from(E)
    # return nx.max_weight_matching(G)
    return xd_matching(V, E)


def extract_cycles(V, E):
    # build adj list
    G = [[] for _ in range(V + 1)]
    for e in E:
        G[e[0]].append(e[1])
        G[e[1]].append(e[0])

    # output cycles
    cycles = []

    visited = [False] * (V + 1)

    # for every vertex not added to output
    for s in range(1, V + 1):
        if visited[s]:
            continue

        # local cycle
        C = [s]
        visited[s] = True

        # set direction of cycle v -> u
        v = s
        u = G[s][0]

        # go around the cycle
        while u != s:
            C.append(u)
            visited[u] = True
            # continue in the same direction (v -> u)
            u, v = G[u][1] if G[u][0] == v else G[u][0], u

        cycles.append(C)

    return cycles


def my_solve(V_in, E_in):

    # calculate vertex degrees
    degree = [0] * (V_in + 1)
    for e in E_in:
        degree[e[0]] += 1
        degree[e[1]] += 1

    # generate extended graph
    V = 0
    E = []
    for v_in in range(1, V_in + 1):
        d = degree[v_in]

        # vertex is the end of a path or is isolated => no solution
        if d < 2:
            return []

        # count new vertices
        V += 2 * d - 2

        # add K_{d,d-2} edges
        for d1 in range(d):
            v = (v_in, 1, d1)
            for d2 in range(d - 2):
                u = (v_in, 2, d2)
                E.append((v, u))

    # connections between K_{d,d-2} graphs are made on "d" side
    # count used vertices on "d" side
    used_degree = [0] * (V_in + 1)

    # connect K graphs
    for e_in in E_in:
        # original edge
        v_in = e_in[0]
        u_in = e_in[1]

        # corresponding vertex in extended graph
        v = (v_in, 1, used_degree[v_in])
        used_degree[v_in] += 1

        u = (u_in, 1, used_degree[u_in])
        used_degree[u_in] += 1

        E.append((v, u))

    M = perfect_matching(V, E)

    # is not perfect => no solution
    if not is_perfect_matching(M, V):
        return []

    # build cycle graph (made of disjoint cycles)
    E_out = []
    for m in M:
        if m[0][0] != m[1][0]:
            E_out.append((m[0][0], m[1][0]))

    # extract list of cycles in graph
    return extract_cycles(V_in, E_out)


runtests(my_solve)
