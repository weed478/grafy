from data import runtests
import networkx as nx


def maximal_matching(E):
    matching = set()
    nodes = set()
    for u, v in E:
        if u not in nodes and v not in nodes and u != v:
            matching.add((u, v))
            nodes.add(u)
            nodes.add(v)
    return matching


def perfect_matching(E):
    G = nx.Graph()
    G.add_edges_from(E)
    return nx.max_weight_matching(G)


def extract_cycles(V, E):
    G = nx.Graph()
    G.add_nodes_from(range(1, V + 1))
    G.add_edges_from(E)
    return [list(nx.dfs_postorder_nodes(G.subgraph(c))) for c in nx.connected_components(G)]


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

    M = perfect_matching(E)

    # is not perfect => no solution
    if 2 * len(M) != V:
        return []

    # build cycle graph (made of disjoint cycles)
    E_out = []
    for m in M:
        if m[0][0] != m[1][0]:
            E_out.append((m[0][0], m[1][0]))

    # extract list of cycles in graph
    return extract_cycles(V_in, E_out)


runtests(my_solve)
