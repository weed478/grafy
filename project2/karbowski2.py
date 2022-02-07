from data import runtests
import networkx as nx
import matplotlib.pyplot as plt


def plot_graph(V, E):
    G = nx.Graph()
    G.add_nodes_from(V)
    G.add_edges_from(E)
    nx.draw(G)
    plt.show()


def maximal_matching(E):
    matching = set()
    nodes = set()
    for u, v in E:
        if u not in nodes and v not in nodes and u != v:
            matching.add((u, v))
            nodes.add(u)
            nodes.add(v)
    return matching


def my_solve_nx(V_in, E_in):

    G_in = nx.Graph()
    G_in.add_nodes_from(range(1, V_in + 1))
    G_in.add_edges_from(E_in)

    G = nx.Graph()
    for v_in in range(1, V_in + 1):
        for d in range(G_in.degree[v_in]):
            G.add_node((v_in, 1, d))
        for d in range(G_in.degree[v_in] - 2):
            G.add_node((v_in, 2, d))

        for d1 in range(G_in.degree[v_in]):
            v = (v_in, 1, d1)
            for d2 in range(G_in.degree[v_in] - 2):
                u = (v_in, 2, d2)
                G.add_edge(v, u)

    used_degree = [0] * (V_in + 1)

    for e_in in E_in:
        v_in = e_in[0]
        u_in = e_in[1]

        v = (v_in, 1, used_degree[v_in])
        used_degree[v_in] += 1

        u = (u_in, 1, used_degree[u_in])
        used_degree[u_in] += 1

        G.add_edge(v, u)

    M = nx.maximal_matching(G)
    if not nx.is_perfect_matching(G, M):
        return []

    G_out = nx.Graph()
    G_out.add_nodes_from(range(1, V_in + 1))
    for m in M:
        if m[0][0] != m[1][0]:
            G_out.add_edge(m[0][0], m[1][0])

    return [list(nx.dfs_postorder_nodes(G_out.subgraph(c))) for c in nx.connected_components(G_out)]


def my_solve(V_in, E_in):

    degree = [0] * (V_in + 1)
    for e in E_in:
        degree[e[0]] += 1
        degree[e[1]] += 1

    V = []
    E = []
    for v_in in range(1, V_in + 1):
        for d in range(degree[v_in]):
            V.append((v_in, 1, d))
        for d in range(degree[v_in] - 2):
            V.append((v_in, 2, d))

        for d1 in range(degree[v_in]):
            v = (v_in, 1, d1)
            for d2 in range(degree[v_in] - 2):
                u = (v_in, 2, d2)
                E.append((v, u))

    used_degree = [0] * (V_in + 1)

    for e_in in E_in:
        v_in = e_in[0]
        u_in = e_in[1]

        v = (v_in, 1, used_degree[v_in])
        used_degree[v_in] += 1

        u = (u_in, 1, used_degree[u_in])
        used_degree[u_in] += 1

        E.append((v, u))

    M = maximal_matching(E)
    E_out = []
    for m in M:
        if m[0][0] != m[1][0]:
            E_out.append((m[0][0], m[1][0]))
    print(E_out)
    print(len(M), len(E))

    return []


runtests(my_solve_nx)

# print(my_solve_nx(6, [
#     (1, 2),
#     (1, 3),
#     (2, 3),
#     (2, 4),
#     (3, 5),
#     (4, 5),
#     (4, 6),
#     (5, 6)
# ]))
