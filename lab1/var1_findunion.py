from findunion import Node, find, union


def findMaxMinEdge(V, E):
    nodes = [None] + [Node(i + 1) for i in range(V)]

    E.sort(key=lambda e: e[2], reverse=True)

    for e in E:
        union(nodes[e[0]], nodes[e[1]])
        if find(nodes[1]) is find(nodes[2]):
            return e[2]
