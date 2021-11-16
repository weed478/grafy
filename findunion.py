class Node:
    def __init__(self, value):
        self.value = value
        self.parent = self
        self.rank = 0


def find(node):
    if node is not node.parent:
        node.parent = find(node.parent)
    return node.parent


def union(A, B):
    rep1 = find(A)
    rep2 = find(B)

    if rep1 == rep2:
        return

    if rep1.rank > rep2.rank:
        rep2.parent = rep1

    elif rep1.rank < rep2.rank:
        rep1.parent = rep2

    else:
        rep2.parent = rep1
        rep1.rank += 1


if __name__ == "__main__":
    A = Node('A')
    B = Node('B')
    C = Node('C')
    D = Node('D')
    E = Node('E')

    print(False, find(A) == find(B))
    union(A, B)
    union(A, C)
    union(D, E)

    print(True, find(B) == find(C))
    print(True, find(D) == find(E))
    print(False, find(A) == find(E))

    union(A, E)
    print(True, find(B) == find(D))

