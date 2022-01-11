import os
from graph import load_and_build_graph
from check_lex import check_lex_bfs


class SetListNode:
    def __init__(self, S, prev, next):
        self.S = S
        self.prev = prev
        self.next = next


def lex_bfs(G):
    V = len(G)
    set_list_root = SetListNode(set(range(V)), None, None)
    set_of_vertex = dict([(i, set_list_root) for i in range(V)])
    output = []

    def pop_from_first_set():
        nonlocal set_list_root
        v = set_list_root.S.pop()
        set_of_vertex[v] = None
        if len(set_list_root.S) == 0:
            set_list_root = set_list_root.next
        return v

    def add_to_output(v):
        output.append(v)

    def edges_from(v):
        for u in G[v].out:
            if set_of_vertex[u] is not None:
                yield u, set_of_vertex[u]

    def move_from_to(w, S, T):
        nonlocal set_list_root

        S.S.remove(w)
        if len(S.S) == 0:
            if S is not set_list_root:
                S.prev.next = S.next
                if S.next is not None:
                    S.next.prev = S.prev
            else:
                set_list_root = set_list_root.next
                if set_list_root is not None:
                    set_list_root.prev = None
        T.S.add(w)
        set_of_vertex[w] = T

    def add_set_before(S, T):
        nonlocal set_list_root

        if T is not set_list_root:
            S.prev = T.prev
            S.next = T
            T.prev.next = S
            T.prev = S
        else:
            S.next = set_list_root
            set_list_root.prev = S
            set_list_root = S

    def get_set_before(S):
        return S.prev

    while set_list_root is not None:
        v = pop_from_first_set()

        add_to_output(v)

        replaced = set()

        for w, S in edges_from(v):
            if id(S) not in replaced:
                T = SetListNode(set(), None, None)
                add_set_before(T, S)
                replaced.add(id(S))
            else:
                T = get_set_before(S)

            move_from_to(w, S, T)

    return output


if __name__ == "__main__":
    for graph_dir in os.listdir("./graphs-lab4"):
        graph_dir = f"./graphs-lab4/{graph_dir}"
        for graph in os.listdir(graph_dir):
            graph_path = f"{graph_dir}/{graph}"
            G = load_and_build_graph(graph_path)
            vs = lex_bfs(G)
            if not check_lex_bfs(G, vs):
                print(f"[FAIL] {graph} {vs}")
    print("Test done")
