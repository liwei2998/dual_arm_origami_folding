#!/usr/bin/env python
"""
Input consists of a simple graph of { node: [list of neighbors] } plus a source and target node.
"""

from collections import deque

def bfs(graph, src, tgt):
    """Return the shortest path from the source (src) to the target (tgt) in the graph"""

    if src in graph is False:
        raise AttributeError("The source '%s' is not in the graph" % src)
    if tgt in graph is False:
        raise AttributeError("The target '%s' is not in the graph" % tgt)

    parents = {src: None}
    queue = deque([src])
    while queue:
        node = queue.popleft()
        print "node",node
        for neighbor in graph[node]:
            if neighbor not in parents:
                parents[neighbor] = node
                queue.append(neighbor)
                if node == tgt:
                    break

    path = [tgt]
    while parents[tgt] is not None:
        path.insert(0, parents[tgt])
        tgt = parents[tgt]

    return path


"""
Test using the graph from:
https://commons.wikimedia.org/wiki/File:Graph_with_Chordless_and_Chorded_Cycles.svg
"""

test = {
    "a": ["b", "f"],
    "b": ["a", "c", "g"],
    "c": ["b", "d", "g", "l"],
    "d": ["c", "e", "k"],
    "e": ["d", "f"],
    "f": ["a", "e"],
    "g": ["b", "c", "h", "l"],
    "h": ["g", "i"],
    "i": ["h", "j", "k"],
    "j": ["i", "k"],
    "k": ["d", "i", "j", "l"],
    "l": ["c", "g", "k"],
}

assert bfs(test, "a", "e") == ['a', 'f', 'e']
assert bfs(test, "a", "i") == ['a', 'b', 'g', 'h', 'i']
assert bfs(test, "a", "l") == ['a', 'b', 'c', 'l']
assert bfs(test, "b", "k") == ['b', 'c', 'd', 'k']
