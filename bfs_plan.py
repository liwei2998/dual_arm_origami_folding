#!/usr/bin/env python
"""
Input consists of a simple graph of { node: [list of neighbors] } plus a source and target node.
"""

from collections import deque
import origami_state_generation as osg

facets1 = {"1":[[[1,1],[0,0]],[[0,0],[-1,1]]],
          "2":[[[1,1],[0,0]],[[0,0],[1,-1]]],
          "3":[[[0,0],[1,-1]],[[0,0],[-1,-1]]],
          "4":[[[0,0],[-1,-1]],[[0,0],[-1,1]]]}

polygen1 = {"1":[[1,1],[0,0],[-1,1]],
            "2":[[1,1],[1,-1],[0,0]],
            "3":[[1,-1],[0,0],[-1,-1]],
            "4":[[0,0],[-1,-1],[-1,1]]}
stack1 = [["1","2","3","4"]]
state1 = {"stack":stack1,"polygen":polygen1,"facet_crease":facets1}
state_dict = {"state1":state1}
state_graph = {"state1":[]}

def bfs(state_graph, src, tgt_stack):
    """Return the shortest path from the source (src) to the target (tgt) in the graph"""

    if src in state_graph is False:
        raise AttributeError("The source '%s' is not in the graph" % src)


    parents = {src: None}
    queue = deque([src])
    k = 2
    # #dynamic generate variable's names, using locals()
    # names = locals()
    while queue:
        node = queue.popleft()
        print "node",node
        state_node = state_dict[node]
        # generate children states for this node
        children_states = osg.generateNextLayerStates(state_node)
        for i in range(len(children_states)):
            # names["state"+str(k)] = children_states[i]
            #store each children states
            state_dict["state"+str(k)] = children_states[i]
            #add children states to state_graph
            state_graph.setdefault(node,[]).append("state"+str(k))
            k += 1
        for neighbor in state_graph[node]:
            if neighbor not in parents:
                parents[neighbor] = node
                queue.append(neighbor)
                if state_node["stack"] == tgt_stack:
                    break
    path = [node]
    while parents[node] is not None:
        path.insert(0, parents[node])
        node = parents[node]

    return path

path = bfs(state_graph,"state1",[['3'],['4'],['1'],['2']])
print "path",path
