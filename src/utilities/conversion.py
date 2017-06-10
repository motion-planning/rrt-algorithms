# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

def convert_edge_set_to_dict(E: set) -> dict:
    """
    Utility function to convert a set of edges into a dict that A* can use
    :param E: set of all edges: {(a, b), ... }
    :return: dict of edge connections: {a: [b, c, ...], ...]
    """
    g = {}
    for e in E:
        parent, child = e[0], e[1]
        if parent in g:
            g[parent].append(child)
        else:
            g[parent] = [child]

    return g
