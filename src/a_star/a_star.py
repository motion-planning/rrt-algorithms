# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from src.a_star.heuristics import heuristic
from src.utilities.geometry import distance_between_points
from src.utilities.queues import PriorityQueue


def a_star_search(graph: dict, start: tuple, goal: tuple) -> (dict, dict):
    """
    A* Graph Search
    Based on: http://www.redblobgames.com/pathfinding/a-star/implementation.html#python-astar
    which was released under an MIT license: http://www.redblobgames.com/
    :param graph: graph in the form vertex: [neighbor_1, neighbor_2, ...]
    :param start: starting vertex
    :param goal: ending vertex
    :return: dict showing where each vertex came from (explored), dict of cost-to-come
    """
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        if current in graph:
            for neighbor in graph[current]:
                # noinspection PyTypeChecker
                new_cost = cost_so_far[current] + distance_between_points(current, neighbor)

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    frontier.put(neighbor, priority)
                    came_from[neighbor] = current

    return came_from, cost_so_far


def reconstruct_path(came_from: dict, start: tuple, goal: tuple) -> list:
    """
    Reconstruct path from start to goal
    :param came_from: graph in the form: vertex: came_from, vertex: came_from
    :param start: starting location
    :param goal: goal location
    :return: sequence of vertices from start to goal
    """
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


def a_star_path(graph: dict, start: tuple, goal: tuple) -> list:
    """

    :param graph: graph in the form vertex: [neighbor_1, neighbor_2, ...]
    :param start: starting vertex
    :param goal: ending vertex
    :return: path from start to goal
    """
    came_from, cost_so_far = a_star_search(graph, start, goal)
    path = reconstruct_path(came_from, start, goal)

    return path
