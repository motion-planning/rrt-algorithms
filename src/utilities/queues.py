import heapq


class PriorityQueue(object):
    def __init__(self):
        """
        Initialize Priority Queue
        """
        self.elements = []

    def empty(self) -> bool:
        """
        Check if self is empty
        :return: True if empty, False otherwise
        """
        return len(self.elements) == 0

    def put(self, item: object, priority: float):
        """
        Put an item on the Queue with given Priority
        :param item: item to push
        :param priority: priority
        """
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> object:
        """
        Get the top item from the Priority Queue
        :return: item with the highest priority
        """
        return heapq.heappop(self.elements)[1]
