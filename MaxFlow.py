import numpy as np


class Graph (object):

    def __init__(self, data=None):
        n, m = np.shape(data)
        assert n == m, "data must be square"

        # edge properties
        self.capacity = data
        self.flow = np.zeros((n, n), dtype=np.int)

        self.size = n

        # vertices properties
        self.vertices = np.array([i for i in range(self.size)])
        self.excess = [0] * self.size
        self.distance = [0] * self.size
        self.level = [-1] * self.size
        self.seen = [0] * self.size

        self.source = 0
        self.sink = self.size - 1

    def _search(self, traverse, origin=None, goal=None, DFS=False):

        # set path origin and goal indexes
        goal = goal or self.sink
        origin = origin or self.source

        # avoid double visit
        visited = [False] * self.size

        # a queue that states the next indexes to visit, starting with origin
        to_visit = [origin]

        # mark origin as visited
        visited[origin] = True

        # to determine the edge; predecessor:u 
        predecessor = origin

        while to_visit:

            # pops the next index to visit, base of the search method chosen
            if DFS:
                current = to_visit.pop(0)
            # BFS
            else:  
                current = to_visit.pop()

            # iterates on all possible steps
            for successor in range(self.size):

                # a viable next step is one that was not visited and that its 
                # residual is possitive
                if (not visited[successor]) and \
                (self.capacity[(current, successor)] \
                    - self.flow[(current, successor)] > 0):

                    # mark as visited and add to the queue to explore
                    visited[successor] = True
                    to_visit.append(successor)

                    # adds to the traverse
                    traverse[successor] = current

        return visited[goal]

    def _max_flow_search_FF(self, origin=None, goal=None, data=False,\
     DFS=False):

        goal = goal or self.sink
        origin = origin or self.source

        #extendend information 
        presented_data = {}

        # initiate vars
        traverse = [-1] * self.size
        max_flow = 0
        iter_count = 0
        path_flow = float('inf')

        # before augmented path were exhusted 
        while self._search(traverse, origin, goal, DFS):

            iter_count += 1
            presented_data[iter_count] = []

            # walk backwards, find the smallest viable flow 
            # for an augmented path
            current = goal
            while current != origin :

                presented_data[iter_count].append(current)
                pred = traverse[current]
                residual_flow = (self.capacity[(pred, current)] - \
                    self.flow[(pred, current)])
                path_flow = min(path_flow, residual_flow)

                current = pred

            # add the current path flow to the total flow
            max_flow += path_flow

            current = goal
            while current != origin :

                # the predecessor for the current index 
                pred = traverse[current]

                self.flow[(pred, current)] += path_flow
                self.flow[(current, pred)] -= path_flow

                current = pred

        if data:
            return {'max_flow' : max_flow, 'iteration': \
            [{i: [0] + presented_data[i][::-1]} \
            for i  in range(1, iter_count+1)]}
        return max_flow

    def EdmondKarp(self, origin=None, goal=None, data=False):

        return self._max_flow_search_FF(origin=origin, goal=goal, \
            data=data, DFS=False)

    def _BFS_using_levels(self, origin=None, goal=None):

        # set path origin and goal indexes

        origin = origin or self.source
        goal = goal or self.sink

        self.level = [-1] * self.size 
        self.level[origin] = 0

        to_visit = [origin]

        while to_visit :

            current = to_visit.pop(0)

            for successor in range(self.size):
                if self.capacity[(current, successor)] - \
                self.flow[(current, successor)] \
                 > 0 and self.level[successor] < 0:
                    self.level[successor] = self.level[current] + 1
                    to_visit.append(successor)


        return self.level

    def _send_flow(self, origin=None, goal=None, max_flow=float('inf')):
        
        goal = goal or self.sink
        origin = origin or self.source

        # sink reached 
        if origin == goal :
            return max_flow

        # traverse all neighbors 
        for successor in range(self.size):

            # if a viable edge, a positive residual and d(u)<d(v)
            if ((self.capacity[(origin, successor)] - \
                self.flow[(origin, successor)] > 0 )
                and (self.level[origin] + 1 == self.level[successor])):
                
                # finds the bounding flow
                current_flow = \
                min(max_flow, self.capacity[(origin, successor)] \
                    - self.flow[(origin, successor)])
                
                path_flow = self._send_flow(successor, goal, current_flow)
                
                # if the flow is viable, update the path.
                if path_flow > 0:

                    self.flow[(origin, successor)] += path_flow
                    self.flow[(successor, origin)] -= path_flow

                    return path_flow
        return 0

    def _max_flow_search_D(self, origin=None, goal=None):

        # initialzing vars
        goal = goal or self.sink
        origin = origin or self.source
        total_flow = 0


        # while there is path on the residual graph
        while True :
            # finds the path using BFS and marks their reachability to the sink
            self.level = self._BFS_using_levels(origin, goal)

            # if reached sink
            if self.level[goal] < 0:
                return total_flow

            # using the above paths to compute flow
            while True:
                path_flow = self._send_flow(origin, goal)

                if path_flow == 0:
                    break

                # incrementing flow
                total_flow += path_flow

    def Dinic(self, origin=None, goal=None):

        return self._max_flow_search_D(origin=origin, goal=goal)

    def _push(self, u, v):
        
        delta_flow = min(self.excess[u], (self.capacity[(u, v)] - \
            self.flow[(u, v)]))

        self.flow[(u, v)] += delta_flow
        self.flow[(v, u)] -= delta_flow

        self.excess[u] -= delta_flow
        self.excess[v] += delta_flow

    def _relabel(self, u):
        
        # initialize 
        min_distance = float('inf')

        for v in range(self.size):
            if (self.capacity[(u, v)] - self.flow[(u, v)]) > 0 :
                min_distance = min(min_distance, self.distance[v])
                self.distance[u] = min_distance + 1

    def _discharge(self, u):

        while self.excess[u] > 0 :

            # iterating through all possibel vertexes
            if self.seen[u] < self.size :

                v = self.seen[u]

                if (self.capacity[(u, v)] - self.flow[(u, v)] > 0) \
                and (self.distance[u] > self.distance[v]):
            
                    self._push(u, v)

                else:
                    self.seen[u] += 1
             
             # if a push is not possible, relabel
            else:
                self._relabel(u)
                self.seen[u] = 0

    def PushRelable(self, origin=None, goal=None):
        
        # initiates vars
        n = self.size

        goal = goal or self.sink
        origin = origin or self.source

        # inter_nodes, not sink or source, are viable for push
        inter_nodes = [i for i in range(n) if i != origin and i != goal]

        # pushes max capacity from source
        self.distance[origin] = n
        self.excess[origin] = float('inf')

        # resolves excess on source neighbors
        for v in range(n):
            self._push(origin, v)


        potential_vertex = 0

        while potential_vertex < len(inter_nodes):
            u = inter_nodes[potential_vertex]

            # resolve ecxess flow from u.
            pred_distance = self.distance[u]
            self._discharge(u)

            # checks whether te vertex was relabled
            if self.distance[u] > pred_distance :

                # move to front selection rule
                # inserts the vertex to the front and set the search back to 0
                inter_nodes.insert(0, inter_nodes.pop(potential_vertex))
                potential_vertex = 0

            else:
                potential_vertex += 1

        return sum (self.flow[origin])

