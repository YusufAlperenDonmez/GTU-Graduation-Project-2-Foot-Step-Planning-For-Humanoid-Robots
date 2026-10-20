import heapq
import math

class AStarPlanner:
    def __init__(self, resolution=0.5):
        self.resolution = resolution

    def plan(self, start_world, goal_world, obstacles, x_range=(0, 20), y_range=(0, 20)):
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range

        start_node = self._snap_to_grid(start_world)
        goal_node = self._snap_to_grid(goal_world)

        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, start_node, [start_world]))
        
        closed_set = set()
        explored_nodes = []
        g_costs = {start_node: 0.0}

        res = self.resolution
        movements = [
            (res, 0), (-res, 0), (0, res), (0, -res),
            (res, res), (-res, res), (res, -res), (-res, -res)
        ]

        iterations = 0
        max_iterations = 25000

        while open_set and iterations < max_iterations:
            iterations += 1
            f, g, current, path = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)
            explored_nodes.append(current)

            if abs(current[0] - goal_node[0]) <= res and abs(current[1] - goal_node[1]) <= res:
                final_path = path + [goal_world]
                return explored_nodes, final_path

            for dx, dy in movements:
                next_node = (round(current[0] + dx, 2), round(current[1] + dy, 2))

                if not (self.x_min <= next_node[0] <= self.x_max and self.y_min <= next_node[1] <= self.y_max):
                    continue

                if self._is_colliding(next_node, obstacles) or self._is_path_colliding(current, next_node, obstacles):
                    continue

                move_cost = math.hypot(dx, dy)
                new_g = g + move_cost

                if next_node not in g_costs or new_g < g_costs[next_node]:
                    g_costs[next_node] = new_g
                    h = math.hypot(next_node[0] - goal_world[0], next_node[1] - goal_world[1])
                    new_f = new_g + h
                    
                    heapq.heappush(open_set, (new_f, new_g, next_node, path + [next_node]))

        print("[A* Warning] Path search reached limit or destination is blocked!")
        return explored_nodes, [start_world, goal_world]

    def _snap_to_grid(self, pt):
        return (round(round(pt[0] / self.resolution) * self.resolution, 2),
                round(round(pt[1] / self.resolution) * self.resolution, 2))

    def _is_colliding(self, pt, obstacles):
        x, y = pt
        clearance = 0.25  # Safety margin around box edges

        for obs in obstacles:
            cx, cy = obs['cx'], obs['cy']
            half_w = (obs['sx'] / 2.0) + clearance
            half_h = (obs['sy'] / 2.0) + clearance
            yaw = obs.get('yaw', 0.0)

            dx = x - cx
            dy = y - cy

            if yaw != 0.0:
                cos_y = math.cos(-yaw)
                sin_y = math.sin(-yaw)
                local_x = dx * cos_y - dy * sin_y
                local_y = dx * sin_y + dy * cos_y
            else:
                local_x = dx
                local_y = dy

            if (-half_w <= local_x <= half_w) and (-half_h <= local_y <= half_h):
                return True

        return False

    def _is_path_colliding(self, p1, p2, obstacles):
        steps = 5
        for i in range(1, steps):
            t = i / float(steps)
            ix = p1[0] * (1 - t) + p2[0] * t
            iy = p1[1] * (1 - t) + p2[1] * t
            if self._is_colliding((ix, iy), obstacles):
                return True
        return False