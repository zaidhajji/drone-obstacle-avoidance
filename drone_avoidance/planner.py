"""Grid map, A* and path shortcutting. No ROS dependency, so it is unit-testable."""
import heapq
import math


class GridMap:
    def __init__(self, cfg, res=0.25, inflation=0.8):
        self.xmin, self.xmax, self.ymin, self.ymax = cfg['arena']
        self.res, self.infl = res, inflation
        self.boxes = cfg.get('boxes', [])
        self.cyls = cfg.get('cylinders', [])
        self.nx = int((self.xmax - self.xmin) / res) + 1
        self.ny = int((self.ymax - self.ymin) / res) + 1
        self.blocked = [[self._hit(*self.to_xy((i, j))) for j in range(self.ny)]
                        for i in range(self.nx)]

    def to_cell(self, p):
        return (round((p[0] - self.xmin) / self.res), round((p[1] - self.ymin) / self.res))

    def to_xy(self, c):
        return (self.xmin + c[0] * self.res, self.ymin + c[1] * self.res)

    def _hit(self, x, y):
        m = self.infl
        if x < self.xmin + m or x > self.xmax - m or y < self.ymin + m or y > self.ymax - m:
            return True
        for cx, cy, sx, sy in self.boxes:
            dx = max(abs(x - cx) - sx / 2, 0.0)
            dy = max(abs(y - cy) - sy / 2, 0.0)
            if math.hypot(dx, dy) < m:
                return True
        for cx, cy, r in self.cyls:
            if math.hypot(x - cx, y - cy) - r < m:
                return True
        return False

    def free(self, c):
        return 0 <= c[0] < self.nx and 0 <= c[1] < self.ny and not self.blocked[c[0]][c[1]]


def astar(gm, start, goal):
    s, g = gm.to_cell(start), gm.to_cell(goal)
    if not (gm.free(s) and gm.free(g)):
        return None
    openq, came, cost = [(0.0, s)], {s: None}, {s: 0.0}
    while openq:
        _, c = heapq.heappop(openq)
        if c == g:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                n = (c[0] + dx, c[1] + dy)
                if not gm.free(n):
                    continue
                if dx and dy and not (gm.free((c[0] + dx, c[1])) and gm.free((c[0], c[1] + dy))):
                    continue  # no corner cutting
                nc = cost[c] + math.hypot(dx, dy)
                if n not in cost or nc < cost[n]:
                    cost[n] = nc
                    came[n] = c
                    heapq.heappush(openq, (nc + math.hypot(g[0] - n[0], g[1] - n[1]), n))
    if g not in came:
        return None
    path, c = [], g
    while c is not None:
        path.append(gm.to_xy(c))
        c = came[c]
    return path[::-1]


def line_of_sight(gm, a, b):
    n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (gm.res / 2)) + 1
    for k in range(n + 1):
        t = k / n
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if not gm.free(gm.to_cell(p)):
            return False
    return True


def smooth(gm, path):
    """Greedy shortcutting: keep only the waypoints needed to stay collision-free."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(gm, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
