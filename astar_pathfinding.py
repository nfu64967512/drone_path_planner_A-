"""
A*尋路演算法模組
實現精確的障礙物避障路徑規劃
"""

import math
import heapq
from typing import List, Tuple, Optional, Set
from logger_utils import logger


class Node:
    """A*演算法節點"""
    def __init__(self, position: Tuple[int, int], parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # 從起點到當前節點的實際成本
        self.h = 0  # 從當前節點到終點的啟發式估計成本
        self.f = 0  # f = g + h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(self.position)


class GridMap:
    """柵格化地圖"""

    def __init__(self, bounds: Tuple[float, float, float, float],
                 resolution: float = 0.5):
        """
        初始化柵格地圖

        Args:
            bounds: (min_lat, max_lat, min_lon, max_lon) 地圖邊界
            resolution: 柵格解析度(公尺) - 每個柵格的邊長
        """
        self.min_lat, self.max_lat, self.min_lon, self.max_lon = bounds
        self.resolution = resolution
        self.earth_radius_m = 111111.0  # 每度約111111公尺

        # 計算柵格大小
        lat_range = self.max_lat - self.min_lat
        lon_range = self.max_lon - self.min_lon
        avg_lat = (self.min_lat + self.max_lat) / 2
        cos_lat = math.cos(math.radians(avg_lat))

        # 轉換為公尺
        lat_meters = lat_range * self.earth_radius_m
        lon_meters = lon_range * self.earth_radius_m * cos_lat

        # 計算柵格數量
        self.grid_height = max(10, int(lat_meters / resolution))
        self.grid_width = max(10, int(lon_meters / resolution))

        # 創建柵格（False=可通行, True=障礙物）
        self.grid = [[False for _ in range(self.grid_width)]
                     for _ in range(self.grid_height)]

        logger.info(f"柵格地圖創建: {self.grid_width}x{self.grid_height} "
                   f"(解析度:{resolution}m)")

    def latlon_to_grid(self, lat: float, lon: float) -> Tuple[int, int]:
        """將經緯度轉換為柵格座標"""
        # 正規化到[0,1]
        norm_lat = (lat - self.min_lat) / (self.max_lat - self.min_lat)
        norm_lon = (lon - self.min_lon) / (self.max_lon - self.min_lon)

        # 轉換為柵格座標
        grid_y = int(norm_lat * (self.grid_height - 1))
        grid_x = int(norm_lon * (self.grid_width - 1))

        # 確保在範圍內
        grid_y = max(0, min(self.grid_height - 1, grid_y))
        grid_x = max(0, min(self.grid_width - 1, grid_x))

        return (grid_y, grid_x)

    def grid_to_latlon(self, grid_y: int, grid_x: int) -> Tuple[float, float]:
        """將柵格座標轉換為經緯度"""
        # 正規化到[0,1]
        norm_lat = grid_y / (self.grid_height - 1)
        norm_lon = grid_x / (self.grid_width - 1)

        # 轉換為經緯度
        lat = self.min_lat + norm_lat * (self.max_lat - self.min_lat)
        lon = self.min_lon + norm_lon * (self.max_lon - self.min_lon)

        return (lat, lon)

    def mark_obstacle(self, polygon: List[Tuple[float, float]],
                      safe_distance: float = 0):
        """標記障礙物區域"""
        # 如果有安全距離，先擴展多邊形（這部分由外部處理）
        # 遍歷柵格，檢查每個柵格中心是否在障礙物內
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                lat, lon = self.grid_to_latlon(y, x)
                if self._point_in_polygon((lat, lon), polygon):
                    self.grid[y][x] = True

    def mark_boundary(self, boundary: List[Tuple[float, float]], invert: bool = True):
        """標記作業邊界（邊界外為障礙）"""
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                lat, lon = self.grid_to_latlon(y, x)
                is_inside = self._point_in_polygon((lat, lon), boundary)

                if invert:
                    # 邊界外視為障礙
                    self.grid[y][x] = not is_inside

    def _point_in_polygon(self, point: Tuple[float, float],
                          polygon: List[Tuple[float, float]]) -> bool:
        """射線法判斷點是否在多邊形內"""
        lat, lon = point
        n = len(polygon)
        inside = False

        p1_lat, p1_lon = polygon[0]
        for i in range(1, n + 1):
            p2_lat, p2_lon = polygon[i % n]
            if lon > min(p1_lon, p2_lon):
                if lon <= max(p1_lon, p2_lon):
                    if lat <= max(p1_lat, p2_lat):
                        if p1_lon != p2_lon:
                            xinters = (lon - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat
                        if p1_lat == p2_lat or lat <= xinters:
                            inside = not inside
            p1_lat, p1_lon = p2_lat, p2_lon

        return inside

    def is_valid(self, grid_y: int, grid_x: int) -> bool:
        """檢查柵格座標是否有效且可通行"""
        if grid_y < 0 or grid_y >= self.grid_height:
            return False
        if grid_x < 0 or grid_x >= self.grid_width:
            return False
        return not self.grid[grid_y][grid_x]


class AStarPathfinder:
    """A*尋路演算法"""

    def __init__(self, grid_map: GridMap):
        self.grid_map = grid_map

    def find_path(self, start_latlon: Tuple[float, float],
                  end_latlon: Tuple[float, float],
                  allow_diagonal: bool = True) -> Optional[List[Tuple[float, float]]]:
        """
        使用A*演算法尋找路徑

        Args:
            start_latlon: 起點經緯度
            end_latlon: 終點經緯度
            allow_diagonal: 是否允許對角線移動

        Returns:
            路徑點列表（經緯度），如果找不到路徑返回None
        """
        # 轉換為柵格座標
        start_grid = self.grid_map.latlon_to_grid(*start_latlon)
        end_grid = self.grid_map.latlon_to_grid(*end_latlon)

        # 檢查起點和終點是否有效
        if not self.grid_map.is_valid(*start_grid):
            logger.warning(f"起點在障礙物內: {start_latlon}")
            # 嘗試找到最近的有效點
            start_grid = self._find_nearest_valid_point(start_grid)
            if start_grid is None:
                return None

        if not self.grid_map.is_valid(*end_grid):
            logger.warning(f"終點在障礙物內: {end_latlon}")
            end_grid = self._find_nearest_valid_point(end_grid)
            if end_grid is None:
                return None

        # 創建起始和目標節點
        start_node = Node(start_grid)
        end_node = Node(end_grid)

        # 初始化開放列表和關閉集合
        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set: Set[Tuple[int, int]] = set()

        # 存儲每個節點的最佳g值
        g_scores = {start_grid: 0}

        # A*主循環
        while open_list:
            # 取出f值最小的節點
            current_node = heapq.heappop(open_list)

            # 如果到達目標
            if current_node.position == end_node.position:
                path = self._reconstruct_path(current_node)
                # 轉換為經緯度
                latlon_path = [self.grid_map.grid_to_latlon(y, x)
                              for y, x in path]
                # 路徑平滑
                smoothed_path = self._smooth_path(latlon_path)
                logger.info(f"A*找到路徑: {len(path)}格 -> {len(smoothed_path)}點")
                return smoothed_path

            # 添加到關閉集合
            closed_set.add(current_node.position)

            # 檢查所有鄰居
            neighbors = self._get_neighbors(current_node, allow_diagonal)

            for neighbor in neighbors:
                if neighbor.position in closed_set:
                    continue

                # 計算新的g值
                tentative_g = current_node.g + self._distance(
                    current_node.position, neighbor.position
                )

                # 如果找到更好的路徑
                if neighbor.position not in g_scores or tentative_g < g_scores[neighbor.position]:
                    neighbor.parent = current_node
                    neighbor.g = tentative_g
                    neighbor.h = self._heuristic(neighbor.position, end_node.position)
                    neighbor.f = neighbor.g + neighbor.h

                    g_scores[neighbor.position] = tentative_g
                    heapq.heappush(open_list, neighbor)

        # 找不到路徑
        logger.warning(f"A*找不到路徑: {start_latlon} -> {end_latlon}")
        return None

    def _find_nearest_valid_point(self, grid_pos: Tuple[int, int],
                                   max_search_radius: int = 20) -> Optional[Tuple[int, int]]:
        """找到最近的有效點"""
        y, x = grid_pos

        for radius in range(1, max_search_radius):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dy) == radius or abs(dx) == radius:
                        ny, nx = y + dy, x + dx
                        if self.grid_map.is_valid(ny, nx):
                            return (ny, nx)

        return None

    def _get_neighbors(self, node: Node, allow_diagonal: bool) -> List[Node]:
        """獲取鄰居節點"""
        y, x = node.position
        neighbors = []

        # 四個方向
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        # 如果允許對角線，添加四個對角方向
        if allow_diagonal:
            directions.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])

        for dy, dx in directions:
            ny, nx = y + dy, x + dx

            if self.grid_map.is_valid(ny, nx):
                # 如果是對角線移動，檢查兩個相鄰格子是否都可通行（避免穿牆）
                if allow_diagonal and abs(dy) == 1 and abs(dx) == 1:
                    if not (self.grid_map.is_valid(y + dy, x) and
                           self.grid_map.is_valid(y, x + dx)):
                        continue

                neighbor = Node((ny, nx), node)
                neighbors.append(neighbor)

        return neighbors

    def _heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """啟發式函數（歐幾里得距離）"""
        dy = pos2[0] - pos1[0]
        dx = pos2[1] - pos1[1]
        return math.sqrt(dx * dx + dy * dy)

    def _distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """計算兩點間距離"""
        dy = pos2[0] - pos1[0]
        dx = pos2[1] - pos1[1]
        # 對角線移動成本為sqrt(2)，直線為1
        return math.sqrt(dx * dx + dy * dy)

    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """重建路徑"""
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return list(reversed(path))

    def _smooth_path(self, path: List[Tuple[float, float]],
                     epsilon: float = 0.00001) -> List[Tuple[float, float]]:
        """
        路徑平滑 - 使用Douglas-Peucker簡化算法
        移除不必要的中間點，保持主要轉折點
        """
        if len(path) <= 2:
            return path

        # 使用簡單的視線檢查法
        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # 嘗試連接到最遠的可見點
            farthest_visible = current_idx + 1

            for test_idx in range(len(path) - 1, current_idx, -1):
                if self._has_line_of_sight(path[current_idx], path[test_idx]):
                    farthest_visible = test_idx
                    break

            if farthest_visible > current_idx + 1:
                # 跳過中間點
                smoothed.append(path[farthest_visible])
                current_idx = farthest_visible
            else:
                # 添加下一個點
                smoothed.append(path[current_idx + 1])
                current_idx += 1

        return smoothed

    def _has_line_of_sight(self, start: Tuple[float, float],
                           end: Tuple[float, float]) -> bool:
        """檢查兩點間是否有視線（無障礙物）"""
        start_grid = self.grid_map.latlon_to_grid(*start)
        end_grid = self.grid_map.latlon_to_grid(*end)

        # 使用Bresenham直線算法檢查中間格子
        points = self._bresenham_line(start_grid, end_grid)

        for y, x in points:
            if not self.grid_map.is_valid(y, x):
                return False

        return True

    def _bresenham_line(self, start: Tuple[int, int],
                        end: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Bresenham直線算法 - 獲取直線經過的所有格子"""
        y0, x0 = start
        y1, x1 = end

        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            points.append((y, x))

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return points
