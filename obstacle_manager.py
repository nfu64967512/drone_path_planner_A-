"""
障礙物管理器模組 - A*尋路演算法版
實現多邊形障礙區域，使用A*演算法保證100%不穿越障礙物
"""

import math
from typing import List, Tuple, Optional
from logger_utils import logger
from astar_pathfinding import GridMap, AStarPathfinder

try:
    from config import Config
except ImportError:
    class Config:
        EARTH_RADIUS_M = 6378137.0

class Obstacle:
    """障礙物資料類 - 多邊形版本"""
    def __init__(self, corners: List[Tuple[float, float]], safe_distance: float = 1.0):
        self.corners = corners  # 障礙物的角點列表 [(lat, lon), ...]
        self.safe_distance = safe_distance  # 安全距離(公尺)
        self.markers = []  # 角點標記列表
        self.polygon = None  # 障礙物多邊形顯示
        self.safe_polygon = None  # 安全範圍多邊形顯示
        self.is_complete = False  # 是否已完成打點

    def add_corner(self, lat: float, lon: float):
        """添加角點"""
        self.corners.append((lat, lon))

    def get_expanded_corners(self, distance_m: float) -> List[Tuple[float, float]]:
        """
        獲取擴展後的多邊形角點(向外擴展指定距離)
        用於生成安全邊界
        """
        if len(self.corners) < 3:
            return self.corners

        # 使用簡單的向外偏移算法
        expanded = []
        n = len(self.corners)

        for i in range(n):
            prev_idx = (i - 1) % n
            next_idx = (i + 1) % n

            curr = self.corners[i]
            prev = self.corners[prev_idx]
            next = self.corners[next_idx]

            # 計算兩條邊的法向量
            v1 = self._normalize_vector(
                self._vector_subtract(curr, prev)
            )
            v2 = self._normalize_vector(
                self._vector_subtract(next, curr)
            )

            # 計算角平分線方向(向外)
            bisector = self._normalize_vector(
                (v1[0] + v2[0], v1[1] + v2[1])
            )

            # 計算法向量(順時針旋轉90度)
            normal = (-bisector[1], bisector[0])

            # 向外擴展
            expanded_point = self._offset_point(curr, normal, distance_m)
            expanded.append(expanded_point)

        return expanded

    def _vector_subtract(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> Tuple[float, float]:
        """向量減法"""
        return (p1[0] - p2[0], p1[1] - p2[1])

    def _normalize_vector(self, v: Tuple[float, float]) -> Tuple[float, float]:
        """向量正規化"""
        length = math.sqrt(v[0]**2 + v[1]**2)
        if length < 1e-10:
            return (0, 0)
        return (v[0] / length, v[1] / length)

    def _offset_point(self, point: Tuple[float, float], direction: Tuple[float, float],
                     distance_m: float) -> Tuple[float, float]:
        """將點沿指定方向偏移指定距離"""
        lat, lon = point
        dir_lat, dir_lon = direction

        # 簡單的平面偏移(對於小範圍足夠精確)
        earth_radius_m = 111111.0
        cos_lat = math.cos(math.radians(lat))

        new_lat = lat + (dir_lat * distance_m / earth_radius_m)
        new_lon = lon + (dir_lon * distance_m / (earth_radius_m * cos_lat))

        return (new_lat, new_lon)


class ObstacleManager:
    """
    障礙物管理器 - A*尋路演算法版

    核心功能:
    1. 管理多邊形障礙區域
    2. 使用A*演算法進行精確避障路徑規劃
    3. 建立柵格地圖並標記障礙物
    4. 保證100%不穿越障礙物
    """

    def __init__(self):
        self.obstacles: List[Obstacle] = []
        self.earth_radius_m = 111111.0  # 每度約111111公尺
        self.current_creating_obstacle: Optional[Obstacle] = None
        self.grid_map: Optional[GridMap] = None
        self.pathfinder: Optional[AStarPathfinder] = None

    def start_new_obstacle(self, safe_distance: float = 1.0) -> Obstacle:
        """開始創建新障礙物"""
        obstacle = Obstacle([], safe_distance)
        self.current_creating_obstacle = obstacle
        return obstacle

    def finish_current_obstacle(self):
        """完成當前障礙物創建"""
        if self.current_creating_obstacle and len(self.current_creating_obstacle.corners) >= 3:
            self.current_creating_obstacle.is_complete = True
            self.obstacles.append(self.current_creating_obstacle)
            logger.info(f"完成障礙物: {len(self.current_creating_obstacle.corners)}個角點")
            self.current_creating_obstacle = None
            return True
        return False

    def cancel_current_obstacle(self):
        """取消當前障礙物創建"""
        self.current_creating_obstacle = None

    def remove_obstacle(self, obstacle: Obstacle):
        """移除障礙物"""
        if obstacle in self.obstacles:
            self.obstacles.remove(obstacle)
            logger.info(f"移除障礙物")
            return True
        return False

    def remove_nearest_obstacle(self, coords: Tuple[float, float], threshold_m: float = 50.0):
        """移除指定座標最近的障礙物"""
        if not self.obstacles:
            return None

        nearest_obs = None
        min_dist = float('inf')

        for obs in self.obstacles:
            if not obs.corners:
                continue
            # 計算到障礙物中心的距離
            center = self._calculate_polygon_center(obs.corners)
            dist = self.calculate_distance(coords, center)
            if dist < min_dist:
                min_dist = dist
                nearest_obs = obs

        if nearest_obs and min_dist < threshold_m:
            self.remove_obstacle(nearest_obs)
            return nearest_obs
        return None

    def _calculate_polygon_center(self, corners: List[Tuple[float, float]]) -> Tuple[float, float]:
        """計算多邊形中心點"""
        if not corners:
            return (0, 0)

        lat_sum = sum(c[0] for c in corners)
        lon_sum = sum(c[1] for c in corners)
        n = len(corners)

        return (lat_sum / n, lon_sum / n)

    def clear_all(self):
        """清除所有障礙物"""
        self.obstacles.clear()
        self.current_creating_obstacle = None
        logger.info("清除所有障礙物")

    def filter_waypoints_with_detour(self, waypoints: List[Tuple[float, float]],
                                     boundary_corners: List[Tuple[float, float]] = None) -> List[Tuple[float, float]]:
        """
        A*演算法障礙物避障

        流程:
        1. 建立柵格地圖並標記障礙物
        2. 檢測航點間的路徑是否穿過障礙區域
        3. 使用A*演算法尋找繞行路徑
        4. 保證100%不穿越障礙物
        """
        if not waypoints or not self.obstacles:
            return waypoints

        # 建立柵格地圖
        self._build_grid_map(waypoints, boundary_corners)

        if not self.grid_map or not self.pathfinder:
            logger.warning("柵格地圖建立失敗，使用原始航點")
            return waypoints

        # 處理航點路徑
        result_waypoints = [waypoints[0]]  # 保留第一個點

        for i in range(len(waypoints) - 1):
            current = waypoints[i]
            next_point = waypoints[i + 1]

            # 檢查這段路徑是否穿過障礙物
            if self._segment_collides_with_obstacles(current, next_point):
                # 使用A*尋找繞行路徑
                detour_path = self.pathfinder.find_path(current, next_point)

                if detour_path and len(detour_path) > 2:
                    # 找到繞行路徑，添加中間點（跳過起點，因為已經添加過）
                    result_waypoints.extend(detour_path[1:])
                    logger.info(f"A*繞行: {i}-{i+1}, 生成{len(detour_path)-2}個中間點")
                else:
                    # 找不到路徑或路徑是直線，直接添加終點
                    if next_point not in result_waypoints:
                        result_waypoints.append(next_point)
            else:
                # 無障礙物，直接添加下一個點
                if next_point not in result_waypoints:
                    result_waypoints.append(next_point)

        logger.info(f"A*避障完成: {len(waypoints)}點 -> {len(result_waypoints)}點")
        return result_waypoints

    def _build_grid_map(self, waypoints: List[Tuple[float, float]],
                        boundary_corners: Optional[List[Tuple[float, float]]] = None):
        """建立柵格地圖並標記障礙物"""
        # 計算地圖邊界
        all_points = waypoints[:]

        if boundary_corners:
            all_points.extend(boundary_corners)

        # 添加所有障礙物角點
        for obs in self.obstacles:
            if obs.is_complete:
                all_points.extend(obs.corners)

        if len(all_points) < 2:
            logger.error("航點數量不足，無法建立柵格地圖")
            return

        # 計算邊界（加上緩衝區）
        lats = [p[0] for p in all_points]
        lons = [p[1] for p in all_points]

        lat_margin = (max(lats) - min(lats)) * 0.1  # 10%緩衝
        lon_margin = (max(lons) - min(lons)) * 0.1

        bounds = (
            min(lats) - lat_margin,
            max(lats) + lat_margin,
            min(lons) - lon_margin,
            max(lons) + lon_margin
        )

        # 創建柵格地圖（解析度0.5公尺）
        self.grid_map = GridMap(bounds, resolution=0.5)

        # 標記作業邊界（如果有提供）
        if boundary_corners and len(boundary_corners) >= 3:
            self.grid_map.mark_boundary(boundary_corners, invert=True)

        # 標記所有障礙物
        for obs in self.obstacles:
            if obs.is_complete and len(obs.corners) >= 3:
                # 使用擴展後的安全邊界
                safe_boundary = obs.get_expanded_corners(obs.safe_distance)
                self.grid_map.mark_obstacle(safe_boundary)
                logger.info(f"標記障礙物: {len(obs.corners)}角點, 安全距離{obs.safe_distance}m")

        # 創建A*尋路器
        self.pathfinder = AStarPathfinder(self.grid_map)
        logger.info(f"柵格地圖建立完成: {self.grid_map.grid_width}x{self.grid_map.grid_height}")

    def _segment_collides_with_obstacles(self, p1: Tuple[float, float],
                                          p2: Tuple[float, float]) -> bool:
        """檢查線段是否與障礙物碰撞（使用柵格地圖）"""
        if not self.grid_map:
            return False

        # 轉換為柵格座標
        start_grid = self.grid_map.latlon_to_grid(*p1)
        end_grid = self.grid_map.latlon_to_grid(*p2)

        # 使用Bresenham算法檢查線段經過的所有格子
        if hasattr(self.pathfinder, '_bresenham_line'):
            points = self.pathfinder._bresenham_line(start_grid, end_grid)

            for y, x in points:
                if not self.grid_map.is_valid(y, x):
                    return True

        return False

    def _identify_scan_segments(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[str, Tuple[int, int]]]:
        """
        識別掃描線段結構

        基於距離分析:
        - 長距離段(>閾值): 掃描線
        - 短距離段(<閾值): 轉向段

        返回: [(segment_type, (start_index, end_index)), ...]
        segment_type: "scan" 或 "turn"
        """
        if len(waypoints) < 2:
            return []

        segments = []

        # 計算所有相鄰點之間的距離
        distances = []
        for i in range(len(waypoints) - 1):
            dist = self.calculate_distance(waypoints[i], waypoints[i + 1])
            distances.append((i, i + 1, dist))

        if not distances:
            return []

        # 找出距離中位數作為閾值
        sorted_dists = sorted([d[2] for d in distances])
        median_dist = sorted_dists[len(sorted_dists) // 2]
        max_dist = sorted_dists[-1]

        # 閾值: 使用最大距離的50%,確保只識別真正的長掃描線
        threshold = max(median_dist * 0.6, max_dist * 0.5)

        # 識別掃描線段
        for idx_start, idx_end, dist in distances:
            if dist > threshold:
                segments.append(("scan", (idx_start, idx_end)))
            else:
                segments.append(("turn", (idx_start, idx_end)))

        logger.info(f"識別掃描結構: {len([s for s in segments if s[0]=='scan'])}條掃描線,閾值={threshold:.2f}m")
        return segments

    def _segment_scan_line(self, p1: Tuple[float, float], p2: Tuple[float, float],
                          obstacles: List[Obstacle],
                          boundary_corners: Optional[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
        """
        將穿過障礙物的掃描線分段

        策略:
        1. 找出掃描線與障礙區域的交點
        2. 在安全邊界外側生成繞行點
        3. 返回: [p1, 繞行點們..., p2]
        """
        if not obstacles:
            return [p1, p2]

        # 處理第一個障礙物
        obstacle = obstacles[0]

        # 獲取安全邊界(障礙區域 + 安全距離)
        safe_boundary = obstacle.get_expanded_corners(obstacle.safe_distance)

        # 檢測線段與障礙區域的交點
        intersections = self._line_polygon_intersections(p1, p2, safe_boundary)

        if len(intersections) < 2:
            # 沒有足夠的交點,嘗試其他障礙物
            remaining = [o for o in obstacles if o != obstacle]
            if remaining:
                return self._segment_scan_line(p1, p2, remaining, boundary_corners)
            return [p1, p2]

        # 生成繞行路徑(沿著安全邊界走)
        detour_points = self._generate_boundary_detour(p1, p2, safe_boundary, intersections)

        if not detour_points:
            logger.warning(f"繞行點生成失敗")
            remaining = [o for o in obstacles if o != obstacle]
            if remaining:
                return self._segment_scan_line(p1, p2, remaining, boundary_corners)
            return [p1, p2]

        # 驗證繞行點是否在作業邊界內
        valid_detour = []
        for dp in detour_points:
            if boundary_corners is None or self.point_in_polygon(dp, boundary_corners):
                valid_detour.append(dp)

        if not valid_detour:
            logger.warning(f"繞行點超出邊界")
            return [p1, p2]

        # 構建當前障礙物的繞行路徑
        current_path = [p1] + valid_detour + [p2]

        # 檢查是否還有其他障礙物需要處理
        remaining = [o for o in obstacles if o != obstacle]
        if not remaining:
            logger.info(f"成功生成繞行路徑: {len(valid_detour)}個繞行點")
            return current_path

        # 遞歸處理剩餘障礙物
        final_path = [current_path[0]]
        for i in range(len(current_path) - 1):
            seg_start = current_path[i]
            seg_end = current_path[i + 1]

            # 檢查這個線段是否與剩餘障礙物碰撞
            colliding = self.check_segment_collision(seg_start, seg_end, remaining)

            if colliding:
                # 遞歸處理這個線段
                sub_path = self._segment_scan_line(seg_start, seg_end, colliding, boundary_corners)
                final_path.extend(sub_path[1:])
            else:
                # 無碰撞,直接添加終點
                final_path.append(seg_end)

        logger.info(f"完整繞行路徑: 處理{len(obstacles)}個障礙物,生成{len(final_path)}個航點")
        return final_path

    def _line_polygon_intersections(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                    polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        計算線段與多邊形的交點

        返回: 交點列表
        """
        intersections = []
        n = len(polygon)

        for i in range(n):
            edge_start = polygon[i]
            edge_end = polygon[(i + 1) % n]

            # 檢測線段與多邊形邊的交點
            intersection = self._line_segment_intersection(p1, p2, edge_start, edge_end)
            if intersection:
                intersections.append(intersection)

        return intersections

    def _line_segment_intersection(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                   p3: Tuple[float, float], p4: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        計算兩條線段的交點

        返回: 交點座標,如果不相交返回None
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if abs(denom) < 1e-10:
            return None  # 平行或重合

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

        if 0 <= t <= 1 and 0 <= u <= 1:
            # 有交點
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)

        return None

    def _generate_boundary_detour(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                  boundary: List[Tuple[float, float]],
                                  intersections: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        生成沿著障礙邊界的繞行路徑

        策略:
        1. 找出進入點和離開點在邊界上的位置
        2. 沿著邊界走,選擇遠離原始掃描線的方向(而非較短路徑)
        3. 返回繞行點序列
        """
        if len(intersections) < 2:
            return []

        # 使用前兩個交點
        entry = intersections[0]
        exit = intersections[1]

        # 找出交點最近的邊界頂點(而非邊)
        entry_vertex = self._find_nearest_vertex(entry, boundary)
        exit_vertex = self._find_nearest_vertex(exit, boundary)

        if entry_vertex is None or exit_vertex is None:
            return []

        # 計算掃描線的中點
        mid_lat = (p1[0] + p2[0]) / 2
        mid_lon = (p1[1] + p2[1]) / 2
        scan_mid = (mid_lat, mid_lon)

        # 生成兩個可能的繞行路徑
        n = len(boundary)

        # 路徑1: 順時針
        path1 = [entry]
        idx = entry_vertex
        while idx != exit_vertex:
            path1.append(boundary[idx])
            idx = (idx + 1) % n
        path1.append(boundary[exit_vertex])
        path1.append(exit)

        # 路徑2: 逆時針
        path2 = [entry]
        idx = entry_vertex
        while idx != exit_vertex:
            path2.append(boundary[idx])
            idx = (idx - 1) % n
        path2.append(boundary[exit_vertex])
        path2.append(exit)

        # 選擇遠離掃描線中點的路徑
        # 計算兩條路徑的中點到掃描線中點的距離
        path1_mid = path1[len(path1) // 2]
        path2_mid = path2[len(path2) // 2]

        dist1 = self.calculate_distance(path1_mid, scan_mid)
        dist2 = self.calculate_distance(path2_mid, scan_mid)

        # 選擇距離較遠的路徑(更安全的繞行)
        if dist1 > dist2:
            return path1
        else:
            return path2

    def _find_nearest_vertex(self, point: Tuple[float, float],
                            polygon: List[Tuple[float, float]]) -> Optional[int]:
        """
        找出點最接近多邊形的哪個頂點

        返回: 頂點的索引
        """
        min_dist = float('inf')
        nearest_vertex = None

        for i, vertex in enumerate(polygon):
            dist = self.calculate_distance(point, vertex)
            if dist < min_dist:
                min_dist = dist
                nearest_vertex = i

        return nearest_vertex

    def _find_nearest_edge(self, point: Tuple[float, float],
                          polygon: List[Tuple[float, float]]) -> Optional[int]:
        """
        找出點最接近多邊形的哪條邊

        返回: 邊的起始索引
        """
        min_dist = float('inf')
        nearest_edge = None
        n = len(polygon)

        for i in range(n):
            edge_start = polygon[i]
            edge_end = polygon[(i + 1) % n]

            dist = self._point_to_segment_distance(point, edge_start, edge_end)
            if dist < min_dist:
                min_dist = dist
                nearest_edge = i

        return nearest_edge

    def _point_to_segment_distance(self, point: Tuple[float, float],
                                   seg_start: Tuple[float, float],
                                   seg_end: Tuple[float, float]) -> float:
        """計算點到線段的距離"""
        px, py = point
        x1, y1 = seg_start
        x2, y2 = seg_end

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return self.calculate_distance(point, seg_start)

        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return self.calculate_distance(point, (closest_x, closest_y))

    def point_in_polygon(self, point: Tuple[float, float],
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

    def check_waypoint_collision(self, waypoint: Tuple[float, float]) -> bool:
        """檢查航點是否在任何障礙區域內"""
        for obstacle in self.obstacles:
            if not obstacle.is_complete or len(obstacle.corners) < 3:
                continue

            # 檢查是否在安全邊界內
            safe_boundary = obstacle.get_expanded_corners(obstacle.safe_distance)
            if self.point_in_polygon(waypoint, safe_boundary):
                return True

        return False

    def check_segment_collision(self, p1: Tuple[float, float],
                               p2: Tuple[float, float],
                               obstacles_to_check: List[Obstacle] = None) -> List[Obstacle]:
        """
        檢查線段是否穿過障礙區域
        返回: 與線段碰撞的障礙物列表
        """
        obstacles = obstacles_to_check if obstacles_to_check is not None else self.obstacles
        colliding_obstacles = []

        for obstacle in obstacles:
            if not obstacle.is_complete or len(obstacle.corners) < 3:
                continue

            # 檢查線段是否與安全邊界相交
            safe_boundary = obstacle.get_expanded_corners(obstacle.safe_distance)

            # 檢查線段端點是否在多邊形內
            if self.point_in_polygon(p1, safe_boundary) or self.point_in_polygon(p2, safe_boundary):
                colliding_obstacles.append(obstacle)
                continue

            # 檢查線段是否與多邊形邊相交
            n = len(safe_boundary)
            for i in range(n):
                edge_start = safe_boundary[i]
                edge_end = safe_boundary[(i + 1) % n]

                if self._line_segment_intersection(p1, p2, edge_start, edge_end):
                    colliding_obstacles.append(obstacle)
                    break

        return colliding_obstacles

    def calculate_distance(self, p1: Tuple[float, float],
                          p2: Tuple[float, float]) -> float:
        """計算兩點間距離(公尺) - 平面近似"""
        lat1, lon1 = p1
        lat2, lon2 = p2

        # 使用平均緯度計算經度縮放
        avg_lat = (lat1 + lat2) / 2
        cos_lat = math.cos(math.radians(avg_lat))

        # 緯度和經度差值(度)
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # 轉換為公尺
        dy = dlat * self.earth_radius_m
        dx = dlon * self.earth_radius_m * cos_lat

        # 計算距離
        distance = math.sqrt(dx*dx + dy*dy)
        return distance
