"""
障礙物UI擴展模組 - 多邊形角點打點版
與主邊界打點方式一致，精確圍出障礙區域
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional, Tuple, List
from obstacle_manager import ObstacleManager, Obstacle
from logger_utils import logger


class ObstacleUIExtension:
    """障礙物UI擴展 - 多邊形角點打點版"""

    def __init__(self, app):
        self.app = app
        self.obstacle_manager = ObstacleManager()

        # 創建狀態
        self.creating_mode = False
        self.delete_mode = False

        # 默認參數
        self.default_safe_distance = 5.0

        # 保存原始地圖點擊處理器
        self.original_map_click_handler = None

    def add_obstacle_ui(self, parent_frame):
        """添加障礙物管理UI - 簡化版"""
        obstacle_frame = ttk.LabelFrame(parent_frame, text="障礙物管理",
                                       padding="10", style='Modern.TLabelframe')
        obstacle_frame.pack(fill=tk.X, pady=(0, 10))

        # 說明
        info = "🚫 點擊地圖標記障礙角點\n   與邊界打點方式相同"
        ttk.Label(obstacle_frame, text=info, justify=tk.LEFT,
                 font=("Segoe UI", 8),
                 foreground=self.app.colors['text_secondary']).pack(anchor=tk.W, pady=(0, 5))

        # 安全距離 (唯一的參數)
        ttk.Label(obstacle_frame, text="安全距離:").pack(anchor=tk.W, pady=(5, 0))
        from ui_components import ModernSlider
        self.safe_slider = ModernSlider(
            obstacle_frame, label="安全距離", from_=0.5, to=20,
            value=self.default_safe_distance, resolution=0.5,
            command=self.on_safe_distance_change, unit="m"
        )
        self.safe_slider.pack(fill=tk.X, pady=2)
        self.app.modern_sliders['obstacle_safe_dist'] = self.safe_slider

        # 按鈕組
        button_frame = tk.Frame(obstacle_frame, bg='white')
        button_frame.pack(fill=tk.X, pady=(10, 0))

        self.create_btn = ttk.Button(button_frame, text="標記障礙點",
                                     command=self.toggle_create_mode,
                                     style='Primary.TButton', width=14)
        self.create_btn.pack(side=tk.LEFT, padx=(0, 5))

        self.finish_btn = ttk.Button(button_frame, text="完成障礙物",
                                     command=self.finish_current_obstacle,
                                     style='Success.TButton', width=14,
                                     state='disabled')
        self.finish_btn.pack(side=tk.LEFT)

        # 第二行按鈕
        button_frame2 = tk.Frame(obstacle_frame, bg='white')
        button_frame2.pack(fill=tk.X, pady=(5, 0))

        ttk.Button(button_frame2, text="刪除最後點",
                  command=self.remove_last_corner,
                  width=14).pack(side=tk.LEFT, padx=(0, 5))

        ttk.Button(button_frame2, text="刪除障礙物",
                  command=self.toggle_delete_mode,
                  style='Warning.TButton', width=14).pack(side=tk.LEFT)

        # 狀態資訊
        self.info_label = ttk.Label(obstacle_frame, text="目前障礙物: 0 個",
                                    font=("Segoe UI", 8),
                                    foreground=self.app.colors['text_secondary'])
        self.info_label.pack(anchor=tk.W, pady=(10, 0))

        self.status_label = ttk.Label(obstacle_frame, text="",
                                      font=("Segoe UI", 8),
                                      foreground=self.app.colors['primary'])
        self.status_label.pack(anchor=tk.W, pady=(2, 0))

    def toggle_create_mode(self):
        """切換創建模式"""
        if self.creating_mode:
            self.exit_create_mode()
        else:
            self.enter_create_mode()

    def enter_create_mode(self):
        """進入創建模式 - 開始新障礙物"""
        self.creating_mode = True
        self.delete_mode = False
        self.create_btn.config(text="取消標記")
        self.finish_btn.config(state='normal')

        # 開始新障礙物
        self.obstacle_manager.start_new_obstacle(self.default_safe_distance)

        # 替換地圖點擊處理器
        self.original_map_click_handler = self.app.on_map_click
        self.app.map.add_left_click_map_command(self.on_create_click)

        self.update_status("開始標記障礙點 - 至少需要3個點")
        logger.info("進入障礙物創建模式 - 開始打點")

    def exit_create_mode(self):
        """退出創建模式"""
        # 取消當前未完成的障礙物
        if self.obstacle_manager.current_creating_obstacle:
            current = self.obstacle_manager.current_creating_obstacle
            # 刪除已打的標記
            for marker in current.markers:
                try:
                    marker.delete()
                except:
                    pass
            current.markers.clear()

            self.obstacle_manager.cancel_current_obstacle()

        self.creating_mode = False
        self.create_btn.config(text="標記障礙點")
        self.finish_btn.config(state='disabled')

        # 恢復原始處理器
        if self.original_map_click_handler:
            self.app.map.add_left_click_map_command(self.original_map_click_handler)
            self.original_map_click_handler = None

        self.update_status("")

    def on_create_click(self, coords):
        """添加障礙物角點"""
        if not self.obstacle_manager.current_creating_obstacle:
            return

        lat, lon = coords
        current = self.obstacle_manager.current_creating_obstacle

        # 添加角點
        current.add_corner(lat, lon)

        # 創建標記
        point_num = len(current.corners)
        marker = self.app.map.set_marker(
            lat, lon,
            text=f"O{point_num}",
            marker_color_circle="#8B5CF6",  # 紫色
            marker_color_outside="#8B5CF6"
        )
        current.markers.append(marker)

        # 更新狀態
        if point_num < 3:
            self.update_status(f"已標記 {point_num} 點 - 還需 {3-point_num} 點")
        else:
            self.update_status(f"已標記 {point_num} 點 - 可以完成障礙物")

        logger.info(f"添加障礙角點 O{point_num}: ({lat:.6f}, {lon:.6f})")

    def remove_last_corner(self):
        """刪除最後一個角點"""
        if not self.obstacle_manager.current_creating_obstacle:
            return

        current = self.obstacle_manager.current_creating_obstacle
        if not current.corners:
            return

        # 刪除最後的標記
        if current.markers:
            last_marker = current.markers.pop()
            try:
                last_marker.delete()
            except:
                pass

        # 刪除最後的角點
        current.corners.pop()

        # 更新狀態
        point_num = len(current.corners)
        if point_num < 3:
            self.update_status(f"已標記 {point_num} 點 - 還需 {3-point_num} 點")
        else:
            self.update_status(f"已標記 {point_num} 點 - 可以完成障礙物")

        logger.info(f"刪除障礙角點，剩餘 {point_num} 點")

    def finish_current_obstacle(self):
        """完成當前障礙物"""
        if not self.obstacle_manager.current_creating_obstacle:
            return

        current = self.obstacle_manager.current_creating_obstacle

        if len(current.corners) < 3:
            self.update_status("至少需要3個角點才能完成障礙物")
            return

        # 完成障礙物
        success = self.obstacle_manager.finish_current_obstacle()

        if success:
            # 清除臨時標記
            for marker in current.markers:
                try:
                    marker.delete()
                except:
                    pass
            current.markers.clear()

            # 創建障礙物顯示
            self.create_obstacle_display(current)

            # 更新資訊
            self.update_info()
            self.update_status(f"完成障礙物 - 共 {len(current.corners)} 個角點")

            # 繼續創建下一個
            self.obstacle_manager.start_new_obstacle(self.default_safe_distance)
            self.update_status("可以繼續標記下一個障礙物")

            logger.info(f"完成障礙物創建: {len(current.corners)} 個角點")

    def create_obstacle_display(self, obstacle: Obstacle):
        """創建障礙物顯示 - 多邊形版"""
        try:
            if len(obstacle.corners) < 3:
                return

            # 計算中心點
            center_lat = sum(c[0] for c in obstacle.corners) / len(obstacle.corners)
            center_lon = sum(c[1] for c in obstacle.corners) / len(obstacle.corners)

            # 中心標記
            center_marker = self.app.map.set_marker(
                center_lat, center_lon,
                text=f"🚫\n{len(obstacle.corners)}點",
                marker_color_circle="#8B5CF6",
                marker_color_outside="#8B5CF6"
            )
            obstacle.markers.append(center_marker)

            # 安全邊界多邊形 (淺紫色外圈)
            safe_boundary = obstacle.get_expanded_corners(obstacle.safe_distance)
            obstacle.safe_polygon = self.app.map.set_polygon(
                safe_boundary,
                fill_color="#E6D8F5",  # 非常淺的紫色
                outline_color="#9370DB",  # 中紫色
                border_width=2
            )

            # 障礙物多邊形 (深紫色內圈)
            obstacle.polygon = self.app.map.set_polygon(
                obstacle.corners,
                fill_color="#B39DDB",  # 中度紫色
                outline_color="#7B1FA2",  # 深紫色
                border_width=3
            )

            # 加入paths以支持地圖操作
            self.app.paths.append(obstacle.safe_polygon)
            self.app.paths.append(obstacle.polygon)

        except Exception as e:
            logger.error(f"創建障礙物顯示失敗: {e}")

    def on_safe_distance_change(self, value):
        """安全距離改變"""
        self.default_safe_distance = value

        # 如果正在創建，更新當前障礙物的安全距離
        if self.obstacle_manager.current_creating_obstacle:
            self.obstacle_manager.current_creating_obstacle.safe_distance = value

        # 重新繪製所有已完成的障礙物
        self.redraw_all_obstacles()

    def redraw_all_obstacles(self):
        """重新繪製所有障礙物"""
        for obstacle in self.obstacle_manager.obstacles:
            if not obstacle.is_complete:
                continue

            # 刪除舊顯示
            if obstacle.polygon:
                if obstacle.polygon in self.app.paths:
                    self.app.paths.remove(obstacle.polygon)
                try:
                    obstacle.polygon.delete()
                except:
                    pass

            if obstacle.safe_polygon:
                if obstacle.safe_polygon in self.app.paths:
                    self.app.paths.remove(obstacle.safe_polygon)
                try:
                    obstacle.safe_polygon.delete()
                except:
                    pass

            # 重新創建 (不需要重新創建中心標記)
            safe_boundary = obstacle.get_expanded_corners(obstacle.safe_distance)
            obstacle.safe_polygon = self.app.map.set_polygon(
                safe_boundary,
                fill_color="#E6D8F5",
                outline_color="#9370DB",
                border_width=2
            )

            obstacle.polygon = self.app.map.set_polygon(
                obstacle.corners,
                fill_color="#B39DDB",
                outline_color="#7B1FA2",
                border_width=3
            )

            self.app.paths.append(obstacle.safe_polygon)
            self.app.paths.append(obstacle.polygon)

    def toggle_delete_mode(self):
        """切換刪除模式"""
        if self.delete_mode:
            self.exit_delete_mode()
        else:
            self.enter_delete_mode()

    def enter_delete_mode(self):
        """進入刪除模式"""
        self.delete_mode = True
        self.creating_mode = False

        self.original_map_click_handler = self.app.on_map_click
        self.app.map.add_left_click_map_command(self.on_delete_click)

        self.update_status("點擊障礙物中心刪除")
        logger.info("進入刪除模式")

    def exit_delete_mode(self):
        """退出刪除模式"""
        self.delete_mode = False

        if self.original_map_click_handler:
            self.app.map.add_left_click_map_command(self.original_map_click_handler)
            self.original_map_click_handler = None

        self.update_status("")

    def on_delete_click(self, coords):
        """刪除障礙物"""
        lat, lon = coords
        removed = self.obstacle_manager.remove_nearest_obstacle((lat, lon), threshold_m=100.0)

        if removed:
            # 從paths移除
            if removed.polygon and removed.polygon in self.app.paths:
                self.app.paths.remove(removed.polygon)
            if removed.safe_polygon and removed.safe_polygon in self.app.paths:
                self.app.paths.remove(removed.safe_polygon)

            # 刪除顯示
            try:
                for marker in removed.markers:
                    marker.delete()
                if removed.polygon:
                    removed.polygon.delete()
                if removed.safe_polygon:
                    removed.safe_polygon.delete()
            except:
                pass

            self.update_info()
            self.update_status("已刪除障礙物")
            logger.info(f"已刪除障礙物")
        else:
            self.update_status("未找到附近的障礙物")

        # 退出刪除模式
        self.exit_delete_mode()

    def clear_all_obstacles(self):
        """清除所有障礙物"""
        for obstacle in self.obstacle_manager.obstacles[:]:
            # 從paths移除
            if obstacle.polygon and obstacle.polygon in self.app.paths:
                self.app.paths.remove(obstacle.polygon)
            if obstacle.safe_polygon and obstacle.safe_polygon in self.app.paths:
                self.app.paths.remove(obstacle.safe_polygon)

            # 刪除顯示
            try:
                for marker in obstacle.markers:
                    marker.delete()
                if obstacle.polygon:
                    obstacle.polygon.delete()
                if obstacle.safe_polygon:
                    obstacle.safe_polygon.delete()
            except:
                pass

        # 清除當前創建中的障礙物
        if self.obstacle_manager.current_creating_obstacle:
            current = self.obstacle_manager.current_creating_obstacle
            for marker in current.markers:
                try:
                    marker.delete()
                except:
                    pass

        self.obstacle_manager.clear_all()
        self.update_info()
        self.update_status("")
        logger.info("已清除所有障礙物")

    def update_info(self):
        """更新資訊"""
        count = len(self.obstacle_manager.obstacles)
        self.info_label.config(text=f"目前障礙物: {count} 個")

    def update_status(self, text: str):
        """更新狀態顯示"""
        self.status_label.config(text=text)

    def apply_obstacle_avoidance(self, waypoints, boundary_corners=None):
        """應用障礙物避讓 (傳入邊界)"""
        if not self.obstacle_manager.obstacles:
            return waypoints

        logger.info(f"應用障礙物繞行: {len(self.obstacle_manager.obstacles)} 個")
        return self.obstacle_manager.filter_waypoints_with_detour(waypoints, boundary_corners)
