import sys
import os
import subprocess
import time
import numpy as np
import cv2
import math

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, 
    QPushButton, QComboBox, QFormLayout, QHBoxLayout, QLineEdit, QGroupBox, QCheckBox
)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage

from astar import AStarPlanner
from dijkstra import DijkstraPlanner

class GazeboStreamWorker(QThread):
    frame_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.running = True
        self.node = Node()
        self.node.subscribe(GzImage, "/simulator/camera/image", self.cb)

    def cb(self, msg):
        try:
            dtype = np.uint8
            img = np.frombuffer(msg.data, dtype=dtype)
            channels = 3 if len(msg.data) == msg.height * msg.width * 3 else 1
            img = img.reshape((msg.height, msg.width, channels))
            if channels == 3:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.frame_signal.emit(img)
        except Exception:
            pass

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class CleanAppDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.grid_size = 20.0
        self.show_auras = False

        self.setWindowTitle(f"Pathfinding Simulation & Control Panel ({int(self.grid_size)}x{int(self.grid_size)} Grid)")
        self.resize(1150, 720)

        main_layout = QHBoxLayout(self)

        # --- LEFT PANEL: Controls & Parameters ---
        control_group = QGroupBox("Configuration & Controls")
        control_layout = QVBoxLayout(control_group)
        form_layout = QFormLayout()

        self.algo_combo = QComboBox()
        self.algo_combo.addItems(["A* Algorithm", "Dijkstra"])
        form_layout.addRow("Algorithm:", self.algo_combo)

        self.start_x = QLineEdit("2.0")
        self.start_y = QLineEdit("2.0")
        form_layout.addRow("Start X:", self.start_x)
        form_layout.addRow("Start Y:", self.start_y)

        self.goal_x = QLineEdit("18.0")
        self.goal_y = QLineEdit("18.0")
        form_layout.addRow("Goal X:", self.goal_x)
        form_layout.addRow("Goal Y:", self.goal_y)

        self.chk_auras = QCheckBox("Show Obstacle Safety Auras")
        self.chk_auras.setChecked(False)
        self.chk_auras.toggled.connect(self.toggle_auras)
        form_layout.addRow("Overlay:", self.chk_auras)

        control_layout.addLayout(form_layout)

        btn_layout_top = QHBoxLayout()
        self.btn_run = QPushButton("Run Simulation")
        self.btn_run.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; padding: 10px;")
        self.btn_run.clicked.connect(self.start_simulation)
        btn_layout_top.addWidget(self.btn_run)

        self.btn_check = QPushButton("Check Positions")
        self.btn_check.setStyleSheet("background-color: #0277BD; color: white; font-weight: bold; padding: 10px;")
        self.btn_check.clicked.connect(self.check_positions)
        btn_layout_top.addWidget(self.btn_check)
        control_layout.addLayout(btn_layout_top)

        self.btn_clear = QPushButton("Clear Path / Reset")
        self.btn_clear.setStyleSheet("background-color: #C62828; color: white; font-weight: bold; padding: 10px;")
        self.btn_clear.clicked.connect(self.clear_map)
        control_layout.addWidget(self.btn_clear)

        self.status_label = QLabel("Status: Map loaded on startup. Ready.")
        control_layout.addWidget(self.status_label)
        
        self.runtime_label = QLabel("Runtime: -- ms")
        self.runtime_label.setStyleSheet("font-weight: bold; color: #0277BD; font-size: 13px; margin-top: 5px;")
        control_layout.addWidget(self.runtime_label)
        
        control_layout.addStretch()

        main_layout.addWidget(control_group, stretch=1)

        # --- RIGHT PANEL: Simulation View Canvas ---
        view_group = QGroupBox(f"Simulation View ({int(self.grid_size)}x{int(self.grid_size)} Grid)")
        view_layout = QVBoxLayout(view_group)
        
        self.video_label = QLabel("Loading Gazebo Map...")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: #111111; color: #888888; font-size: 14px;")
        self.video_label.setFixedSize(640, 640)
        
        view_layout.addWidget(self.video_label, alignment=Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(view_group, stretch=2)

        self.gz_process = None
        self.stream_worker = None
        
        self.explored_nodes = []
        self.calculated_path = []
        self.explore_index = 0
        self.path_anim_index = 0
        self.is_exploring = False
        self.execution_time_ms = 0.0

        self.obstacles = self.load_obstacles_from_file()
        self.generate_sdf_from_obstacles()
        self.launch_preview_gazebo()

    def toggle_auras(self, checked):
        self.show_auras = checked

    def load_obstacles_from_file(self):
        obstacles = []
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "obstacles.txt")
        if not os.path.exists(file_path):
            file_path = r"C:\Simulator\obstacles.txt"

        if not os.path.exists(file_path):
            return obstacles

        try:
            with open(file_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = [p.strip() for p in line.split(",")]
                    nums = []
                    for p in parts:
                        try:
                            nums.append(float(p))
                        except ValueError:
                            pass
                    if len(nums) >= 5:
                        cx, cy, sx, sy, yaw = nums[:5]
                        obstacles.append({
                            'cx': cx, 'cy': cy,
                            'sx': sx, 'sy': sy,
                            'yaw': yaw
                        })
        except Exception as e:
            print(f"[Error] Failed to parse obstacles.txt: {e}")
        
        return obstacles

    def generate_sdf_from_obstacles(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        sdf_path = os.path.join(base_dir, "pathfinding_2d.sdf")

        sdf_content = [
            '<?xml version="1.0" ?>',
            '<sdf version="1.6">',
            '  <world name="pathfinding_2d">',
            '    <physics name="1ms" type="ignored">',
            '      <max_step_size>0.001</max_step_size>',
            '      <real_time_factor>1.0</real_time_factor>',
            '    </physics>',
            '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
            '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
            '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
            '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
            '      <render_engine>ogre</render_engine>',
            '    </plugin>',
            '    <light type="directional" name="sun">',
            '      <pose>10 10 20 0 0 0</pose>',
            '      <diffuse>0.8 0.8 0.8 1</diffuse>',
            '      <specular>0.2 0.2 0.2 1</specular>',
            '      <direction>-0.5 0.1 -0.9</direction>',
            '      <cast_shadows>true</cast_shadows>',
            '    </light>',
            '    <model name="ground_plane">',
            '      <static>true</static>',
            '      <pose>10 10 0 0 0 0</pose>',
            '      <link name="link">',
            '        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>',
            '        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>',
            '          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>'
        ]

        colors = [
            "0.9 0.2 0.2 1", "0.2 0.5 0.9 1", "0.9 0.6 0.1 1",
            "0.5 0.2 0.8 1", "0.2 0.8 0.8 1", "0.8 0.2 0.8 1",
            "0.8 0.5 0.2 1", "0.2 0.8 0.3 1", "0.3 0.7 0.9 1",
            "0.9 0.4 0.6 1", "0.4 0.8 0.2 1", "0.8 0.3 0.3 1"
        ]

        for idx, obs in enumerate(self.obstacles):
            cx, cy, sx, sy, yaw = obs['cx'], obs['cy'], obs['sx'], obs['sy'], obs['yaw']
            color = colors[idx % len(colors)]
            
            obstacle_xml = f"""
    <model name="obstacle_{idx+1}">
      <pose>{cx} {cy} 0.05 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} 0.1</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} 0.1</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse><emissive>{color}</emissive></material>
        </visual>
      </link>
    </model>"""
            sdf_content.append(obstacle_xml)

        sdf_content.append("""
    <model name="overhead_camera">
      <static>true</static>
      <pose>10.0 10.0 20.0 0 1.57079632679 1.57079632679</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>0.927295218</horizontal_fov>
            <image><width>640</width><height>640</height></image>
            <clip><near>0.1</near><far>100</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
          <topic>simulator/camera/image</topic>
        </sensor>
      </link>
    </model>
  </world>
</sdf>""")

        try:
            with open(sdf_path, "w") as f:
                f.write("\n".join(sdf_content))
            print(f"[Success] Generated pathfinding_2d.sdf with {len(self.obstacles)} obstacles from obstacles.txt")
        except Exception as e:
            print(f"[Error] Failed to generate pathfinding_2d.sdf: {e}")

    def launch_preview_gazebo(self):
        if self.gz_process is not None:
            return

        subprocess.run(["taskkill", "/f", "/t", "/im", "gz-sim-main.exe"], capture_output=True, shell=True)

        base_dir = os.path.dirname(os.path.abspath(__file__))
        world_file = os.path.join(base_dir, "pathfinding_2d.sdf")
        
        self.gz_process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", world_file], 
            shell=True
        )

        self.stream_worker = GazeboStreamWorker()
        self.stream_worker.frame_signal.connect(self.update_frame)
        self.stream_worker.start()

    def is_point_colliding(self, pt):
        x, y = pt
        margin = 0.25

        for obs in self.obstacles:
            cx, cy = obs['cx'], obs['cy']
            half_w = (obs['sx'] / 2.0) + margin
            half_h = (obs['sy'] / 2.0) + margin
            yaw = obs['yaw']

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

    def check_positions(self):
        try:
            sx = float(self.start_x.text())
            sy = float(self.start_y.text())
            gx = float(self.goal_x.text())
            gy = float(self.goal_y.text())
        except ValueError:
            self.status_label.setText("Status: Error - Invalid coordinate format!")
            return

        start_inside = self.is_point_colliding((sx, sy))
        goal_inside = self.is_point_colliding((gx, gy))

        msg_parts = []
        if start_inside:
            msg_parts.append("⚠️ START is INSIDE an obstacle or aura!")
        else:
            msg_parts.append("✅ START is safe.")

        if goal_inside:
            msg_parts.append("⚠️ GOAL is INSIDE an obstacle or aura!")
        else:
            msg_parts.append("✅ GOAL is safe.")

        self.status_label.setText(" | ".join(msg_parts))

    def start_simulation(self):
        try:
            sx = float(self.start_x.text())
            sy = float(self.start_y.text())
            gx = float(self.goal_x.text())
            gy = float(self.goal_y.text())
        except ValueError:
            self.status_label.setText("Status: Error - Invalid coordinates!")
            return

        if self.is_point_colliding((sx, sy)) or self.is_point_colliding((gx, gy)):
            self.status_label.setText("Status: Cannot run! Position inside obstacle safety aura.")
            return

        selected_algo = self.algo_combo.currentText()
        self.status_label.setText(f"Status: Exploring space ({selected_algo})...")
        self.btn_run.setEnabled(False)

        start_pos = (sx, sy)
        goal_pos = (gx, gy)

        t_start = time.perf_counter()

        if selected_algo == "A* Algorithm":
            planner = AStarPlanner(resolution=0.5)
            self.explored_nodes, self.calculated_path = planner.plan(
                start_pos, goal_pos, self.obstacles, 
                x_range=(0, self.grid_size), 
                y_range=(0, self.grid_size)
            )
        elif selected_algo == "Dijkstra":
            planner = DijkstraPlanner(resolution=0.5)
            self.explored_nodes, self.calculated_path = planner.plan(
                start_pos, goal_pos, self.obstacles, 
                x_range=(0, self.grid_size), 
                y_range=(0, self.grid_size)
            )

        t_end = time.perf_counter()
        self.execution_time_ms = (t_end - t_start) * 1000.0
        self.runtime_label.setText(f"Runtime ({selected_algo}): {self.execution_time_ms:.2f} ms")

        self.explore_index = 0
        self.path_anim_index = 0
        self.is_exploring = True

    def clear_map(self):
        self.explored_nodes = []
        self.calculated_path = []
        self.explore_index = 0
        self.path_anim_index = 0
        self.is_exploring = False
        self.execution_time_ms = 0.0
        self.status_label.setText("Status: Path cleared. Map running.")
        self.runtime_label.setText("Runtime: -- ms")
        self.btn_run.setEnabled(True)

    def world_to_pixel(self, wx, wy, image_width=640, image_height=640):
        scale = image_width / self.grid_size
        px = int(round(wx * scale))
        py = int(round(image_height - (wy * scale)))
        return (px, py)

    def draw_positive_grid(self, img):
        h, w, _ = img.shape
        scale = w / self.grid_size

        step = 2 if self.grid_size <= 20 else 5
        for x_m in range(0, int(self.grid_size) + 1, step):
            px = int(x_m * scale)
            if 0 <= px <= w:
                cv2.line(img, (px, 0), (px, h), (45, 45, 45), 1)
                cv2.putText(img, f"{x_m}", (px - 8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        for y_m in range(0, int(self.grid_size) + 1, step):
            py = int(h - (y_m * scale))
            if 0 <= py <= h:
                cv2.line(img, (0, py), (w, py), (45, 45, 45), 1)
                cv2.putText(img, f"{y_m}", (5, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        cv2.putText(img, "(0,0)", (5, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    def draw_obstacle_auras(self, overlay):
        if not self.show_auras:
            return

        aura_margin = 0.25

        for obs in self.obstacles:
            cx, cy = obs['cx'], obs['cy']
            hw = (obs['sx'] / 2.0) + aura_margin
            hh = (obs['sy'] / 2.0) + aura_margin
            yaw = obs['yaw']

            corners_local = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

            pixel_corners = []
            for lx, ly in corners_local:
                if yaw != 0.0:
                    cos_y = math.cos(yaw)
                    sin_y = math.sin(yaw)
                    wx = cx + (lx * cos_y - ly * sin_y)
                    wy = cy + (lx * sin_y + ly * cos_y)
                else:
                    wx = cx + lx
                    wy = cy + ly

                px, py = self.world_to_pixel(wx, wy)
                pixel_corners.append([px, py])

            pts = np.array(pixel_corners, np.int32).reshape((-1, 1, 2))
            cv2.polylines(overlay, [pts], isClosed=True, color=(255, 255, 255), thickness=2)

    def update_frame(self, cv_img):
        self.draw_positive_grid(cv_img)

        overlay = cv_img.copy()

        self.draw_obstacle_auras(overlay)

        try:
            sx, sy = float(self.start_x.text()), float(self.start_y.text())
            gx, gy = float(self.goal_x.text()), float(self.goal_y.text())
            spx, spy = self.world_to_pixel(sx, sy)
            gpx, gpy = self.world_to_pixel(gx, gy)
            
            start_color = (0, 0, 255) if self.is_point_colliding((sx, sy)) else (255, 255, 0)
            goal_color = (0, 0, 255) if self.is_point_colliding((gx, gy)) else (255, 0, 255)

            cv2.circle(overlay, (spx, spy), 8, start_color, -1)
            cv2.putText(overlay, "S", (spx-4, spy+4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
            cv2.circle(overlay, (gpx, gpy), 8, goal_color, -1)
            cv2.putText(overlay, "G", (gpx-4, gpy+4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        except ValueError:
            pass

        if self.is_exploring and self.explored_nodes:
            chunk_size = max(1, len(self.explored_nodes) // 50)
            end_idx = min(self.explore_index + chunk_size, len(self.explored_nodes))
            
            for i in range(end_idx):
                ex, ey = self.explored_nodes[i]
                epx, epy = self.world_to_pixel(ex, ey)
                cv2.circle(overlay, (epx, epy), 6, (255, 180, 120), -1)

            self.explore_index = end_idx
            
            if self.explore_index >= len(self.explored_nodes):
                self.is_exploring = False
                self.status_label.setText(f"Status: Path found ({self.execution_time_ms:.2f} ms). Traversing...")

        elif not self.is_exploring and self.explored_nodes:
            for ex, ey in self.explored_nodes:
                epx, epy = self.world_to_pixel(ex, ey)
                cv2.circle(overlay, (epx, epy), 6, (255, 180, 120), -1)

            if len(self.calculated_path) > 1:
                pixel_points = [self.world_to_pixel(pt[0], pt[1]) for pt in self.calculated_path]
                
                for i in range(len(pixel_points) - 1):
                    cv2.line(overlay, pixel_points[i], pixel_points[i+1], (0, 220, 0), 3)

                start_px = pixel_points[0]
                goal_px = pixel_points[-1]
                
                cv2.circle(overlay, start_px, 8, (255, 255, 0), -1)
                cv2.putText(overlay, "START", (start_px[0]-10, start_px[1]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                cv2.circle(overlay, goal_px, 8, (255, 0, 255), -1)
                cv2.putText(overlay, "GOAL", (goal_px[0]-10, goal_px[1]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

                if self.path_anim_index < len(pixel_points):
                    robot_px = pixel_points[self.path_anim_index]
                    cv2.circle(overlay, robot_px, 9, (0, 255, 255), -1)
                    cv2.circle(overlay, robot_px, 14, (0, 165, 255), 2)
                    self.path_anim_index += 1
                else:
                    self.path_anim_index = len(pixel_points) - 1
                    if "Completed" not in self.status_label.text():
                        self.status_label.setText(f"Status: Completed ({self.execution_time_ms:.2f} ms)")
                        self.btn_run.setEnabled(True)

        if self.execution_time_ms > 0.0:
            cv2.putText(overlay, f"Runtime: {self.execution_time_ms:.2f} ms", (450, 625), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        cv2.addWeighted(overlay, 0.45, cv_img, 0.55, 0, cv_img)

        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_image = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

    def closeEvent(self, event):
        if self.stream_worker:
            self.stream_worker.stop()
        if self.gz_process:
            try:
                subprocess.run(f"taskkill /f /t /pid {self.gz_process.pid}", capture_output=True, shell=True)
            except Exception:
                pass
        subprocess.run(["taskkill", "/f", "/t", "/im", "gz-sim-main.exe"], capture_output=True, shell=True)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    dashboard = CleanAppDashboard()
    dashboard.show()
    sys.exit(app.exec())