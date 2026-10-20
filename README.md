# 2D Pathfinding Simulation & Control Panel (PyQt6 & Gazebo Harmonic)

An interactive 2D pathfinding simulation dashboard developed for bipedal robotics and motion planning research. This project couples a **PyQt6 control GUI** with **Gazebo Harmonic** via OpenCV image streaming over Gazebo Transport (`gz-transport13`).

The platform uses a single source of truth for obstacle management, dynamically compiling world files, overlaying configuration-space ($\mathcal{C}$-space) safety margins, and benchmarking real-time path search algorithms ($A^*$ vs. Dijkstra).

---

## 📸 Key Capabilities & Features

* **Single Source of Truth (`obstacles.txt`):** No hardcoded SDF XML blocks. All obstacle positions, dimensions, and orientations (`cx, cy, sx, sy, yaw`) are maintained strictly inside `obstacles.txt`.
* **Dynamic World Compiler:** `main.py` parses `obstacles.txt` on startup and dynamically generates `pathfinding_2d.sdf` before spawning Gazebo.
* **Calibrated Parallax-Free Camera:** An overhead camera mounted at $Z = 20.0\text{ m}$ with an adjusted horizontal FOV ($0.927295\text{ rad}$) and orthogonal projection eliminates perspective parallax, ensuring a 1:1 pixel overlay match between OpenCV projections and 3D Gazebo space.
* **Dynamic Grid & Axis Inversion Correction:** Fully handles OpenCV Y-axis image coordinate inversion (`py = height - wy * scale`) and renders a $20 \times 20\text{ m}$ positive spatial grid overlay.
* **Safety Aura Overlays ($\mathcal{C}$-space):** Includes an interactive UI checkbox to toggle white Oriented Bounding Box (OBB) safety frames ($0.25\text{ m}$ margin) rendered directly over obstacles.
* **Algorithm Performance Comparison:** Supports real-time execution of both $A^*$ **Algorithm** (Euclidean distance heuristic) and **Dijkstra** (uninformed search) with millisecond-precision runtime benchmarks (`time.perf_counter()`).
* **Research Benchmark Map:** Includes a 24-element obstacle world replicating the environment from *Hornung et al. (IEEE-RAS Humanoids 2012)* for footstep planning evaluation against local minima.

---

## 💻 Environment Setup & Dependency Installation

### Option A: Using Pixi (Automated Environment - Recommended)

1. **Install Pixi:**
   ```powershell
   iwr -useb https://pixi.sh/install.ps1 | iex
   ```

2. **Verify Pixi Installation:**
   ```powershell
   pixi --version
   ```

3. **Initialize Pixi Project from Scratch:**
   ```bash
   # Navigate to your project root folder
   cd C:\Simulator

   # Initialize a new Pixi project (creates pixi.toml)
   pixi init
   ```

4. **Add Dependencies to Pixi Environment:**
   ```bash
   # Add Python and Core Computer Vision / GUI Packages
   pixi add python pyqt numpy opencv

   # Add PyPI Dependencies (Gazebo Transport & Message Bindings)
   pixi add --pypi gz-transport13 gz-msgs10

   # Install all configured dependencies into the local environment (.pixi)
   pixi install
   ```

## 📁 Project Directory Structure

```text
gazebo-pathfinding-simulator/
├── main.py            # Main PyQt6 control panel, Gazebo subscriber, & SDF compiler
├── astar.py           # Custom A* grid planner with OBB clearance checking
├── dijkstra.py        # Custom Dijkstra planner (uninformed search baseline)
├── obstacles.txt      # Source of truth for map obstacle configurations
├── pathfinding_2d.sdf # Dynamically generated Gazebo world file (auto-built on startup)
├── pixi.toml          # Pixi environment configuration file
└── README.md          # Project documentation
```

---

## 🚀 How to Run the Simulator

### Step 1: Terminate Existing Background Processes

To prevent port locks or dual-instance conflicts with Gazebo Transport, clear any lingering Gazebo processes:

* **Windows (PowerShell / Command Prompt):**
  ```powershell
  taskkill /f /t /im gz-sim-main.exe
  ```
* **Linux / macOS:**
  ```bash
  killall -9 gz-sim-main
  ```

### Step 2: Launch the Simulation Dashboard

Run the main application entry point using Pixi or Python:

```bash
# Using Pixi (Recommended)
pixi run python main.py

```

---

## 🎮 Interactive Dashboard & Control Panel

The dashboard is split into two primary panels: the **Configuration & Controls** sidebar on the left and the **640x640 Simulation View Canvas** on the right.

### Control Panel Interface (`Left Window`)

1. **Algorithm Selection Dropdown (`QComboBox`):**
   * **`A* Algorithm`:** Runs grid search weighted by Euclidean distance heuristic ($h(n) = \sqrt{\Delta x^2 + \Delta y^2}$). Highly efficient directed search.
   * **`Dijkstra`:** Runs unweighted grid search ($h(n) = 0$). Explores radially in all directions to demonstrate baseline search tree expansion volume.

2. **Coordinate Inputs (`QLineEdit`):**
   * **`Start X` / `Start Y`:** Set starting agent pose in meters (Default: `2.0`, `2.0` | Benchmark: `10.0`, `0.8`).
   * **`Goal X` / `Goal Y`:** Set target destination pose in meters (Default: `18.0`, `18.0` | Benchmark: `6.0`, `17.0`).

3. **Overlay Settings Checkbox (`QCheckBox`):**
   * **`Show Obstacle Safety Auras`:** Toggles white Oriented Bounding Box (OBB) outline overlays ($0.25\text{ m}$ clearance margin) directly over rendered obstacles on the canvas (Default: Unchecked/OFF).

4. **Action Buttons:**
   * **`Check Positions`:** Validates whether the current Start and Goal coordinates lie inside any obstacle bounding box or safety aura margin before executing search.
   * **`Run Simulation`:** Starts pathfinding execution, triggers real-time search node expansion rendering, traces the solved trajectory, and animates agent traversal.
   * **`Clear Path / Reset`:** Clears all rendered exploration nodes, solved paths, trajectory animations, and resets execution metrics.

5. **Telemetry & Runtime Readout:**
   * **`Status` Label:** Displays system state (`Ready`, `Exploring...`, `Traversing...`, `Completed`).
   * **`Runtime` Label:** Displays exact algorithm execution time in milliseconds (`Runtime (A* Algorithm): XX.XX ms`) computed via `time.perf_counter()`.

---

### Simulation View Canvas (`Right Window`)

* **Live Stream Viewer:** Receives 30 FPS RGB video frames directly from Gazebo's `/simulator/camera/image` topic over `gz-transport13`.
* **Coordinate Grid System:** Draws a positive $20 \times 20\text{ m}$ spatial grid overlay with labeled axis meters and $(0,0)$ origin at the bottom-left corner.
* **Node Expansion Visualization:** Displays light-blue candidate search nodes as the algorithm expands outward through free space.
* **Trajectory Animation:** Renders the computed green trajectory path once solved, along with an animated yellow agent marker tracking step-by-step progress from Start (`S`) to Goal (`G`).
* **HUD Performance Badge:** Renders an on-canvas runtime readout (`Runtime: XX.XX ms`) in the bottom-right corner of the video frame.

---

## 📊 Modifying Obstacles

To add, remove, or modify obstacles in the world, edit `obstacles.txt` directly. Do **not** edit `pathfinding_2d.sdf` manually.

### Format in `obstacles.txt`:

```text
# Format: type, x_center, y_center, width_x, height_y, yaw_radians
box, 10.0, 7.0, 1.5, 2.5, 0.0
box, 14.0, 11.0, 3.0, 0.6, 0.785
```

When `main.py` launches, it parses `obstacles.txt`, updates internal collision-checking structures, and automatically rewrites `pathfinding_2d.sdf` for Gazebo.
