import math
import heapq
import time
import serial

# =================================================================================
#   PART 1: REAL ROBOT INTERFACE (THE "DRIVER")
#   - Establishes Serial Connection
#   - Sends Velocity Commands
#   - Estimates Position (Dead Reckoning)
# =================================================================================

class RealRobot:
    def __init__(self, port='COM3', baudrate=115200):
        print(f"Attempting to connect to robot on {port}...")
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2) # Wait for Arduino to reset/boot
            print(f"SUCCESS: Connected to {port}")
            # Clear buffer
            self.arduino.reset_input_buffer()
        except Exception as e:
            print(f"ERROR: Could not connect to Arduino. Running in DUMMY MODE.\nDetails: {e}")
            self.arduino = None

        # Dead Reckoning State (Where we think we are)
        # Note: In a real advanced robot, this comes from Encoder Feedback
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update = time.time()

    def set_velocity(self, vx, vy, omega):
        """
        Sends velocity command <vx, vy, omega> to Arduino.
        Also updates internal position estimate.
        """
        # 1. Update our estimated position based on previous command duration
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        # Simple Euler Integration (Estimating position change)
        # x_new = x_old + (v_x_world * dt)
        # v_x_world = vx * cos(theta) - vy * sin(theta)
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += omega * dt

        # 2. Send command to Arduino
        if self.arduino:
            # Format: <vx,vy,omega> e.g., <0.200,0.000,0.500>
            command = f"<{vx:.3f},{vy:.3f},{omega:.3f}>"
            try:
                self.arduino.write(command.encode())
            except Exception as e:
                print(f"Serial Write Error: {e}")

    def get_pose(self):
        """Returns [x, y, theta]"""
        return [self.x, self.y, self.theta]

    def stop(self):
        self.set_velocity(0, 0, 0)
        print("Robot Stopped.")

# =================================================================================
#   PART 2: CONFIGURATION & CONSTANTS
# =================================================================================

GRID_RES    = 0.25           # Each grid cell is 0.5 meters x 0.5 meters
FLOOR_SIZE  = 2.0          # Total map size (10m x 10m)
GRID_SIZE   = int(FLOOR_SIZE / GRID_RES) # 20x20 Grid

# PID / Pure Pursuit Tuning
ANGLE_TOL   = 0.15          # Radians (~8 degrees) tolerance
KP_LIN      = 0.8           # Proportional Gain for Linear Speed
KP_YAW      = 2.0           # Proportional Gain for Rotation
MAX_VEL     = 0.3           # Max Linear Speed (m/s) - Keep low for testing!
MAX_OMEGA   = 1.0           # Max Rotational Speed (rad/s)

# =================================================================================
#   PART 3: D* LITE PATH PLANNER (THE "BRAIN")
# =================================================================================

class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal  = goal
        self.km    = 0
        self.g     = {}
        self.rhs   = {}
        self.U     = []
        # 8-Directional Movement (Up, Down, Left, Right, Diagonals)
        self.moves = [(0,1),(0,-1),(1,0),(-1,0), (1,1),(1,-1),(-1,1),(-1,-1)]
        
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                self.g[(r,c)] = float('inf')
                self.rhs[(r,c)] = float('inf')
        
        self.rhs[goal] = 0
        heapq.heappush(self.U,(self.calculate_key(goal),goal))
        self.last = start
        self.compute_shortest_path()

    def heuristic(self,u,v): 
        # Euclidean distance heuristic
        return math.hypot(u[0]-v[0],u[1]-v[1])

    def calculate_key(self,s):
        k1 = min(self.g[s],self.rhs[s]) + self.heuristic(self.start,s) + self.km
        k2 = min(self.g[s],self.rhs[s])
        return (k1, k2)

    def update_vertex(self,u):
        if u != self.goal:
            self.rhs[u] = min([self.g[n]+self.heuristic(u,n) for n in self.neighbors(u) if self.is_free(n)] or [float('inf')])
        
        # Remove u from priority queue if it exists
        if [x for x in self.U if x[1]==u]: 
            self.U = [x for x in self.U if x[1]!=u]
            heapq.heapify(self.U)
            
        if self.g[u] != self.rhs[u]: 
            heapq.heappush(self.U,(self.calculate_key(u),u))

    def neighbors(self,u):
        for d in self.moves:
            v=(u[0]+d[0],u[1]+d[1])
            if 0<=v[0]<GRID_SIZE and 0<=v[1]<GRID_SIZE: yield v

    def is_free(self,cell): 
        # 1 = Free, 0 = Obstacle
        return self.grid[cell[0]][cell[1]]==1

    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            if not self.U: break
            k_old,s = heapq.heappop(self.U)
            
            if k_old < self.calculate_key(s): 
                heapq.heappush(self.U,(self.calculate_key(s),s))
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for n in self.neighbors(s): self.update_vertex(n)
            else:
                self.g[s] = float('inf')
                self.update_vertex(s)
                for n in self.neighbors(s): self.update_vertex(n)

    def path(self):
        if self.g[self.start] == float('inf'): return None
        path=[self.start]; s=self.start
        
        # Simple gradient descent to reconstruct path
        while s!=self.goal:
            nbrs=[n for n in self.neighbors(s) if self.is_free(n)]
            if not nbrs: return None
            s=min(nbrs,key=lambda n:self.g[n]+self.heuristic(s,n))
            path.append(s)
            if len(path) > 200: break # Safety break for loops
        return path

# =================================================================================
#   PART 4: HELPER FUNCTIONS (COORDINATE TRANSFORMS)
# =================================================================================

def real_to_grid(x, y):
    """Converts Real World Meters to Grid Cell indices"""
    # Offset so (0,0) is in the middle of the room if desired, 
    # OR assuming (0,0) is bottom-left corner of the map.
    # Here assuming (0,0) real world is at Grid Cell (0,0) for simplicity.
    gx = int(math.floor(x / GRID_RES))
    gy = int(math.floor(y / GRID_RES))
    return max(0,min(GRID_SIZE-1,gy)), max(0,min(GRID_SIZE-1,gx))

def grid_to_world(cell):
    """Converts Grid Cell indices to Real World Meters (Center of cell)"""
    row, col = cell # row is Y, col is X usually in matrices
    # But let's map: Col -> X, Row -> Y
    x = (col + 0.5) * GRID_RES 
    y = (row + 0.5) * GRID_RES 
    return x, y

def wrap_pi(a): 
    return (a + math.pi) % (2 * math.pi) - math.pi

def clamp(v, lo, hi): 
    return max(lo, min(hi, v))

# =================================================================================
#   PART 5: MAIN EXECUTION
# =================================================================================

if __name__ == "__main__":
    
    # --- 1. SETUP ROBOT ---
    # !!! IMPORTANT: CHANGE 'COM3' TO YOUR ARDUINO PORT !!!
    robot = RealRobot(port='COM3') 
    
    # --- 2. USER CONFIGURATION (GRID SETTINGS) ---
    
    # A. DEFINE POINTS IN GRID COORDINATES (Col=X, Row=Y)
    # Using 0-19 scale (since GRID_SIZE=20)
    START_GRID  = (7, 7)    # Start near bottom-left
    TARGET_GRID = (0, 0)    # Go to middle-right
    
    # B. DEFINE OBSTACLES (List of Grid Cells)
    # Format: [(row, col), (row, col)...]
    OBSTACLE_CELLS = [
        (0, 1)
    ]
    
    # --- 3. INITIALIZATION ---

    # Set Robot Internal Position to Match Start Grid
    sx, sy = grid_to_world(START_GRID)
    robot.x = sx
    robot.y = sy
    robot.theta = 0.0 # Facing East
    
    # Initialize Map
    grid_map = [[1]*GRID_SIZE for _ in range(GRID_SIZE)]
    
    # Apply Obstacles
    print(f"Adding {len(OBSTACLE_CELLS)} obstacles...")
    for obs in OBSTACLE_CELLS:
        r, c = obs
        if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
            grid_map[r][c] = 0 # 0 means Blocked
            print(f" - Blocked Cell ({r}, {c})")

    # Initialize Planner
    planner = DStarLite(grid_map, START_GRID, TARGET_GRID)
    
    # Calculate Target World Coordinates
    tx, ty = grid_to_world(TARGET_GRID)

    print("-" * 40)
    print(f"Start: {START_GRID} (World: {sx:.2f}m, {sy:.2f}m)")
    print(f"Goal:  {TARGET_GRID} (World: {tx:.2f}m, {ty:.2f}m)")
    print("-" * 40)
    print("Starting Navigation in 3 seconds...")
    time.sleep(3)
    
    # --- 4. CONTROL LOOP ---
    try:
        while True:
            # A. Get Estimated Position
            rx, ry, rtheta = robot.get_pose()
            current_cell = real_to_grid(rx, ry)
            
            # B. Check Goal Status
            dist_to_goal = math.hypot(tx - rx, ty - ry)
            
            if dist_to_goal < 0.15: # 15cm Tolerance
                print("SUCCESS: Goal Reached!")
                robot.stop()
                break
                
            if current_cell == TARGET_GRID:
                print("SUCCESS: Entered Target Cell!")
                robot.stop()
                break

            # C. Re-Plan
            planner.start = current_cell
            path_cells = planner.path()
            
            if not path_cells or len(path_cells) < 2:
                print("No path found or goal too close/blocked.")
                robot.stop()
                break
            
            # D. Calculate Control Output (Pure Pursuit)
            target_cell = path_cells[1] # Next immediate Step
            wx, wy = grid_to_world(target_cell)
            
            dx = wx - rx
            dy = wy - ry
            
            # Linear Velocity (World Frame)
            vx_world = KP_LIN * dx
            vy_world = KP_LIN * dy
            
            # Speed Limiting
            vmag = math.hypot(vx_world, vy_world)
            if vmag > MAX_VEL:
                scale = MAX_VEL / vmag
                vx_world *= scale
                vy_world *= scale
            
            # E. Transform to Robot Local Frame
            # Vx_local = Forward, Vy_local = Strafe Left
            # Rotation Matrix:
            # [cos  sin]
            # [-sin cos]
            vx_local = vx_world * math.cos(rtheta) + vy_world * math.sin(rtheta)
            vy_local = -vx_world * math.sin(rtheta) + vy_world * math.cos(rtheta)
            
            # F. Rotation Control (Keep Facing Forward / 0.0 radians)
            target_yaw = 0.0 
            err_yaw = wrap_pi(target_yaw - rtheta)
            omega = clamp(KP_YAW * err_yaw, -MAX_OMEGA, MAX_OMEGA)
            
            # G. Send Command
            robot.set_velocity(vx_local, vy_local, omega)
            
            # Debug Print (Optional)
            # print(f"Pos: {rx:.2f},{ry:.2f} -> Tgt: {target_cell}")
            
            time.sleep(0.05) # 20Hz Loop

    except KeyboardInterrupt:
        print("\nEMERGENCY STOP TRIGGERED")
        robot.stop()
    except Exception as e:
        print(f"\nCRITICAL ERROR: {e}")
        robot.stop()