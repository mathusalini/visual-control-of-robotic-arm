import math
import heapq
import time
import serial

# =================================================================================
#   PART 1: REAL ROBOT INTERFACE
# =================================================================================

class RealRobot:
    def __init__(self, port='COM3', baudrate=115200):
        print(f"Attempting to connect to robot on {port}...")
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2) # Wait for Arduino to reset/boot
            print(f"SUCCESS: Connected to {port}")
            self.arduino.reset_input_buffer()
        except Exception as e:
            print(f"ERROR: Could not connect to Arduino. Running in DUMMY MODE.\nDetails: {e}")
            self.arduino = None

        # Dead Reckoning State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update = time.time()

    def set_velocity(self, vx, vy, omega):
        # 1. Update Position Estimate
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += omega * dt

        # 2. Send Command
        if self.arduino:
            # Format: <vx,vy,omega>
            command = f"<{vx:.3f},{vy:.3f},{omega:.3f}>"
            try:
                self.arduino.write(command.encode())
            except Exception:
                pass

    def get_pose(self):
        return [self.x, self.y, self.theta]

    def stop(self):
        self.set_velocity(0, 0, 0)
        print("Robot Stopped.")

# =================================================================================
#   PART 2: CONFIGURATION
# =================================================================================

GRID_RES    = 0.25           # Meters per cell
FLOOR_SIZE  = 2.0          # 10x10 meters
GRID_SIZE   = int(FLOOR_SIZE / GRID_RES) # 20x20 Grid

# PID / Control Tuning
ANGLE_TOL   = 0.15          
KP_LIN      = 0.8           
KP_YAW      = 2.0           
MAX_VEL     = 0.3           # m/s
MAX_OMEGA   = 1.0           # rad/s

# =================================================================================
#   PART 3: D* LITE PATH PLANNER
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
        return math.hypot(u[0]-v[0],u[1]-v[1])

    def calculate_key(self,s):
        k1 = min(self.g[s],self.rhs[s]) + self.heuristic(self.start,s) + self.km
        k2 = min(self.g[s],self.rhs[s])
        return (k1, k2)

    def update_vertex(self,u):
        if u != self.goal:
            self.rhs[u] = min([self.g[n]+self.heuristic(u,n) for n in self.neighbors(u) if self.is_free(n)] or [float('inf')])
        
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
        while s!=self.goal:
            nbrs=[n for n in self.neighbors(s) if self.is_free(n)]
            if not nbrs: return None
            s=min(nbrs,key=lambda n:self.g[n]+self.heuristic(s,n))
            path.append(s)
            if len(path) > 200: break 
        return path

# =================================================================================
#   PART 4: HELPERS
# =================================================================================

def real_to_grid(x, y):
    gx = int(math.floor(x / GRID_RES))
    gy = int(math.floor(y / GRID_RES))
    return max(0,min(GRID_SIZE-1,gy)), max(0,min(GRID_SIZE-1,gx))

def grid_to_world(cell):
    row, col = cell 
    x = (col + 0.5) * GRID_RES 
    y = (row + 0.5) * GRID_RES 
    return x, y

def wrap_pi(a): return (a + math.pi) % (2 * math.pi) - math.pi
def clamp(v, lo, hi): return max(lo, min(hi, v))

# =================================================================================
#   PART 5: MAIN EXECUTION
# =================================================================================

if __name__ == "__main__":
    
    # 1. SETUP
    robot = RealRobot(port='COM3') # <--- CHECK YOUR PORT
    
    # 2. USER CONFIGURATION
    START_GRID  = (7, 7)    
    TARGET_GRID = (0, 0)    
    
    OBSTACLE_CELLS = [
        (5, 1),(1,1)
         
    ]
    
    # 3. INITIALIZATION
    sx, sy = grid_to_world(START_GRID)
    robot.x = sx
    robot.y = sy
    robot.theta = 0.0 
    
    grid_map = [[1]*GRID_SIZE for _ in range(GRID_SIZE)]
    for obs in OBSTACLE_CELLS:
        r, c = obs
        if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
            grid_map[r][c] = 0 

    planner = DStarLite(grid_map, START_GRID, TARGET_GRID)
    tx, ty = grid_to_world(TARGET_GRID)

    # -------------------------------------------------------------
    #   PRINT PATH BEFORE MOVING
    # -------------------------------------------------------------
    print("\n" + "="*40)
    print("       INITIAL PATH PLAN GENERATED")
    print("="*40)
    
    initial_path = planner.path()
    
    if initial_path:
        print(f"Start Point: {START_GRID}")
        print(f"Target Point: {TARGET_GRID}")
        print(f"Path Length: {len(initial_path)} steps\n")
        print("Full Path (Row, Col):")
        print(initial_path)
    else:
        print("CRITICAL ERROR: No path found! Check obstacles.")
        exit()
        
    print("="*40)
    print("Starting Navigation in 5 seconds...")
    time.sleep(5)
    
    # 4. CONTROL LOOP
    try:
        while True:
            # A. Get State
            rx, ry, rtheta = robot.get_pose()
            current_cell = real_to_grid(rx, ry)
            
            # B. Check Goal
            dist_to_goal = math.hypot(tx - rx, ty - ry)
            if dist_to_goal < 0.15: 
                print("SUCCESS: Goal Reached!")
                robot.stop()
                break
                
            if current_cell == TARGET_GRID:
                print("SUCCESS: Entered Target Cell!")
                robot.stop()
                break

            # C. Update Path
            planner.start = current_cell
            path_cells = planner.path()
            
            if not path_cells or len(path_cells) < 2:
                print("Path obstructed or finished.")
                robot.stop()
                break
            
            # D. Pure Pursuit Control
            target_cell = path_cells[1] 
            wx, wy = grid_to_world(target_cell)
            
            dx = wx - rx
            dy = wy - ry
            
            vx_world = KP_LIN * dx
            vy_world = KP_LIN * dy
            
            vmag = math.hypot(vx_world, vy_world)
            if vmag > MAX_VEL:
                scale = MAX_VEL / vmag
                vx_world *= scale
                vy_world *= scale
            
            # Robot Frame Transform
            vx_local = vx_world * math.cos(rtheta) + vy_world * math.sin(rtheta)
            vy_local = -vx_world * math.sin(rtheta) + vy_world * math.cos(rtheta)
            
            # Heading Control
            omega = clamp(KP_YAW * wrap_pi(0.0 - rtheta), -MAX_OMEGA, MAX_OMEGA)
            
            # Send
            robot.set_velocity(vx_local, vy_local, omega)
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\nEMERGENCY STOP")
        robot.stop()