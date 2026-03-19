import heapq
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
import time

GRID_SIZE = 70


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(pos, grid):
    """Get valid 8-directional neighbors."""
    size = grid.shape[0]
    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]
    neighbors = []
    for dr, dc in directions:
        nr, nc = pos[0]+dr, pos[1]+dc
        if 0 <= nr < size and 0 <= nc < size and grid[nr][nc] == 0:
            cost = 1.414 if abs(dr)+abs(dc) == 2 else 1.0
            neighbors.append(((nr, nc), cost))
    return neighbors


def astar(grid, start, goal):
    """A* pathfinding — returns path or None."""
    size = grid.shape[0]
    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]
    move_cost = {(dr,dc): (1.414 if abs(dr)+abs(dc)==2 else 1.0)
                 for dr, dc in directions}

    open_set = [(heuristic(start, goal), 0, start, [start])]
    visited = set()

    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            return path
        for dr, dc in directions:
            nr, nc = current[0]+dr, current[1]+dc
            if 0 <= nr < size and 0 <= nc < size and grid[nr][nc] == 0:
                neighbor = (nr, nc)
                if neighbor not in visited:
                    new_g = g + move_cost[(dr,dc)]
                    heapq.heappush(open_set,
                        (new_g + heuristic(neighbor, goal), new_g, neighbor, path + [neighbor]))
    return None


class DynamicObstacleEnvironment:
    """
    Simulates a battlefield with dynamic obstacles.
    Obstacles can appear/disappear as UGV navigates.
    """
    def __init__(self, size, start, goal,
                 initial_density=0.15, dynamic_rate=0.02):
        self.size = size
        self.start = start
        self.goal = goal
        self.dynamic_rate = dynamic_rate
        self.grid = np.zeros((size, size), dtype=int)
        self.ugv_pos = start
        self.path_taken = [start]
        self.replans = 0
        self.total_nodes = 0
        self.steps = 0
        self.frames = []   
                   
        placed = 0
        target = int(size * size * initial_density)
        while placed < target:
            r, c = random.randint(0, size-1), random.randint(0, size-1)
            if (r,c) != start and (r,c) != goal and self.grid[r][c] == 0:
                self.grid[r][c] = 1
                placed += 1

        self.current_plan = astar(self.grid, self.start, self.goal)

    def update_dynamic_obstacles(self):
        """Randomly add/remove obstacles to simulate dynamic environment."""
        changes = []
        for _ in range(int(self.size * self.size * self.dynamic_rate)):
            r, c = random.randint(0, self.size-1), random.randint(0, self.size-1)
            if (r,c) == self.ugv_pos or (r,c) == self.goal:
                continue
            # Toggle obstacle
            self.grid[r][c] = 1 - self.grid[r][c]
            changes.append((r,c))
        return changes

    def replan(self):
        """Replan from current UGV position."""
        self.replans += 1
        return astar(self.grid, self.ugv_pos, self.goal)

    def is_path_blocked(self, path):
        """Check if any cell on the remaining path is now an obstacle."""
        if path is None:
            return True
        for cell in path:
            if self.grid[cell[0]][cell[1]] == 1:
                return True
        return False

    def run(self, max_steps=500):
        """
        Simulate UGV navigation with dynamic replanning.
        Returns: (success, path_taken, replans, steps)
        """
        plan = self.current_plan
        if plan is None:
            print("  No initial path found!")
            return False, self.path_taken, self.replans, self.steps

        plan_index = 1  # Next step in plan

        for step in range(max_steps):
            self.steps = step

            # Save frame for visualization
            if step % 10 == 0:
                self.frames.append((
                    np.copy(self.grid),
                    list(self.path_taken),
                    list(plan) if plan else []
                ))

            # Check if goal reached
            if self.ugv_pos == self.goal:
                print(f"  ✅ Goal reached in {step} steps! Replans: {self.replans}")
                return True, self.path_taken, self.replans, self.steps

            # Update dynamic obstacles
            self.update_dynamic_obstacles()

            # Check if current plan is still valid
            remaining_plan = plan[plan_index:] if plan else []
            if self.is_path_blocked(remaining_plan):
                plan = self.replan()
                plan_index = 1
                if plan is None:
                    print(f"  ⚠️  No path found after replan at step {step}. Waiting...")
                    continue

            # Move one step
            if plan and plan_index < len(plan):
                next_pos = plan[plan_index]
                if self.grid[next_pos[0]][next_pos[1]] == 0:
                    self.ugv_pos = next_pos
                    self.path_taken.append(self.ugv_pos)
                    plan_index += 1
                else:
                    plan = self.replan()
                    plan_index = 1

        print(f"  ❌ Max steps reached. Replans: {self.replans}")
        return False, self.path_taken, self.replans, self.steps


def visualize_result(env, success, title="UGV Dynamic Navigation"):
    """Static visualization of final result."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(title, fontsize=14, fontweight='bold')

    # Final grid state
    display = np.copy(env.grid).astype(float)
    for (r,c) in env.path_taken:
        display[r][c] = 2
    display[env.start[0]][env.start[1]] = 3
    if env.ugv_pos != env.goal:
        display[env.ugv_pos[0]][env.ugv_pos[1]] = 5
    display[env.goal[0]][env.goal[1]] = 4

    cmap = plt.cm.colors.ListedColormap(
        ['white', 'black', 'cyan', 'green', 'red', 'yellow'])
    bounds = [-0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    axes[0].imshow(display, cmap=cmap, norm=norm, origin='upper')
    axes[0].set_title('Final Grid State & Path Taken', fontweight='bold')
    patches = [
        mpatches.Patch(color='green',  label='Start'),
        mpatches.Patch(color='red',    label='Goal'),
        mpatches.Patch(color='yellow', label='UGV Current Pos'),
        mpatches.Patch(color='cyan',   label='Path Taken'),
        mpatches.Patch(color='black',  label='Obstacle'),
    ]
    axes[0].legend(handles=patches, loc='upper right', fontsize=7)
    axes[0].set_xlabel('X (km)')
    axes[0].set_ylabel('Y (km)')

    # Metrics bar chart
    metrics = {
        'Steps\nTaken': env.steps,
        'Replans\nCount': env.replans,
        'Path\nLength': len(env.path_taken),
    }
    colors = ['steelblue', 'orange', 'green']
    bars = axes[1].bar(metrics.keys(), metrics.values(), color=colors, edgecolor='black')
    axes[1].set_title('Measures of Effectiveness (MoE)', fontweight='bold')
    axes[1].set_ylabel('Value')
    for bar, val in zip(bars, metrics.values()):
        axes[1].text(bar.get_x() + bar.get_width()/2,
                     bar.get_height() + 1, str(val),
                     ha='center', fontweight='bold', fontsize=11)

    status = "SUCCESS ✅" if success else "FAILED ❌"
    axes[1].set_xlabel(f'Navigation Result: {status}', fontsize=12, fontweight='bold',
                       color='green' if success else 'red')

    plt.tight_layout()
    plt.savefig('ugv_dynamic_obstacles.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Saved: 'ugv_dynamic_obstacles.png'")


def print_moe(env, success):
    print(f"\n  {'─'*45}")
    print(f"  Navigation Result : {'SUCCESS ✅' if success else 'FAILED ❌'}")
    print(f"  Total Steps       : {env.steps}")
    print(f"  Replanning Count  : {env.replans}")
    print(f"  Path Cells Visited: {len(env.path_taken)}")
    print(f"  Final Position    : {env.ugv_pos}")
    print(f"  Goal              : {env.goal}")


# ─── MAIN ───────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    random.seed(7)

    START = (5, 5)
    GOAL  = (64, 64)

    print("=" * 60)
    print("   UGV PATHFINDING - DYNAMIC OBSTACLES (Replanning A*)")
    print(f"   Grid: {GRID_SIZE}x{GRID_SIZE} | Start: {START} | Goal: {GOAL}")
    print("=" * 60)

    print("\n[Simulating dynamic environment...]")
    t0 = time.time()

    env = DynamicObstacleEnvironment(
        size=GRID_SIZE,
        start=START,
        goal=GOAL,
        initial_density=0.15,   
        dynamic_rate=0.01       
    )

    success, path_taken, replans, steps = env.run(max_steps=1000)

    print(f"  Total time: {time.time()-t0:.2f}s")
    print_moe(env, success)

    visualize_result(env, success,
                     title=f"UGV Dynamic Navigation | Replans: {replans} | Steps: {steps}")
