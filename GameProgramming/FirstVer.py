import math
import sys
import numpy as np
import pygame


# ------------------------
# Simulation parameters
# ------------------------

# Grid resolution (water height field)
GRID_WIDTH = 80
GRID_HEIGHT = 60
CELL_SIZE = 10  # pixels per cell

# Window size
SCREEN_WIDTH = GRID_WIDTH * CELL_SIZE
SCREEN_HEIGHT = GRID_HEIGHT * CELL_SIZE

# Water physics parameters
DX = 1.0        # spatial step (arbitrary units)
DT = 0.03       # time step
C = 2.0         # wave propagation speed
BASE_DAMPING = 0.02  # global damping coefficient

# Precompute constant for Laplacian term
C2_DT2_DX2 = (C * C) * (DT * DT) / (DX * DX)

# Absorbing boundary: thickness (in cells)
BOUNDARY_THICKNESS = 6
MAX_BOUNDARY_DAMPING = 0.3  # extra damping near borders

# Height to color scaling
HEIGHT_COLOR_SCALE = 0.5  # larger -> colors change less with height


# ------------------------
# Rigid body (sphere) parameters
# ------------------------

SPHERE_RADIUS = 0.8    # vertical radius (in same units as height)
SPHERE_MASS = 1.0
GRAVITY = 9.8 * 0.3     # scaled gravity for visual stability

BUOYANCY_STRENGTH = 30.0  # k_b in F_buoy = k_b * depth
DRAG_COEFF = 5.0          # vertical drag in water

IMPULSE_SCALE = 0.15      # how strongly sphere motion disturbs the water
IMPULSE_RADIUS = 4.0      # in grid cells


class WaterField:
    def __init__(self, nx, ny):
        self.nx = nx
        self.ny = ny

        # Height fields: previous and current
        self.h_prev = np.zeros((nx, ny), dtype=np.float32)
        self.h = np.zeros((nx, ny), dtype=np.float32)

        # Precompute boundary damping map alpha_{i,j}
        self.alpha = np.zeros((nx, ny), dtype=np.float32)
        self._init_boundary_damping()

    def _init_boundary_damping(self):
        for i in range(self.nx):
            for j in range(self.ny):
                dist_left = i
                dist_right = self.nx - 1 - i
                dist_top = j
                dist_bottom = self.ny - 1 - j
                dist_edge = min(dist_left, dist_right, dist_top, dist_bottom)

                if dist_edge < BOUNDARY_THICKNESS:
                    # Normalize: 0 at inner boundary, 1 at the edge
                    t = 1.0 - (dist_edge / float(BOUNDARY_THICKNESS))
                    self.alpha[i, j] = t * MAX_BOUNDARY_DAMPING
                else:
                    self.alpha[i, j] = 0.0

    def step(self):
        """Advance water simulation by one time step using discrete wave equation."""
        nx, ny = self.nx, self.ny
        h = self.h
        h_prev = self.h_prev

        h_next = np.zeros_like(h)

        # Interior cells update (exclude outermost boundary)
        for i in range(1, nx - 1):
            for j in range(1, ny - 1):
                lap = (
                    h[i + 1, j] + h[i - 1, j]
                    + h[i, j + 1] + h[i, j - 1]
                    - 4.0 * h[i, j]
                )

                local_damping = BASE_DAMPING + self.alpha[i, j]

                h_next[i, j] = (
                    (2.0 - local_damping * DT) * h[i, j]
                    - (1.0 - local_damping * DT) * h_prev[i, j]
                    + C2_DT2_DX2 * lap
                )

        # Boundary cells: strong absorption (clamp to zero height)
        h_next[0, :] = 0.0
        h_next[-1, :] = 0.0
        h_next[:, 0] = 0.0
        h_next[:, -1] = 0.0

        # Roll time levels
        self.h_prev, self.h = self.h, h_next

    def sample_height(self, x, y):
        """
        Bilinearly interpolate water height at continuous coordinates (x, y),
        where x in [0, nx-1], y in [0, ny-1].
        """
        # Clamp to valid range minus 1 for interpolation
        x = max(0.0, min(self.nx - 1.001, x))
        y = max(0.0, min(self.ny - 1.001, y))

        i0 = int(math.floor(x))
        j0 = int(math.floor(y))
        i1 = min(i0 + 1, self.nx - 1)
        j1 = min(j0 + 1, self.ny - 1)

        tx = x - i0
        ty = y - j0

        h00 = self.h[i0, j0]
        h10 = self.h[i1, j0]
        h01 = self.h[i0, j1]
        h11 = self.h[i1, j1]

        h0 = h00 * (1.0 - tx) + h10 * tx
        h1 = h01 * (1.0 - tx) + h11 * tx

        return h0 * (1.0 - ty) + h1 * ty

    def add_impulse(self, x, y, strength):
        """
        Apply a localized disturbance around (x, y) in grid coordinates.
        Strength can be positive or negative.
        """
        cx = x
        cy = y
        radius = IMPULSE_RADIUS

        i_min = max(1, int(math.floor(cx - radius)))
        i_max = min(self.nx - 2, int(math.ceil(cx + radius)))
        j_min = max(1, int(math.floor(cy - radius)))
        j_max = min(self.ny - 2, int(math.ceil(cy + radius)))

        for i in range(i_min, i_max + 1):
            for j in range(j_min, j_max + 1):
                dx = i - cx
                dy = j - cy
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < radius:
                    w = 1.0 - (dist / radius)
                    self.h[i, j] += strength * w


class Sphere:
    def __init__(self, x, y, z=5.0):
        self.x = x  # horizontal position in grid coordinates
        self.y = y
        self.z = z  # vertical position
        self.vz = 0.0

    def reset(self, x, y, z=5.0, vz=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.vz = vz

    def update(self, water: WaterField):
        """
        Update sphere vertical motion under gravity, buoyancy, and drag.
        Also feed disturbance back into water when entering it.
        """
        # Sample water height under sphere
        water_h = water.sample_height(self.x, self.y)

        # Depth of bottom point of sphere below water surface
        bottom_z = self.z - SPHERE_RADIUS
        depth = water_h - bottom_z  # >0 means submerged

        # Compute forces
        F_gravity = -SPHERE_MASS * GRAVITY
        F_buoy = 0.0
        F_drag = 0.0

        if depth > 0.0:
            # Simple linear buoyancy model
            F_buoy = BUOYANCY_STRENGTH * depth

            # Submergence factor for drag (0 to 1)
            submergence = max(0.0, min(1.0, depth / (2.0 * SPHERE_RADIUS)))
            F_drag = -DRAG_COEFF * submergence * self.vz

            # Disturb water based on downward velocity
            if self.vz < 0.0:
                impulse = IMPULSE_SCALE * (-self.vz) * submergence
                water.add_impulse(self.x, self.y, -impulse)

        Fz = F_gravity + F_buoy + F_drag

        # Integrate motion
        az = Fz / SPHERE_MASS
        self.vz += az * DT
        self.z += self.vz * DT

        # Simple floor to prevent sphere from falling too low
        min_z = -5.0
        if self.z < min_z:
            self.z = min_z
            if self.vz < 0.0:
                self.vz *= -0.3  # small bounce

    def get_screen_position(self):
        """
        Map sphere (x, y) in grid coordinates to screen position.
        """
        sx = int(self.x * CELL_SIZE + CELL_SIZE * 0.5)
        sy = int(self.y * CELL_SIZE + CELL_SIZE * 0.5)
        return sx, sy


def height_to_color(h):
    """
    Map water height to an RGB color.
    Base level 0 -> mid blue; positive -> lighter, negative -> darker.
    """
    t = 0.5 + 0.5 * (h / HEIGHT_COLOR_SCALE)
    t = max(0.0, min(1.0, t))
    v = int(t * 255)
    # Blue-ish colormap
    return (0, v, 255)


def draw_water(screen, water: WaterField):
    nx, ny = water.nx, water.ny
    for i in range(nx):
        for j in range(ny):
            h = water.h[i, j]
            color = height_to_color(h)
            rect = pygame.Rect(
                i * CELL_SIZE,
                j * CELL_SIZE,
                CELL_SIZE,
                CELL_SIZE,
            )
            screen.fill(color, rect)


def draw_sphere(screen, sphere: Sphere, water: WaterField):
    sx, sy = sphere.get_screen_position()
    # Map vertical position relative to water to color/alpha
    water_h = water.sample_height(sphere.x, sphere.y)
    bottom_z = sphere.z - SPHERE_RADIUS
    depth = water_h - bottom_z

    if depth > 0:
        # In water: more submerged -> darker
        t = max(0.0, min(1.0, depth / (2.0 * SPHERE_RADIUS)))
        color = (int(255 * (1.0 - t)), int(100 * (1.0 - t)), 255)
    else:
        # In air
        color = (255, 200, 0)

    radius_px = int(CELL_SIZE * 0.6)
    pygame.draw.circle(screen, color, (sx, sy), radius_px)


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("2.5D Water Height Field + Sphere Prototype (Pygame)")

    clock = pygame.time.Clock()

    water = WaterField(GRID_WIDTH, GRID_HEIGHT)
    sphere = Sphere(GRID_WIDTH / 2.0, GRID_HEIGHT / 2.0, z=8.0)

    running = True
    while running:
        # --------------
        # Handle events
        # --------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    # Reset water and sphere
                    water = WaterField(GRID_WIDTH, GRID_HEIGHT)
                    sphere = Sphere(GRID_WIDTH / 2.0, GRID_HEIGHT / 2.0, z=8.0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    # Left click: drop sphere from above at mouse position
                    mx, my = event.pos
                    gx = mx / CELL_SIZE
                    gy = my / CELL_SIZE
                    sphere.reset(gx, gy, z=8.0, vz=0.0)
                elif event.button == 3:
                    # Right click: add manual disturbance
                    mx, my = event.pos
                    gx = mx / CELL_SIZE
                    gy = my / CELL_SIZE
                    water.add_impulse(gx, gy, strength=-0.5)

        # --------------
        # Simulation step
        # --------------
        sphere.update(water)
        water.step()

        # --------------
        # Rendering
        # --------------
        draw_water(screen, water)
        draw_sphere(screen, sphere, water)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
