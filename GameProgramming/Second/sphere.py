from config import*
from water import Water1D
import math
# ============================
# 강체(구)
# ============================

class Sphere:
    def __init__(self, x, z):
        self.x = float(x)
        self.z = float(z)
        self.vz = 0.0
        self.water_h_for_force = 0.0

    def update(self, water: Water1D):
        water_h_raw = water.sample_height(self.x)
        self.water_h_for_force += BUOY_HEIGHT_SMOOTH * (water_h_raw - self.water_h_for_force)
        bottom_z = self.z - SPHERE_RADIUS
        
        # 실제 잠김 깊이
        depth_raw = water_h_raw - bottom_z

        # 중력, 부력, 항력
        F_gravity = -SPHERE_MASS * GRAVITY
        F_buoy = 0.0
        F_drag = 0.0
        
        # V_sub = (π h^2 / 3) * (3R - h) = π h^2 (R - h/3)
        # F_buoy ≈ ρ * g * V_sub
        # 잠긴 높이 h (0 ~ 2R) : 구의 "바닥"에서 물 표면까지의 거리

        if depth_raw > 0.0:
            h = max(0.0, min(depth_raw, 2.0 * SPHERE_RADIUS))

            V_sub = math.pi * h * h * (SPHERE_RADIUS - h / 3.0)
            F_buoy = BUOYANCY_STRENGTH * V_sub

            # 잠긴 비율은 실제 깊이 기준 (수중 항력, 임펄스에는 그대로 반영)
            submergence = max(0.0, min(1.0, V_sub / SPHERE_VOLUME))
            F_drag = -DRAG_COEFF * submergence * self.vz

            if self.vz < 0.0:
                impulse = IMPULSE_SCALE * (-self.vz) * submergence
                water.add_impulse(self.x, -impulse)

        Fz = F_gravity + F_buoy + F_drag

        az = Fz / SPHERE_MASS
        self.vz += az * DT
        self.z += self.vz * DT

        # 너무 아래로 떨어지는 것 방지용 바닥:
        # 수면이 내려갈 수 있는 최소 높이보다 조금만 더 아래까지만 허용
        min_z = -MAX_WATER_HEIGHT - SPHERE_RADIUS * 0.5
        if self.z < min_z:
            self.z = min_z
            if self.vz < 0.0:
                self.vz *= -0.2

    
        # x: 격자 index -> 화면 x
        # z: 월드 -> 화면 y (위가 0, 아래로 증가)
    def get_screen_pos(self):
        sx = int(self.x * CELL_SIZE + CELL_SIZE * 0.5)
        sy = int(MID_Y - self.z * Z_SCALE)
        return sx, sy
    

    
        # 부력에 쓸 깊이는 살짝 클램프
        # depth_for_buoy = max(0.0, min(depth_raw, MAX_BUOY_DEPTH))