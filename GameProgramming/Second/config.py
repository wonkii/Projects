import math
# ============================
# 기본 설정
# ============================

NUM_POINTS = 200          # 수면을 이루는 점 개수
CELL_SIZE = 4         # 점 사이 화면 간격

SCREEN_WIDTH = NUM_POINTS * CELL_SIZE # 총 SCREEN의 길이 800
SCREEN_HEIGHT = 600 # SCREEN의 높이
MID_Y = SCREEN_HEIGHT // 2  # 화면 가운데 수평선을 수면 높이의 초깃값으로 활용 예정

# 월드 좌표계에서 z(위/아래)를 화면 픽셀로 바꿀 때의 스케일
Z_SCALE = 40.0  # 1 단위 높이 = 30 픽셀 

# 수면 높이 클램프 (수면이 이 범위를 넘지 않게 제한)
MAX_WATER_HEIGHT = 5.0  # h in [-5, 5] 안으로 제한


# ============================
# 물(1D Height Field) 물리 파라미터
# ============================

# 파동방정식에 필요한 dx, dt 그리고 v(파동의 속도)
DX = 1.0           # 수평 격자 간격 
DT = 0.03          # 시간 스텝
C = 20.0           # 파동의 속도

# 파동방정식. 수면 움직임의 핵심
C2_DT2_DX2 = (C * C) * (DT * DT) / (DX * DX)


# 수면 전체에 걸리는 기본 감쇠 계수
BASE_DAMPING = 0.004  # water.py의 Step함수에서 파동 움직임 계산 시 사용

BOUNDARY_THICKNESS = 2     # 양 끝에서부터의 흡수 영역
MAX_BOUNDARY_DAMPING = 0.1 # 경계 쪽에서 추가로 더해질 감쇠 계수

IMPULSE_RADIUS = 2.0       # 강체가 줄 수면 교란 반경 (격자 index 단위)

# ============================
# 강체(구) 물리 파라미터
# ============================

SPHERE_RADIUS = 0.6   # 구 반지름
SPHERE_MASS = 3

# 구 전체 부피 부력 계산 및 항력 비율 계산 시 사용
SPHERE_VOLUME = (4.0 / 3.0) * math.pi * (SPHERE_RADIUS ** 3)  

GRAVITY = 9.8             # 중력 가속도
BUOYANCY_STRENGTH = 40.0   # 부력 계수 
DRAG_COEFF = 10.0          # 수중 항력

# 강체와 수면 사이 작용하는 방정식 계산 시 사용되는 변수
BUOY_HEIGHT_SMOOTH = 0.1        # 공이 느끼는 수면의 부드러움
# MAX_BUOY_DEPTH = 0.7 * SPHERE_RADIUS  # 부력 계산에 쓸 최대 잠김 깊이

# 수면에 주는 충격 계산 시 사용
IMPULSE_SCALE = 0.1    # 강체가 수면을 누를 때 생성하는 파동 세기

#(원래 (물의 밀도 * g)이지만 온도에 따라 달라지는 물 밀도 계산 안함)