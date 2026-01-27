import numpy as np
import pandas as pd

# Parametry z quadruped_config.hpp
THIGH_LENGTH = 0.09
SHIN_LENGTH = 0.09
LOOP_RATE_HZ = 30
DT = 1.0 / LOOP_RATE_HZ
BASE_STEP_LENGTH = 0.08
DEFAULT_GROUND_Y = -0.13
STEP_HEIGHT = 0.05
STEP_DURATION = 1.0

def calculate_ik(target_x, target_y):
    """Implementacja logiki z IK_solver.cpp"""
    D_sq = target_x**2 + target_y**2
    D = np.sqrt(D_sq)
    
    # Sprawdzenie zasięgu (Workspace check)
    if D > (THIGH_LENGTH + SHIN_LENGTH) or D < abs(THIGH_LENGTH - SHIN_LENGTH):
        return None, None
    
    # Prawo Cosinusów dla kolana
    cos_beta = (THIGH_LENGTH**2 + SHIN_LENGTH**2 - D_sq) / (2 * THIGH_LENGTH * SHIN_LENGTH)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    angle_knee_rad = np.pi - beta
    
    # Prawo Cosinusów dla biodra
    phi = np.arctan2(target_y, target_x)
    cos_alpha = (D_sq + THIGH_LENGTH**2 - SHIN_LENGTH**2) / (2 * D * THIGH_LENGTH)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)
    
    angle_hip_rad = (np.pi / 2.0) + alpha - phi
    return np.degrees(angle_hip_rad), np.degrees(angle_knee_rad)

def generate_step_data():
    results = []
    num_steps = int(STEP_DURATION / DT)
    
    # 1. FAZA LOTU (Swing Phase) - noga idzie do przodu i unosi się
    step_start_x = -BASE_STEP_LENGTH / 2.0
    step_end_x = BASE_STEP_LENGTH / 2.0
    
    for i in range(num_steps + 1):
        t = i * DT
        progress = min(t / STEP_DURATION, 1.0)
        
        # Ruch X: od -0.04m do +0.04m
        x = step_start_x + (progress * (step_end_x - step_start_x))
        # Ruch Y: Sinusoida (unoszenie o 0.05m)
        arc_height = STEP_HEIGHT * np.sin(np.pi * progress)
        y = DEFAULT_GROUND_Y + arc_height
        
        hip_deg, knee_deg = calculate_ik(x, y)
        results.append(["Swing", round(t, 3), round(x, 4), round(y, 4), round(hip_deg, 2), round(knee_deg, 2)])

    # 2. FAZA PODPARCIA (Stance Phase) - noga wraca po ziemi, pchając robota
    for i in range(1, num_steps + 1):
        t = i * DT
        progress = min(t / STEP_DURATION, 1.0)
        
        # Ruch X: od +0.04m do -0.04m
        x = step_end_x - (progress * (step_end_x - step_start_x))
        y = DEFAULT_GROUND_Y # Noga na ziemi
        
        hip_deg, knee_deg = calculate_ik(x, y)
        results.append(["Stance", round(STEP_DURATION + t, 3), round(x, 4), round(y, 4), round(hip_deg, 2), round(knee_deg, 2)])
        
    return results

# Generowanie i wyświetlanie wyników
columns = ["Faza", "Czas [s]", "X [m]", "Y [m]", "Biodro [deg]", "Kolano [deg]"]
data = generate_step_data()
df = pd.DataFrame(data, columns=columns)

print(df.to_string(index=False))