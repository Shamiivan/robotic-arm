import numpy as np

def get_angles(coordinates):


    x0 , x1 = -100, 100
    y0, y1 = 0,100
    z0, z1 = 0, 100
    p = 20
    d = 10

    x, y, z = coordinates
    r = (z - z0) / (z1 - z0) * (p + d - abs(p - d)) + abs(p - d)
    # r = (z - z0) / (z1 - z0)
    # print("R", r)
    # v =(p + d - abs(p - d)) + abs(p - d)
    # print("V", v)
    # r = r * v
    print("R", r)
    alpha = np.arccos((r**2 + p**2 - d**2) / (2 * r * p))
    alpha = np.arccos((r**2 + p**2 - d**2) / (2 * r * p))
    beta = np.arccos((p**2 + d**2 - r**2) / (2 * p * d))
    theta_y = 0.5 * np.pi - np.arccos(2 / (x1 - x0) * (x - (x1 + x0) / 2))
    theta_x = np.arcsin(1 / (y1 - y0) * (y - y0))
    
    theta_y = theta_y * 180/np.pi 
    theta_x = (theta_x + alpha) * 180/np.pi
    beta = beta * 180/np.pi
    return [theta_y, theta_x, beta] 


print(get_angles([0,0,0]))
