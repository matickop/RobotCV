import time
import math
import numpy as np
import rtde_control
import rtde_receive

#---PARAMETRI
host = '172.20.10.4'   # IP robota
acc = 0.2
vel = 0.2

home_p = [math.radians(90),
          math.radians(-90),
          math.radians(90),
          math.radians(-90),
          math.radians(-90),
          math.radians(0)]

pk1 = [-0.21160, -0.6619, 0.009, 0, 3.13, 0]   # startna pozicija

#---GENERACIJA POZICIJ

def testna_mreza_stirih(a, b, start_poz):
    poz = np.zeros((a, b, 6))
    dx = 0
    for i in range(a):
        dy = 0
        for j in range(b):
            poz[i, j] = start_poz
            poz[i, j, 1] += dy
            poz[i, j, 0] += dx
            dy += 0.07877
        dx += 0.0686
    return poz

#---FUNKCIJE POMIKI

def homing(rtde_c):
    """Home robot"""
    rtde_c.moveJ(home_p, acc, vel)

def random_move(rtde_c):
    """Premik na eno testno pozicijo"""
    rtde_c.moveL([-0.21160, -0.6619, 0.04, 0, 3.13, 0], acc, vel)

def move_to_position(rtde_c, pozicije, safe_z=0.04, work_z=0.009):
    """Loop cez vse pozicije v mrezi"""
    for i in range(pozicije.shape[0]):
        for j in range(pozicije.shape[1]):
            target_p = pozicije[i, j].copy()

            # safe pozicija
            safe_p = target_p.copy()
            safe_p[2] = safe_z

            # 1. gor na safe
            rtde_c.moveL(safe_p, acc, vel)
            time.sleep(0.5)

            # 2. dol na work
            work_p = target_p.copy()
            work_p[2] = work_z
            rtde_c.moveL(work_p, acc, vel)
            time.sleep(0.5)

            # >>> tu lahko dodas gripper logiko <<<

            # 3. nazaj gor
            rtde_c.moveL(safe_p, acc, vel)
            time.sleep(0.5)

    # nazaj na home
    rtde_c.moveJ(home_p, acc, vel)

#---

if __name__ == "__main__":
    # Ustvari RTDE kontrolni interface
    rtde_c = rtde_control.RTDEControlInterface(host)
    rtde_r = rtde_receive.RTDEReceiveInterface(host)  # lahko branje stanja, ni obvezno

    print("Homing...")
    homing(rtde_c)

    #print("Random premik...")
    #random_move(rtde_c)

    #print("Generiranje mreze...")
    #mreza = testna_mreza_stirih(2, 2, pk1)

    #print("Premikanje cez mrezo...")
    #move_to_position(rtde_c, mreza)

    print("Disconnecting...")
    rtde_c.disconnect()
