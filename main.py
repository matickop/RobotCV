import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from robot_module import MyRobot
from kamera_module import MyCamera
import cv2
import os
import threading
import shutil

stop_event = threading.Event()

cam = MyCamera()
robot = MyRobot("192.168.3.102")  
pobrani_koti = [None]*8  # 4 koti prve palete + 4 druge

def shuffling_kosckov():
    stop_event.clear()
    random_safe, random_drop = robot.generiraj_random_joint_mreze(robot.paleta2_safe_joint, robot.paleta2_drop_joint)
    robot.homing()
    print("homing")
    robot.gripper_close()

    for i in range(robot.paleta2_safe_joint.shape[0]):
        for j in range(robot.paleta2_safe_joint.shape[1]):
            if stop_event.is_set():
                print("STOP - prekinitev programa")
                return
            #pot do koscka:
            path = [
                list(robot.paleta1_safe_joint[i, j]) + [1.2, 0.6, 0],
                list(robot.paleta1_work_joint[i, j]) + [0.3, 0.2, 0]
            ]
            robot.rtde_c.moveJ(path)
            robot.gripper_open() 
            time.sleep(0.2)

            path = [
                list(robot.paleta1_work_joint[i, j]) + [1.2, 0.6, 0.01],
                list(robot.paleta1_safe_joint[i, j]) + [1.2, 0.2, 0.01],
                list(random_safe[i, j]) + [1.2, 0.6, 0.01],
                list(random_drop[i, j]) + [1.2, 0.2, 0.0]
            ]
            robot.rtde_c.moveJ(path)
            robot.gripper_close()
            time.sleep(0.2)
            path = [
                list(random_drop[i,j]) + [1.2, 0.6, 0.01],
                list(random_safe[i, j]) + [1.2, 0.6, 0.0]                
            ]
            robot.rtde_c.moveJ(path)


    robot.homing()

def pobiranje_s_kamero():
    stop_event.clear()
    robot.homing()
    print("Homing") 
    robot.gripper.move_and_wait_for_pos(229, speed=200, force=2)
    print("Gripper closed")

    #kreiram array za pozicije slike v paleti 1 --> array z [0,0]...[3,5]
    flat_map = [[i, j] for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
    #zanka za zajem slike in vse ostalo :D
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_event.is_set():
                print("Cikel prekinjen s STOP gumbom")
                return
            # 1) Gre nad kos za kamero
            path = [
                list(robot.paleta2_kam_safe_joint[i, j]) + [1.2, 0.5, 0.0],
                list(robot.paleta2_kam_joint[i, j]) + [1.2, 0.5, 0.0]
            ]
            robot.rtde_c.moveJ(path)

            # 2) Zajame sliko in template matcha
            robot.ring_ON()
            cam.capture_image()
            robot.ring_OFF()
            slika, score_match = cam.template_match(cam.template_path, show=False)
            slika = slika.split(".")[0]
            idx, kot = slika.split("_")
            idx = int(idx)
            kot = int(kot)
            print(idx, kot)

            #gre samo nad tocko kamor bi postavil sliko
            if flat_map[idx] is not None:
                # 3) Pick iz palete 2 - rotiranje 
                target_safe = robot.paleta2_safe_joint[i, j].copy()
                target_safe[5] += np.deg2rad(kot)
                target_work = robot.paleta2_work_joint[i, j].copy()
                target_work[5] += np.deg2rad(kot)
                path = [
                    list(robot.paleta2_kam_safe_joint[i, j]) + [1.2, 0.6, 0.0],
                    list(target_safe) + [1.2, 0.6, 0.0],
                    list(target_work) + [0.2, 0.2, 0.0],
                ]
                robot.rtde_c.moveJ(path)
                print("premikam se nad sliko")
                robot.gripper_open()
                print("odpiram gripper")
                time.sleep(0.3)


                # 4) Dvig + pot do cilja v paleti 1
                row, col = flat_map[idx]
                path = [
                    list(target_safe) + [1.2, 0.6, 0.01],
                    list(robot.paleta1_safe_joint[row, col]) + [1.2, 0.6, 0.01],
                    list(robot.paleta1_drop_joint[row, col]) + [1.2, 0.2, 0.0]
                ]

                # 5) Place
                robot.rtde_c.moveJ(path)
                robot.gripper_close()
                time.sleep(0.3)

                    # 6) Dvig nad odlagališče
                path = [
                    list(robot.paleta1_drop_joint[row, col]) + [1.2, 0.6, 0.01],
                    list(robot.paleta1_safe_joint[row, col]) + [1.2, 0.6, 0.0]
                ]
                robot.rtde_c.moveJ(path)
                    
                #ko polozi sliko, se v matriki pozicij namesto indeksov appenda None
                flat_map[idx] = None
            else:
                print("Slike ni mogoce postaviti na zapolnjeno mesto")
                
            
    #homing nazaj
    robot.homing()

def celoten_loop():
    stop_event.clear()
    while True:
        if stop_event.is_set():
            break
        shuffling_kosckov()
        if stop_event.is_set():
            break
        pobiranje_s_kamero()

def zajem_celotne_slike():
    """Slika vsak kos posebaj za referenco"""
    stop_event.clear()
    robot.homing()
    print("Homing") 
    robot.gripper.move_and_wait_for_pos(229, speed=200, force=2)
    print("Gripper closed")

    save_dir = "zajeta_celotna_slika"

    if os.path.exists(save_dir):
        shutil.rmtree(save_dir)
    os.makedirs(save_dir, exist_ok=True)

    #zanka za zajem slike in vse ostalo :D
    ind_slik = np.array([i for i in range(24)]).reshape(4, 6)
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_event.is_set():
                print("Cikel prekinjen s STOP gumbom")
                return
            # 1) Gre nad kos za kamero
            path = [
                list(robot.paleta2_kam_safe_joint[i, j]) + [1.2, 0.5, 0.0],
                list(robot.paleta2_kam_joint[i, j]) + [1.2, 0.5, 0.0]
            ]
            robot.rtde_c.moveJ(path)
            # 2) Zajame sliko
            idx = ind_slik[i,j]
            base_filename = f"{idx}_0.png"
            base_path = os.path.join(save_dir, base_filename)
            print(base_path)
            robot.ring_ON()
            cam.capture_image_celotna(filename=base_filename, save_dir=save_dir)
            robot.ring_OFF()
            path = [
                list(robot.paleta2_kam_joint[i, j]) + [1.2, 0.5, 0.0],
                list(robot.paleta2_kam_safe_joint[i, j]) + [1.2, 0.5, 0.0]
            ]
            robot.rtde_c.moveJ(path)

    generiranje_rotacije_slik(folder="zajeta_celotna_slika")

    robot.homing()

def rotate(img, angle):
    h, w = img.shape[:2]
    center = (w // 2, h // 2)

    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(img, M, (w, h))

    return rotated

def generiranje_rotacije_slik(folder: str):
    for filename in os.listdir(folder):
        if filename.endswith("_0.png"):
            file_path = os.path.join(folder, filename) 
            img = cv2.imread(file_path)
            idx = filename.split("_0")[0] # TUKI JE PROBLEM!

            rotations = {
                "-90": rotate(img, 90), #kompenzacija ker -90 ni ccw ampak cw
                "90": rotate(img, -90),
                "180": rotate(img, 180)
            }

        for angle, rotated_img in rotations.items():
            new_filename = f"{idx}_{angle}.png"
            save_path = os.path.join(folder, new_filename)
            cv2.imwrite(save_path, rotated_img)
            print(f"saved: {save_path}")

def fine_move_tcp(dx=0.0, dy=0.0, dz=0.0):
    tcp = robot.get_actual_tcp_pose()
    tcp[0] += dx
    tcp[1] += dy
    tcp[2] += dz
    tcp[3] = 0.0
    tcp[4] = 3.14
    tcp[5] = 0.0
    robot.move_fine(tcp)
    

