import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import FreeSimpleGUI as sg
from robot_module import MyRobot
from kamera_module import MyCamera
import cv2
import os

stop_requested = False

cam = MyCamera()

robot = MyRobot("192.168.3.102")  
pobrani_koti = [None]*8  # 4 koti prve palete + 4 druge

def shuffling_kosckov():
    stop_requested = False
    random_safe, random_work = robot.generiraj_random_joint_mreze(robot.paleta2_safe_joint, robot.paleta2_work_joint)
    robot.homing()
    print("homing")
    robot.gripper_close()

    for i in range(robot.paleta2_safe_joint.shape[0]):
        for j in range(robot.paleta2_safe_joint.shape[1]):
            if stop_requested:
                print("STOP - prekinitev programa")
                return
            #pot do koscka:
            path = [
                list(robot.paleta1_safe_joint[i, j]) + [1.2, 0.6, 0.01],
                list(robot.paleta1_work_joint[i, j]) + [1.2, 0.6, 0]
            ]
            robot.rtde_c.moveJ(path)
            robot.gripper_open() 
            time.sleep(0.2)

            path = [
                list(robot.paleta1_work_joint[i, j]) + [1.2, 0.6, 0.01],
                list(robot.paleta1_safe_joint[i, j]) + [1.2, 0.6, 0.01],
                list(random_safe[i, j]) + [1.2, 0.6, 0.01],
                list(random_work[i, j]) + [1.2, 0.6, 0.0]
            ]
            robot.rtde_c.moveJ(path)
            robot.gripper_close()
            time.sleep(0.2)
            path = [
                list(random_work[i,j]) + [1.2, 0.6, 0.01],
                list(random_safe[i, j]) + [1.2, 0.6, 0.0]                
            ]
            robot.rtde_c.moveJ(path)


    robot.homing()

def pobiranje_s_kamero():
    stop_requested = False

    robot.homing()
    print("Homing") 
    robot.gripper.move_and_wait_for_pos(229, speed=200, force=2)
    print("Gripper closed")

    #kreiram array za pozicije slike v paleti 1 --> array z [0,0]...[3,5]
    flat_map = [[i, j] for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
    #zanka za zajem slike in vse ostalo :D
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_requested:
                print("Cikel prekinjen s STOP gumbom")
                return
            # 1) Gre nad kos za kamero
            path = [
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
                    list(target_safe) + [1.2, 0.6, 0.0]
                ]
                robot.rtde_c.moveJ(path)
                robot.gripper_open()
                time.sleep(0.3)


                # 4) Dvig + pot do cilja v paleti 1
                row, col = flat_map[idx]
                path = [
                    list(robot.paleta2_safe_joint[i, j]) + [1.2, 0.6, 0.01],
                    list(robot.paleta1_safe_joint[row, col]) + [1.2, 0.6, 0.01],
                    list(robot.paleta1_work_joint[row, col]) + [1.2, 0.6, 0.0]
                ]

                # 5) Place
                robot.rtde_c.moveJ(path)
                robot.gripper_close()
                time.sleep(0.3)

                    # 6) Dvig nad odlagališče
                path = [
                    list(robot.paleta1_work_joint[row, col]) + [1.2, 0.6, 0.01],
                    list(robot.paleta1_safe_joint[row, col]) + [1.2, 0.6, 0.0]
                ]
                robot.rtde_c.moveJ(path)
                    
                #ko polozi sliko, se v matriki pozicij namesto indeksov appenda None
                flat_map[idx] = None
            else:
                print("Slike ni mogoce postaviti na zapolnjeno mesto")
                
            
    #homing nazaj
    robot.homing()

def zajem_celotne_slike():
    """Slika vsak kos posebaj za referenco"""
    stop_requested = False

    robot.homing()
    print("Homing") 
    robot.gripper.move_and_wait_for_pos(229, speed=200, force=2)
    print("Gripper closed")

    save_dir = "zajeta_celotna_slika"
    os.makedirs(save_dir, exist_ok=True)

    #zanka za zajem slike in vse ostalo :D
    ind_slik = np.array([i for i in range(24)]).reshape(4, 6)
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_requested:
                print("Cikel prekinjen s STOP gumbom")
                return
            # 1) Gre nad kos za kamero
            path = [
                list(robot.paleta2_kam_joint[i, j]) + [1.2, 0.5, 0.0]
            ]
            robot.rtde_c.moveJ(path)
            # 2) Zajame sliko
            idx = ind_slik[i,j]
            base_filename = f"{idx}_0.png"
            base_path = os.path.join(save_dir, base_filename)
            robot.ring_ON()
            cam.capture_image(filename=base_path)
            robot.ring_OFF()

            img = cv2.imread(base_path)

            for kot in [-90, 90, 180]:
                if kot == -90:
                    rotated = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                
                elif kot == 90:
                    rotated = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

                elif kot == 180:
                    rotated = cv2.rotate(img, cv2.ROTATE_180)

            rotated_filename = f"{idx}_{kot}.png"
            rotated_path = os.path.join(save_dir, rotated_filename)
            cv2.imwrite(rotated_path, rotated)
            print(f"Shranjena rotacija {kot}°: {rotated_path}")


            