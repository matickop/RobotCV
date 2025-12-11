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
    if os.path.exists("flat_map.npy"):
        flat_map = np.load("flat_map.npy", allow_pickle=True)
        flat2d = np.full((4,6), None, dtype=object)
        if np.all(flat_map==None):
            print(f"[INFO] Vsi koščki so pobrani, začenjam shufflanje...")
            random_safe, random_drop = robot.generiraj_random_joint_mreze(robot.paleta2_safe, robot.paleta2_drop)
            robot.homing()
            print("homing")
            robot.gripper_close()
            #kreiram flat_map kjer so vsi noter None, da se nemore ustavit kar tako
            for i in range(robot.paleta2_safe_joint.shape[0]):
                for j in range(robot.paleta2_safe_joint.shape[1]):
                    if stop_event.is_set():
                        print("STOP - prekinitev programa")
                        return
                    #pot do koscka:
                    path = [
                        list(robot.paleta1_safe_joint[i, j]) + [1.2, 0.8, 0],
                        list(robot.paleta1_work_joint[i, j]) + [0.3, 0.3, 0]
                    ]
                    robot.rtde_c.moveJ(path)
                    robot.gripper_open() 

                    path = [
                        list(robot.paleta1_work_joint[i, j]) + [1.2, 0.8, 0.01],
                        list(robot.paleta1_safe_joint[i, j]) + [1.2, 0.3, 0.01],
                        list(random_safe[i, j]) + [1.2, 0.8, 0.01],
                        list(random_drop[i, j]) + [1.2, 0.3, 0.0]
                    ]
                    robot.rtde_c.moveJ(path)
                    robot.gripper_close()
                    path = [
                        list(random_drop[i,j]) + [1.2, 0.8, 0.01],
                        list(random_safe[i, j]) + [1.2, 0.3, 0.0]                
                    ]
                    robot.rtde_c.moveJ(path)

                    flat2d[i,j] = [i,j]  #nastavim flat2d na i,j
                    np.save("flat_map.npy", flat2d.flatten())

            flat_map = [[i, j] for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
            flat_map = np.array(flat_map, dtype=object)
            np.save("flat_map.npy", flat_map)
            print(f"[INFO] Shuffling complete. New flat_map.npy created.")
            robot.homing()

def pobiranje_s_kamero():
    stop_event.clear()
    robot.homing()
    print("Homing") 
    robot.gripper.move_and_wait_for_pos(229, speed=200, force=2)
    print("Gripper closed")
    # ime za csv se kreira na zacetku
    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
    ime = f"Template_CSV_datoteke/Template_matching_scores_{timestr}"
    # preverimo ce obstaja flat_map
    if os.path.exists("flat_map.npy"):
        flat_map = np.load("flat_map.npy", allow_pickle=True)
        flat_map = np.array(flat_map, dtype=object)
        print(f"[INFO] Naložena obstoječa flat_map.npy: {flat_map}")
    #zanka za zajem slike in vse ostalo :D
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_event.is_set():
                print("Cikel prekinjen s STOP gumbom")
                return
            # 1) Gre nad kos za kamero
            path = [
                list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.2, 0.8, 0.0],
                list(map(float, robot.paleta2_kam_joint[i, j])) + [1.2, 0.5, 0.0]
            ]
            robot.rtde_c.moveJ(path)

            # 2) Zajame sliko in template matcha
            robot.ring_ON()
            cam.capture_image()
            robot.ring_OFF()
            slika, score_match, slovar = cam.template_match(cam.template_path, show=False)
            # zapisem top 5 slik
            with open(ime, "a", encoding="utf8") as f:
                row = ",".join([f"{k}={v}" for k, v in list(slovar.items())[:5]])
                f.write(row + "\n")

            if slika == "template_prazna.png":
                print(f"Pozicija je prazna, preskakujem...")
                path = [
                    list(map(float, robot.paleta2_kam_joint[i, j])) + [1.2, 0.5, 0.01],
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.2, 0.5, 0.0],
                ]
                robot.rtde_c.moveJ(path)
                continue
            
            try:
                slika = slika.split(".")[0]
                idx, kot = slika.split("_")
                idx = int(idx)
                kot = int(kot)
                print(idx, kot)
            except Exception as e:
                print(f"Neveljavna slika ali indeks: {slika} | Error: {e}")
                continue

            #Gre nad koscek, katerege je zajel s kamero(ce je koscke bil prepoznan)
            if flat_map[idx] is not None:
                # 3) Pick iz palete 2 - rotiranje 
                target_safe = robot.rtde_c.getInverseKinematics(robot.pomik_rotacija(robot.paleta2_safe[i, j], kot)[0], robot.home_p)
                target_work = robot.rtde_c.getInverseKinematics(robot.pomik_rotacija(robot.paleta2_work[i, j], kot)[0], robot.home_p)
                path = [
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.2, 0.8, 0.01],
                    list(map(float, target_safe)) + [1.2, 0.8, 0.005],
                    list(map(float, target_work)) + [0.2, 0.3, 0.0],
                ]
                robot.rtde_c.moveJ(path)
                print("premikam se nad sliko")
                robot.gripper_open()
                print("odpiram gripper")


                # 4) Dvig + pot do cilja v paleti 1
                row, col = flat_map[idx]
                print(row, col)
                print(robot.paleta1_safe_joint[row, col])
                path = [
                    list(map(float, target_safe)) + [1.2, 0.8, 0.01],
                    list(map(float, robot.paleta1_safe_joint[row, col])) + [1.2, 0.8, 0.005],
                    list(map(float, robot.paleta1_drop_joint[row, col])) + [1.2, 0.3, 0.0],
                ]

                # 5) Place
                robot.rtde_c.moveJ(path)
                robot.gripper_close()

                    # 6) Dvig nad odlagališče
                path = [
                    list(map(float, robot.paleta1_drop_joint[row, col])) + [1.2, 0.8, 0.005],
                    list(map(float, robot.paleta1_safe_joint[row, col])) + [1.2, 0.6, 0.0],
                ]
                robot.rtde_c.moveJ(path)
                    
                #ko polozi sliko, se v matriki pozicij namesto indeksov appenda None
                flat_map[idx] = None
                np.save("flat_map.npy", flat_map)
            else:
                print("Slike ni mogoce postaviti na zapolnjeno mesto")
                
    if not np.all(flat_map==None):
        print(f"[INFO] Pobiranje končano, vsi koščki niso pobrani, kljub temu so vsi v flat_map-u nastavljeni na None.")
        flat_map = [None for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
        np.save("flat_map.npy", flat_map)

    #homing nazaj
    robot.homing()

def celoten_loop():
    stop_event.clear()
    while True:
        if os.path.exists("flat_map.npy"):
            zasedenost = np.load("flat_map.npy", allow_pickle=True)
            if stop_event.is_set():
                print("[INFO] stopping program")
                break
            if np.all(zasedenost==None):
                print(f"[INFO] Vsi koščki so pobrani, začenjam shufflanje...")
                shuffling_kosckov()
            else:
                if stop_event.is_set():
                    print("[INFO] stopping program")
                    break
                print(f"[INFO] Koščki so na plaleti 2, začenjam pobiranje s kamero...")
                pobiranje_s_kamero()
        else:
            print(f"[INFO] Datoteka flat_map.npy ne obstaja, kreiram novo in začenjam pobiranje s kamero...")
            flat_map = [[i, j] for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
            flat_map = np.array(flat_map, dtype=object)
            np.save("flat_map.npy", flat_map)
            pobiranje_s_kamero()
        time.sleep(0.5)

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
            idx = filename.split("_0")[0] # TUKI JE PROBLEM! - kje je problem?
        

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
    

