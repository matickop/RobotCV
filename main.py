import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from robot_module import MyRobot
from kamera_module import MyCamera
import cv2
import os
import threading
import shutil
import logger
import ids


stop_event = threading.Event()

log = logger.Logger() # globalni logger
cam = MyCamera(save_dir="zajeti") # objekt kamere
robot = MyRobot("192.168.3.102") # objekt robota 
pobrani_koti = [None]*8  # 4 koti prve palete + 4 druge

# Pomožne funkcije za preverjanje zapolnjenosti, kreiranje csv datoteko...
def koscki_pobrani():
    """
    Ko so vsi koščki pobrani, je datoteka "zapolnjenost_paleta1.npy" polna s None vrednostmi.
    V primeru da niso vsi koscki pobrani, je treba najprej pobrati vse koscke, ko jih enkrat poberemo in razmečemo,
    je datoteka polna z pozicijam, kam bo treba, po prepoznavi koscek odloziti.
    V primeru da datoteke ni, se ustvari nova, ki je polna z None vrednostmi, kar pomeni da še ni noben koscek pobran.
    """
    file_name = "zapolnjenost_paleta1.npy"
    try:
        zapolnjenost_paleta1 = np.load((file_name), allow_pickle=True)
        log.event("file_check", "DEBUG", f"Loaded {file_name} successfully.")
        if np.all(zapolnjenost_paleta1 == None):
            log.event("paleta1_polna", "INFO", "Vsi koscki so pobrani.")

            return True
        else:
            log.event("paleta1_ni_polna", "INFO", f"Stevilo nepobranih kosckov: {np.sum(zapolnjenost_paleta1 != None)//2}")

            return False

    except FileNotFoundError:
        log.event("file_not_found", "DEBUG", f"File {file_name} not found.")
        zapolnjenost_paleta1 = np.array([[i, j] for i in range(4) for j in range(6)])
        np.save("zapolnjenost_paleta1.npy", zapolnjenost_paleta1)
        log.event("file_created", "DEBUG", f"File {file_name} created and initialized with positions.")

        return False


def kreiranje_csv_datoteke():
    """
    Kreira ime za CSV datoteko, ki se uporablja za shranjevanje rezultatov template matchinga.
    Ime datoteke vključuje časovni žig, da se zagotovi unikatnost vsake datoteke.
     - Če datoteka že obstaja, se ustvari nova z drugačnim časovnim žigom.
     - Vsi dogodki, povezani s kreiranjem datoteke, se beležijo v logu.
    """
    os.makedirs("Template_CSV_datoteke", exist_ok=True)
    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
    ime = f"Template_CSV_datoteke/Template_matching_scores_{timestr}.csv"

    log.event("file_created", "INFO",
            message=f"CSV datoteka ustvarjena: {ime}")

    return ime



def shuffling_kosckov():
    """ 
    Funkcija, ki razmece koscke iz palete 1 na paleto 2.
    Za preverjanje ce so koscki na paleti je zgolj samo flat_map.npy, kateri se prazni tekom pobiranja kosckov, ce se ni vsak
    koscek zabelezil kot pobran, je treba najprej pobrati vse koscke. Ni nobenega preverjanja, kako so koscki orientirani,
    predpostavljam da so pravilno.
    """

    stop_event.clear()

    session_cid = ids.new_id("Shuffling")
    log.event("shuffling_start", "INFO", "Začel se je proces shufflanja kosckov iz palete 1 na paleto 2.", cmd_id=session_cid)

    if not koscki_pobrani():
        log.event("paleta1_ni_polna", "WARNING", "Shuffling se ne more začeti, ker niso vsi koscki pobrani.", cmd_id=session_cid)

        return
    
    log.event("paleta1_prazna", "INFO",
              message="Vsi koščki pobrani, pripravljam shuffling...",
              cmd_id=session_cid)
    
    random_safe, random_drop = robot.generiraj_random_joint_mreze(
        robot.paleta2_safe, robot.paleta2_drop)
    rows, cols = robot.paleta2_safe_joint.shape[:2]
    zapolnjenost_paleta1_2d = np.load("zapolnjenost_paleta1.npy", allow_pickle=True).reshape(rows, cols, 2)

    cid = ids.new_id("homing")
    log.motion(cmd_id=cid, method="homing", status="started",actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})
    robot.homing()
    log.motion(cmd_id=cid, method="homing", status="completed", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})

    cid = ids.new_id("gripper")
    log.gripper(cmd_id=cid, method="gripper_close", status="started", level="INFO", extra={"session": session_cid})
    robot.gripper_close()
    log.gripper(cmd_id=cid, method="gripper_close", status="completed", level="INFO", extra={"session": session_cid})

    for i in range(rows): # Začetek iteracije prestavljanja iz palete 1 na paleto 2
        for j in range(cols):

            if stop_event.is_set():
                log.event("stop_requested", "WARNING", f"Shuffling prekinjen pri [{i},{j}] koscku", cmd_id=session_cid)
                return
            
            cid = ids.new_id("moveJ")
            path = [
                list(robot.paleta1_safe_joint[i, j]) + [1.6, 2.1, 0.01],
                list(robot.paleta1_work_joint[i, j]) + [0.5, 2.1, 0]
            ] # Pod do koscka na paleti 1
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_piece"})
            robot.rtde_c.moveJ(path)
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_piece"})  

            cid = ids.new_id("gripper")
            log.gripper(cmd_id=cid, method="gripper_open", status="started", level="INFO", extra={"session": session_cid, "step": "gripper_close_piece"})
            robot.gripper_open() 
            log.gripper(cmd_id=cid, method="gripper_open", status="completed", level="INFO", extra={"session": session_cid, "step": "gripper_close_piece"})

            cid = ids.new_id("moveJ")
            path = [
                list(robot.paleta1_work_joint[i, j]) + [1.6, 2.1, 0.01],
                list(robot.paleta1_safe_joint[i, j]) + [1.6, 2.1, 0.01],
                list(random_safe[i, j]) + [1.6, 2.1, 0.01],
                list(random_drop[i, j]) + [1.6, 2.1, 0.0]
            ]
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_drop"})
            robot.rtde_c.moveJ(path)
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_drop"})

            cid = ids.new_id("gripper")
            log.gripper(cmd_id=cid, method="gripper_close", status="started", level="INFO", extra={"session": session_cid, "step": "gripper_close_drop"})
            robot.gripper_close()
            log.gripper(cmd_id=cid, method="gripper_close", status="completed", level="INFO", extra={"session": session_cid, "step": "gripper_close_drop"})

            cid = ids.new_id("moveJ")
            path = [
                list(random_drop[i,j]) + [1.6, 2.1, 0.01],
                list(random_safe[i,j]) + [1.6, 2.1, 0.05]                
            ]
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe"})
            robot.rtde_c.moveJ(path)
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe"})

            zapolnjenost_paleta1_2d[i,j] = [i,j]  #nastavim flat2d na i,j
            zapolnjenost_paleta1_2d = np.array(zapolnjenost_paleta1_2d, dtype=object)
            np.save("zapolnjenost_paleta1.npy", zapolnjenost_paleta1_2d.flatten())
            log.event("file_saved", "INFO", f"Updated zapolnjenost_paleta1.npy after moving piece [{i},{j}]", cmd_id=session_cid)

    log.event("shuffling_complete", "INFO", "Shuffling complete.", cmd_id=session_cid)

    cid = ids.new_id("homing")
    log.motion(cmd_id=cid, method="homing", status="started", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})
    robot.homing()
    log.motion(cmd_id=cid, method="homing", status="completed", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})


def pobiranje_s_kamero():
    """
    Funkcija, ki pobira koscke s kamero. Gre nad vsak koscek na paleti 2, zajame sliko, prepozna koscek in ga pobere, če je prepoznan.

    """
    stop_event.clear()
    
    robot.homing()
    print("Homing") 
    robot.gripper.move_and_wait_for_pos(229, speed=200, force=2)
    print("Gripper closed")

    # Ime za csv se kreira na zacetku
    os.makedirs("Template_CSV_datoteke", exist_ok=True)
    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
    ime = f"Template_CSV_datoteke/Template_matching_scores_{timestr}.csv"

    # Preverimo ce obstaja flat_map in ce če je napolnjen z pozicijami
    if os.path.exists("flat_map.npy"):
        flat_map = np.load("flat_map.npy", allow_pickle=True)
        flat_map = np.array(flat_map, dtype=object)
        print(f"[INFO] Naložena obstoječa flat_map.npy: {flat_map}")
    else:
        flat_map = [[i, j] for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
        flat_map = np.array(flat_map, dtype=object)
        np.save("flat_map.npy", flat_map)
        print(f"[INFO] Kreirana nova flat_map.npy: {flat_map}")

    #zanka za zajem slike in vse ostalo :D
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_event.is_set():
                print("Cikel prekinjen s STOP gumbom")
                return
            
            # 1) Gre nad kos za kamero
            path = [
                list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.2, 2.1, 0.01],
                list(map(float, robot.paleta2_kam_joint[i, j])) + [1.2, 2.1, 0.0]
            ]
            robot.rtde_c.moveJ(path)

            # 2) Zajame sliko in template matcha
            robot.ring_ON()
            cam.capture_image()
            robot.ring_OFF()
            slika, score_match, top_coarse, top_full = cam.template_match_multiscale(show=False)
            # zapisem top 5 slik
            with open(ime, "a", encoding="utf8") as f:
                f.write("coarse:" + ",".join([f"{k}={v:.6f}" for k, v in top_coarse]) + "\n")
                f.write("full:" + ",".join([f"{k}={v:.6f}" for k, v in top_full]) + "\n")

            if slika == "template_prazna.png":
                print("Pozicija je prazna, preskakujem...")
                path = [
                    list(map(float, robot.paleta2_kam_joint[i, j])) + [1.6, 2.1, 0.01],
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.1, 0.0],
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
            if np.all(flat_map[idx] != None):
                # 3) Pick iz palete 2 - rotiranje 
                target_safe = robot.rtde_c.getInverseKinematics(robot.pomik_rotacija(robot.paleta2_safe[i, j], kot)[0], robot.home_p)
                target_work = robot.rtde_c.getInverseKinematics(robot.pomik_rotacija(robot.paleta2_work[i, j], kot)[0], robot.home_p)
                path = [
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.1, 0.01],
                    list(map(float, target_safe)) + [1.6, 2.1, 0.005],
                    list(map(float, target_work)) + [0.2, 2.1, 0.0],
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
                    list(map(float, target_safe)) + [1.6, 2.1, 0.01],
                    list(map(float, robot.paleta1_safe_joint[row, col])) + [1.6, 2.1, 0.005],
                    list(map(float, robot.paleta1_drop_joint[row, col])) + [1.6, 2.1, 0.0],
                ]

                # 5) Place
                robot.rtde_c.moveJ(path)
                robot.gripper_close()

                    # 6) Dvig nad odlagališče
                path = [
                    list(map(float, robot.paleta1_drop_joint[row, col])) + [1.6, 2.1, 0.005],
                    list(map(float, robot.paleta1_safe_joint[row, col])) + [1.6, 2.1, 0.0],
                ]
                robot.rtde_c.moveJ(path)
                    
                #ko polozi sliko, se v matriki pozicij namesto indeksov appenda None
                flat_map[idx] = None
                np.save("flat_map.npy", flat_map)

            elif np.all(flat_map[idx] == None):
                print(f"[INFO] Pozicija {idx} je že zapolnjena, nadaljujem...")
                path = [
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.1, 0.005]
                ]
                robot.rtde_c.moveJ(path)
                print("[INFO] Premikam se nad sliko - varna pozicija")

                
    if not np.all(flat_map==None):
        print("[INFO] Pobiranje končano, vsi koščki niso pobrani, kljub temu so vsi v flat_map-u nastavljeni na None.")
        flat_map = [None for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
        np.save("flat_map.npy", flat_map)

    #homing nazaj
    robot.homing()


def celoten_loop():
    stop_event.clear()
    while True:
        if os.path.exists("flat_map.npy"):
            zasedenost = np.load("flat_map.npy", allow_le=True)
            if stop_event.is_set():
                print("[INFO] stopping program")
                break
            if np.all(zasedenost==None):
                print("[INFO] Vsi koščki so pobrani, začenjam shufflanje...")
                shuffling_kosckov()
            else:
                if stop_event.is_set():
                    print("[INFO] stopping program")
                    break
                print("[INFO] Koščki so na plaleti 2, začenjam pobiranje s kamero...")
                pobiranje_s_kamero()
        else:
            print("[INFO] Datoteka flat_map.npy ne obstaja, kreiram novo in začenjam pobiranje s kamero...")
            flat_map = [[i, j] for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
            flat_map = np.array(flat_map, dtype=object)
            np.save("flat_map.npy", flat_map)
            pobiranje_s_kamero()
        time.sleep(0.5)


def prepoznava_slik():
    """
    Premiki samo za prepoznavo slik, brez pobiranja in odlaganja. Za testiranje template matchinga in pozicij kamer.
    """
    stop_event.clear()
    print("[INFO] Začenjam prepoznavo slik... najprej homing")
    robot.homing()
    print("[INFO] Homing completed.")

    # Ime za csv se kreira na zacetku
    print("[INFO] Kreiram ime za CSV datoteko...")
    os.makedirs("Template_CSV_datoteke", exist_ok=True)
    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
    ime = f"Template_CSV_datoteke/Template_matching_scores_{timestr}.csv"

    # Zanka za premikanje in primerjavo med sliko, prav tako se vsak score beleži v .csv datoteko
    for i in range(robot.paleta2_kam_joint.shape[0]):
        for j in range(robot.paleta2_kam_joint.shape[1]):
            if stop_event.is_set():
                print("Cikel prekinjen s STOP gumbom")
                return
            
            # 1) gre nad kos za kamero(safe pozicija) - podamo pot - path
            path = [
                list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.3, 0.01],
                list(map(float, robot.paleta2_kam_joint[i, j])) + [1.6, 2.3, 0.0]
            ]
            robot.rtde_c.moveJ(path)

            # 2) zajame sliko in template matcha, score zapiše v .csv datoteko
            robot.ring_ON()
            cam.capture_image()
            robot.ring_OFF()
            slika, score_match, top_coarse, top_full = cam.template_match_multiscale(top_n=3)
            print("[INFO] Slika zajeta, izvajam template matching...")

            print("[INFO] Zapisujem v .csv")
            with open(ime, "a", encoding="utf8") as f:
                f.write("coarse:" + ",".join([f"{k}={v:.6f}" for k, v in top_coarse]) + "\n")
                f.write("full:" + ",".join([f"{k}={v:.6f}" for k, v in top_full]) + "\n")
            print(f"[INFO] Slika: {slika} | Score: {score_match:.6f}")

            # 3) Gre nazaj na safe pozicijo nad kos - zadnji del zanke, gre naprej na naslednjo iteracijo
            robot.rtde_c.moveJ(list(map(float, robot.paleta2_kam_safe_joint[i,j])), 1.6, 2.3)
            
    # homing
    print("[INFO] Prepoznavanje slik končano, izvajam homing...")
    robot.homing()   

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
                list(robot.paleta2_kam_safe_joint[i, j]) + [1.2, 1.4, 0.0],
                list(robot.paleta2_kam_joint[i, j]) + [1.2, 1.4, 0.0]
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
                list(robot.paleta2_kam_joint[i, j]) + [1.2, 1.4, 0.0],
                list(robot.paleta2_kam_safe_joint[i, j]) + [1.2, 1.4, 0.0]
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
    

