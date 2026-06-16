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
gui = None

# Pomožne funkcije za preverjanje zapolnjenosti, kreiranje csv datoteko...
def koscki_pobrani():
    """
    Ko so vsi koščki pobrani, je datoteka "zapolnjenost_paleta1.npy" polna s -1 vrednostmi.
    V primeru da niso vsi koscki pobrani, je treba najprej pobrati vse koscke, ko jih enkrat poberemo in razmečemo,
    je datoteka polna z pozicijam, kam bo treba, po prepoznavi koscek odloziti.
    V primeru da datoteke ni, se ustvari nova, ki je polna z -1 vrednostmi, kar pomeni da še ni noben koscek pobran.
    """
    file_name = "zapolnjenost_paleta1.npy"
    try:
        zapolnjenost_paleta1 = np.load(file_name)
        log.event("file_check", "DEBUG", f"Loaded {file_name} successfully.")
        if np.all(zapolnjenost_paleta1 == -1):
            log.event("paleta1_polna", "INFO", "Vsi koscki so pobrani.")

            return True
        else:
            log.event("paleta1_ni_polna", "INFO", f"Stevilo nepobranih kosckov: {np.sum(zapolnjenost_paleta1 != -1)//2}")

            return False

    except FileNotFoundError:
        log.event("file_not_found", "DEBUG", f"File {file_name} not found.")
        zapolnjenost_paleta1 = np.array([[i, j] for i in range(4) for j in range(6)], dtype=np.int64)
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
    Za preverjanje ce so koscki na paleti je zgolj samo zapolnjenost_paleta1.npy kateri se prazni tekom pobiranja kosckov, ce se ni vsak
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
    zapolnjenost_paleta1_2d = np.load("zapolnjenost_paleta1.npy").reshape(rows, cols, 2)

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
                list(random_safe[i,j]) + [1.6, 2.1, 0.0]                
            ]
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe"})
            robot.rtde_c.moveJ(path)
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe"})

            zapolnjenost_paleta1_2d[i,j] = [i,j]  #nastavim flat2d na i,j
            zapolnjenost_paleta1_2d = np.array(zapolnjenost_paleta1_2d)
            np.save("zapolnjenost_paleta1.npy", zapolnjenost_paleta1_2d.reshape(-1, 2))
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

    session_cid = ids.new_id("Pobiranje_s_kamero")

    if not koscki_pobrani():
        log.event("paleta1_ni_polna", "WARNING", "Vsi koscki niso pobrani ali pa datoteke ni, potrebno naloziti zapolnjenost palete 1!", cmd_id=session_cid)
        zapolnjenost_paleta1 = np.load("zapolnjenost_paleta1.npy")
        log.event("file_check", "DEBUG", f"Loaded zapolnjenost_paleta1.npy: {zapolnjenost_paleta1}", cmd_id=session_cid)

    cid = ids.new_id("homing") # homing za začetek pobiranja s kamero
    log.motion(cmd_id=cid, method="homing", status="started",actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})
    robot.homing()
    log.motion(cmd_id=cid, method="homing", status="completed", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})

    cid = ids.new_id("gripper") # zapremo gripper
    log.gripper(cmd_id=cid, method="gripper_close", status="started", level="INFO", extra={"session": session_cid})
    robot.gripper_close()
    log.gripper(cmd_id=cid, method="gripper_close", status="completed", level="INFO", extra={"session": session_cid})

    ime = kreiranje_csv_datoteke() # kreiramo csv datoteko za shranjevanje rezultatov template matchinga

    rows, cols = robot.paleta2_kam_joint.shape[:2]

    for i in range(rows): #zanka za zajem slike in vse ostalo :D
        for j in range(cols):
            
            if stop_event.is_set():
                log.event("stop_requested", "WARNING", f"Shuffling prekinjen pri [{i},{j}] koscku", cmd_id=session_cid)
                return
            
            # gre nad kos za kamero
            cid = ids.new_id("moveJ")
            log.event("move_to_camera_position", "INFO", f"Moving to camera position for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid})
            path = [
                list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.2, 2.1, 0.01],
                list(map(float, robot.paleta2_kam_joint[i, j])) + [1.2, 2.1, 0.0]
            ] # pot do pozicije nad kosckom
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_camera_position"})
            robot.rtde_c.moveJ(path) # premik nad koscek
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_camera_position"})

            # zajame sliko in template matcha
            cid = ids.new_id("capture_and_match")
            log.event("capture_and_match", "INFO", f"Capturing image and matching for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid})
            log.event("ring_on", "INFO", f"Turning ring ON for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid, "step": "ring_on"})
            robot.ring_ON()
            log.camera(cmd_id=cid, method="capture_image", status="started", level="INFO", extra={"session": session_cid, "step": "capture_image"})
            cam.capture_image()
            log.camera(cmd_id=cid, method="capture_image", status="completed", level="INFO", extra={"session": session_cid, "step": "capture_image"})
            robot.ring_OFF()
            log.event("ring_off", "INFO", f"Turning ring OFF for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid, "step": "ring_off"})

            log.camera(cmd_id=cid, method="template_match_multiscale", status="started", level="INFO", extra={"session": session_cid, "step": "template_match"})
            slika, score_match, top_coarse, top_full = cam.template_match_multiscale(show=False)
            log.camera(cmd_id=cid, method="template_match_multiscale", status="completed", level="INFO", extra={"session": session_cid, "step": "template_match", "matched_piece": slika, "match_score": score_match})

            # zapisem top 5 slik
            with open(ime, "a", encoding="utf8") as f:
                f.write("coarse:" + ",".join([f"{k}={v:.6f}" for k, v in top_coarse]) + "\n")
                f.write("full:" + ",".join([f"{k}={v:.6f}" for k, v in top_full]) + "\n")

            if slika == "template_prazna.png": # ce ni koscka, se premakne na safe pozicijo nad kosckom in nadaljuje z naslednjim kosckom
                cid = ids.new_id("no_piece_detected")
                log.event("no_piece_detected", "WARNING", f"No piece detected at position [{i},{j}]. Moving to next position.", cmd_id=cid, extra={"session": session_cid})

                cid = ids.new_id("moveJ")
                log.event("move_to_safe_position", "INFO", f"Moving to safe position for piece [{i},{j}] since no piece was detected.", cmd_id=cid, extra={"session": session_cid, "step": "move_to_safe_position"})
                path = [
                    list(map(float, robot.paleta2_kam_joint[i, j])) + [1.6, 2.1, 0.01],
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.1, 0.0],
                ]
                log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position"})
                robot.rtde_c.moveJ(path)
                log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position"})
                
                continue
            
            cid = ids.new_id("piece_detected") # ce je koscek detektiran, se ga poskuša indeksirati, da se dobi indeks in kot, ki sta potrebna za pobiranje koscka
            log.event("piece_detected", "INFO", f"Indexing detected piece for position [{i},{j}]: {slika}", cmd_id=cid, extra={"session": session_cid})
            try:
                log.camera(cmd_id=cid, method="index_piece", status="started", level="INFO", extra={"session": session_cid, "detected_piece": slika})
                slika = slika.split(".")[0]
                idx, kot = slika.split("_")
                idx = int(idx)
                kot = int(kot)
                log.camera(cmd_id=cid, method="index_piece", status="completed", level="INFO", extra={"session": session_cid, "detected_piece": slika, "indexed_piece": (idx, kot)})
            except Exception as e:
                log.camera(cmd_id=cid, method="index_piece", status="error", level="ERROR", extra={"session": session_cid, "detected_piece": slika, "error": str(e)})
                continue

            # gre nad koscek, katerege je zajel s kamero
            cid = ids.new_id("moveJ")
            if np.all(zapolnjenost_paleta1[idx] != -1):
                # 3) Pick iz palete 2 - rotiranje 
                log.event("move_to_piece_position", "INFO", f"Moving to piece position for piece [{i},{j}] with index {idx} and angle {kot}", cmd_id=cid, extra={"session": session_cid})
                target_safe = robot.rtde_c.getInverseKinematics(robot.pomik_rotacija(robot.paleta2_safe[i, j], kot)[0], robot.home_p)
                target_work = robot.rtde_c.getInverseKinematics(robot.pomik_rotacija(robot.paleta2_work[i, j], kot)[0], robot.home_p)
                path = [
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.1, 0.01],
                    list(map(float, target_safe)) + [1.6, 2.1, 0.005],
                    list(map(float, target_work)) + [0.2, 2.1, 0.0],
                ]

                log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_piece_position"})
                robot.rtde_c.moveJ(path)
                log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_piece_position"})
                
                cid = ids.new_id("gripper")
                log.gripper(cmd_id=cid, method="gripper_open", status="started", level="INFO", extra={"session": session_cid, "step": "gripper_open"})
                robot.gripper_open()
                log.gripper(cmd_id=cid, method="gripper_open", status="completed", level="INFO", extra={"session": session_cid, "step": "gripper_open"})

                # 4) Dvig + pot do cilja v paleti 1
                cid = ids.new_id("moveJ")
                log.event("move_to_drop_position", "INFO", f"Moving to drop position for piece [{i},{j}] with index {idx} and angle {kot}", cmd_id=cid, extra={"session": session_cid})
                row, col = zapolnjenost_paleta1[idx] # iz arraya zapolnjenost_paleta1 z indeksom slike dobimo pozicijo, kamor je treba koscek odloziti
                log.event("get_drop_position", "INFO", f"Getting drop position for piece with index {idx} at position [{row},{col}] on paleta 1", cmd_id=cid, extra={"session": session_cid})
                log.event("drop_position_details", "DEBUG", f"Drop position details for piece with index {idx}: safe_joint={robot.paleta1_safe_joint[row, col]}, drop_joint={robot.paleta1_drop_joint[row, col]}", cmd_id=cid, extra={"session": session_cid})
                path = [
                    list(map(float, target_safe)) + [1.6, 2.1, 0.01],
                    list(map(float, robot.paleta1_safe_joint[row, col])) + [1.6, 2.1, 0.005],
                    list(map(float, robot.paleta1_drop_joint[row, col])) + [1.6, 2.1, 0.0],
                ]

                log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_drop_position"})
                robot.rtde_c.moveJ(path)
                log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_drop_position"})    

                cid = ids.new_id("gripper")
                log.gripper(cmd_id=cid, method="gripper_close", status="started", level="INFO", extra={"session": session_cid, "step": "gripper_close"})
                robot.gripper_close()
                log.gripper(cmd_id=cid, method="gripper_close", status="completed", level="INFO", extra={"session": session_cid, "step": "gripper_close"})

                # 6) Dvig nad odlagališče
                cid = ids.new_id("moveJ")
                log.event("move_to_safe_position_after_drop", "INFO", f"Moving to safe position after dropping piece [{i},{j}] with index {idx} and angle {kot}", cmd_id=cid, extra={"session": session_cid})
                path = [
                    list(map(float, robot.paleta1_drop_joint[row, col])) + [1.6, 2.1, 0.005],
                    list(map(float, robot.paleta1_safe_joint[row, col])) + [1.6, 2.1, 0.0],
                ]

                log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position_after_drop"})
                robot.rtde_c.moveJ(path)
                log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position_after_drop"})

                #ko polozi sliko, se v matriki pozicij namesto indeksov appenda None
                log.event("update_zapolnjenost_paleta1", "INFO", f"Updating zapolnjenost_paleta1 for piece with index {idx}", cmd_id=cid, extra={"session": session_cid})
                zapolnjenost_paleta1[idx] = -1
                log.event("zapolnjenost_paleta1_updated", "DEBUG", f"Updated zapolnjenost_paleta1[{idx}] = {zapolnjenost_paleta1[idx]}", cmd_id=cid, extra={"session": session_cid})
                np.save("zapolnjenost_paleta1.npy", zapolnjenost_paleta1)
                log.event("file_saved", "INFO", f"Saved updated zapolnjenost_paleta1.npy after dropping piece with index {idx}", cmd_id=cid, extra={"session": session_cid})

            elif np.all(zapolnjenost_paleta1[idx] == -1):
                log.event("piece_already_picked", "WARNING", f"Piece with index {idx} at position [{i},{j}] has already been picked according to zapolnjenost_paleta1. Moving to safe position.", cmd_id=cid, extra={"session": session_cid})
                path = [
                    list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.1, 0.005]
                ]

                log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position_already_picked"})
                robot.rtde_c.moveJ(path)
                log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position_already_picked"}) 
                log.event("moved_to_safe_position", "INFO", f"Moved to safe position for piece with index {idx} at position [{i},{j}]", cmd_id=cid, extra={"session": session_cid})

                
    # if not np.all(zapolnjenost_paleta1==None):ss
    #     print("[INFO] Pobiranje končano, vsi koščki niso pobrani, kljub temu so vsi v flat_map-u nastavljeni na None.")
    #     flat_map = [None for i in range(robot.paleta2_kam_joint.shape[0]) for j in range(robot.paleta2_kam_joint.shape[1])]
    #     np.save("flat_map.npy", flat_map)

    cid = ids.new_id("homing") # homing za konec pobiranja s kamero
    log.event("pobiranje_complete", "INFO", "Pobiranje s kamero complete. Performing homing.", cmd_id=cid, extra={"session": session_cid})
    log.motion(cmd_id=cid, method="homing", status="started", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "homing_after_pobiranje"})
    robot.homing()
    log.motion(cmd_id=cid, method="homing", status="completed", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "homing_after_pobiranje"})


def celoten_loop():
    """
    Glavna funkcija programa za nemoteno delovanje pobiranja in razmetavanja sestavljanke.
    """

    log.event("program_start", "INFO", "Program started. Entering main loop.")
    stop_event.clear()

    while True:
        if koscki_pobrani():
            cid = ids.new_id("shuffling")
            log.event("shuffling_start", "INFO", "Vsi koščki so pobrani, začenjam shufflanje...", cmd_id=cid)
            shuffling_kosckov()

        else:
            cid = ids.new_id("pobiranje")
            log.event("pobiranje_start", "INFO", "Niso vsi koščki pobrani, začenjam pobiranje s kamero...", cmd_id=cid)
            pobiranje_s_kamero()

        time.sleep(0.5)


def prepoznava_slik():
    """
    Premiki samo za prepoznavo slik, brez pobiranja in odlaganja. Za testiranje template matchinga in pozicij kamer.
    """

    stop_event.clear()

    session_cid = ids.new_id("Prepoznava_slik")
    
    cid = ids.new_id("homing")
    log.event("prepoznava_start", "INFO", "Začela se je prepoznavanje slik s kamero.", cmd_id=session_cid)
    log.motion(cmd_id=cid, method="homing", status="started", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})
    robot.homing()
    log.motion(cmd_id=cid, method="homing", status="completed", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})

    # Ime za csv se kreira na zacetku
    cid = ids.new_id("create_csv")
    log.event("create_csv", "INFO", "Creating CSV file for template matching scores.", cmd_id=cid, extra={"session": session_cid})
    ime = kreiranje_csv_datoteke()

    rows, cols = robot.paleta2_kam_joint.shape[:2]

    for i in range(rows): # Zanka za premikanje in primerjavo med sliko, prav tako se vsak score beleži v .csv datoteko
        for j in range(cols):
        
            if stop_event.is_set():
                log.event("shuffling_start", "INFO", "Začel se je proces shufflanja kosckov iz palete 1 na paleto 2.", cmd_id=session_cid)
                return
            
            # 1) gre nad kos za kamero(safe pozicija) - podamo pot - path
            cid = ids.new_id("moveJ")
            log.event("move_to_camera_position", "INFO", f"Moving to camera position for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid})
            path = [
                list(map(float, robot.paleta2_kam_safe_joint[i, j])) + [1.6, 2.3, 0.01],
                list(map(float, robot.paleta2_kam_joint[i, j])) + [1.6, 2.3, 0.0]
            ]
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_camera_position"})    
            robot.rtde_c.moveJ(path)
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=path[-1], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_camera_position"})  

            # 2) zajame sliko in template matcha, score zapiše v .csv datoteko
            cid = ids.new_id("capture_and_match")
            log.event("capture_and_match", "INFO", f"Capturing image and matching for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid})
            log.event("ring_on", "INFO", f"Turning ring ON for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid, "step": "ring_on"})
            robot.ring_ON()
            log.camera(cmd_id=cid, method="capture_image", status="started", level="INFO", extra={"session": session_cid, "step": "capture_image"})
            cam.capture_image()
            log.camera(cmd_id=cid, method="capture_image", status="completed", level="INFO", extra={"session": session_cid, "step": "capture_image"})
            robot.ring_OFF()
            log.event("ring_off", "INFO", f"Turning ring OFF for piece [{i},{j}]", cmd_id=cid, extra={"session": session_cid, "step": "ring_off"})
            # Po cam.capture_image() in robot.ring_OFF() — prikaži zajeto sliko:
            if gui is not None:
                try:
                    raw = cv2.imread(cam.template_path)
                    if raw is not None:
                        from PIL import Image as PILImage
                        rgb = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
                        gui.update_image(PILImage.fromarray(rgb))
                except Exception:
                    pass

            log.camera(cmd_id=cid, method="template_match_multiscale", status="started", level="INFO", extra={"session": session_cid, "step": "template_match"})
            slika, score_match, top_coarse, top_full = cam.template_match_multiscale(show=False)
            log.camera(cmd_id=cid, method="template_match_multiscale", status="completed", level="INFO", extra={"session": session_cid, "step": "template_match", "matched_piece": slika, "match_score": score_match})
            
            if gui is not None:
                gui.update_match_panel(
                    best_name=slika,
                    best_score=score_match,
                    top3=top_full[:3]
                )
            # Se povezava slike z GUI da se obarva koscek
            if slika and slika != "template_prazna.png":
                try:
                    idx = int(slika.split(".")[0].split("_")[0])
                    gui.set_piece_placed(idx)
                except Exception:
                    pass

            log.event("csv_write", "INFO", f"Template match score for piece [{i},{j}]: {score_match:.6f}", cmd_id=cid, extra={"session": session_cid, "matched_piece": slika, "match_score": score_match})
            with open(ime, "a", encoding="utf8") as f:
                f.write("coarse:" + ",".join([f"{k}={v:.6f}" for k, v in top_coarse]) + "\n")
                f.write("full:" + ",".join([f"{k}={v:.6f}" for k, v in top_full]) + "\n")
            print(f"[INFO] Slika: {slika} | Score: {score_match:.6f}")

            # 3) Gre nazaj na safe pozicijo nad kos - zadnji del zanke, gre naprej na naslednjo iteracijo
            cid = ids.new_id("moveJ")
            log.event("move_to_safe_position", "INFO", f"Moving to safe position for piece [{i},{j}] after capturing and matching", cmd_id=cid, extra={"session": session_cid})
            log.motion(cmd_id=cid, method="moveJ", status="started", target_q=list(map(float, robot.paleta2_kam_safe_joint[i,j])) + [1.6, 2.3, 0.01], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position_after_capture"})  
            robot.rtde_c.moveJ(list(map(float, robot.paleta2_kam_safe_joint[i,j])), 1.6, 2.3)
            log.motion(cmd_id=cid, method="moveJ", status="completed", target_q=list(map(float, robot.paleta2_kam_safe_joint[i,j])) + [1.6, 2.3, 0.01], actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid, "step": "move_to_safe_position_after_capture"})    
            
    # homing
    cid = ids.new_id("homing")
    log.event("prepoznava_complete", "INFO", "Prepoznavanje slik s kamero complete. Performing homing.", cmd_id=cid, extra={"session": session_cid})
    log.motion(cmd_id=cid, method="homing", status="started", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})
    robot.homing()   
    log.motion(cmd_id=cid, method="homing", status="completed", actual_q=robot.rtde_r.getActualQ(), level="INFO", extra={"session": session_cid})  

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
    

