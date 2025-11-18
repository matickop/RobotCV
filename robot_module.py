import os
import math
import numpy as np
import rtde_control
import rtde_receive
import rtde_io
import dashboard_client
import random
import robotiq_gripper
from scipy.spatial.transform import Rotation as R
import time
import threading


class MyRobot:
    def __init__(self, host):
        #ROBOT PARAM:
        self.rob_freq = 500.0
        #WATCHDOG PARAM
        self.watchdog_flag = threading.Event()
        self.watchdog_thread = None
        #RTDE connectionP
        self.rtde_c = rtde_control.RTDEControlInterface(host, self.rob_freq)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
        self.rtde_io = rtde_io.RTDEIOInterface(host)
        self.dash = dashboard_client.DashboardClient(host, 29999)
        self.dash.connect(2000)
        ##REUPLOAD SCRIPTS - CLEAR PROTECTIVE STOP
        #self.rtde_c.reuploadScript()
        #START WATCHDOG
        print("[INFO] Povezava z robotom uspešna")
        self.rtde_c.setWatchdog(10.0) # 10 --> 10Hz --> 0.1s
        self.watchdog_thread = threading.Thread(target=self.watchdog_kicker, daemon=True)
        self.watchdog_thread.start()
        print(f"[INFO] Watchdog nastavljen na 10Hz")
        # gripper connection
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(host, 63352)
        # parametri hitrosti
        self.acc = 0.2
        self.accq = 0.4
        self.vel = 0.4
        self.velq = 0.6
        # home position
        self.home_p = [ math.radians(-90),
                        math.radians(-90),
                        math.radians(-90),
                        math.radians(-90),
                        math.radians(90),
                        math.radians(0)]
        # HOMING
        self.homing()
        self.ensure_home(max_attempts=3, tol=0.05)
        # GRIPPER AKTIVIRAN
        self.gripper.activate()
        self.gripper_close()
        # generacija mrez
        self._load_paleta(1) #atributi se shranijo v 3 palete glede na z offset, vse so pretvorjene v joint space:    
                            # paleta1_safe/_joint, paleta1_work/_joint, paleta1_kam/joint 
        self._load_paleta(2)# isto sam da so paleta2_xsdasdsad
        print(f"[INFO] Mreže generirane")
        # loadanje kotov
        if os.path.exists("koti.npy"):
            self.pobrani_koti = np.load("koti.npy", allow_pickle=True).tolist()
        else:
            self.pobrani_koti = [None] * 8

        # ostali parametri
        self.kamera_y = 0.1
        self.safe_z = 0.04
        self.work_z = 0.0112
        self.drop_z = 0.0152
        self.kamera_z = 0.0245
        self.freedrive_active = False
        self.payload_mass = 1.15
        self.cog = [0, -0.004, 0.045]
        self.inital_tcp_rotation = np.array([0, 3.14, 0])
        self.tcp_rotation_paleta1 = None
        self.tcp_rotation_paleta2 = None 
        self.rtde_c.setPayload(self.payload_mass, self.cog)

    def watchdog_kicker(self):
        varnostni_faktor = 0.01 # 10ms
        perioda = 0.1
        ciljni_cas = perioda - varnostni_faktor
        while not self.watchdog_flag.is_set():
            start_time = time.monotonic()
            try:
                self.rtde_c.kickWatchdog()
            except rtde_control.RTDEException as e:
                print(f"[ERROR] Napaka WATCHDOG - prekinjena povezava z robotom: {e}")
                break
            elapsed = time.monotonic() - start_time
            sleep_time = ciljni_cas - elapsed
            if sleep_time > 0:
                sleep_time -= varnostni_faktor
                time.sleep(sleep_time)

    def reconnect(self, host="192.168.3.102"):
        try:
            self.rtde_c.reconnect()
            self.rtde_r.reconnect()
            self.rtde_io.reconnect()
            self.dash.connect(2000)
            self.rtde_c.setPayload(self.payload_mass, self.cog)
            print(f"[INFO] RTDE Services reconnceted, starting watchdog")

            self.rtde_c.setWatchdog(10.0) # 10 --> 10Hz --> 0.1s
            self.watchdog_thread = threading.Thread(target=self.watchdog_kicker, daemon=True)
            self.watchdog_thread.start()
        except Exception as e:
            print(f"Reconnect failed {e}")
            return
   
    def _load_paleta(self, ime):
        """Naloži vse mreže za paleto (1 ali 2) in nastavi atribute self.paletaX_safe, ..."""
        suffixes = ["safe", "work", "kam", "kam_safe", "drop"]
        for suf in suffixes:
            try:
                pose = np.load(f"mreza_paleta{ime}_{suf}.npy")
                joints = np.load(f"mreza_paleta{ime}_{suf}_joint.npy")
            except FileNotFoundError:
                pose, joints = None, None
            # nastavi atribute, npr. self.paleta1_safe, self.paleta1_safe_joint
            setattr(self, f"paleta{ime}_{suf}", pose)
            setattr(self, f"paleta{ime}_{suf}_joint", joints)


    def homing(self):  
        self.rtde_c.moveJ(self.home_p, self.accq, self.velq)

    def initialize(self):
        self.homing()
        self.gripper.activate()

    def activate_freedrive(self):
        self.rtde_c.setPayload(self.payload_mass, self.cog)
        self.rtde_c.freedriveMode(free_axes=[1, 1, 1, 0, 0, 0])
        self.freedrive_active = True
    
    def deactivate_freedrive(self):
        self.rtde_c.endFreedriveMode()
        self.freedrive_active = False
    
    def get_actual_tcp_pose(self):
        return np.array(self.rtde_r.getActualTCPPose())
    
    def rotate_pose_local_z(self, pose, angle):
        """
        Vrne nov pose = [x,y,z, rx,ry,rz] kjer je na TCP dodana lokalna rotacija
        za angle_deg (v stopinjah) okoli lokalne Z osi. Pozicija x,y,z ostane enaka.
        pose: iterable length=6 (meters, radians for rotvec)
        """
        pose = np.asarray(pose, dtype=float)
        pos = pose[:3].copy()
        rotvec = pose[3:].copy()   # axis-angle (rx,ry,rz)

        # trenutna rotacija kot scipy Rotation
        R0 = R.from_rotvec(rotvec)

        # lokalna rotacija okoli orodnega Z (v rotvec obliki)
        Rz = R.from_euler('z', angle, degrees=True)

        # za lokalno rotacijo uporabimo R_new = R0 * Rz
        Rnew = R0 * Rz

        new_rotvec = Rnew.as_rotvec()
        return np.concatenate((pos, new_rotvec))

    def generiranje_mreze(self, a, b, koti, paleta):
        poz = np.zeros((a, b, 6))
        zl, zd, sl, sd = [np.array(k, dtype=float) for k in koti]

        #rotacija mreze okoli koordinatnega sistema robota
        vektor_x = (sl[:2] - zl[:2])
        vektor_x /= np.linalg.norm(vektor_x)

        rotacija_mreze = np.degrees(np.arctan2(vektor_x[1], vektor_x[0]))
        R0 = R.from_rotvec(self.inital_tcp_rotation)
        Rz = R.from_euler("z", rotacija_mreze, degrees=True)
        Rnew = Rz*R0 
        new_rotvec = Rnew.as_rotvec()
        
        if paleta == "1":
            self.tcp_rotation_paleta1 = new_rotvec
            tcp_rot = self.tcp_rotation_paleta1
        elif paleta == "2":
            self.tcp_rotation_paleta2 = new_rotvec
            tcp_rot = self.tcp_rotation_paleta2

        print(f"TCP rotacija okoli palete1 je: {self.tcp_rotation_paleta1}")
        print(f"TCP rotacija okoli palete2 je: {self.tcp_rotation_paleta2}")

        mreza_safe  = np.zeros((a, b, 6))
        mreza_work  = np.zeros((a, b, 6))
        mreza_kam   = np.zeros((a, b, 6))
        mreza_kam_safe = np.zeros((a, b, 6))
        mreza_drop = np.zeros((a, b, 6))

        for i in range(a):
            v_left = zl - (zl - sl)*(i/(a-1))
            v_right = zd - (zd - sd)*(i/(a-1))
            for j in range(b):
                poz[i,j] = v_left + (v_right - v_left)*(j/(b - 1))
                poz[i,j][2] = 0.0095
                poz[i,j][3:] = tcp_rot
                #safe
                pos_safe = poz[i,j].copy()
                pos_safe[2] = self.safe_z
                mreza_safe[i, j] = pos_safe
                
                #work
                pos_work = poz[i, j].copy()
                pos_work[2] = self.work_z
                mreza_work[i, j] = pos_work

                #drop
                pos_drop = poz[i,j].copy()
                pos_drop[2] = self.drop_z
                mreza_drop[i, j] = pos_drop

                #kamera
                pos_kam = poz[i,j].copy()
                pos_kam[1] += self.kamera_y
                pos_kam[2] = self.kamera_z
                mreza_kam[i, j] = pos_kam

                #kamera safe
                pos_kam_safe= poz[i,j].copy()
                pos_kam_safe[1] += self.kamera_y
                pos_kam_safe[2] = self.safe_z
                mreza_kam_safe[i, j] = pos_kam_safe 



        return mreza_safe, mreza_work, mreza_kam, mreza_kam_safe, mreza_drop
    
    def pripravi_in_shrani_paleto(self, ime, koti, oznaka, a=4, b=6):
        # generiranje mrež (pose)
        safe, work, kam, kam_safe, drop = self.generiranje_mreze(a, b, koti, oznaka)
        # pretvorba v joint
        safe_joint  = self.pretvori_v_joint_mreze(safe)
        work_joint  = self.pretvori_v_joint_mreze(work)
        kam_joint   = self.pretvori_v_joint_mreze(kam)
        kam_safe_joint = self.pretvori_v_joint_mreze(kam_safe)
        drop_joint = self.pretvori_v_joint_mreze(drop)

        # shrani
        np.save(f"mreza_{ime}_safe.npy", safe)
        np.save(f"mreza_{ime}_work.npy", work)
        np.save(f"mreza_{ime}_kam.npy", kam)
        np.save(f"mreza_{ime}_kam_safe.npy", kam_safe)
        np.save(f"mreza_{ime}_drop.npy", drop)

        np.save(f"mreza_{ime}_safe_joint.npy", safe_joint)
        np.save(f"mreza_{ime}_work_joint.npy", work_joint)
        np.save(f"mreza_{ime}_kam_joint.npy", kam_joint)
        np.save(f"mreza_{ime}_kam_safe_joint.npy", kam_safe_joint)
        np.save(f"mreza_{ime}_drop_joint.npy", drop_joint)

        return safe, work, kam, kam_safe, drop, safe_joint, work_joint, kam_joint, kam_safe_joint, drop_joint


    def pretvori_v_joint_mreze(self, mreza_pose):
        """
        Pretvori mrežo TCP pozicij v mrežo joint konfiguracij.
        """
        mreza_joint = np.zeros_like(mreza_pose)
        q_seed = self.rtde_r.getActualQ()

        for i in range(mreza_joint.shape[0]):
            for j in range(mreza_joint.shape[1]):
                q_target = self.rtde_c.getInverseKinematics(mreza_pose[i, j], q_seed)
                # izberi varnega kandidata za wrist6
                print("q_target type:", type(q_target), "q_target[5]:", q_target[5])
                q_target[5] = self.pick_wrist6_candidate(q_target[5], q_seed[5])
                mreza_joint[i, j] = q_target
                q_seed = q_target  # seed za naslednjo točko

        return mreza_joint 

    def pick_wrist6_candidate(self, q6_target, q6_current, limit_deg=180, alpha=0.002):
        # Evaluate q6_target + k*2π for k in {-1, 0, +1}
        candidates = [q6_target + k*2*np.pi for k in (-1, 0, 1)]
        def cost(q6):
            dist = abs(q6 - q6_current)
            # penalize proximity to soft limits (±limit_deg)
            q6_deg = np.degrees(q6)
            proximity = max(0.0, abs(q6_deg) - (limit_deg - 20))  # start penalizing near the edge
            return dist + alpha * proximity
        # choose the candidate with minimum cost, but discard those beyond hard limit
        hard_limit_deg = 250
        feasible = [c for c in candidates if abs(np.degrees(c)) <= hard_limit_deg]
        if not feasible:
            feasible = candidates
        return min(feasible, key=cost)

    def generiraj_random_joint_mreze(self, safe_joint, work_joint):
        """
        Sprejme dve obstoječi joint mreži (safe in work) in vrne
        naključno premešani mreži, kjer ima vsaka točka še random rotacijo q6.
        """
        a, b, _ = safe_joint.shape
        n = a * b

        # splošči
        flat_safe = safe_joint.reshape(n, 6).copy()
        flat_work = work_joint.reshape(n, 6).copy()

        # permutacija
        perm = np.random.permutation(n)
        flat_safe = flat_safe[perm]
        flat_work = flat_work[perm]

        # možne rotacije okoli zapestja
        rotations = [-np.pi/2, 0.0, np.pi/2, np.pi]

        q_seed = self.rtde_r.getActualQ()

        for i in range(n):
            ang = np.random.choice(rotations)

            # dodaj rotacijo na q6 (joint 5 v Python indeksu = 5)
            flat_safe[i][5] = self.pick_wrist6_candidate(flat_safe[i][5] + ang, q_seed[5])
            flat_work[i][5] = self.pick_wrist6_candidate(flat_work[i][5] + ang, flat_safe[i][5])

            q_seed = flat_safe[i]  # seed za naslednjo točko

        # nazaj v obliko
        random_safe = flat_safe.reshape(a, b, 6)
        random_work = flat_work.reshape(a, b, 6)

        return random_safe, random_work

    def move_fine(self, position):
        self.rtde_c.moveL(position, speed = 0.2, acceleration=0.3)

    def move_with_random_rotation(self, position):
        position[2] = self.safe_z
        q_position = self.rtde_c.getInverseKinematics(position)
        possible_rotation = [-np.pi/2, 0, np.pi/2, np.pi]
        ang = random.choice(possible_rotation)
        q_position[5] += ang
        self.rtde_c.moveJ(q_position, self.accq, self.velq)

    def joint_move_to_position(self, position):
        self.rtde_c.moveJ(position, self.accq, self.velq)

    def move_to_position(self, position): # "Varna" pozicija, z je vec kot dovolj visok
        self.rtde_c.moveJ(position, self.accq, self.velq)

    def pick_and_place_position(self, position): # S to pozicijo se koscek pobere in odlozi
        position[2] = self.work_z
        q_position = self.rtde_c.getInverseKinematics(position)
        self.rtde_c.moveJ(q_position, self.acc, self.vel)

    def move_to_kamera_position(self, position): # Pozicija kamera glede z offsetom v x(in y) in zadostna visina
        position[1] = position[1] + self.kamera_y
        position[2] = self.kamera_z
        q_position = self.rtde_c.getInverseKinematics(position)
        self.rtde_c.moveJ(q_position, self.accq, self.velq)

    def gripper_open(self):
        self.gripper.move_and_wait_for_pos(209, speed=255, force=120)

    def gripper_close(self):
        self.gripper.move_and_wait_for_pos(229, speed=255, force=120)

    def move_with_blend(self, positions, acc=1.2, vel=0.5, blend=0.01):
        """
        Izvede zaporedje joint pozicij z blendingom.
        positions: seznam joint vektorjev (brez gripperja!)
        """
        path = []
        for q in positions:
            q = q.tolist() if hasattr(q, "tolist") else list(q)
            path.append(q + [acc, vel, blend])
        self.rtde_c.moveJ(path)  

    def ring_ON(self):
        self.rtde_io.setStandardDigitalOut(0, True)

    def ring_OFF(self):
        self.rtde_io.setStandardDigitalOut(0, False)

    def teachMode(self):
        self.rtde_c.teachMode()
        self.freedrive_active = True
        
    def endTeachMode(self):
        self.rtde_c.endTeachMode()
        self.freedrive_active = False

    def disconnect(self):
        print(f"[INFO] Prekinjam povezavo z robotom in kamero")
        # Flag za ustavitev watchdog threada
        self.watchdog_flag.set()
        print("[INFO] Čakam da se watchdog zaustavi")
        if self.watchdog_thread and self.watchdog_thread.is_alive():
            self.watchdog_thread.join(timeout=1.5)
        # Prekinjamo povezavo z robotom - vsemi interfaci in kamero
        try:
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
            self.gripper.disconnect()
            print("[INFO] Control, recieve in IO uspešno disconnectano")
        except Exception as e:
            print(f"[WARN] Prišlo je do napake med prekinjanjem povezave: {e}")

        if self.dash.safetystatus() == "Safetystatus: PROTECTIVE_STOP":
            time.sleep(5)
            self.dash.unlockProtectiveStop()
            self.dash.disconnect()
            print(f"[INFO] PS CLEARED: SHUTING DOWN")


    def is_at_home(self, tol=0.01):
        actual = np.array(self.rtde_r.getActualQ())
        home = np.array(self.home_p)
        # Izračunaj razliko med jointi, če je vsa manjša od tolerance, je OK
        return np.all(np.abs(actual - home) < tol)
    
    def ensure_home(self, max_attempts=3, tol=0.05):
        for attempt in range(max_attempts):
            self.homing()
            if self.is_at_home(tol=tol):
                print(f"[INFO] Robot is at home (after {attempt+1} attempts)")
                return
            else:
                print(f"[WARN] Robot not at home after attempt {attempt+1}")
                time.sleep(0.5)
        raise RuntimeError("[ERROR] Robot could not reach home position after multiple attempts!")
    
    def unlock_protective_stop(self):
        """Protective stop se unlocka"""
        host = "192.168.3.102"
        try:
            self.dash.unlockProtectiveStop()
            print(f"[INFO] Protective stop cleared, reconnecting to RTDE...")
            time.sleep(5)
            try:
                self.rtde_c.reconnect()
                self.rtde_r.reconnect()
                self.rtde_io.reconnect()
                time.sleep(5)
                self.rtde_c.reuploadScript()
                print(f"[INFO] Successfully connected to RTDE")
            except Exception as e:
                print(f"[ERROR] Prišlo je do napake pri reconnectanju: {e}")
            
            try:
                self.watchdog_thread = threading.Thread(target=self.watchdog_kicker, daemon=True)
                self.watchdog_thread.start()
            except Exception as e:
                print(f"[ERROR] Vzpostavitev watchdoga ni uspela: {e}")
        except Exception as e:
            print(f"[ERROR] Protective stop not cleared: {e}")
            return
    
    def protective_stop(self):
        """prisilni stop, ni isto kot emergency stop"""
        self.rtde_c.triggerProtectiveStop()
        self.watchdog_flag.set()
        time.sleep(1)
        try:
            if self.rtde_c.isConnected():
                self.rtde_c.disconnect()
                print("rtde_c disconnected")
            else:
                print("rtde_c je ze disconnectan")
        except Exception as e:
            print(f"[ERROR] Prišlo je do napake disconnectanja: {e}")

        print(f"[WARN] PROTECTIVE STOP: Povezava z RTDE prekinjena, počisti protective stop")
        