import time
import math
import numpy as np
import rtde_control
import rtde_receive
import random
import robotiq_gripper
from scipy.spatial.transform import Rotation as R


class MyRobot:
    def __init__(self, host):
        #RTDE connection
        self.rtde_c = rtde_control.RTDEControlInterface(host)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
        # gripper connection
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(host, 63352)
        # parametri
        self.acc = 0.2
        self.vel = 0.4
        try:
            self.paleta1 = np.load("mreza_paleta1.npy")
            self.paleta2 = np.load("mreza_paleta2.npy")
            print("Obe mreži za paleti sta naloženi")
        except:
            print("Nobene mreže nimaš shranjene")
            self.paleta1 = None
            self.paleta2 = None
            pass
        self.kamera_y = 0.098348
        self.safe_z = 0.04
        self.work_z = 0.0098
        self.kamera_z = 0.01935
        self.freedrive_active = False
        self.home_p = [ math.radians(-90),
                        math.radians(-90),
                        math.radians(-90),
                        math.radians(-90),
                        math.radians(90),
                        math.radians(0)]
        self.payload_mass = 1.15
        self.cog = [0, -0.004, 0.045]
        self.inital_tcp_rotation = np.array([0, 3.14, 0])
        self.tcp_rotation_paleta1 = None
        self.tcp_rotation_paleta2 = None 
        self.rtde_c.setPayload(self.payload_mass, self.cog)

    def reconnect(self, host="192.168.1.102"):
        try:
            try:
                self.rtde_c.disconnect()
            except:
                pass
            try:
                self.gripper.disconnect()
            except:
                pass

            self.rtde_c = rtde_control.RTDEControlInterface(host)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
            self.gripper = robotiq_gripper.RobotiqGripper()
            self.gripper.connect(host, 63352)

            self.rtde_c.setPayload(self.payload_mass, self.cog)

            print("robot successfully reconnected and payload set.")
            return True

        except Exception as e:
            print(f"Reconnect failed {e}")
            return False
   
        

    def homing(self):  
        self.rtde_c.moveJ(self.home_p, self.acc, self.vel)

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
        zl, zd, sl, sd = koti

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
        #generacija mreze palet
        for i in range(a):
            v_left = zl - (zl - sl)*(i/(a-1))
            v_right = zd - (zd - sd)*(i/(a-1))
            for j in range(b):
                poz[i,j] = v_left + (v_right - v_left)*(j/(b - 1))
                poz[i,j][2] = 0.0095
                poz[i,j][3:] = tcp_rot



        return poz

    def generiranje_nakljucne_mreze(self, a, b, paleta):
        """Koda naredi matriko nakljucnih pozicij"""
        grid = paleta

        flat_poz = [grid[i, j].copy() for i in range(a) for j in range(b)]
        random.shuffle(flat_poz)

        mozne_rot = [-90, 0, 90]

        random_mreza = np.zeros_like(grid)

        idx = 0
        for i in range(a):
            for j in range(b):
                p = flat_poz[idx]
                idx += 1

                ang = random.choice(mozne_rot)
                p_rot = self.rotate_pose_local_z(p, ang)
                random_mreza[i, j] = p_rot

        return random_mreza
    
    def shuffling_kosckov(self, safe_z=0.04, work_z=0.0098):
        """"Robot pobere kosckek na paleti 1 in ga postavni na naključno mesto na paleti2"""
        random_paleta2 = self.generiranje_nakljucne_mreze(self.paleta2.shape[0],    
                                                           self.paleta2.shape[1],   
                                                           self.paleta2)

        self.homing()
        print("homing")
        self.gripper.activate()
        print("gripper activated")
        self.gripper.move_and_wait_for_pos(229, speed=180, force=2)

        for i in range(self.paleta1.shape[0]):
            for j in range(self.paleta1.shape[1]):
                pick_p = self.paleta1[i, j].copy()
                place_p = random_paleta2[i, j].copy()

                safe_p = pick_p.copy()
                safe_p[2] = safe_z

                safe_place = place_p.copy()
                safe_place[2] = safe_z

                work_pick = pick_p.copy()
                work_pick[2] = work_z

                work_place = place_p.copy()
                work_place[2] = work_z

                # 1. gre nad koscek
                self.rtde_c.moveL(safe_p, self.acc, self.vel)
                print(safe_p)

                # 2. gre v utor s prijemalom
                self.rtde_c.moveL(work_pick, self.acc, self.vel)
                print(work_pick)
                
                # 3. open gripper
                self.gripper.move_and_wait_for_pos(211, speed=150, force=2)

                #4. dvigne kosck
                self.rtde_c.moveL(safe_p, self.acc, self.vel)

                #5. pozicija na paleti2, gre nad njo
                self.rtde_c.moveL(safe_place, self.acc, self.vel)

                #6. gre dol do prave visine
                self.rtde_c.moveL(work_place, self.acc, self.vel)

                #7. zapre gripper
                self.gripper.move_and_wait_for_pos(229, speed=150, force=2)

                #8. gre nad kosck
                self.rtde_c.moveL(safe_place, self.acc, self.vel)

                #tle in gre jovo na novo
        

        # nazaj na home
        self.homing()
        self.rtde_c.disconnect()
        self.gripper.disconnect()

    def pobiranje_kamera(self, safe_z=0.04, work_z=0.0098, kamera_z=0.01935):
        """Robot gre nad paleto2, nad vsak koscke, zajame sliko, jo pregleda in koscek postavi na pravilno mesto"""

        self.homing()
        print("Homing")
        self.gripper.move_and_wait_for_pos(229, speed=180, force=2)
        print("Gripper closed")
        #zanka za zajem slike in vse ostalo :D
        for i in range(self.paleta1.shape[0]):
            for j in range(self.paleta1.shape[1]):
                paleta1_p = self.paleta1[i,j].copy()
                paleta2_p = self.paleta2[i,j].copy()
                #parametri pozicij - safe, work, kamera --> različne višine
                kamera_p = paleta2_p.copy()
                kamera_p[2] = kamera_z
                kamera_p[1] = kamera_p[0] + self.kamera_y

                pick_p = paleta2_p.copy()
                pick_safe_p = pick_p.copy()
                pick_safe_p[2] = safe_z
                pick_work_p = pick_p.copy()
                pick_work_p[2] = work_z

                place_p = paleta1_p.copy()
                place_safe_p = place_p.copy()
                place_safe_p[2] = safe_z
                place_work_p = place_p.copy()
                place_work_p[2] = work_z

                # Gre na safe pozicijo nad koscek z koordinato kamere --> offset v x,y,z
                self.rtde_c.moveL(kamera_p, self.acc, self.vel)
                # Zajame sliko

                # Obdela sliko in vrne pozicijo v matriki + rotacijo --> iz mreze paleta 1 vzame pravilno lokacijo

                # Iz place



    def move_to_position(self, position):
        position[2] = self.safe_z
        self.rtde_c.moveL(position, self.acc, self.vel)

    def pick_and_place_position(self, position):
        position[2] = self.work_z
        self.rtde_c.moveL(position, self.acc, self.vel)

    def move_to_kamera_position(self, position):
        position[1] = position[1] + self.kamera_y
        position[2] = self.kamera_z
        self.rtde_c.moveL(position, self.acc, self.vel)

    def gripper_open(self):
        self.gripper.move_and_wait_for_pos(211, speed=150, force=2)

    def gripper_close(self):
        self.gripper.move_and_wait_for_pos(229, speed=150, force=2)
        

    def disconnect(self):
        self.rtde_c.disconnect()
        self.gripper.disconnect()