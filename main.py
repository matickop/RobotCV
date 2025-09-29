import time
import math
import numpy as np
import rtde_control
import rtde_receive
import random
import robotiq_gripper
from scipy.spatial.transform import Rotation as R
import FreeSimpleGUI as sg
from robot_module import MyRobot
from kamera_module import MyCamera
import timeit

cam = MyCamera()

robot = MyRobot("192.168.3.102")  
pobrani_koti = [None]*8  # 4 koti prve palete + 4 druge

layout = [
    [sg.Text("RobotCV GUI", font=("Helvetica", 24), expand_x=True, justification='center')],
    [sg.Text("Robot control:")],
    [sg.Button("Reconnect", s=14)],
    [sg.Button("Home", s=14), sg.Button("Shuffle kosckov", s=14), sg.Button("FreeDrive", s=14), sg.B("Pobiranje koščkov s kamero")],
    [sg.B("Izhodiščna točka palete 1"), sg.B("Izhodiščna točka palete 2")],
    [sg.Text("Camera control:")],
    [sg.Button("Zajem slike", s=14), sg.B("Template matching", s=14), sg.B("Disconnect camera", s=14), sg.B("Connect camera", s=14)],
    [sg.Text("Prva paleta:")],
    [sg.Button(f"Kot {i+1}") for i in range(4)],
    [sg.Text("Druga paleta:")],
    [sg.Button(f"Kot {i+5}") for i in range(4)],
    [sg.Multiline("", size=(70,10), key="-KOTI-", disabled=True)],
    [sg.Button("Generiraj mrežo palete 1", s=18), sg.Button("Shrani paleto 1", s=14), sg.Button("Naloži paleto 1", s=14)],
    [sg.Button("Generiraj mrežo palete 2", s=18), sg.Button("Shrani paleto 2", s=14), sg.Button("Naloži paleto 2", s=14)],
    [sg.B("Generacija random mreže")],
]

window = sg.Window("RobotCV GUI", layout, resizable=True, finalize=True)

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        robot.disconnect()
        break
    
    if event ==  "Home":
        robot.homing()

    if event == "Reconnect":
        robot.reconnect()

    if event == "FreeDrive":
        if not robot.freedrive_active:
            robot.activate_freedrive()
            window[event].update(text="FreeDrive ON", button_color=('white','red'))
        else:
            robot.deactivate_freedrive()
            window[event].update(text="FreeDrive", button_color=('white','green'))

    elif event.startswith("Kot"):
        idx = int(event.split(" ")[1])-1
        tcp = robot.get_actual_tcp_pose()
        pobrani_koti[idx] = tcp.copy()
        # osveži prikaz
        prikaz = ""
        for i, k in enumerate(pobrani_koti):
            if k is not None:
                prikaz += f"Kot {i+1}: {k}\n"
        window["-KOTI-"].update(prikaz)

    elif event == "Generiraj mrežo palete 1":
        if any(k is None for k in pobrani_koti[:4]):
            sg.popup("Najprej poberi vse kote prve palete!")
            continue
                # generiranje mreže za paleto 1
        robot.paleta1 = robot.generiranje_mreze(4, 6, pobrani_koti[:4], "1")
        sg.popup("Mreža palete 1 generirana!")

    elif event == "Generiraj mrežo palete 2":
        if any(k is None for k in pobrani_koti[4:]):
            sg.popup("Najprej poberi vse kote druge palete!")
            continue

        # generiranje mreže za paleto 2
        robot.paleta2 = robot.generiranje_mreze(4, 6, pobrani_koti[4:], "2")
        sg.popup("Mreža palete 2 generirana!")

    elif event == "Shrani paleto 1":
        np.save("mreza_paleta1.npy", robot.paleta1)

    elif event == "Shrani paleto 2":
        np.save("mreza_paleta2.npy", robot.paleta2)

    elif event == "Naloži paleto 1":
        robot.paleta1 = np.load("mreza_paleta1.npy")
        print(robot.paleta1)
        test = np.zeros_like(robot.paleta1)
        for i in range(robot.paleta1.shape[0]):
            for j in range(robot.paleta1.shape[1]):
                test[i,j] = robot.rtde_c.getInverseKinematics(robot.paleta1[i, j])
        print(test)

    elif event == "Naloži paleto 2":
        robot.paleta2 = np.load("mreza_paleta2.npy")
        print(robot.paleta2)

    if event == "Shuffle kosckov":
        robot.shuffling_kosckov()


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
                #
                



    if event == "Zajem slike":
        cam.capture_image()

    if event == "Disconnect camera":
        cam.release()
        print("kamera disconnectana")

    if event == "Connect camera":
        cam.connect()
        print("kamera connectana")

    if event == "Template matching":
        cam.template_match(cam.template_path, show=False)
        print("template matcham")

    if event == "Pobiranje koščkov s kamero":
        robot.homing()
        print("Homing")
        robot.gripper.activate()
        robot.gripper.move_and_wait_for_pos(229, speed=180, force=2)
        print("Gripper closed")
        #kreiram array za pozicije slike v paleti --> array z [0,0]...[3,5]
        flat_map = [[i, j] for i in range(robot.paleta1.shape[0]) for j in range(robot.paleta1.shape[1])]
        #zanka za zajem slike in vse ostalo :D
        for i in range(robot.paleta1.shape[0]):
            for j in range(robot.paleta1.shape[1]):
                paleta1_p = robot.paleta1[i,j].copy()
                paleta2_p = robot.paleta2[i,j].copy()

                # Gre na safe pozicijo nad koscek z koordinato kamere --> offset v x,y,z
                robot.move_to_kamera_position(paleta2_p)

                # Zajame sliko
                cam.capture_image()

                # Obdela sliko in vrne pozicijo v matriki + rotacijo --> iz mreze paleta 1 vzame pravilno lokacijo
                slika, score_match = cam.template_match(cam.template_path, show=False)
                slika = slika.split(".")[0]
                idx, kot = slika.split("_")
                idx = int(idx)
                print(idx)
                kot = int(kot)
                print(kot)

                #gre samo nad tocko kamor bi postavil sliko
                if flat_map[idx] is not None:
                    #ce ni None gre nad rezo slikice drgac pa skipa naprej
                    robot.move_to_position(robot.paleta2[i, j])

                    #tuki pa potem gre na pozicijo kam bi jo postavu
                    row, col = flat_map[idx]
                    robot.move_to_position(robot.paleta1[row, col])
                    #place-a sliko
                    robot.pick_and_place_position(robot.paleta1[row,col])
                    #se vrne nad sliko
                    robot.move_to_position(robot.paleta1[row,col])
                    
                    #ko polozi sliko, se v matriki pozicij namesto indeksov appenda None
                    flat_map[idx] = None
                else:
                    print("Slike ni mogoce postaviti na zapolnjeno mesto")
                    
                
        #homing nazaj
        robot.homing()

    if event == "Generacija random mreže":
        robot.generiranje_nakljucne_mreze(robot.paleta2.shape[0], robot.paleta2.shape[1], robot.paleta2)
        print(robot.generiranje_nakljucne_mreze(robot.paleta2.shape[0], robot.paleta2.shape[1], robot.paleta2))

    if event == "Izhodiščna točka palete 1":
        robot.move_to_position(robot.paleta1[0,0])
        print(robot.paleta1[0,0])
        print(robot.rtde_r.getActualQ())
        print(robot.rtde_c.getInverseKinematics(robot.paleta1[0,0]))

    if event == "Izhodiščna točka palete 2":
        robot.move_to_position(robot.paleta2[0,0])