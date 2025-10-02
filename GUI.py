import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import FreeSimpleGUI as sg
from robot_module import MyRobot
from kamera_module import MyCamera
import test_main
import threading

stop_requested = False
running_thread = None

cam = MyCamera()

robot = MyRobot("192.168.3.102")  
pobrani_koti = [None]*8  # 4 koti prve palete + 4 druge

layout = [
    [sg.Text("RobotCV GUI", font=("Helvetica", 24), expand_x=True, justification='center')],
    [sg.Text("Robot control:")],
    [sg.Button("Reconnect", s=14), sg.B("Activate gripper"), sg.B("STOP",button_color=("white", "red"), s=14)],
    [sg.Button("Home", s=14), sg.Button("Shuffle kosckov", s=14), sg.Button("FreeDrive", s=14), sg.B("Pobiranje koščkov s kamero"), sg.B("Zajem celotne slike")],
    [sg.B("Izhodiščna točka palete 1"), sg.B("Izhodiščna točka palete 2")],
    [sg.Text("Camera control:")],
    [sg.Button("Zajem slike", s=14), sg.B("Template matching", s=14), sg.B("Disconnect camera", s=14), sg.B("Connect camera", s=14)],
    [sg.Text("Prva paleta:")],
    [sg.Button(f"Kot {i+1}") for i in range(4)],
    [sg.Text("Druga paleta:")],
    [sg.Button(f"Kot {i+5}") for i in range(4)],
    [sg.Multiline("", size=(70,10), key="-KOTI-", disabled=True)],
    [sg.Button("Generiraj mrežo palete 1", s=18), sg.Button("Naloži paleto 1", s=14)],
    [sg.Button("Generiraj mrežo palete 2", s=18), sg.Button("Naloži paleto 2", s=14)],
    [sg.B("Generacija random mreže")]
]

window = sg.Window("RobotCV GUI", layout, resizable=True, finalize=True)

while True:
    event, values = window.read()

    if event == sg.WIN_CLOSED:
        robot.disconnect()
        break
    
    if event ==  "Home":
        robot.homing()

    if event == "Activate gripper":
        robot.gripper.activate()

    if event == "Reconnect":
        robot.reconnect()

    if event == "STOP":
        test_main.stop_requested = True

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
        robot.pobrani_koti[idx] = tcp.copy()
        np.save("koti.npy", np.array(robot.pobrani_koti, dtype=object))
        # osveži prikaz
        prikaz = ""
        for i, k in enumerate(robot.pobrani_koti):
            if k is not None:
                prikaz += f"Kot {i+1}: {k}\n"
        window["-KOTI-"].update(prikaz)

    elif event == "Generiraj mrežo palete 1":
        if any(k is None for k in robot.pobrani_koti[:4]):
            sg.popup("Najprej poberi vse kote prve palete!")
            continue
        # generating
        (robot.paleta1_safe,
        robot.paleta1_work,
        robot.paleta1_kam,
        robot.paleta1_safe_joint,
        robot.paleta1_work_joint,
        robot.paleta1_kam_joint) = robot.pripravi_in_shrani_paleto("paleta1", robot.pobrani_koti[:4], "1")
        sg.popup("Mreža palete 1 generirana!")

    elif event == "Generiraj mrežo palete 2":
        if any(k is None for k in robot.pobrani_koti[4:]):
            sg.popup("Najprej poberi vse kote druge palete!")
            continue

        (robot.paleta2_safe,
        robot.paleta2_work,
        robot.paleta2_kam,
        robot.paleta2_safe_joint,
        robot.paleta2_work_joint,
        robot.paleta2_kam_joint) = robot.pripravi_in_shrani_paleto("paleta2", robot.pobrani_koti[4:], "2")
        sg.popup("Mreža palete 2 generirana!")

    elif event == "Naloži palete 1":
        robot._load_paleta(1)
        sg.popup("Paleta 1 naložena")

    elif event == "Naloži palete 2":
        robot._load_paleta(2)
        sg.popup("Paleta 2 naložena!")

    if event == "Shuffle kosckov":
        threading.Thread(target=test_main.shuffling_kosckov, daemon=True).start()

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
        threading.Thread(target=test_main.pobiranje_s_kamero, daemon=True).start()
        
    if event == "Generacija random mreže":
        robot.generiranje_nakljucne_mreze(robot.paleta2.shape[0], robot.paleta2.shape[1], robot.paleta2)
        print(robot.generiranje_nakljucne_mreze(robot.paleta2.shape[0], robot.paleta2.shape[1], robot.paleta2))

    if event == "Izhodiščna točka palete 1":
        robot.move_to_position(robot.paleta1_safe_joint[0,0])

    if event == "Izhodiščna točka palete 2":
        robot.move_to_position(robot.paleta2_safe_joint[0,0])

    if event == "Zajem celotne slike":
        test_main.zajem_celotne_slike()