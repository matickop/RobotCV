import time
import os
import numpy as np
import threading
import customtkinter as ctk
from tkinter import messagebox
import tkinter as tk
import cv2
from PIL import Image, ImageTk, ImageEnhance
from pathlib import Path
import main

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

BG    = "#002855"
DARK  = "#0a1929"
BORDER= "#1a3a5c"

ROWS, COLS = 4, 6
N_PIECES   = ROWS * COLS  # 24
PIECE_PX   = 56


class RobotTouchGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.overrideredirect(True)
        self.geometry("1024x600+0+0")
        self.bind("<Escape>", lambda e: self.on_exit())
        self.configure(fg_color=BG)

        self.btn_h    = 60
        self.font_btn = ("Arial", 16, "bold")
        self.vtipkani_pin = ""

        self._btns_need_robot = []
        self._btns_need_cam   = []
        self._btns_need_both  = []

        self._piece_state  = [0] * N_PIECES
        self._pulse_active = [False] * N_PIECES
        self._pulse_on     = [False] * N_PIECES

        self._piece_imgs       = {}
        self._piece_imgs_grey  = {}
        self._piece_canvas_ids = {}
        self._piece_labels     = {}

        self._load_piece_images()

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, minsize=80)
        self.grid_rowconfigure(0, minsize=60)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, minsize=80)

        # Header
        header = tk.Frame(self, bg=BG, highlightthickness=0, bd=0)
        header.grid(row=0, column=0, columnspan=2, sticky="nsew")
        tk.Label(header, text="KOLEKTOR", font=("Arial", 22, "bold"),
            fg="white", bg=BG).pack(side="left", padx=30, pady=10)
        self.lbl_naslov = tk.Label(header, text="Functions",
            font=("Arial", 22, "bold"), fg="#7eaad4", bg=BG)
        self.lbl_naslov.pack(side="left", padx=20, pady=10)

        # Kontejner
        self.container = tk.Frame(self, bg=BG, highlightthickness=0, bd=0)
        self.container.grid(row=1, column=0, sticky="nsew", padx=20, pady=10)
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for name in ("Functions", "Vision", "Connections", "Settings"):
            f = tk.Frame(self.container, bg=BG, highlightthickness=0, bd=0)
            f.grid(row=0, column=0, sticky="nsew")
            self.frames[name] = f

        self.frame_functions   = self.frames["Functions"]
        self.frame_vision      = self.frames["Vision"]
        self.frame_connections = self.frames["Connections"]
        self.frame_settings    = self.frames["Settings"]

        # Nav
        nav_frame = tk.Frame(self, bg=BG, width=80, highlightthickness=0, bd=0)
        nav_frame.grid(row=1, column=1, sticky="ns", padx=(0, 10))
        nav_center = tk.Frame(nav_frame, bg=BG, highlightthickness=0, bd=0)
        nav_center.pack(expand=True)
        for i, tab in enumerate(["Functions", "Vision", "Connections", "Settings"], 1):
            ctk.CTkButton(nav_center, text=str(i), width=50, height=50,
                font=("Arial", 20, "bold"),
                command=lambda t=tab: self.prikazi_zavihek(t)).pack(pady=15)

        # Bottom
        bottom = tk.Frame(self, bg=BG, highlightthickness=0, bd=0)
        bottom.grid(row=2, column=0, columnspan=2, sticky="ew", padx=20, pady=10)
        bottom.grid_columnconfigure(0, weight=1)
        bottom.grid_columnconfigure(1, weight=1)
        bottom.grid_columnconfigure(2, weight=1)

        ctk.CTkButton(bottom, text="PREKINITEV", fg_color="#d32f2f", hover_color="#9a0007",
            height=50, width=150, font=self.font_btn,
            command=self.prekinitev_programa).grid(row=0, column=0, sticky="w")

        self.btn_homing = ctk.CTkButton(bottom, text="HOMING", fg_color="#f57c00",
            hover_color="#e65100", height=50, width=150, font=self.font_btn,
            command=lambda: self.run_thread(main.robot.homing))
        self.btn_homing.grid(row=0, column=1)
        self._btns_need_robot.append(self.btn_homing)

        ctk.CTkButton(bottom, text="EXIT", fg_color="#37474f", hover_color="#263238",
            height=50, width=150, font=self.font_btn,
            command=self.on_exit).grid(row=0, column=2, sticky="e")

        self.setup_functions_tab()
        self.setup_vision_tab()
        self.setup_connections_tab()
        self.setup_settings_tab()

        self.prikazi_zavihek("Functions")
        self._posodobi_gumbe()

        main.gui = self

    # ═══════════════════════════════════════════════════════════
    # POPUP DIALOG (nadomestek za messagebox — deluje z overrideredirect)
    # ═══════════════════════════════════════════════════════════
    def _popup(self, naslov: str, sporocilo: str, barva_roba: str = BORDER):
        """Univerzalni modal popup ki deluje tudi z overrideredirect(True)."""
        popup = tk.Toplevel(self)
        popup.overrideredirect(True)
        popup.configure(bg=barva_roba)

        # Centriraj na glavno okno PRED grab_set — sicer "window not viewable"
        self.update_idletasks()
        x = self.winfo_x() + self.winfo_width()  // 2 - 210
        y = self.winfo_y() + self.winfo_height() // 2 - 90
        popup.geometry(f"420x180+{x}+{y}")

        inner = tk.Frame(popup, bg=DARK, padx=20, pady=16)
        inner.pack(fill="both", expand=True, padx=1, pady=1)

        tk.Label(inner, text=naslov, font=("Arial", 13, "bold"),
            fg="#7eaad4", bg=DARK).pack(anchor="w", pady=(0, 6))
        tk.Label(inner, text=sporocilo, font=("Arial", 11),
            fg="white", bg=DARK, wraplength=380, justify="left").pack(anchor="w")

        ctk.CTkButton(inner, text="OK", width=100, height=34,
            command=popup.destroy).pack(pady=(14, 0))

        # Zdaj ko je okno nastavljeno, ga pokazi in nastavi grab
        popup.update_idletasks()
        popup.lift()
        popup.focus_force()
        try:
            popup.grab_set()
        except Exception:
            pass  # ce grab ne dela (redko), popup se vseeno pokaze

    def _popup_napaka(self, naslov: str, sporocilo: str):
        self._popup(naslov, sporocilo, barva_roba="#d32f2f")

    def _popup_opozorilo(self, naslov: str, sporocilo: str):
        self._popup(naslov, sporocilo, barva_roba="#f57c00")

    # ═══════════════════════════════════════════════════════════
    # SLIKE KOŠČKOV
    # ═══════════════════════════════════════════════════════════
    def _load_piece_images(self):
        base     = Path(__file__).resolve().parent
        col_dir  = base / "koscki"
        grey_dir = base / "koscki_grey"

        for idx in range(N_PIECES):
            cp = col_dir  / f"{idx:02d}.png"
            gp = grey_dir / f"{idx:02d}.png"
            try:
                pil_c = Image.open(cp).resize((PIECE_PX, PIECE_PX))
                self._piece_imgs[idx] = ImageTk.PhotoImage(pil_c)
            except Exception:
                self._piece_imgs[idx] = None
            try:
                pil_g = Image.open(gp).resize((PIECE_PX, PIECE_PX))
                self._piece_imgs_grey[idx] = ImageTk.PhotoImage(pil_g)
            except Exception:
                self._piece_imgs_grey[idx] = None

    # ═══════════════════════════════════════════════════════════
    # STANJE GUMBOV
    # ═══════════════════════════════════════════════════════════
    def _posodobi_gumbe(self):
        r = getattr(main.robot, "is_connected", False)
        c = getattr(main.cam,   "is_connected", False)
        for btn in self._btns_need_robot:
            btn.configure(state="normal" if r else "disabled")
        for btn in self._btns_need_cam:
            btn.configure(state="normal" if c else "disabled")
        for btn in self._btns_need_both:
            btn.configure(state="normal" if (r and c) else "disabled")

    # ═══════════════════════════════════════════════════════════
    # PREKLAPLJANJE ZAVIHKOV
    # ═══════════════════════════════════════════════════════════
    def prikazi_zavihek(self, ime):
        for frame in self.frames.values():
            frame.grid_remove()
        self.frames[ime].grid(row=0, column=0, sticky="nsew")
        self.lbl_naslov.configure(text=ime)

    # ═══════════════════════════════════════════════════════════
    # TAB 1: FUNCTIONS
    # ═══════════════════════════════════════════════════════════
    def setup_functions_tab(self):
        vbox = tk.Frame(self.frame_functions, bg=BG, highlightthickness=0, bd=0)
        vbox.pack(side="left", fill="y", padx=20, pady=60)

        btn_program = ctk.CTkButton(vbox, text="Program", fg_color="#1565c0",
            hover_color="#0d47a1", height=self.btn_h, width=250, font=self.font_btn,
            command=self.start_avtomatski_cikel)
        btn_program.pack(pady=10, anchor="w")
        self._btns_need_both.append(btn_program)

        btn_pobiranje = ctk.CTkButton(vbox, text="Pobiranje",
            height=self.btn_h, width=250, font=self.font_btn,
            command=lambda: self.run_and_switch(main.pobiranje_s_kamero,
                "Poteka pobiranje s pomočjo kamere..."))
        btn_pobiranje.pack(pady=10, anchor="w")
        self._btns_need_both.append(btn_pobiranje)

        btn_shuffle = ctk.CTkButton(vbox, text="Shuffle",
            height=self.btn_h, width=250, font=self.font_btn,
            command=lambda: self.run_and_switch(main.shuffling_kosckov,
                "Mešanje koščkov sestavljanke na paleti..."))
        btn_shuffle.pack(pady=10, anchor="w")
        self._btns_need_robot.append(btn_shuffle)

        btn_detection = ctk.CTkButton(vbox, text="Detection", fg_color="#37474f",
            height=self.btn_h, width=250, font=self.font_btn,
            command=self._start_detekcija)
        btn_detection.pack(pady=10, anchor="w")
        self._btns_need_both.append(btn_detection)

    def _start_detekcija(self):
        """Reset puzzle grid in začni detekcijo."""
        self.reset_puzzle()
        self.run_and_switch(main.prepoznava_slik, "Prepoznavanje koščkov...")

    # ═══════════════════════════════════════════════════════════
    # TAB 2: VISION
    # ═══════════════════════════════════════════════════════════
    def setup_vision_tab(self):
        self.frame_vision.grid_columnconfigure(0, weight=5)
        self.frame_vision.grid_columnconfigure(1, weight=7)
        self.frame_vision.grid_rowconfigure(1, weight=1)

        self.lbl_status = tk.Label(self.frame_vision,
            text="Sistem v pripravljenosti...",
            font=("Arial", 13, "bold"), fg="#7eaad4", bg=BG)
        self.lbl_status.grid(row=0, column=0, columnspan=2,
            sticky="w", padx=12, pady=(8, 2))

        # LEVO: zajeta slika
        left = tk.Frame(self.frame_vision, bg=BG, highlightthickness=0, bd=0)
        left.grid(row=1, column=0, sticky="nsew", padx=(8, 4), pady=8)
        left.grid_rowconfigure(1, weight=1)
        left.grid_columnconfigure(0, weight=1)

        tk.Label(left, text="ZAJETA SLIKA", font=("Arial", 10, "bold"),
            fg="#7eaad4", bg=BG).grid(row=0, column=0, sticky="w", pady=(0, 3))

        self.lbl_image = ctk.CTkLabel(left,
            text="[ Čakanje na zajem slike ]",
            font=("Arial", 12), fg_color=DARK,
            width=320, height=380, corner_radius=6)
        self.lbl_image.grid(row=1, column=0, sticky="nsew")

        # DESNO: puzzle grid + match info
        right = tk.Frame(self.frame_vision, bg=BG, highlightthickness=0, bd=0)
        right.grid(row=1, column=1, sticky="nsew", padx=(4, 8), pady=8)
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)

        # Puzzle grid
        puzzle_wrap = tk.Frame(right, bg=BG, highlightthickness=0, bd=0)
        puzzle_wrap.grid(row=0, column=0, sticky="n", pady=(0, 8))

        tk.Label(puzzle_wrap, text="SESTAVLJANKA", font=("Arial", 10, "bold"),
            fg="#7eaad4", bg=BG).pack(anchor="w", pady=(0, 4))

        grid_w = COLS * PIECE_PX + (COLS - 1) * 2
        grid_h = ROWS * PIECE_PX + (ROWS - 1) * 2

        self._puzzle_canvas = tk.Canvas(puzzle_wrap,
            width=grid_w, height=grid_h,
            bg=DARK, highlightthickness=1, highlightbackground=BORDER)
        self._puzzle_canvas.pack()

        for idx in range(N_PIECES):
            r = idx // COLS
            c = idx % COLS
            x = c * (PIECE_PX + 2)
            y = r * (PIECE_PX + 2)
            img = self._piece_imgs_grey.get(idx)
            if img:
                cid = self._puzzle_canvas.create_image(x, y, anchor="nw", image=img)
            else:
                cid = self._puzzle_canvas.create_rectangle(
                    x, y, x + PIECE_PX, y + PIECE_PX,
                    fill="#1a2a3a", outline="#1a3a5c")
            self._piece_canvas_ids[idx] = cid

        self._lbl_counter = tk.Label(puzzle_wrap, text="0 / 24 koščkov",
            font=("Arial", 10, "bold"), fg="#7eaad4", bg=BG)
        self._lbl_counter.pack(anchor="e", pady=(4, 0))

        # TOP 3 (samo to — brez referenčne slike in brez Slika/Score boxa)
        top3_wrap = tk.Frame(right, bg=BG, highlightthickness=0, bd=0)
        top3_wrap.grid(row=1, column=0, sticky="new")
        top3_wrap.grid_columnconfigure(0, weight=1)

        tk.Label(top3_wrap, text="TOP 3 UJEMANJA", font=("Arial", 10, "bold"),
            fg="#7eaad4", bg=BG).grid(row=0, column=0, sticky="w", pady=(4, 4))

        top3_f = tk.Frame(top3_wrap, bg=BG, highlightthickness=0, bd=0)
        top3_f.grid(row=1, column=0, sticky="ew")
        top3_f.grid_columnconfigure(1, weight=1)

        self._top3_rows = []
        fill_colors = ["#1565c0", "#1976d2", "#1e88e5"]
        for i in range(3):
            row_f = tk.Frame(top3_f, bg=BG, highlightthickness=0, bd=0)
            row_f.grid(row=i, column=0, columnspan=3, sticky="ew", pady=2)
            row_f.grid_columnconfigure(1, weight=1)

            tk.Label(row_f, text=f"{i+1}.", width=2,
                font=("Arial", 9, "bold"), fg="#7eaad4", bg=BG, anchor="e"
            ).grid(row=0, column=0, padx=(0, 4))

            bar_bg = tk.Frame(row_f, bg=DARK, height=18,
                highlightthickness=1, highlightbackground=BORDER)
            bar_bg.grid(row=0, column=1, sticky="ew")
            bar_bg.grid_propagate(False)

            bar_fill = tk.Frame(bar_bg, bg=fill_colors[i], height=18)
            bar_fill.place(x=0, y=0, relheight=1, relwidth=0)

            lbl_name = tk.Label(bar_bg, text="—", font=("Arial", 8),
                fg="white", bg=DARK, anchor="w")
            lbl_name.place(x=4, y=0, relheight=1)

            lbl_score = tk.Label(row_f, text="", width=7,
                font=("Arial", 8), fg="#aaaaaa", bg=BG, anchor="w")
            lbl_score.grid(row=0, column=2, padx=(4, 0))

            self._top3_rows.append((bar_fill, lbl_name, lbl_score, bar_bg))

    # ═══════════════════════════════════════════════════════════
    # TAB 3: CONNECTIONS
    # ═══════════════════════════════════════════════════════════
    def setup_connections_tab(self):
        conn_box = tk.Frame(self.frame_connections, bg=BG, highlightthickness=0, bd=0)
        conn_box.pack(anchor="w", padx=30, pady=30, fill="x")

        row_robot = tk.Frame(conn_box, bg=BG, highlightthickness=0, bd=0)
        row_robot.pack(fill="x", pady=15)
        ctk.CTkButton(row_robot, text="CONNECT ROBOT", height=50, width=200,
            font=self.font_btn, fg_color="#607d8b", hover_color="#455a64",
            command=self.connect_robot).pack(side="left")
        self.lbl_status_robot = tk.Label(row_robot, text="Ni povezano",
            font=("Arial", 18, "bold"), fg="gray", bg=BG)
        self.lbl_status_robot.pack(side="left", padx=20)

        row_cam = tk.Frame(conn_box, bg=BG, highlightthickness=0, bd=0)
        row_cam.pack(fill="x", pady=15)
        ctk.CTkButton(row_cam, text="CONNECT KAMERA", height=50, width=200,
            font=self.font_btn, fg_color="#607d8b", hover_color="#455a64",
            command=self.connect_kamera).pack(side="left")
        self.lbl_status_cam = tk.Label(row_cam, text="Ni povezano",
            font=("Arial", 18, "bold"), fg="gray", bg=BG)
        self.lbl_status_cam.pack(side="left", padx=20)

    # ═══════════════════════════════════════════════════════════
    # TAB 4: SETTINGS
    # ═══════════════════════════════════════════════════════════
    def setup_settings_tab(self):
        self.lock_frame = tk.Frame(self.frame_settings, bg=BG,
            highlightthickness=0, bd=0)
        self.lock_frame.pack(expand=True, fill="both")

        frame_prikaz = tk.Frame(self.lock_frame, bg=BG, highlightthickness=0, bd=0)
        frame_prikaz.pack(pady=(10, 5), anchor="center")

        tk.Label(frame_prikaz, text="ZAŠČITENO OBMOČJE - Vnesite Admin PIN",
            font=("Arial", 16, "bold"), fg="white", bg=BG).pack(pady=5)

        self.lbl_pin_display = ctk.CTkLabel(frame_prikaz, text="",
            font=("Arial", 32, "bold"), fg_color="black", text_color="white",
            width=240, height=50, corner_radius=5)
        self.lbl_pin_display.pack(pady=5)

        frame_num_pad = tk.Frame(self.lock_frame, width=340, height=300,
            bg=BG, highlightthickness=0, bd=0)
        frame_num_pad.pack(pady=(5, 10), anchor="center")
        frame_num_pad.pack_propagate(False)

        for r in range(4):
            frame_num_pad.grid_rowconfigure(r, weight=1)
        for c in range(3):
            frame_num_pad.grid_columnconfigure(c, weight=1)

        gumbi = ["7","8","9","4","5","6","1","2","3","C","0","OK"]
        for idx, tekst in enumerate(gumbi):
            r, c = divmod(idx, 3)
            if tekst == "C":
                b_color, h_color = "#9e9e9e", "#757575"
            elif tekst == "OK":
                b_color, h_color = "#2e7d32", "#1b5e20"
            else:
                b_color, h_color = ctk.ThemeManager.theme["CTkButton"]["fg_color"]
            ctk.CTkButton(frame_num_pad, text=tekst, font=("Arial", 24, "bold"),
                fg_color=b_color, hover_color=h_color,
                command=lambda t=tekst: self.pin_pad_pritisk(t)
            ).grid(row=r, column=c, padx=5, pady=5, sticky="nsew")

        self.admin_content = tk.Frame(self.frame_settings, bg=BG,
            highlightthickness=0, bd=0)
        self.admin_content.grid_columnconfigure((0, 1), weight=1)

        # Stolpec 0: Umerjanje palet
        frame_palete = tk.Frame(self.admin_content, bg=BG,
            highlightthickness=0, bd=0)
        frame_palete.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        tk.Label(frame_palete, text="Umerjanje Palet",
            font=("Arial", 14, "bold"), fg="white", bg=BG).pack(pady=(0, 6))

        tk.Label(frame_palete, text="Paleta 1 — koti",
            font=("Arial", 10), fg="#7eaad4", bg=BG).pack(anchor="w", padx=4)
        grid_p1 = tk.Frame(frame_palete, bg=BG, highlightthickness=0, bd=0)
        grid_p1.pack(fill="x", pady=(2, 6), padx=4)
        for i in range(4):
            btn = ctk.CTkButton(grid_p1, text=f"K{i+1}", width=55, height=36,
                command=lambda i=i: self.poberi_kot(i))
            btn.pack(side="left", padx=3, expand=True)
            self._btns_need_robot.append(btn)

        btn_mreza1 = ctk.CTkButton(frame_palete, text="Generiraj mrežo — Paleta 1",
            height=38, font=("Arial", 11, "bold"), command=self.gen_mreza_p1)
        btn_mreza1.pack(pady=3, fill="x", padx=4)
        self._btns_need_robot.append(btn_mreza1)

        tk.Label(frame_palete, text="Paleta 2 — koti",
            font=("Arial", 10), fg="#7eaad4", bg=BG).pack(anchor="w", padx=4, pady=(10, 0))
        grid_p2 = tk.Frame(frame_palete, bg=BG, highlightthickness=0, bd=0)
        grid_p2.pack(fill="x", pady=(2, 6), padx=4)
        for i in range(4):
            btn = ctk.CTkButton(grid_p2, text=f"K{i+5}", width=55, height=36,
                command=lambda i=i: self.poberi_kot(i + 4))
            btn.pack(side="left", padx=3, expand=True)
            self._btns_need_robot.append(btn)

        btn_mreza2 = ctk.CTkButton(frame_palete, text="Generiraj mrežo — Paleta 2",
            height=38, font=("Arial", 11, "bold"), command=self.gen_mreza_p2)
        btn_mreza2.pack(pady=3, fill="x", padx=4)
        self._btns_need_robot.append(btn_mreza2)

        load_row = tk.Frame(frame_palete, bg=BG, highlightthickness=0, bd=0)
        load_row.pack(fill="x", padx=4, pady=(8, 0))
        ctk.CTkButton(load_row, text="Naloži P1", height=32, width=100,
            fg_color="#37474f", command=lambda: self._nalozi_paleto(1)
        ).pack(side="left", padx=3, expand=True)
        ctk.CTkButton(load_row, text="Naloži P2", height=32, width=100,
            fg_color="#37474f", command=lambda: self._nalozi_paleto(2)
        ).pack(side="left", padx=3, expand=True)

        # Stolpec 1: Ročna kontrola
        frame_kontrola = tk.Frame(self.admin_content, bg=BG,
            highlightthickness=0, bd=0)
        frame_kontrola.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        tk.Label(frame_kontrola, text="Ročna kontrola",
            font=("Arial", 14, "bold"), fg="white", bg=BG).pack(pady=(0, 10))

        self.btn_freedrive = ctk.CTkButton(frame_kontrola, text="Vklopi FreeDrive",
            fg_color="#2e7d32", hover_color="#1b5e20", height=44,
            font=("Arial", 13, "bold"), command=self.toggle_freedrive)
        self.btn_freedrive.pack(pady=6, fill="x", padx=10)
        self._btns_need_robot.append(self.btn_freedrive)

        btn_gripper = ctk.CTkButton(frame_kontrola, text="Aktiviraj Gripper",
            height=44, font=("Arial", 13, "bold"), command=self.activate_gripper)
        btn_gripper.pack(pady=6, fill="x", padx=10)
        self._btns_need_robot.append(btn_gripper)

        btn_capture = ctk.CTkButton(frame_kontrola, text="Capture slika",
            height=44, fg_color="#1565c0", font=("Arial", 13, "bold"),
            command=self.capture_sliko)
        btn_capture.pack(pady=6, fill="x", padx=10)
        self._btns_need_cam.append(btn_capture)

        ctk.CTkButton(frame_kontrola, text="Reset Puzzle Prikaz",
            height=36, fg_color="#455a64", font=("Arial", 11),
            command=self.reset_puzzle).pack(pady=(16, 6), fill="x", padx=10)

        ctk.CTkButton(self.admin_content, text="Ponovno zakleni",
            fg_color="#d32f2f", command=self.ponovno_zakleni
        ).grid(row=1, column=0, columnspan=2, pady=10)

    # ═══════════════════════════════════════════════════════════
    # PUZZLE METODE
    # ═══════════════════════════════════════════════════════════
    def set_piece_detecting(self, idx: int):
        if not (0 <= idx < N_PIECES):
            return
        self._piece_state[idx] = 1
        self._pulse_active[idx] = True
        self._pulse_on[idx] = True
        self.after(0, lambda: self._pulse_tick(idx))

    def set_piece_placed(self, idx: int):
        if not (0 <= idx < N_PIECES):
            return
        self._piece_state[idx] = 2
        self._pulse_active[idx] = False
        self.after(0, lambda: self._draw_piece(idx, colored=True))
        self.after(0, self._update_counter)

    def reset_puzzle(self):
        for idx in range(N_PIECES):
            self._piece_state[idx] = 0
            self._pulse_active[idx] = False
        self.after(0, self._redraw_all_pieces)
        self.after(0, self._update_counter)

    def _pulse_tick(self, idx):
        if not self._pulse_active[idx]:
            return
        self._pulse_on[idx] = not self._pulse_on[idx]
        self._draw_piece(idx, colored=self._pulse_on[idx])
        self.after(400, lambda: self._pulse_tick(idx))

    def _draw_piece(self, idx: int, colored: bool):
        img = self._piece_imgs[idx] if colored else self._piece_imgs_grey[idx]
        if img is None:
            return
        cid = self._piece_canvas_ids.get(idx)
        if cid is not None:
            self._puzzle_canvas.itemconfig(cid, image=img)

    def _redraw_all_pieces(self):
        for idx in range(N_PIECES):
            self._draw_piece(idx, colored=(self._piece_state[idx] == 2))

    def _update_counter(self):
        placed = sum(1 for s in self._piece_state if s == 2)
        self._lbl_counter.configure(text=f"{placed} / {N_PIECES} koščkov")

    # ═══════════════════════════════════════════════════════════
    # UPDATE METODE
    # ═══════════════════════════════════════════════════════════
    def update_image(self, pil_image):
        def do():
            ctk_img = ctk.CTkImage(
                light_image=pil_image, dark_image=pil_image, size=(320, 380))
            self.lbl_image.configure(image=ctk_img, text="")
        self.after(0, do)

    def update_match_panel(self, best_name: str, best_score: float, top3: list):
        """Posodobi samo TOP 3 prikaz pod sestavljanko."""
        def do():
            scores = [s for _, s in top3] if top3 else [1]
            worst  = max(scores) or 1
            for i, (bar_fill, lbl_name, lbl_score, _) in enumerate(self._top3_rows):
                if i < len(top3):
                    name, score = top3[i]
                    rel = max(0.05, min(1.0, 1.0 - score / worst))
                    bar_fill.place(relwidth=rel)
                    lbl_name.configure(text=name)
                    lbl_score.configure(text=f"{score:.4f}")
                else:
                    bar_fill.place(relwidth=0)
                    lbl_name.configure(text="—")
                    lbl_score.configure(text="")
        self.after(0, do)

    # ═══════════════════════════════════════════════════════════
    # LOGIKA
    # ═══════════════════════════════════════════════════════════
    def prekinitev_programa(self):
        self._popup_opozorilo("Prekinitev", "Klicana funkcija za prekinitev programa!")

    def pin_pad_pritisk(self, tipka):
        self.lbl_pin_display.configure(text_color="white")
        if tipka == "C":
            self.vtipkani_pin = ""
            self.lbl_pin_display.configure(text="")
        elif tipka == "OK":
            if self.vtipkani_pin == "1234":
                self.lock_frame.pack_forget()
                self.admin_content.pack(expand=True, fill="both")
                self.vtipkani_pin = ""
                self.lbl_pin_display.configure(text="")
            else:
                self.vtipkani_pin = ""
                self.lbl_pin_display.configure(text="NAPAČEN PIN",
                    text_color="#d32f2f")
                self.after(2000, lambda: self.lbl_pin_display.configure(
                    text="", text_color="white")
                    if self.lbl_pin_display.cget("text") == "NAPAČEN PIN"
                    else None)
        else:
            if self.lbl_pin_display.cget("text") == "NAPAČEN PIN":
                self.lbl_pin_display.configure(text="", text_color="white")
            if len(self.vtipkani_pin) < 6:
                self.vtipkani_pin += tipka
            self.lbl_pin_display.configure(text="*" * len(self.vtipkani_pin))

    def ponovno_zakleni(self):
        self.admin_content.pack_forget()
        self.lbl_pin_display.configure(text="", text_color="white")
        self.lock_frame.pack(expand=True, fill="both")

    def run_thread(self, target_func):
        threading.Thread(target=target_func, daemon=True).start()

    def run_and_switch(self, target_func, text_status="Proces v teku..."):
        self.prikazi_zavihek("Vision")
        self.lbl_status.configure(text=text_status)
        self.run_thread(target_func)

    def start_avtomatski_cikel(self):
        self.reset_puzzle()
        self.run_and_switch(main.celoten_loop,
            "Avtomatski cikel: Robot avtonomno razvršča koščke...")

    def activate_gripper(self):
        def worker():
            try:
                main.robot.gripper.activate()
                main.robot.gripper_close()
            except Exception as e:
                self.after(0, lambda err=e: self._popup_napaka(
                    "Napaka", f"Gripper error: {err}"))
        self.run_thread(worker)

    def toggle_freedrive(self):
        def worker():
            try:
                if not main.robot.freedrive_active:
                    main.robot.teachMode()
                    self.after(0, lambda: self.btn_freedrive.configure(
                        text="FreeDrive VKLOPLJEN", fg_color="#d32f2f"))
                else:
                    main.robot.endTeachMode()
                    self.after(0, lambda: self.btn_freedrive.configure(
                        text="Vklopi FreeDrive", fg_color="#2e7d32"))
            except Exception as e:
                self.after(0, lambda err=e: self._popup_napaka(
                    "Napaka", f"FreeDrive error: {err}"))
        self.run_thread(worker)

    def poberi_kot(self, idx):
        def worker():
            try:
                tcp = main.robot.get_actual_tcp_pose()
                main.robot.pobrani_koti[idx] = tcp.copy()
                np.save("koti.npy",
                    np.array(main.robot.pobrani_koti, dtype=object))
                self.after(0, lambda: self._popup(
                    "Shranjeno", f"Kot {idx+1} uspešno zabeležen."))
            except Exception as e:
                self.after(0, lambda err=e: self._popup_napaka(
                    "Napaka", f"Napaka pri shranjevanju kota: {err}"))
        self.run_thread(worker)

    def gen_mreza_p1(self):
        def worker():
            try:
                if any(k is None for k in main.robot.pobrani_koti[:4]):
                    self.after(0, lambda: self._popup_opozorilo(
                        "Opozorilo", "Najprej poberi vse 4 kote Palete 1!"))
                    return
                self.after(0, lambda: self.lbl_status.configure(
                    text="Generiranje mreže P1... (traja 1-2 min)"))
                main.robot.pripravi_in_shrani_paleto(
                    "paleta1", main.robot.pobrani_koti[:4], "1")
                self.after(0, lambda: self.lbl_status.configure(
                    text="Sistem v pripravljenosti..."))
                self.after(0, lambda: self._popup(
                    "Uspeh", "Mreža Palete 1 shranjena!"))
            except Exception as e:
                import traceback
                print(traceback.format_exc())
                self.after(0, lambda err=e: self._popup_napaka(
                    "Napaka", f"{err}"))
        self.run_thread(worker)

    def gen_mreza_p2(self):
        def worker():
            try:
                if any(k is None for k in main.robot.pobrani_koti[4:8]):
                    self.after(0, lambda: self._popup_opozorilo(
                        "Opozorilo", "Najprej poberi vse 4 kote Palete 2!"))
                    return
                self.after(0, lambda: self.lbl_status.configure(
                    text="Generiranje mreže P2... (traja 1-2 min)"))
                main.robot.pripravi_in_shrani_paleto(
                    "paleta2", main.robot.pobrani_koti[4:8], "2")
                self.after(0, lambda: self.lbl_status.configure(
                    text="Sistem v pripravljenosti..."))
                self.after(0, lambda: self._popup(
                    "Uspeh", "Mreža Palete 2 shranjena!"))
            except Exception as e:
                import traceback
                print(traceback.format_exc())
                self.after(0, lambda err=e: self._popup_napaka(
                    "Napaka", f"{err}"))
        self.run_thread(worker)

    def _nalozi_paleto(self, n):
        def worker():
            try:
                main.robot._load_paleta(n)
                self.after(0, lambda: self._popup(
                    "Naloženo", f"Paleta {n} naložena iz diska."))
            except Exception as e:
                import traceback
                print(traceback.format_exc())
                self.after(0, lambda err=e: self._popup_napaka(
                    "Napaka", f"Napaka pri nalaganju: {err}"))
        self.run_thread(worker)

    def connect_robot(self):
        def worker():
            try:
                ok = main.robot.connect()
                if ok:
                    self.after(0, lambda: self.lbl_status_robot.configure(
                        fg="#2e7d32", text="Povezano ✅"))
                else:
                    self.after(0, lambda: self.lbl_status_robot.configure(
                        fg="#d32f2f", text="Napaka pri povezavi ❌"))
            except Exception:
                self.after(0, lambda: self.lbl_status_robot.configure(
                    fg="#d32f2f", text="Napaka ❌"))
            self.after(0, self._posodobi_gumbe)
        self.lbl_status_robot.configure(fg="yellow", text="Povezujem...")
        self.run_thread(worker)

    def connect_kamera(self):
        def worker():
            try:
                ok = main.cam.connect()
                if ok:
                    self.after(0, lambda: self.lbl_status_cam.configure(
                        fg="#2e7d32", text="Povezano ✅"))
                else:
                    self.after(0, lambda: self.lbl_status_cam.configure(
                        fg="#d32f2f", text="Napaka pri povezavi ❌"))
            except Exception:
                self.after(0, lambda: self.lbl_status_cam.configure(
                    fg="#d32f2f", text="Napaka ❌"))
            self.after(0, self._posodobi_gumbe)
        self.lbl_status_cam.configure(fg="yellow", text="Povezujem...")
        self.run_thread(worker)

    def capture_sliko(self):
        def worker():
            try:
                save_path = main.cam.capture_image()
                raw_frame = cv2.imread(save_path)
                if raw_frame is None:
                    raise RuntimeError(f"cv2.imread failed: {save_path}")
                rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(rgb)
                self.update_image(pil_img)
                self.after(0, lambda: self.prikazi_zavihek("Vision"))
                self.after(0, lambda: self.lbl_status.configure(
                    text="Zajeta slika prikazana."))
            except Exception as e:
                self.after(0, lambda err=e: self._popup_napaka(
                    "Capture", f"Napaka: {err}"))
        self.run_thread(worker)

    def on_exit(self):
        def worker():
            try:
                main.robot.disconnect()
                main.cam.release()
            except Exception:
                pass
            self.after(0, self.destroy)
        self.run_thread(worker)


if __name__ == "__main__":
    app = RobotTouchGUI()
    app.mainloop()