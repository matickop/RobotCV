import os
import cv2
import numpy as np
from pypylon import pylon
from pypylon import genicam
import time


class MyCamera:
    """
    Modul za zajem slike s pypylon kamero in template matching.
    """

    def __init__(self, save_dir: str = None, camera_index: int = 0):
        """
        save_dir: mapa kamor se shranjujejo zajete slike
        camera_index: (če imaš več kamer lahko spremeniš)
        """
        if save_dir is None:
            save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zajeti")
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)

        # Inicializacija kamere
        tlf = pylon.TlFactory.GetInstance()
        self.cam = pylon.InstantCamera(tlf.CreateFirstDevice())
        self.cam.MaxNumBuffer = 20
        time.sleep(1)
        self.cam.Open()

        self.template_path = None
        self.search_dir = "zajeta_celotna_slika"

        self.image_cache = {}
        self.preload_images()
    # ----------------------------------------------------------------------
    def preload_images(self):
        """Naloži vse slike iz search_dir v RAM enkrat za vselej."""
        for fname in os.listdir(self.search_dir):
            fpath = os.path.join(self.search_dir, fname)
            img = cv2.imread(fpath)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.image_cache[fname] = gray
        print(f"[INFO] Prednaloženih {len(self.image_cache)} slik.")

    def capture_image(self, filename: str = "zajeta_slika.png", timeout_ms: int = 20000) -> str:
        """
        Zajame eno sliko in jo shrani v self.save_dir
        Vrne pot do shranjene slike.
        """
        try:
            iterations = 5
            for i in range(iterations):
                result = self.cam.GrabOne(timeout_ms)
                if result.GrabSucceeded():
                    img = pylon.PylonImage()
                    img.AttachGrabResultBuffer(result)
                    save_path = os.path.join(self.save_dir, filename)
                    img.Save(pylon.ImageFileFormat_Png, save_path)
                    img.Release()
                    result.Release()
                    self.template_path = save_path
                    print(f"[INFO] Slika zajeta in shranjena v: {save_path}")
                    return save_path
                else:
                    print("ITERACIJA")
                    #Timeout
                    time.sleep(1)
        except Exception as e:
            print(f"[ERROR] Camera grab failed: {e}")
            # Optionally: reconnect logic here!
            raise
    # ----------------------------------------------------------------------

    def template_match(self, template_path: str,
                       method=cv2.TM_SQDIFF_NORMED, show: bool = True):
        """
        Poišče najboljše ujemanje med template_path in vsemi slikami v search_dir.
        Vrne (najboljsa_datoteka, najboljsi_score).
        """
        template = cv2.imread(self.template_path)
        if template is None:
            raise FileNotFoundError(f"Template ne obstaja: {template_path}")
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        h, w = template_gray.shape[:2]

        najboljsi_score = 1
        najboljsa_sestavljanka = None
        najboljsa_datoteka = None
        najboljsa_lokacija = None
        slovar_slik = {}
        
        for name, img in self.image_cache.items():
            if img is None:
                continue
            res = cv2.matchTemplate(img, template_gray, method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            slovar_slik[name] = min_val

            if min_val <  najboljsi_score:
                najboljsi_score = min_val
                najboljsa_sestavljanka = img.copy()
                najboljsa_datoteka = name
                najboljsa_lokacija = min_loc

        if najboljsa_sestavljanka is None:
            print("Ni bilo najdenega ujemanja.")
            return None, None
        # uredimo slovar score-ov od najmansega do najvecjega
        urejen_slovar = dict(sorted(slovar_slik.items(), key=lambda item: item[1]))
        
        # Označi najboljše ujemanje
        top_left = najboljsa_lokacija
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(najboljsa_sestavljanka, top_left, bottom_right, (0, 0, 255), 3)

        print(f"Najboljša sestavljanka: {najboljsa_datoteka}, score={najboljsi_score:.3f}")

        if show:
            cv2.imshow("Najboljse ujemanje", najboljsa_sestavljanka)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return najboljsa_datoteka, najboljsi_score, urejen_slovar

    # ----------------------------------------------------------------------

    def release(self):
        """Zapre povezavo s kamero."""
        try:
            self.cam.Close()
        except Exception as e:
            print(f"[ERROR] Prišlo je do napake med prekinjanjem povezave s kamero: {e}")

    def connect(self):
        """Odpre kamero"""
        self.cam.Open()

    def capture_image_celotna(self, filename: str = "zajeta_slika.png", save_dir: str = None, timeout_ms: int = 20000) -> str:
        """
        Zajame eno sliko in jo shrani v self.save_dir
        Vrne pot do shranjene slike.
        """
        if save_dir is None:
            save_dir = "zajeta_celotna_slika"
            os.makedirs(save_dir, exist_ok=True)
        
        self.cam.Open()
        result = self.cam.GrabOne(timeout_ms)
        if result.GrabSucceeded():
            img = pylon.PylonImage()
            img.AttachGrabResultBuffer(result)
            save_path = os.path.join(save_dir, filename)
            img.Save(pylon.ImageFileFormat_Png, save_path)
            img.Release()
            result.Release()
            self.template_path = save_path

            return save_path
        else:
            raise RuntimeError(
                f"Grab failed: {result.ErrorCode} {result.ErrorDescription}"
            )
        