from concurrent.futures import ThreadPoolExecutor
import os
import cv2
import numpy as np
from pypylon import pylon
from pypylon import genicam
import time
import ids
import logger

log = logger.Logger()


class MyCamera:
    """
    Modul za zajem slik s kamero Basler in izvajanje algoritmov za prepoznavo slik.
        - capture_image: zajame eno sliko in jo shrani
        - template_match: poišče najboljše ujemanje med zajeto sliko in slikami v search_dir
        - template_match_multiscale: optimizirana, hitrejsa verzija template_match-a
    """

    def __init__(self, save_dir: str = None, camera_index: int = 0):
        """
        Inicializira kamero in ustvari mapo za shranjevanje slik.
        """

        self.init_id = ids.new_id("camera_init")
        log.event("camera_init", "INFO", "Initializing camera.", cmd_id=self.init_id)

        self.save_dir = save_dir
        log.event("camera_init", "INFO", f"Uporabljam {self.save_dir} za shranjevanje slik, v primeru da ga ni, kreiram novega.", cmd_id=self.init_id)
        os.makedirs(self.save_dir, exist_ok=True)

        log.event("camera_init", "INFO", "Povezovanje z kamero...", cmd_id=self.init_id)
        tlf = pylon.TlFactory.GetInstance() # inicializacija kamere
        log.event("camera_init", "INFO", "Kreiram instanco cam", cmd_id=self.init_id)
        self.cam = pylon.InstantCamera(tlf.CreateFirstDevice())
        log.event("camera_init", "INFO", "Povezava z kamero vzpostavljena.", cmd_id=self.init_id)
        log.event("camera_init", "INFO", "Nastavljam MaxNumBuffer na 20, da se zmanjša možnost timeoutov pri zajemu slike.", cmd_id=self.init_id)
        self.cam.MaxNumBuffer = 20
        time.sleep(1)
        log.event("camera_init", "INFO", "Zaganjam aktivno povezavo s kamero", cmd_id=self.init_id)
        self.cam.Open()
        log.event

        log.event("camera_init", "INFO", "Nastavitev template_path in search_dir", cmd_id=self.init_id)
        self.template_path = None
        self.search_dir = "zajeta_celotna_slika"

        log.event("camera_init", "INFO", "Prednaložim slike v več resolucijah za hitrejše template matchanje.", cmd_id=self.init_id)
        self.image_cache = {}
        self.preload_images_multiscale()
        log.event("camera_init", "INFO", "Inicializacija kamere zaključena.", cmd_id=self.init_id)

    # ----------------------------------------------------------------------
    def preload_images(self):
        """
        Prednaloži slike v self.image_cache
        """

        log.event("camera_init", "INFO", "Prednaložim slike v self.image_cache.", cmd_id=self.init_id)

        for fname in os.listdir(self.search_dir):
            fpath = os.path.join(self.search_dir, fname)
            img = cv2.imread(fpath)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.image_cache[fname] = gray
        log.event("camera_init", "INFO", f"Prednaloženih {len(self.image_cache)} slik.", cmd_id=self.init_id)

    def preload_images_multiscale(self):
        """
        Naloži slike v več resolucijah
        """

        log.event("camera_init", "INFO", "Naloži slike v več resolucijah.", cmd_id=self.init_id)

        for fname in os.listdir(self.search_dir):
            fpath = os.path.join(self.search_dir, fname)
            img = cv2.imread(fpath)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
            # Kreiraj image pyramid (različne resolucije)
            self.image_cache[fname] = {
                'full': gray,
                'half': cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA),
                'quarter': cv2.resize(gray, None, fx=0.1, fy=0.1, interpolation=cv2.INTER_AREA)
            }

        log.event("camera_init", "INFO", f"Prednaloženih {len(self.image_cache)} slik(multi-scale)", cmd_id=self.init_id)


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

    def template_match(self,
                       method=cv2.TM_SQDIFF_NORMED, show: bool = True):
        """
        Poišče najboljše ujemanje med template_path in vsemi slikami v search_dir.
        Vrne (najboljsa_datoteka, najboljsi_score).
        """
        template = cv2.imread(self.template_path)
        if template is None:
            raise FileNotFoundError(f"Template ne obstaja: {self.template_path}")
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        h, w = template_gray.shape[:2]

        najboljsi_score = 1
        najboljsa_sestavljanka = None
        najboljsa_datoteka = None
        najboljsa_lokacija = None
        full_scores = []
        
        for name, pyr in self.image_cache.items():
            img = pyr["full"]
            if img is None:
                continue
            res = cv2.matchTemplate(img, template_gray, method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            full_scores.append((name, min_val))

            if min_val <  najboljsi_score:
                najboljsi_score = min_val
                najboljsa_sestavljanka = img.copy()
                najboljsa_datoteka = name
                najboljsa_lokacija = min_loc

        if najboljsa_sestavljanka is None:
            print("Ni bilo najdenega ujemanja.")
            return None, None
        # uredimo slovar score-ov od najmansega do najvecjega
        top_full = sorted(full_scores, key=lambda item: item[1])
        
        # Označi najboljše ujemanje
        top_left = najboljsa_lokacija
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(najboljsa_sestavljanka, top_left, bottom_right, (0, 0, 255), 3)

        print(f"Najboljša sestavljanka: {najboljsa_datoteka}, score={najboljsi_score:.3f}")

        if show:
            cv2.imshow("Najboljse ujemanje", najboljsa_sestavljanka)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return najboljsa_datoteka, najboljsi_score, [], top_full[:5]
    

    def template_match_multiscale(self, 
                              top_n: int = 10, 
                              max_workers: int = 8,
                              show: bool = False):
        """
        Multi-scale matching:
        FAZA 1: Matching na 1/5 resoluciji
        FAZA 2: Matching na full resoluciji
        """
        template = cv2.imread(self.template_path)
        if template is None:
            raise FileNotFoundError(f"Template ne obstaja: {self.template_path}")
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        h, w = template_gray.shape[:2]

        template_quarter = cv2.resize(template_gray, None, fx=0.1, fy=0.1, 
                                    interpolation=cv2.INTER_AREA)
        
        coarse_scores = []
        
        for name, img_pyramid in self.image_cache.items():
            img_quarter = img_pyramid['quarter']
            
            # Template matching na nizki resoluciji (HITRO!)
            res = cv2.matchTemplate(img_quarter, template_quarter, cv2.TM_SQDIFF_NORMED)
            min_val, _, _, _ = cv2.minMaxLoc(res)
            
            coarse_scores.append((name, min_val))
        
        # Sortiraj in vzemi top N kandidatov
        coarse_scores.sort(key=lambda x: x[1])
        top_candidates = [name for name, score in coarse_scores[:top_n]]

        najboljsi_score = float('inf')
        najboljsa_datoteka = None
            
        def match_full_scale(name):
            """Match na full resoluciji"""
            img_full = self.image_cache[name]['full']
            res = cv2.matchTemplate(img_full, template_gray, cv2.TM_SQDIFF_NORMED)
            min_val, _, min_loc, _ = cv2.minMaxLoc(res)
            return (name, min_val, min_loc)
        
        # Parallel matching samo na top kandidatih
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            results = list(executor.map(match_full_scale, top_candidates))
        
        final_scores = [(name, score) for name, score, _ in results]
        final_scores.sort(key=lambda x: x[1])

        # Najdi najboljši rezultat
        for name, score, loc in results:
            if score < najboljsi_score:
                najboljsi_score = score
                najboljsa_datoteka = name
                
        print(f"Najboljša: {najboljsa_datoteka}, score={najboljsi_score:.6f}")
        
        return najboljsa_datoteka, najboljsi_score, coarse_scores[:5], final_scores[:5]

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
        