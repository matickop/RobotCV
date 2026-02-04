    import logging
    import os
    import csv
    from datetime import datetime

    # Ustvari mapo logs, če ne obstaja
    if not os.path.exists('logs'):
        os.makedirs('logs')

    # Ime datoteke glede na datum
    LOG_FILENAME = datetime.now().strftime('logs/robot_log_%Y-%m-%d.log')
    CSV_FILENAME = datetime.now().strftime('logs/motion_data_%Y-%m-%d.csv')

    # --- 1. KONFIGURACIJA GLAVNEGA LOGGERJA (Tekstovni log) ---
    def setup_logger(name):
        """Vrne logger instanco za določen modul."""
        logger = logging.getLogger(name)
        
        # Preprečimo podvajanje handlerjev, če funkcijo kličemo večkrat
        if not logger.handlers:
            logger.setLevel(logging.DEBUG)
            
            # Format: ČAS | NIVO | MODUL | SPOROČILO
            formatter = logging.Formatter('%(asctime)s | %(levelname)-8s | %(name)-10s | %(message)s', datefmt='%H:%M:%S')

            # Handler za datoteko (vse)
            file_handler = logging.FileHandler(LOG_FILENAME, encoding='utf-8')
            file_handler.setFormatter(formatter)
            file_handler.setLevel(logging.DEBUG)

            # Handler za konzolo (samo INFO in naprej, da ne smetimo terminala preveč)
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            console_handler.setLevel(logging.INFO)

            logger.addHandler(file_handler)
            logger.addHandler(console_handler)
        
        return logger

    # --- 2. KONFIGURACIJA CSV LOGGERJA (Podatkovni log za gibe) ---
    class MotionLogger:
        def __init__(self):
            self.filename = CSV_FILENAME
            # Če datoteka še ne obstaja, zapiši glavo
            if not os.path.exists(self.filename):
                with open(self.filename, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Timestamp", "Context", "Target_Q_or_Pose", "Actual_TCP_Pose"])

        def log_move(self, context, target, actual_pose):
            """
            Context: Kje v kodi se je zgodilo (npr. 'Shuffle Pick')
            Target: Kam smo hoteli iti (list)
            Actual_Pose: Kje smo dejansko (list [x,y,z,rx,ry,rz])
            """
            now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            
            # Formatiranje seznamov v string za lepši CSV
            t_str = str([f"{x:.4f}" for x in target]) if target is not None else "None"
            a_str = str([f"{x:.4f}" for x in actual_pose]) if actual_pose is not None else "None"

            with open(self.filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([now, context, t_str, a_str])

    # Globalna instanca za gibe
    motion_log = MotionLogger()