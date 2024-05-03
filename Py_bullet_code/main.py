import pybullet as p
import time
import math
import numpy as np
from math import *
import keyboard
import simulation_pybullet as simulation
import threading
import serial

DEG2RAD = math.pi/180.0 
RAD2DEG = 180.0/math.pi

# Configuration du port série
port = 'COM4'
baud_rate = 19200

# Tentative d'ouverture du port série
try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    if ser.is_open:
        print(f"Port ouvert sur {ser.name}")
except Exception as e:
    print(f"Impossible d'ouvrir le port {port}: {e}")
    exit(1)

def send_data(data):
    """Envoyer des données en hexadécimal via le port série."""
    if ser.is_open:
        ser.write(bytearray(data))
        print("Données envoyées:", data)

def receive_data():
    """Recevoir des données du port série et afficher uniquement les trames de 7 octets commençant par 0xADDD."""
    buffer = bytearray()  # Buffer pour accumuler les données
    while True:
        if ser.is_open:
            # Lire les données disponibles
            incoming_data = ser.read(ser.in_waiting or 1)
            if incoming_data:
                buffer += incoming_data

                # Tant qu'il y a suffisamment de données pour potentiellement former une trame
                while len(buffer) >= 7:
                    # Vérifier si les deux premiers octets de la trame correspondent à 0xADDD
                    if buffer[0] == 0xAD and buffer[1] == 0xDD:
                        # Extraire une trame de 7 octets
                        frame = buffer[:7]
                        print(f"Trame reçue: {frame.hex()}")
                        # Supprimer les 7 octets traités du buffer
                        buffer = buffer[7:]
                    else:
                        # Si la trame ne commence pas par 0xADDD, supprimer le premier octet
                        buffer.pop(0)

        threading.Event().wait(1)

def on_closing(root):
    """Gérer la fermeture de l'interface graphique."""
    if messagebox.askokcancel("Quitter", "Voulez-vous quitter l'application ?"):
        if ser.is_open:
            ser.close()
        root.destroy()
        exit(0)

si = simulation.SimulationPyBullet()

def main() :
    while True :
        si.run_simulation()
        dk_joint_angle = si.get_dk_angle()
        if si.verification_apply  :
            if si._state == simulation.State.CHOOSE_MODE:
                data = [0xA8, 0x89, 0x01, 0x00, 0xFA, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                checksum = sum(data[:18]) % 256
                data[18] = checksum
                send_data(data)
            else : 
                J1 = int(si.result_base_joint_angle * RAD2DEG)
                if J1 < 0:
                    speed_J1 = 143
                else : 
                    speed_J1 = 15
                J2 = int(si.result_shoulder_joint_angle * RAD2DEG) 
                if J2 < 0:
                    speed_J2 = 143
                else : 
                    speed_J2 = 15
                J3 = int(si.result_elbow_joint_angle * RAD2DEG) 
                if J3 < 0:
                    speed_J3 = 143
                else : 
                    speed_J3 = 15
                J4 = int(si.result_rot_joint_angle * RAD2DEG) 
                if J4 < 0:
                    speed_J4 = 143
                else : 
                    speed_J4 = 15
                J5 = int(si.result_tilt_joint_angle * RAD2DEG) 
                if J5 < 0:
                    speed_J5 = 143
                else : 
                    speed_J5 = 15
                #J6 = int(si.result_base_joint_angle * RAD2DEG) si nécessaire
                #if J6 < 0:
                #    speed_J6 = 0x8F
                #else : 
                #    speed_J6 = 0x0F

                # Conservez tous les éléments comme entiers pour le calcul du checksum
                data = [0xA8, 0x89, 0x01, 0x00, 0xFD, 0xEF, speed_J1, abs(J1), speed_J2, abs(J2), speed_J3, abs(J3), speed_J4, abs(J4), speed_J5, abs(J5), 0x0F, 0x00, 0x00]
                checksum = sum(data[:18]) % 256
                data[18] = checksum
                send_data(data)
                print("Joint Angles:", dk_joint_angle)
        si.close_simulation()


# Utilisation de la classe SimulationPyBullet
if __name__ == "__main__": 
    receiver_thread = threading.Thread(target=receive_data, daemon=True)
    receiver_thread.start()
    main()


# (-1.7453292519943295, 2.6179938779914944, -2.6179938779914944, -3.141592653589793, 3.490658503988659)




#si.verification_init == True 
#si._state == 'INIT'