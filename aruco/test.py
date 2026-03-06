import cv2
import numpy as np
import json
import os
from camera_utils import estimate_ArUco_marker_pose

# 1. Φόρτωση παραμέτρων από το JSON
json_file = "camera_params.json"
if not os.path.exists(json_file):
    print(f"Σφάλμα: Το αρχείο {json_file} δεν βρέθηκε!")
    exit()

with open(json_file, "r") as f:
    data = json.load(f)

# Επιλογή παραμέτρων για την κάμερα 4
if "4" not in data["camera_parameters"]:
    print("Σφάλμα: Δεν υπάρχουν δεδομένα για την κάμερα '4' στο JSON!")
    exit()

params = data["camera_parameters"]["4"]
intrinsic_mat = np.array(params["camera_matrix"])
dist_coeffs = np.array(params["dist_coeffs"])

# 2. Άνοιγμα εξωτερικής κάμερας (Index 1)
cap = cv2.VideoCapture(1)

print("--- ΕΛΕΓΧΟΣ ΑΚΡΙΒΕΙΑΣ ---")
print("Πίεσε 'q' για έξοδο.")
print("Οι αποστάσεις εμφανίζονται σε εκατοστά (cm).")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Αδυναμία λήψης εικόνας.")
        break

    # 3. Ανίχνευση Marker (Ορίζουμε 30mm για να υπολογιστεί σωστά το Z)
    # Χρησιμοποιούμε το κλειδί "4X4_50/30mm" που αντιστοιχεί στο λεξικό σου
    transform, processed_frame = estimate_ArUco_marker_pose(
        frame, "4X4_50/30mm", intrinsic_mat, dist_coeffs
    )

    # 4. Υπολογισμός και εμφάνιση συντεταγμένων
    # Ελέγχουμε αν ο πίνακας μετασχηματισμού δεν είναι ο μοναδιαίος (δηλαδή αν βρέθηκε marker)
    if not np.array_equal(transform, np.eye(4)):
        # Η 4η στήλη του transform matrix περιέχει τα X, Y, Z σε μέτρα
        x_cm = transform[0, 3] * 100
        y_cm = transform[1, 3] * 100
        z_cm = transform[2, 3] * 100
        
        # Εκτύπωση στο τερματικό για έλεγχο με μεζούρα
        print(f"Position -> X: {x_cm:6.2f} | Y: {y_cm:6.2f} | Z (Distance): {z_cm:6.2f} cm", end='\r')

    # Εμφάνιση του frame με τους άξονες
    cv2.imshow("Logitech C310 - ArUco Tracking", processed_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\nΗ διαδικασία ολοκληρώθηκε.")
