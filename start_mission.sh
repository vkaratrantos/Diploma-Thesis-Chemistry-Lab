#!/bin/bash

# 1. Καθαρισμός παλιών συνδέσεων (αν υπάρχουν)
echo "🧹 Καθαρισμός παλιών συνδέσεων..."
killall socat 2>/dev/null
rm /tmp/virtual_robot 2>/dev/null

# 2. Έναρξη του εικονικού καλωδίου
echo "🔗 Σύνδεση με το Ρομπότ (192.168.123.20)..."
# Τρέχει στο παρασκήνιο (&)
socat pty,link=/tmp/virtual_robot,raw tcp:192.168.123.20:8889 &

# Περιμένουμε 2 δευτερόλεπτα να σταθεροποιηθεί η σύνδεση
sleep 2

# 3. Εκτέλεση του Python Script σου
echo "🚀 Ξεκινάει το πρόγραμμα ελέγχου..."
python3 robot_control.py

# 4. Όταν κλείσει το Python script, κλείνουμε και τη σύνδεση
echo "🛑 Αποσύνδεση..."
killall socat
