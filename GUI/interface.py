import tkinter as tk
from tkinter import ttk, messagebox
import datetime
import subprocess
import threading
import sys
import os

HOME = os.path.expanduser("~")

# Notice we changed 'ros2 launch' to 'ros2 run', and targeted the executable directly!
ROBOT_CMD = [
    "bash", 
    "-i", 
    "-c", 
    f"cd {HOME}/ws_moveit2 && source install/setup.bash && cd {HOME}/elephant_robots_ws && source install/setup.bash && ros2 run my_robot_control simple_move"
]

# Configuration

REAGENTS = {
    "Tube 1": "Copper Sulfate (CuSO4)",
    "Tube 2": "Sodium Hydroxide (NaOH)",
    "Tube 3": "Sodium Carbonate (Na2CO3)",
    "Tube 4": "Hydrochloric Acid (HCl)",
    "Tube 5": "Ammonia (NH3)"
}

# Updated to store just the Marker IDs required for the sequence
RECIPES = {
    "Copper Hydroxide - Cu(OH)2": [1, 2],
    "Copper Carbonate - CuCO3": [1, 3],
    "Carbon Dioxide - CO2": [3, 4],
    "Ammonium Chloride - NH4Cl": [4, 5],
    "Royal Blue Complex - [Cu(NH3)4]2+": [1, 5],
    "Neutralization": [4, 2]
}

# Colors

COLOR_BG = "#0a0a0a"
COLOR_PANEL = "#141414"
COLOR_BLUE_NEON = "#0066cc"
COLOR_BLUE_DARK = "#004488"
COLOR_TEXT_MAIN = "#ffffff"
COLOR_TEXT_DIM = "#888888"
COLOR_BORDER = "#ffffff"  

class LabApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Automated Chemical Synthesis")
        self.root.geometry("1200x800") 
        self.root.configure(bg=COLOR_BG)

        self.manual_batch = [] 
        self.robot_process = None

        # Start the C++ Subprocess
        self.start_robot_process()

        # Header
        header_frame = tk.Frame(root, bg=COLOR_BG, pady=20)
        header_frame.pack(fill="x")
        
        tk.Label(header_frame, text="CHEMICAL SYNTHESIS", font=("Arial", 36, "bold"), bg=COLOR_BG, fg=COLOR_BLUE_NEON).pack()
        tk.Label(header_frame, text="AUTOMATED FLUID HANDLING SYSTEM", font=("Arial", 16), bg=COLOR_BG, fg=COLOR_TEXT_DIM).pack(pady=(0, 10))
        tk.Frame(root, bg=COLOR_BORDER, height=2).pack(fill="x", padx=40, pady=(0, 20))

        # Tabs
        style = ttk.Style()
        style.theme_use('alt') 
        style.configure("TNotebook", background=COLOR_BG, borderwidth=0)
        style.configure("TNotebook.Tab", background="#222", foreground="#aaa", padding=[10, 15], font=("Arial", 16, "bold"), width=35, anchor="center")
        style.map("TNotebook.Tab", background=[("selected", COLOR_BLUE_NEON)], foreground=[("selected", "white")])

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(expand=False, fill="both", padx=40, pady=10)
        self.tab_auto_container = tk.Frame(self.notebook, bg=COLOR_BORDER) 
        self.tab_manual_container = tk.Frame(self.notebook, bg=COLOR_BORDER) 
        self.tab_auto = tk.Frame(self.tab_auto_container, bg=COLOR_PANEL)
        self.tab_auto.pack(fill="both", expand=True, padx=2, pady=2) 
        self.tab_manual = tk.Frame(self.tab_manual_container, bg=COLOR_PANEL)
        self.tab_manual.pack(fill="both", expand=True, padx=2, pady=2) 
        
        self.notebook.add(self.tab_auto_container, text="AUTO SYNTHESIS")
        self.notebook.add(self.tab_manual_container, text="MANUAL CONTROL")
        
        self.build_auto_tab()
        self.build_manual_tab()

        self.log("System Ready. UI Initialized.")

    # --- ROBOT COMMUNICATION LOGIC ---

    def start_robot_process(self):
        """Launches the C++ file in the background and listens to it."""
        import time # Ensure time is imported
        try:
            self.robot_process = subprocess.Popen(
                ROBOT_CMD,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            # Run a thread to echo the C++ prints to the terminal
            threading.Thread(target=self.monitor_robot_output, daemon=True).start()

            # --- NEW CRASH CATCHER ---
            # Wait half a second to see if the launch command instantly fails
            time.sleep(0.5)
            if self.robot_process.poll() is not None:
                # The process died. Grab the error output.
                out, _ = self.robot_process.communicate()
                print(f"\n[!!!] FATAL: Robot process crashed instantly (Exit Code: {self.robot_process.returncode})")
                print(f"[!!!] BASH ERROR LOG:\n{out}")
                
        except Exception as e:
            print(f"[-] ERROR STARTING ROBOT SCRIPT: {e}")
            print(f"[-] ERROR STARTING ROBOT SCRIPT: {e}")

    def monitor_robot_output(self):
        """Constantly reads the stdout of the C++ script and prints it to your terminal."""
        if self.robot_process:
            for line in self.robot_process.stdout:
                # Print directly to terminal so you see MoveIt planning outputs
                sys.stdout.write(line)
                sys.stdout.flush()

    def send_command(self, cmd):
        """Sends a single string command to the C++ std::cin pipe."""
        if self.robot_process and self.robot_process.poll() is None:
            self.robot_process.stdin.write(cmd + "\n")
            self.robot_process.stdin.flush()
        else:
            print(f"[-] WARNING: Cannot send '{cmd}', robot script is not running.")

    def queue_tube_sequence(self, marker_id):
        """Generates and sends the pick, place, and pour sequence for a specific marker."""
        commands = [
            "o",               # Open Gripper
            f"m {marker_id}",  # Go to the requested tube
            "c",               # Close Gripper
            "m 6",             # Go to Mix Tube
            "p",               # Pour
            f"m {marker_id}",  # Return tube to its original spot
            "o"                # Open Gripper
        ]
        for cmd in commands:
            self.send_command(cmd)

    def log(self, message):
        """Logs high-level Python events to your terminal."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"\n[{timestamp}] [GUI] {message}\n")

    # --- AUTO MODE ---
    
    def build_auto_tab(self):
        container = tk.Frame(self.tab_auto, bg=COLOR_PANEL)
        container.pack(expand=True, fill="both", pady=40)

        for recipe_name in RECIPES:
            btn = tk.Button(container, text=recipe_name, width=50, height=2, 
                            bg="#222", fg="white", font=("Arial", 16),
                            activebackground=COLOR_BLUE_NEON, activeforeground="black", 
                            relief="flat", bd=0,
                            command=lambda r=recipe_name: self.run_recipe(r))
            btn.pack(pady=10)

    def run_recipe(self, recipe_name):
        markers = RECIPES[recipe_name]
        self.log(f"Auto-Sequence Started: {recipe_name}")
        
        for marker in markers:
            self.log(f"Queuing sequence for Marker {marker}...")
            self.queue_tube_sequence(marker)
            
        self.log("All commands for recipe sent to robot buffer.")

    # --- MANUAL MODE ---
    
    def build_manual_tab(self):
        main_frame = tk.Frame(self.tab_manual, bg=COLOR_PANEL)
        main_frame.pack(fill="both", expand=True, padx=40, pady=30)

        controls_frame = tk.Frame(main_frame, bg=COLOR_PANEL)
        controls_frame.pack()

        for i, (pump_id, chem_name) in enumerate(REAGENTS.items()):
            tk.Label(controls_frame, text=f"{pump_id} | {chem_name}", width=30, anchor="w", 
                     bg=COLOR_PANEL, fg="#ccc", font=("Arial", 14)).grid(row=i, column=0, padx=15, pady=20)

            btn_add = tk.Button(controls_frame, text="ADD", bg="#333", fg="white", width=10, height=1, relief="flat",
                                font=("Arial", 12, "bold"), 
                                activebackground=COLOR_BLUE_NEON, activeforeground="black",
                                command=lambda p=pump_id, n=chem_name: self.add_to_batch(p, n))
            btn_add.grid(row=i, column=1, padx=15)

        self.batch_lbl = tk.Label(main_frame, text="BATCH: 0 ITEMS PENDING", font=("Arial", 18, "bold"), bg=COLOR_PANEL, fg=COLOR_TEXT_DIM)
        self.batch_lbl.pack(pady=30)

        btn_exec = tk.Button(main_frame, text="EXECUTE PROTOCOL", bg=COLOR_BLUE_DARK, fg="white", 
                             font=("Arial", 18, "bold"), width=35, height=2, relief="flat",
                             activebackground=COLOR_BLUE_NEON, activeforeground="black",
                             command=self.execute_batch)
        btn_exec.pack(side="bottom", pady=20)

    def add_to_batch(self, pump_id, name):
        self.manual_batch.append(pump_id)
        self.log(f"Manual Added: {name} ({pump_id})")
        self.batch_lbl.config(text=f"BATCH: {len(self.manual_batch)} ITEM(S) PENDING", fg="#00aaff")

    def execute_batch(self):
        if not self.manual_batch:
            messagebox.showwarning("Error", "No ingredients added.")
            return

        self.log("Starting Manual Dispense Protocol...")
        for tube_string in self.manual_batch:
            # Extract the number from "Tube 1", "Tube 2", etc.
            marker_id = tube_string.split(" ")[1]
            self.log(f"Queuing sequence for Marker {marker_id}...")
            self.queue_tube_sequence(marker_id)
            
        self.log("Manual Protocol commands sent to robot buffer.")
        
        # Reset batch
        self.manual_batch.clear()
        self.batch_lbl.config(text="BATCH: 0 ITEMS PENDING", fg=COLOR_TEXT_DIM)

if __name__ == "__main__":
    root = tk.Tk()
    app = LabApp(root)
    root.mainloop()
