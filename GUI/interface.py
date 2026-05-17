import tkinter as tk
from tkinter import ttk, messagebox
import datetime
import threading
import sys

# --- ROS 2 LIBRARIES ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configuration
REAGENTS = {
    "Tube 1": "Copper Sulfate (CuSO4)",
    "Tube 2": "Sodium Hydroxide (NaOH)",
    "Tube 3": "Sodium Carbonate (Na2CO3)",
    "Tube 4": "Hydrochloric Acid (HCl)",
    "Tube 5": "Ammonia (NH3)"
}

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
        self.root.geometry("1200x850") 
        self.root.configure(bg=COLOR_BG)

        self.manual_batch = [] 

        # --- NATIVE ROS 2 INITIALIZATION ---
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('gui_commander_node')
        self.publisher = self.ros_node.create_publisher(String, '/gui_commands', 10)
        
        # Ensure ROS 2 shuts down cleanly when you close the window
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

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

        self.log("System Ready. Connected directly to ROS 2 Network.")

    def on_close(self):
        """Safely shuts down the ROS 2 node when the GUI is closed."""
        self.log("Shutting down GUI Node...")
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def log(self, message):
        """Logs high-level Python events to your terminal."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] [GUI] {message}")

    # --- THE MAGIC PIPELINE ---
    def send_command(self, cmd):
        """Publishes the command instantly as a native ROS 2 node."""
        msg = String()
        msg.data = str(cmd)
        self.publisher.publish(msg)
        self.log(f"Command fired -> {cmd}")

    def queue_tube_sequence(self, marker_id):
        """Sends the pick, place, and pour sequence step-by-step."""
        # Using a slight delay thread so it doesn't slam the network all at once
        def sequence():
            import time
            commands = ["o", f"m {marker_id}", "c", "m 6", "p", f"m {marker_id}", "o"]
            for cmd in commands:
                self.send_command(cmd)
                # Give the C++ script a tiny moment to register each command
                time.sleep(0.1) 
                
        threading.Thread(target=sequence, daemon=True).start()

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
            self.queue_tube_sequence(marker)

    # --- MANUAL MODE ---
    def build_manual_tab(self):
        main_frame = tk.Frame(self.tab_manual, bg=COLOR_PANEL)
        main_frame.pack(fill="both", expand=True, padx=40, pady=20)

        # DIRECT OVERRIDE PANEL
        override_frame = tk.LabelFrame(main_frame, text=" DIRECT ROBOT OVERRIDE ", 
                                       bg=COLOR_PANEL, fg=COLOR_BLUE_NEON, font=("Arial", 14, "bold"), pady=15, padx=15)
        override_frame.pack(fill="x", pady=(0, 20))

        actions_frame = tk.Frame(override_frame, bg=COLOR_PANEL)
        actions_frame.pack(pady=5)

        tk.Button(actions_frame, text="OPEN GRIPPER", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("o")).grid(row=0, column=0, padx=10)
        tk.Button(actions_frame, text="CLOSE GRIPPER", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("c")).grid(row=0, column=1, padx=10)
        tk.Button(actions_frame, text="POUR LIQUID", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("p")).grid(row=0, column=2, padx=10)
        tk.Button(actions_frame, text="GO TO MIXER", bg="#333", fg="white", width=15, font=("Arial", 12, "bold"), 
                  command=lambda: self.send_command("m 6")).grid(row=0, column=3, padx=10)

        targets_frame = tk.Frame(override_frame, bg=COLOR_PANEL)
        targets_frame.pack(pady=10)

        for i in range(1, 6):
            tk.Button(targets_frame, text=f"GO TO TUBE {i}", bg="#222", fg="white", width=12, font=("Arial", 10), 
                      command=lambda m=i: self.send_command(f"m {m}")).grid(row=0, column=i-1, padx=5)

        # MANUAL BATCH
        batch_frame = tk.LabelFrame(main_frame, text=" MANUAL BATCH PROTOCOL ", 
                                    bg=COLOR_PANEL, fg=COLOR_TEXT_DIM, font=("Arial", 14, "bold"), pady=15, padx=15)
        batch_frame.pack(fill="both", expand=True)

        controls_frame = tk.Frame(batch_frame, bg=COLOR_PANEL)
        controls_frame.pack()

        for i, (pump_id, chem_name) in enumerate(REAGENTS.items()):
            tk.Label(controls_frame, text=f"{pump_id} | {chem_name}", width=30, anchor="w", 
                     bg=COLOR_PANEL, fg="#ccc", font=("Arial", 14)).grid(row=i, column=0, padx=15, pady=10)

            btn_add = tk.Button(controls_frame, text="ADD TO SEQUENCE", bg="#333", fg="white", width=18, height=1, relief="flat",
                                font=("Arial", 12, "bold"), 
                                activebackground=COLOR_BLUE_NEON, activeforeground="black",
                                command=lambda p=pump_id, n=chem_name: self.add_to_batch(p, n))
            btn_add.grid(row=i, column=1, padx=15)

        self.batch_lbl = tk.Label(batch_frame, text="SEQUENCE: 0 ITEMS PENDING", font=("Arial", 16, "bold"), bg=COLOR_PANEL, fg=COLOR_TEXT_DIM)
        self.batch_lbl.pack(pady=20)

        btn_exec = tk.Button(batch_frame, text="EXECUTE SEQUENCE", bg=COLOR_BLUE_DARK, fg="white", 
                             font=("Arial", 18, "bold"), width=35, height=2, relief="flat",
                             activebackground=COLOR_BLUE_NEON, activeforeground="black",
                             command=self.execute_batch)
        btn_exec.pack(side="bottom", pady=10)

    def add_to_batch(self, pump_id, name):
        self.manual_batch.append(pump_id)
        self.log(f"Manual Added: {name}")
        self.batch_lbl.config(text=f"SEQUENCE: {len(self.manual_batch)} ITEM(S) PENDING", fg="#00aaff")

    def execute_batch(self):
        if not self.manual_batch:
            messagebox.showwarning("Error", "No ingredients added.")
            return

        self.log("Starting Manual Dispense Protocol...")
        for tube_string in self.manual_batch:
            marker_id = tube_string.split(" ")[1]
            self.queue_tube_sequence(marker_id)
            
        self.manual_batch.clear()
        self.batch_lbl.config(text="SEQUENCE: 0 ITEMS PENDING", fg=COLOR_TEXT_DIM)

if __name__ == "__main__":
    root = tk.Tk()
    app = LabApp(root)
    root.mainloop()
