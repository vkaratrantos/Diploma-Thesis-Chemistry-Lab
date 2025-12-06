import tkinter as tk
from tkinter import ttk, messagebox
import datetime

# Configuration

REAGENTS = {
    "Tube 1": "Copper Sulfate (CuSO4)",
    "Tube 2": "Sodium Hydroxide (NaOH)",
    "Tube 3": "Sodium Carbonate (Na2CO3)",
    "Tube 4": "Hydrochloric Acid (HCl)",
    "Tube 5": "Ammonia (NH3)"
}

RECIPES = {
    "Copper Hydroxide - Cu(OH)2": {"formula": "CuSO4 + 2NaOH -> Cu(OH)2", "mix": {"Tube 1": 20, "Tube 2": 20}},
    "Copper Carbonate - CuCO3": {"formula": "CuSO4 + Na2CO3 -> CuCO3", "mix": {"Tube 1": 20, "Tube 3": 20}},
    "Carbon Dioxide - CO2": {"formula": "2HCl + Na2CO3 -> CO2 + ...", "mix": {"Tube 4": 25, "Tube 3": 25}},
    "Ammonium Chloride - NH4Cl": {"formula": "HCl + NH3 -> NH4Cl", "mix": {"Tube 4": 10, "Tube 5": 10}},
    "Royal Blue Complex - [Cu(NH3)4]2+": {"formula": "CuSO4 + 4NH3 -> [Cu(NH3)4]2+", "mix": {"Tube 1": 15, "Tube 5": 30}},
    "Neutralization": {"formula": "HCl + NaOH -> NaCl + H2O", "mix": {"Tube 4": 25, "Tube 2": 25}}
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
        self.root.geometry("1200x1000")
        self.root.configure(bg=COLOR_BG)

        self.manual_batch = {} 

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
        self.build_log_area()

    def build_log_area(self):
        outer_container = tk.Frame(self.root, bg=COLOR_BG, pady=20, padx=40)
        outer_container.pack(side="bottom", fill="both", expand=True)
        lbl = tk.Label(outer_container, text="SYSTEM ACTIVITY LOG", font=("Arial", 16, "bold"), bg=COLOR_BG, fg="white", anchor="w")
        lbl.pack(fill="x", pady=(0, 5))
        border_frame = tk.Frame(outer_container, bg=COLOR_BORDER)
        border_frame.pack(fill="both", expand=True)
        inner_frame = tk.Frame(border_frame, bg=COLOR_PANEL)
        inner_frame.pack(fill="both", expand=True, padx=2, pady=2) 
        self.log_text = tk.Text(inner_frame, height=18, bg=COLOR_PANEL, fg="#cccccc", 
                                font=("Arial", 14), bd=0, relief="flat", padx=15, pady=15)
        self.log_text.pack(fill="both", expand=True)

        # Tags
        self.log_text.tag_configure("time", foreground=COLOR_TEXT_DIM, font=("Arial", 14))
        self.log_text.tag_configure("blue_bold", foreground="#00aaff", font=("Arial", 14, "bold"))
        self.log_text.tag_configure("normal", foreground=COLOR_TEXT_MAIN)
        self.log_text.tag_configure("success", foreground="#00ff00", font=("Arial", 14, "bold"))

        self.log("System Ready. Extended Log Area Loaded.", "normal")

    def log(self, message, style="normal"):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}]  ", "time")
        
        if "ENGAGED" in message or "ACTIVATED" in message or "Added" in message:
            style = "blue_bold"
        elif "Completed" in message or "SUCCESS" in message:
            style = "success"

        self.log_text.insert(tk.END, message + "\n\n", style)
        self.log_text.see(tk.END)

    # Auto Mode 
    def build_auto_tab(self):
        # Container for the buttons
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
        target = RECIPES[recipe_name]
        self.log(f"Auto-Sequence Started: {recipe_name}", "normal")
        for pump, amount in target['mix'].items():
            chem_name = REAGENTS[pump]
            self.log(f"PUMP {pump} ENGAGED -> {amount}ml ({chem_name})")
        self.log("Sequence Completed Successfully.")

    # Manual Mode
    def build_manual_tab(self):
        main_frame = tk.Frame(self.tab_manual, bg=COLOR_PANEL)
        main_frame.pack(fill="both", expand=True, padx=40, pady=30)

        controls_frame = tk.Frame(main_frame, bg=COLOR_PANEL)
        controls_frame.pack()

        self.sliders = {} 

        for i, (pump_id, chem_name) in enumerate(REAGENTS.items()):
            tk.Label(controls_frame, text=f"{pump_id} | {chem_name}", width=30, anchor="w", 
                     bg=COLOR_PANEL, fg="#ccc", font=("Arial", 14)).grid(row=i, column=0, padx=15, pady=20)

            # Sliders
            s = tk.Scale(controls_frame, from_=0, to=50, orient="horizontal", length=350, 
                         bg=COLOR_PANEL, fg=COLOR_BLUE_NEON, highlightthickness=0, 
                         troughcolor="#000", activebackground=COLOR_BLUE_NEON,
                         font=("Arial", 18, "bold")) 
            s.grid(row=i, column=1, padx=25)
            self.sliders[pump_id] = s

            btn_add = tk.Button(controls_frame, text="ADD", bg="#333", fg="white", width=10, height=1, relief="flat",
                                font=("Arial", 12, "bold"), 
                                activebackground=COLOR_BLUE_NEON, activeforeground="black",
                                command=lambda p=pump_id, n=chem_name: self.add_to_batch(p, n))
            btn_add.grid(row=i, column=2, padx=15)

        self.batch_lbl = tk.Label(main_frame, text="BATCH: 0 ITEMS PENDING", font=("Arial", 18, "bold"), bg=COLOR_PANEL, fg=COLOR_TEXT_DIM)
        self.batch_lbl.pack(pady=30)

        btn_exec = tk.Button(main_frame, text="EXECUTE PROTOCOL", bg=COLOR_BLUE_DARK, fg="white", 
                             font=("Arial", 18, "bold"), width=35, height=2, relief="flat",
                             activebackground=COLOR_BLUE_NEON, activeforeground="black",
                             command=self.execute_batch)
        btn_exec.pack(side="bottom", pady=20)

    def add_to_batch(self, pump_id, name):
        slider = self.sliders[pump_id]
        amount = slider.get()
        if amount == 0: return

        if pump_id in self.manual_batch:
            self.manual_batch[pump_id] += amount
        else:
            self.manual_batch[pump_id] = amount

        self.log(f"Manual Added: {amount}ml of {name}", "normal")
        self.batch_lbl.config(text=f"BATCH: {len(self.manual_batch)} ITEM(S) PENDING", fg="#00aaff")
        slider.set(0)

    def execute_batch(self):
        if not self.manual_batch:
            messagebox.showwarning("Error", "No volumes added.")
            return

        self.log("Starting Manual Dispense Protocol...", "normal")
        for pump, amount in self.manual_batch.items():
            name = REAGENTS[pump]
            self.log(f"PUMP {pump} ENGAGED -> {amount}ml ({name})")
        self.log("Manual Protocol Completed Successfully.")
        self.manual_batch = {}
        self.batch_lbl.config(text="BATCH: 0 ITEMS PENDING", fg=COLOR_TEXT_DIM)

if __name__ == "__main__":
    root = tk.Tk()
    app = LabApp(root)
    root.mainloop()
