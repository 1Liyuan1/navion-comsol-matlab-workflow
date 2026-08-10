import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from math import cos, pi, sin, sqrt
from pathlib import Path
from time import monotonic

from devices import Gamepad, JakaRobot, UsbCamera
from magnetic_control import CalibrationGrid, CurrentSolver, Vector3
from vision_processing import detect_black_ring_center


class MagneticRobotControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.app_bg = "#eef2f6"
        self.header_bg = "#eef2f6"
        self.panel_bg = "#eef2f6"
        self.text_color = "#101828"
        self.title("Magnetic Robot Control")
        self.minsize(1024, 660)
        self.header_canvas = None
        self.header_left_logo_photo = None
        self.header_left_logo_item = None
        self.header_logo_photo = None
        self.header_logo_item = None
        self.header_title_item = None
        self.grid_data = None
        self.robot = None
        self.robot_connect_button = None
        self.gamepad = Gamepad()
        self.gamepad_connect_button = None
        self.gamepad_help_photo = None
        self.gamepad_polling = False
        self.gamepad_last_command_time = 0
        self.gamepad_command_interval = 0.25
        self.gamepad_last_position_time = 0
        self.gamepad_joint_index = 0
        self.gamepad_control_target = "Cartesian"
        self.gamepad_last_hat_y = 0
        self.gamepad_last_a = False
        self.gamepad_last_b = False
        self.gamepad_mapping = tk.StringVar(value="Xbox/XInput")
        self.gamepad_a_button = tk.StringVar(value="0")
        self.gamepad_b_button = tk.StringVar(value="1")
        self.gamepad_left_x_axis = tk.StringVar(value="0")
        self.gamepad_left_y_axis = tk.StringVar(value="1")
        self.gamepad_z_axis = tk.StringVar(value="3")
        self.gamepad_speed = tk.StringVar(value="2.0")
        self.gamepad_step = tk.StringVar(value="0.1")
        self.camera = UsbCamera()
        self.camera_photo = None
        self.vision_status = tk.StringVar(value="Circle detection is idle.")
        self.vision_deviation = tk.StringVar(value="--")
        self.vision_centroid = tk.StringVar(value="Centroid: --")
        self.current_plot_canvas = None
        self.matrix_path = tk.StringVar()
        self.magnetic_status = tk.StringVar(value="Load an actuation matrix CSV.")
        self.robot_status = tk.StringVar(value="Robot disconnected.")
        self.robot_control_mode = tk.StringVar(value="常规")
        self.gamepad_status = tk.StringVar(value="Gamepad disconnected.")
        self.gamepad_axes = tk.StringVar(value="Left X/Y: 0.00, 0.00    Right Y: 0.00")
        self.gamepad_selected_joint = tk.StringVar(value="Selected joint: J1")
        self.gamepad_active_control = tk.StringVar(value="Control target: Cartesian X/Y by left stick, Z by right stick")
        self.gamepad_mode_display = tk.StringVar(value="Mode: Cartesian")
        self.gamepad_raw_input = tk.StringVar(value="Raw input: --")
        self.gamepad_last_command = tk.StringVar(value="Last jog: --")
        self.gamepad_tcp_position = tk.StringVar(value="TCP: --")
        self.gamepad_joint_position = tk.StringVar(value="Joints: --")
        self.generator_status = tk.StringVar(value="FY8300S protocol is not configured. Output is locked.")
        self.magnetic_moment = tk.StringVar(value="")
        self.magnetic_entries = {}
        self.pose_entries = []
        self.joint_entries = []
        self._configure_styles()
        self._load_header_logos()
        self._load_header_logo()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._close_application)

    def _configure_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=self.app_bg, foreground=self.text_color, font=("Segoe UI", 10))
        style.configure("App.TFrame", background=self.app_bg)
        style.configure("App.TLabelframe", background=self.panel_bg, borderwidth=1, relief="solid")
        style.configure("App.TLabelframe.Label", background=self.panel_bg, foreground=self.text_color, font=("Segoe UI", 11, "bold"))
        style.configure("App.TLabel", background=self.panel_bg, foreground=self.text_color)
        style.configure("App.TButton", padding=(12, 6))

    def _build_ui(self):
        self.configure(background=self.app_bg)
        header = tk.Canvas(self, height=96, highlightthickness=0, bd=0, background=self.header_bg)
        header.pack(fill="x", side="top")
        self.header_canvas = header
        if self.header_left_logo_photo is not None:
            self.header_left_logo_item = header.create_image(0, 0, anchor="w", image=self.header_left_logo_photo)
        self.header_title_item = header.create_text(
            0,
            0,
            text="Three-Coil Magnetic Control System",
            fill=self.text_color,
            font=("Segoe UI", 24, "bold"),
        )
        if self.header_logo_photo is not None:
            self.header_logo_item = header.create_image(0, 0, anchor="e", image=self.header_logo_photo)
        header.create_line(0, 95, 5000, 95, fill="#d7dde5")
        header.bind("<Configure>", self._layout_header)

        workspace = ttk.Frame(self, padding=(14, 10, 14, 14), style="App.TFrame")
        workspace.pack(fill="both", expand=True)
        workspace.columnconfigure(0, weight=1, uniform="column")
        workspace.columnconfigure(1, weight=1, uniform="column")
        workspace.rowconfigure(0, weight=1)
        workspace.rowconfigure(1, weight=1)

        camera_panel = ttk.LabelFrame(workspace, text="Camera Preview", padding=12, style="App.TLabelframe")
        robot_panel = ttk.LabelFrame(workspace, text="", padding=12, style="App.TLabelframe")
        magnetic_panel = ttk.LabelFrame(workspace, text="Magnetic Field", padding=12, style="App.TLabelframe")
        generator_panel = ttk.LabelFrame(workspace, text="Signal Generator", padding=12, style="App.TLabelframe")
        camera_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))
        robot_panel.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(6, 0))
        magnetic_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 6))
        generator_panel.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(6, 0))
        self._build_camera_panel(camera_panel)
        self._build_robot_panel(robot_panel)
        self._build_magnetic_panel(magnetic_panel)
        self._build_generator_panel(generator_panel)

    def _load_header_logos(self):
        assets_dir = Path(__file__).with_name("assets")
        self.header_left_logo_photo = self._load_header_image(assets_dir / "sysu_logo.png", 72, transparent_threshold=10)

    def _load_header_logo(self):
        assets_dir = Path(__file__).with_name("assets")
        self.header_logo_photo = self._load_header_image(assets_dir / "jiang_lab_logo.png", 72, transparent_threshold=245)

    def _load_header_image(self, image_path, target_height, transparent_threshold):
        if not image_path.exists():
            return None
        try:
            from PIL import Image, ImageTk

            image = Image.open(image_path).convert("RGBA")
            pixels = []
            for red, green, blue, alpha in image.getdata():
                if red >= transparent_threshold and green >= transparent_threshold and blue >= transparent_threshold:
                    pixels.append((red, green, blue, 0))
                else:
                    pixels.append((red, green, blue, alpha))
            image.putdata(pixels)
            bbox = image.getbbox()
            if bbox is not None:
                image = image.crop(bbox)
            scale = target_height / float(image.height)
            target_width = max(1, int(round(image.width * scale)))
            image = image.resize((target_width, target_height), Image.Resampling.LANCZOS)
            return ImageTk.PhotoImage(image=image)
        except Exception:
            try:
                return tk.PhotoImage(file=str(image_path))
            except Exception:
                return None

    def _layout_header(self, event):
        if self.header_canvas is None:
            return
        width = max(1, event.width)
        height = max(1, event.height)
        title_x = width // 2
        title_y = height // 2 + 1
        self.header_canvas.coords(self.header_title_item, title_x, title_y)
        if self.header_left_logo_photo is not None and self.header_left_logo_item is not None:
            self.header_canvas.coords(self.header_left_logo_item, 14, height // 2 + 2)
        if self.header_logo_photo is not None and self.header_logo_item is not None:
            logo_x = width - 14
            logo_y = height // 2 + 2
            self.header_canvas.coords(self.header_logo_item, logo_x, logo_y)

    def _build_camera_panel(self, content):
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=0)
        content.columnconfigure(2, weight=1)
        content.rowconfigure(0, weight=1)
        self.camera_preview = tk.Label(content, text="Camera not connected", background="#161b22", foreground="#d0d7de", font=("Segoe UI", 13), width=52, height=14)
        self.camera_preview.grid(row=0, column=0, columnspan=4, sticky="nsew")
        ttk.Label(content, text="Device index").grid(row=1, column=0, sticky="w", pady=(10, 0))
        self.camera_index = ttk.Entry(content, width=8)
        self.camera_index.insert(0, "0")
        self.camera_index.grid(row=1, column=1, sticky="w", padx=(8, 12), pady=(10, 0))
        ttk.Button(content, text="Start camera", command=self._start_camera).grid(row=1, column=2, pady=(10, 0))
        ttk.Button(content, text="Stop camera", command=self._stop_camera).grid(row=1, column=3, padx=(8, 0), pady=(10, 0))
        ttk.Label(content, text="Circle deviation (px)").grid(row=2, column=0, sticky="w", pady=(12, 2))
        self.vision_deviation_entry = ttk.Entry(content, textvariable=self.vision_deviation, width=14, state="readonly")
        self.vision_deviation_entry.grid(row=2, column=1, sticky="w", padx=(8, 12), pady=(12, 2))
        ttk.Label(content, textvariable=self.vision_centroid).grid(row=2, column=2, columnspan=2, sticky="w", pady=(12, 2))
        ttk.Label(content, textvariable=self.vision_status, wraplength=520).grid(row=3, column=0, columnspan=4, sticky="w", pady=(6, 0))

    def _build_robot_panel(self, content):
        content.rowconfigure(0, weight=1)
        content.columnconfigure(0, weight=1)
        scroller = tk.Canvas(content, highlightthickness=0, bd=0, background=self.panel_bg)
        scrollbar = ttk.Scrollbar(content, orient="vertical", command=scroller.yview)
        scroller.configure(yscrollcommand=scrollbar.set)
        scroller.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        scroll_content = ttk.Frame(scroller, style="App.TFrame")
        scroll_window = scroller.create_window((0, 0), window=scroll_content, anchor="nw")
        scroll_content.bind(
            "<Configure>",
            lambda event: scroller.configure(scrollregion=scroller.bbox("all")),
        )
        scroller.bind(
            "<Configure>",
            lambda event: scroller.itemconfigure(scroll_window, width=event.width),
        )
        scroller.bind("<Enter>", lambda event: self._bind_mousewheel(scroller))
        scroller.bind("<Leave>", lambda event: self._unbind_mousewheel())

        content = scroll_content
        content.columnconfigure(0, weight=1)
        header = ttk.Frame(content, style="App.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="JAKA Robot", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="Control mode").grid(row=0, column=1, sticky="e", padx=(8, 6))
        self.robot_mode_combo = ttk.Combobox(header, textvariable=self.robot_control_mode, width=10, state="readonly")
        self.robot_mode_combo["values"] = ("常规", "手动")
        self.robot_mode_combo.grid(row=0, column=2, sticky="e")
        self.robot_mode_combo.bind("<<ComboboxSelected>>", self._on_robot_mode_changed)

        common = ttk.Frame(content, style="App.TFrame")
        common.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(common, text="Robot IP").grid(row=0, column=0, sticky="w")
        self.robot_ip = ttk.Entry(common, width=22)
        self.robot_ip.insert(0, "10.5.5.100")
        self.robot_ip.grid(row=0, column=1, sticky="w", padx=(8, 12))
        self.robot_connect_button = tk.Button(common, text="Connect", command=self._connect_robot, width=12)
        self.robot_connect_button.grid(row=0, column=2, padx=(0, 6))
        self._set_status_button(self.robot_connect_button, "idle", "Connect")
        ttk.Button(common, text="Disconnect", command=self._disconnect_robot).grid(row=0, column=3)
        ttk.Label(common, textvariable=self.robot_status).grid(row=1, column=0, columnspan=4, sticky="w", pady=(8, 8))
        ttk.Button(common, text="上电", command=self._power_on_robot).grid(row=2, column=0, sticky="w", pady=(0, 4))
        ttk.Button(common, text="断电", command=self._power_off_robot).grid(row=2, column=1, sticky="w", pady=(0, 4))
        ttk.Button(common, text="上使能", command=self._enable_robot).grid(row=2, column=2, sticky="w", padx=(0, 6), pady=(0, 4))
        ttk.Button(common, text="断使能", command=self._disable_robot).grid(row=2, column=3, sticky="w", pady=(0, 4))
        ttk.Button(common, text="Read TCP", command=self._read_tcp).grid(row=3, column=0, sticky="w")
        ttk.Button(common, text="Stop motion", command=self._stop_motion).grid(row=3, column=1, sticky="w")
        ttk.Separator(common, orient="horizontal").grid(row=4, column=0, columnspan=4, sticky="ew", pady=(10, 8))

        self.robot_mode_container = ttk.Frame(content, style="App.TFrame")
        self.robot_mode_container.grid(row=2, column=0, sticky="nsew")
        self.robot_mode_container.columnconfigure(0, weight=1)
        self.robot_regular_frame = ttk.Frame(self.robot_mode_container, style="App.TFrame")
        self.robot_manual_frame = ttk.Frame(self.robot_mode_container, style="App.TFrame")
        self._build_regular_robot_controls(self.robot_regular_frame)
        self._build_manual_robot_controls(self.robot_manual_frame)
        self._show_robot_mode()

    def _build_regular_robot_controls(self, content):
        notebook = ttk.Notebook(content)
        notebook.grid(row=0, column=0, sticky="nsew")
        tcp_frame = ttk.Frame(notebook, padding=(0, 6, 0, 0), style="App.TFrame")
        joint_frame = ttk.Frame(notebook, padding=(0, 6, 0, 0), style="App.TFrame")
        notebook.add(tcp_frame, text="TCP")
        notebook.add(joint_frame, text="Joints")
        ttk.Label(tcp_frame, text="TCP target").grid(row=0, column=0, sticky="w")
        for column, label in enumerate(("X", "Y", "Z", "RX", "RY", "RZ")):
            ttk.Label(tcp_frame, text=label).grid(row=1, column=column, sticky="w", pady=(6, 2))
            entry = ttk.Entry(tcp_frame, width=10)
            entry.insert(0, "0")
            entry.grid(row=2, column=column, padx=(0, 5))
            self.pose_entries.append(entry)
        ttk.Label(tcp_frame, text="Speed").grid(row=3, column=0, sticky="w", pady=(10, 2))
        self.robot_speed = ttk.Entry(tcp_frame, width=10)
        self.robot_speed.insert(0, "20")
        self.robot_speed.grid(row=3, column=1, sticky="w", pady=(10, 2))
        ttk.Button(tcp_frame, text="Move TCP", command=self._move_tcp).grid(row=3, column=2, sticky="w", padx=(8, 0), pady=(10, 2))
        ttk.Label(joint_frame, text="Joint target").grid(row=0, column=0, sticky="w")
        for column, label in enumerate(("J1", "J2", "J3", "J4", "J5", "J6")):
            ttk.Label(joint_frame, text=label).grid(row=1, column=column, sticky="w", pady=(6, 2))
            entry = ttk.Entry(joint_frame, width=10)
            entry.insert(0, "0")
            entry.grid(row=2, column=column, padx=(0, 5))
            self.joint_entries.append(entry)
        ttk.Label(joint_frame, text="Speed").grid(row=3, column=0, sticky="w", pady=(10, 2))
        ttk.Label(joint_frame, text="Use TCP speed").grid(row=3, column=1, sticky="w", pady=(10, 2))
        ttk.Button(joint_frame, text="Read joints", command=self._read_joints).grid(row=3, column=2, sticky="w", padx=(8, 0), pady=(10, 2))
        ttk.Button(joint_frame, text="Move joints", command=self._move_joints).grid(row=3, column=3, sticky="w", padx=(8, 0), pady=(10, 2))

    def _build_manual_robot_controls(self, content):
        content.columnconfigure(1, weight=1)
        self.gamepad_connect_button = tk.Button(content, text="Connect gamepad", command=self._connect_gamepad, width=16)
        self.gamepad_connect_button.grid(row=0, column=0, sticky="w")
        self._set_status_button(self.gamepad_connect_button, "idle", "Connect gamepad")
        self.gamepad_mapping_combo = ttk.Combobox(content, textvariable=self.gamepad_mapping, width=16, state="readonly")
        self.gamepad_mapping_combo["values"] = ("Xbox/XInput", "Beitong/DInput")
        self.gamepad_mapping_combo.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.gamepad_mapping_combo.bind("<<ComboboxSelected>>", self._on_gamepad_mapping_changed)
        ttk.Button(content, text="?", width=3, command=self._show_gamepad_help).grid(row=0, column=2, sticky="e")
        ttk.Label(content, textvariable=self.gamepad_status).grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 2))
        self.gamepad_mode_label = tk.Label(content, textvariable=self.gamepad_mode_display, anchor="center", width=22)
        self.gamepad_mode_label.grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 2))
        self._update_gamepad_mode_display()
        ttk.Button(content, text="Cartesian", command=self._set_gamepad_cartesian_mode).grid(row=2, column=2, sticky="w", padx=(8, 0), pady=(4, 2))
        ttk.Button(content, text="Joint", command=self._set_gamepad_joint_mode).grid(row=2, column=3, sticky="w", pady=(4, 2))
        ttk.Label(content, text="Jog speed").grid(row=3, column=0, sticky="w", pady=(6, 2))
        ttk.Spinbox(content, from_=0.05, to=10.0, increment=0.05, textvariable=self.gamepad_speed, width=8).grid(row=3, column=1, sticky="w", pady=(6, 2))
        ttk.Label(content, text="Jog step").grid(row=3, column=2, sticky="w", padx=(8, 0), pady=(6, 2))
        ttk.Spinbox(content, from_=0.1, to=5.0, increment=0.1, textvariable=self.gamepad_step, width=8).grid(row=3, column=3, sticky="w", pady=(6, 2))
        ttk.Label(content, text="X/Y/Z axes").grid(row=4, column=0, sticky="w", pady=2)
        self.gamepad_left_x_axis_combo = ttk.Combobox(content, textvariable=self.gamepad_left_x_axis, width=4, state="readonly")
        self.gamepad_left_x_axis_combo["values"] = tuple(str(index) for index in range(8))
        self.gamepad_left_x_axis_combo.grid(row=4, column=1, sticky="w", pady=2)
        self.gamepad_left_y_axis_combo = ttk.Combobox(content, textvariable=self.gamepad_left_y_axis, width=4, state="readonly")
        self.gamepad_left_y_axis_combo["values"] = tuple(str(index) for index in range(8))
        self.gamepad_left_y_axis_combo.grid(row=4, column=2, sticky="w", pady=2)
        self.gamepad_z_axis_combo = ttk.Combobox(content, textvariable=self.gamepad_z_axis, width=8, state="readonly")
        self.gamepad_z_axis_combo["values"] = tuple(str(index) for index in range(8))
        self.gamepad_z_axis_combo.grid(row=4, column=3, sticky="w", pady=2)
        ttk.Label(content, text="A/B buttons").grid(row=5, column=0, sticky="w", pady=2)
        self.gamepad_a_button_combo = ttk.Combobox(content, textvariable=self.gamepad_a_button, width=4, state="readonly")
        self.gamepad_a_button_combo["values"] = tuple(str(index) for index in range(16))
        self.gamepad_a_button_combo.grid(row=5, column=1, sticky="w", pady=2)
        self.gamepad_b_button_combo = ttk.Combobox(content, textvariable=self.gamepad_b_button, width=4, state="readonly")
        self.gamepad_b_button_combo["values"] = tuple(str(index) for index in range(16))
        self.gamepad_b_button_combo.grid(row=5, column=2, sticky="w", pady=2)
        ttk.Label(content, textvariable=self.gamepad_axes).grid(row=6, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(content, textvariable=self.gamepad_raw_input, wraplength=520).grid(row=7, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(content, textvariable=self.gamepad_active_control, font=("Segoe UI", 10, "bold")).grid(row=8, column=0, columnspan=4, sticky="w", pady=(6, 2))
        ttk.Label(content, textvariable=self.gamepad_selected_joint).grid(row=9, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(content, textvariable=self.gamepad_last_command, wraplength=520).grid(row=10, column=0, columnspan=4, sticky="w", pady=2)
        ttk.Label(content, textvariable=self.gamepad_tcp_position, wraplength=520).grid(row=11, column=0, columnspan=4, sticky="w", pady=(6, 2))
        ttk.Label(content, textvariable=self.gamepad_joint_position, wraplength=520).grid(row=12, column=0, columnspan=4, sticky="w", pady=2)

    def _on_robot_mode_changed(self, event=None):
        self._show_robot_mode()
        if self.robot_control_mode.get() == "手动":
            self._connect_gamepad(silent=True)

    def _on_gamepad_mapping_changed(self, event=None):
        if self.gamepad_mapping.get() == "Beitong/DInput":
            self.gamepad_a_button.set("1")
            self.gamepad_b_button.set("2")
            self.gamepad_z_axis.set("2")
        else:
            self.gamepad_a_button.set("0")
            self.gamepad_b_button.set("1")
            self.gamepad_z_axis.set("3")

    def _set_gamepad_cartesian_mode(self):
        self.gamepad_control_target = "Cartesian"
        self.gamepad_last_a = False
        self.gamepad_last_b = False
        self._update_gamepad_mode_display()

    def _set_gamepad_joint_mode(self):
        self.gamepad_control_target = "Joint"
        self.gamepad_last_a = False
        self.gamepad_last_b = False
        self._update_gamepad_mode_display()

    def _show_robot_mode(self):
        self.robot_regular_frame.grid_forget()
        self.robot_manual_frame.grid_forget()
        if self.robot_control_mode.get() == "手动":
            self.robot_manual_frame.grid(row=0, column=0, sticky="nsew")
            self._start_gamepad_polling()
        else:
            self.robot_regular_frame.grid(row=0, column=0, sticky="nsew")
            self.gamepad_control_target = "Cartesian"
            if self.gamepad.is_open:
                self.gamepad.close()
            self.gamepad_status.set("Gamepad disconnected.")
            self._set_status_button(self.gamepad_connect_button, "idle", "Connect gamepad")

    def _bind_mousewheel(self, canvas):
        canvas.bind_all("<MouseWheel>", lambda event: canvas.yview_scroll(int(-1 * (event.delta / 120)), "units"))

    def _unbind_mousewheel(self):
        self.unbind_all("<MouseWheel>")

    def _build_magnetic_panel(self, content):
        content.columnconfigure(0, weight=1)
        ttk.Label(content, text="Actuation matrix CSV").grid(row=0, column=0, sticky="w")
        ttk.Entry(content, textvariable=self.matrix_path, width=54).grid(row=1, column=0, columnspan=4, sticky="ew", pady=(2, 8))
        ttk.Button(content, text="Browse", command=self._browse_matrix).grid(row=1, column=4, padx=(8, 0))
        for row, section in enumerate(("Position", "Target field"), start=2):
            ttk.Label(content, text=section).grid(row=row, column=0, sticky="w", pady=(8, 2))
            for column, axis in enumerate(("X", "Y", "Z"), start=1):
                ttk.Label(content, text=axis).grid(row=row, column=column, sticky="w")
                entry = ttk.Entry(content, width=12)
                entry.insert(0, "0")
                entry.grid(row=row + 1, column=column, padx=(0, 8))
                self.magnetic_entries[(section, axis)] = entry
        ttk.Label(content, text="Maximum current (A)").grid(row=6, column=0, sticky="w", pady=(10, 2))
        self.max_current = ttk.Entry(content, width=12)
        self.max_current.insert(0, "1.0")
        self.max_current.grid(row=6, column=1, sticky="w", pady=(10, 2))
        ttk.Label(content, text="Rotation frequency (Hz)").grid(row=6, column=2, sticky="w", pady=(10, 2))
        self.rotation_frequency = ttk.Entry(content, width=12)
        self.rotation_frequency.insert(0, "1.0")
        self.rotation_frequency.grid(row=6, column=3, sticky="w", pady=(10, 2))
        ttk.Label(content, text="Periods").grid(row=7, column=0, sticky="w", pady=(10, 2))
        self.rotation_periods = ttk.Entry(content, width=12)
        self.rotation_periods.insert(0, "1")
        self.rotation_periods.grid(row=7, column=1, sticky="w", pady=(10, 2))
        ttk.Button(content, text="Calculate current trajectory", command=self._calculate_current_trajectory).grid(row=7, column=2, columnspan=2, sticky="w", pady=(10, 2))
        ttk.Label(content, text="Capsule magnet moment |m| (A·m^2)").grid(row=8, column=0, sticky="w", pady=(10, 2))
        self.magnetic_moment_entry = ttk.Entry(content, textvariable=self.magnetic_moment, width=12)
        self.magnetic_moment_entry.grid(row=8, column=1, sticky="w", pady=(10, 2))
        ttk.Button(content, text="Drive matrix view", command=self._open_driver_matrix_viewer).grid(row=8, column=2, columnspan=2, sticky="w", pady=(10, 2))
        ttk.Label(content, text="Trajectory: Bx=Bxy cos(2 pi f t), By=Bxy sin(2 pi f t), Bz=bias.", wraplength=490).grid(row=9, column=0, columnspan=5, sticky="w", pady=(8, 4))
        self.current_plot_area = ttk.Frame(content)
        self.current_plot_area.grid(row=10, column=0, columnspan=5, sticky="nsew")
        ttk.Label(content, textvariable=self.magnetic_status, justify="left", wraplength=490).grid(row=11, column=0, columnspan=5, sticky="w", pady=(6, 0))

    def _build_generator_panel(self, content):
        ttk.Label(content, text="FY8300S serial port").grid(row=0, column=0, sticky="w")
        self.generator_port = ttk.Combobox(content, width=14, state="readonly")
        self.generator_port["values"] = tuple("COM{0}".format(index) for index in range(1, 8))
        self.generator_port.set("COM3")
        self.generator_port.grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(content, text="Three channels: output disabled until the verified protocol and current-to-voltage calibration are supplied.", wraplength=500).grid(row=1, column=0, columnspan=4, sticky="w", pady=(12, 8))
        ttk.Label(content, textvariable=self.generator_status, foreground="#a33a3a", wraplength=500).grid(row=2, column=0, columnspan=4, sticky="w")

    def _browse_matrix(self):
        path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not path:
            return
        try:
            self.grid_data = CalibrationGrid.from_csv(path)
            self.matrix_path.set(path)
            self.magnetic_status.set("Loaded. Position range: {0} to {1}".format(self.grid_data.minimum_position, self.grid_data.maximum_position))
        except Exception as error:
            messagebox.showerror("Import failed", str(error))

    def _start_camera(self):
        try:
            self.camera.open(int(self.camera_index.get()))
            self._update_camera_frame()
        except Exception as error:
            messagebox.showerror("Camera start failed", str(error))

    def _update_camera_frame(self):
        if not self.camera.is_open:
            return
        frame = self.camera.read_rgb()
        if frame is not None:
            try:
                from PIL import Image, ImageTk

                detection, mask = detect_black_ring_center(frame)
                image = self._overlay_detection(frame, detection)
                image = Image.fromarray(image)
                image.thumbnail((500, 300))
                self.camera_photo = ImageTk.PhotoImage(image=image)
                self.camera_preview.configure(image=self.camera_photo, text="")
                self._update_vision_fields(detection)
            except ImportError:
                self._stop_camera()
                messagebox.showerror("Camera display failed", "Camera preview requires Pillow. Run: python -m pip install -r requirements.txt")
                return
            except Exception as error:
                self.vision_status.set("Circle detection failed: {0}".format(error))
                self.vision_deviation.set("--")
                self.vision_centroid.set("Centroid: --")
        self.after(30, self._update_camera_frame)

    def _stop_camera(self):
        self.camera.close()
        self.camera_photo = None
        self.camera_preview.configure(image="", text="Camera not connected")
        self.vision_status.set("Circle detection is idle.")
        self.vision_deviation.set("--")
        self.vision_centroid.set("Centroid: --")

    def _overlay_detection(self, frame, detection):
        try:
            import cv2
        except ImportError:
            return frame
        overlay = frame.copy()
        height, width = overlay.shape[:2]
        center_x = width // 2
        center_y = height // 2
        cv2.drawMarker(overlay, (center_x, center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)
        if detection is not None and detection.detected and detection.centroid_x is not None and detection.centroid_y is not None:
            centroid = (int(round(detection.centroid_x)), int(round(detection.centroid_y)))
            cv2.circle(overlay, centroid, 8, (0, 255, 0), 2)
            cv2.line(overlay, (center_x, center_y), centroid, (0, 165, 255), 2)
            cv2.putText(overlay, "offset={0:.1f}px".format(detection.deviation or 0.0), (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(overlay, "offset={0:.1f}px".format(detection.deviation or 0.0), (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        return overlay

    def _update_vision_fields(self, detection):
        if detection is None or not detection.detected or detection.centroid_x is None or detection.centroid_y is None or detection.deviation is None:
            self.vision_deviation.set("--")
            self.vision_centroid.set("Centroid: --")
            self.vision_status.set("Black circle not detected.")
            return
        self.vision_deviation.set("{0:.2f}".format(detection.deviation))
        self.vision_centroid.set("Centroid: ({0:.1f}, {1:.1f})".format(detection.centroid_x, detection.centroid_y))
        self.vision_status.set("Detected ring area: {0:d} px, contour area: {1:.1f}".format(detection.mask_area, detection.contour_area))

    def _magnetic_vector(self, section):
        return Vector3(*(float(self.magnetic_entries[(section, axis)].get()) for axis in ("X", "Y", "Z")))

    def _open_driver_matrix_viewer(self):
        try:
            if self.grid_data is None:
                raise ValueError("Load an actuation matrix CSV first.")
            position = self._magnetic_vector("Position")
            matrix = self.grid_data.interpolate(position)
        except Exception as error:
            messagebox.showerror("View driver matrix failed", str(error))
            return

        viewer = tk.Toplevel(self)
        viewer.title("Driver Matrix Viewer")
        viewer.configure(background=self.app_bg)
        viewer.geometry("760x420")
        viewer.transient(self)
        viewer.grab_set()

        container = ttk.Frame(viewer, padding=16, style="App.TFrame")
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        title = ttk.Label(container, text="Three-Coil Driver Matrix", font=("Segoe UI", 14, "bold"))
        title.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        ttk.Label(container, text="Current position: ({0:.4f}, {1:.4f}, {2:.4f})".format(position.x, position.y, position.z)).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 8))

        matrix_frame = ttk.LabelFrame(container, text="Actuation matrix A(p)", padding=12, style="App.TLabelframe")
        matrix_frame.grid(row=2, column=0, sticky="nsew", padx=(0, 8))
        for row_index, label in enumerate(("Bx", "By", "Bz")):
            ttk.Label(matrix_frame, text=label).grid(row=row_index + 1, column=0, sticky="w", padx=(0, 8), pady=4)
        for column_index, label in enumerate(("I1", "I2", "I3")):
            ttk.Label(matrix_frame, text=label).grid(row=0, column=column_index + 1, sticky="w", padx=6, pady=(0, 4))
        for row_index in range(3):
            for column_index in range(3):
                value = matrix.values[row_index][column_index]
                ttk.Label(matrix_frame, text="{0:.6g}".format(value), relief="solid", padding=(6, 2)).grid(row=row_index + 1, column=column_index + 1, padx=4, pady=4, sticky="nsew")

        info_frame = ttk.LabelFrame(container, text="Capsule robot model", padding=12, style="App.TLabelframe")
        info_frame.grid(row=2, column=1, sticky="nsew", padx=(8, 0))
        info_frame.columnconfigure(1, weight=1)

        ttk.Label(info_frame, text="Permanent magnet moment |m| (A·m^2)").grid(row=0, column=0, sticky="w", pady=(0, 4))
        moment_entry = ttk.Entry(info_frame, width=16)
        moment_entry.insert(0, self.magnetic_moment.get().strip())
        moment_entry.grid(row=0, column=1, sticky="w", pady=(0, 4))
        ttk.Label(info_frame, text="Magnetic moment vector").grid(row=1, column=0, sticky="w", pady=(8, 4))
        ttk.Label(info_frame, text="m = |m| · u_m").grid(row=1, column=1, sticky="w", pady=(8, 4))
        ttk.Label(info_frame, text="Magnetic torque").grid(row=2, column=0, sticky="w", pady=(8, 4))
        ttk.Label(info_frame, text="τ = m × B").grid(row=2, column=1, sticky="w", pady=(8, 4))
        ttk.Label(info_frame, text="Magnetic force").grid(row=3, column=0, sticky="w", pady=(8, 4))
        ttk.Label(info_frame, text="F = ∇(m · B)").grid(row=3, column=1, sticky="w", pady=(8, 4))
        ttk.Label(info_frame, text="For a unique capsule robot, the magnitude |m| can be treated as a fixed calibrated parameter.", wraplength=290).grid(row=4, column=0, columnspan=2, sticky="w", pady=(12, 0))

        button_frame = ttk.Frame(container, style="App.TFrame")
        button_frame.grid(row=3, column=0, columnspan=2, sticky="e", pady=(14, 0))
        ttk.Button(button_frame, text="Close", command=viewer.destroy).pack()

    def _calculate_current_trajectory(self):
        try:
            if self.grid_data is None:
                raise ValueError("Load an actuation matrix CSV first.")
            position = self._magnetic_vector("Position")
            initial_field = self._magnetic_vector("Target field")
            frequency = float(self.rotation_frequency.get())
            periods = float(self.rotation_periods.get())
            if frequency <= 0 or periods <= 0:
                raise ValueError("Rotation frequency and periods must be positive.")
            xy_amplitude = sqrt(initial_field.x * initial_field.x + initial_field.y * initial_field.y)
            if xy_amplitude == 0:
                raise ValueError("The XY target-field amplitude must be greater than zero for rotation.")
            matrix = self.grid_data.interpolate(position)
            solver = CurrentSolver(float(self.max_current.get()))
            samples = max(200, int(200 * periods))
            duration = periods / frequency
            times, i1, i2, i3 = [], [], [], []
            limited = False
            for index in range(samples + 1):
                time = duration * index / samples
                angle = 2.0 * pi * frequency * time
                target = Vector3(xy_amplitude * cos(angle), xy_amplitude * sin(angle), initial_field.z)
                solution = solver.solve(matrix, target)
                times.append(time)
                i1.append(solution.currents.x)
                i2.append(solution.currents.y)
                i3.append(solution.currents.z)
                limited = limited or solution.is_limited
            self._render_current_plot(times, i1, i2, i3)
            self.magnetic_status.set("Bxy = {0:.6f}, Bz bias = {1:.6f}, f = {2:.6f} Hz\nCurrent range: I1 [{3:.4f}, {4:.4f}], I2 [{5:.4f}, {6:.4f}], I3 [{7:.4f}, {8:.4f}] A\nCurrent limit reached: {9}".format(xy_amplitude, initial_field.z, frequency, min(i1), max(i1), min(i2), max(i2), min(i3), max(i3), limited))
        except Exception as error:
            messagebox.showerror("Trajectory calculation failed", str(error))

    def _render_current_plot(self, times, i1, i2, i3):
        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except ImportError as error:
            raise RuntimeError("Current plot requires matplotlib. Run: python -m pip install -r requirements.txt") from error
        if self.current_plot_canvas is not None:
            self.current_plot_canvas.get_tk_widget().destroy()
        figure = Figure(figsize=(5.7, 3.0), dpi=100)
        axis = figure.add_subplot(111)
        axis.plot(times, i1, label="I1")
        axis.plot(times, i2, label="I2")
        axis.plot(times, i3, label="I3")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Current (A)")
        axis.set_title("Three-coil current trajectory")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
        figure.tight_layout()
        self.current_plot_canvas = FigureCanvasTkAgg(figure, master=self.current_plot_area)
        self.current_plot_canvas.draw()
        self.current_plot_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _connect_robot(self):
        try:
            self.robot = JakaRobot(self.robot_ip.get().strip())
            self.robot.connect()
            self.robot_status.set("Connected to " + self.robot_ip.get().strip())
            self._set_status_button(self.robot_connect_button, "ok", "Connected")
        except Exception as error:
            self.robot = None
            self.robot_status.set("Robot connection failed.")
            self._set_status_button(self.robot_connect_button, "error", "Connect failed")
            messagebox.showerror("Robot connection failed", str(error))

    def _disconnect_robot(self):
        try:
            if self.robot is not None:
                self.robot.disconnect()
            self.robot = None
            self.robot_status.set("Robot disconnected.")
            self._set_status_button(self.robot_connect_button, "idle", "Connect")
        except Exception as error:
            self._set_status_button(self.robot_connect_button, "error", "Disconnect failed")
            messagebox.showerror("Robot disconnect failed", str(error))

    def _power_and_enable(self):
        if messagebox.askyesno("Confirm robot enable", "Power on and enable the robot now?"):
            self._run_robot_action("Robot powered on and enabled.", lambda: self._require_robot().power_on_and_enable())

    def _power_on_robot(self):
        if messagebox.askyesno("确认上电", "现在给机械臂上电吗？"):
            self._run_robot_action("Robot powered on.", lambda: self._require_robot().power_on())

    def _power_off_robot(self):
        if messagebox.askyesno("确认断电", "现在给机械臂断电吗？"):
            self._run_robot_action("Robot powered off.", lambda: self._require_robot().power_off())

    def _enable_robot(self):
        if messagebox.askyesno("确认上使能", "现在给机械臂上使能吗？"):
            self._run_robot_action("Robot enabled.", lambda: self._require_robot().enable())

    def _disable_robot(self):
        if messagebox.askyesno("确认断使能", "现在给机械臂断使能吗？"):
            self._run_robot_action("Robot disabled.", lambda: self._require_robot().disable())

    def _read_tcp(self):
        try:
            pose = self._require_robot().get_tcp_pose()
            for entry, value in zip(self.pose_entries, pose):
                entry.delete(0, tk.END)
                entry.insert(0, str(value))
            self.robot_status.set("TCP pose read successfully.")
        except Exception as error:
            messagebox.showerror("Read TCP failed", str(error))

    def _move_tcp(self):
        try:
            pose = tuple(float(entry.get()) for entry in self.pose_entries)
            speed = float(self.robot_speed.get())
            if messagebox.askyesno("Confirm TCP move", "Command the robot to move to the entered TCP pose?"):
                self._require_robot().move_linear(pose, speed, blocking=False)
                self.robot_status.set("TCP motion command sent.")
        except Exception as error:
            messagebox.showerror("TCP move failed", str(error))

    def _read_joints(self):
        try:
            joints = self._require_robot().get_joint_position()
            for entry, value in zip(self.joint_entries, joints):
                entry.delete(0, tk.END)
                entry.insert(0, str(value))
            self.robot_status.set("Joint positions read successfully.")
        except Exception as error:
            messagebox.showerror("Read joints failed", str(error))

    def _move_joints(self):
        try:
            joints = tuple(float(entry.get()) for entry in self.joint_entries)
            speed = float(self.robot_speed.get())
            if messagebox.askyesno("Confirm joint move", "Command the robot to move to the entered joint angles?"):
                self._require_robot().move_joint(joints, speed, blocking=False)
                self.robot_status.set("Joint motion command sent.")
        except Exception as error:
            messagebox.showerror("Joint move failed", str(error))

    def _stop_motion(self):
        self._run_robot_action("Stop-motion command sent.", lambda: self._require_robot().stop())

    def _connect_gamepad(self, silent=False):
        try:
            if not self.gamepad.is_open:
                self.gamepad.open(0)
            state = self.gamepad.poll()
            self.gamepad_status.set("Gamepad connected: {0}".format(state.name))
            self._set_status_button(self.gamepad_connect_button, "ok", "Gamepad OK")
            self._start_gamepad_polling()
        except Exception as error:
            self.gamepad_status.set("Gamepad disconnected.")
            self._set_status_button(self.gamepad_connect_button, "error", "Gamepad failed")
            if not silent:
                messagebox.showerror("Gamepad connection failed", str(error))

    def _start_gamepad_polling(self):
        if self.gamepad_polling:
            return
        self.gamepad_polling = True
        self.after(50, self._poll_gamepad)

    def _poll_gamepad(self):
        self.gamepad_polling = False
        if self.robot_control_mode.get() != "手动":
            return
        state = self.gamepad.poll()
        state = self._apply_gamepad_mapping(state)
        if state.connected:
            self.gamepad_status.set("Gamepad connected: {0} | axes {1}, buttons {2}, hats {3}".format(state.name, len(state.axes), len(state.buttons), len(state.hats)))
            self._set_status_button(self.gamepad_connect_button, "ok", "Gamepad OK")
            self._update_gamepad_inputs(state)
            self._handle_gamepad_robot_command(state)
        else:
            self.gamepad_status.set("Gamepad disconnected.")
            self._set_status_button(self.gamepad_connect_button, "error", "Gamepad failed")
        self._update_gamepad_robot_position()
        self._start_gamepad_polling()

    def _update_gamepad_inputs(self, state):
        self.gamepad_axes.set(
            "Left X/Y: {0:+.2f}, {1:+.2f}    Right Y: {2:+.2f}".format(
                state.left_x,
                state.left_y,
                state.right_y,
            )
        )
        active_buttons = [str(index) for index, pressed in enumerate(state.buttons) if pressed]
        axes_text = ", ".join("{0}:{1:+.2f}".format(index, value) for index, value in enumerate(state.axes[:8]))
        hats_text = ", ".join("{0}:{1}".format(index, hat) for index, hat in enumerate(state.hats))
        self.gamepad_raw_input.set(
            "Raw axes [{0}] | pressed buttons [{1}] | hats [{2}]".format(
                axes_text or "--",
                ", ".join(active_buttons) or "--",
                hats_text or "--",
            )
        )
        if state.hat_y != 0 and self.gamepad_last_hat_y == 0:
            self.gamepad_joint_index = (self.gamepad_joint_index - state.hat_y) % 6
        if state.button_a and not self.gamepad_last_a:
            self.gamepad_control_target = "Joint"
        if state.button_b and not self.gamepad_last_b:
            self.gamepad_control_target = "Cartesian"
        self.gamepad_last_hat_y = state.hat_y
        self.gamepad_last_a = state.button_a
        self.gamepad_last_b = state.button_b
        self.gamepad_selected_joint.set("Selected joint: J{0}".format(self.gamepad_joint_index + 1))
        if self.gamepad_control_target == "Joint":
            self.gamepad_active_control.set("Control target: J{0} jog by left stick X".format(self.gamepad_joint_index + 1))
        else:
            self.gamepad_active_control.set("Control target: Cartesian X/Y by left stick, Z by right stick")
        self._update_gamepad_mode_display()

    def _apply_gamepad_mapping(self, state):
        if not state.connected:
            return state
        axes = state.axes
        buttons = state.buttons
        x_axis = self._selected_axis(self.gamepad_left_x_axis.get(), 0)
        y_axis = self._selected_axis(self.gamepad_left_y_axis.get(), 1)
        z_axis = self._selected_gamepad_z_axis()
        a_button = self._selected_button(self.gamepad_a_button.get(), 0)
        b_button = self._selected_button(self.gamepad_b_button.get(), 1)
        left_x = axes[x_axis] if len(axes) > x_axis else 0.0
        left_y = -(axes[y_axis] if len(axes) > y_axis else 0.0)
        right_y = -(axes[z_axis] if len(axes) > z_axis else 0.0)
        button_a = buttons[a_button] if len(buttons) > a_button else False
        button_b = buttons[b_button] if len(buttons) > b_button else False
        return type(state)(
            connected=state.connected,
            name=state.name,
            left_x=left_x,
            left_y=left_y,
            right_y=right_y,
            hat_x=state.hat_x,
            hat_y=state.hat_y,
            button_a=button_a,
            button_b=button_b,
            axes=state.axes,
            buttons=state.buttons,
            hats=state.hats,
        )

    def _handle_gamepad_robot_command(self, state):
        now = monotonic()
        if now - self.gamepad_last_command_time < self.gamepad_command_interval:
            return
        command = self._gamepad_command_from_state(state)
        if command is None:
            return
        try:
            robot = self._require_robot()
            mode, axis, value = command
            speed = self._safe_float(self.gamepad_speed.get(), 2.0, 0.05, 10.0)
            if mode == "cartesian":
                self._send_gamepad_cartesian_step(robot, axis, speed, value)
            else:
                self._send_gamepad_joint_step(robot, axis, speed, value)
            self.gamepad_last_command_time = now
            self.gamepad_command_interval = max(0.25, min(1.2, abs(value) / max(speed, 0.05) + 0.08))
        except Exception as error:
            self.robot_status.set("Gamepad jog failed: {0}".format(error))
            self.gamepad_last_command.set("Last jog failed: {0}".format(error))

    def _send_gamepad_cartesian_step(self, robot, axis, speed, value):
        pose = list(robot.get_tcp_pose())
        pose[axis] += value
        robot.move_linear(tuple(pose), speed, blocking=False)
        axis_name = ("X", "Y", "Z", "RX", "RY", "RZ")[axis]
        self.gamepad_last_command.set(
            "Last move: TCP {0} {1:+.3f}, speed {2:.2f}, target {3:.3f}".format(axis_name, value, speed, pose[axis])
        )

    def _send_gamepad_joint_step(self, robot, axis, speed, value):
        joints = list(robot.get_joint_position())
        joints[axis] += value
        robot.move_joint(tuple(joints), speed, blocking=False)
        self.gamepad_last_command.set(
            "Last move: J{0} {1:+.3f}, speed {2:.2f}, target {3:.3f}".format(axis + 1, value, speed, joints[axis])
        )

    def _gamepad_command_from_state(self, state):
        deadzone = 0.25
        step = self._safe_float(self.gamepad_step.get(), 0.5, 0.1, 5.0)
        if self.gamepad_control_target == "Joint":
            if abs(state.left_x) < deadzone:
                return None
            return ("joint", self.gamepad_joint_index, state.left_x * step)
        candidates = (
            (0, state.left_x),
            (1, state.left_y),
            (2, state.right_y),
        )
        axis, value = max(candidates, key=lambda item: abs(item[1]))
        if abs(value) < deadzone:
            return None
        return ("cartesian", axis, value * step)

    def _selected_gamepad_z_axis(self):
        return self._selected_axis(self.gamepad_z_axis.get(), 3)

    def _selected_axis(self, value, default):
        try:
            number = int(value)
        except ValueError:
            return default
        return max(0, min(7, number))

    def _selected_button(self, value, default):
        try:
            number = int(value)
        except ValueError:
            return default
        return max(0, min(15, number))

    def _update_gamepad_mode_display(self):
        if not hasattr(self, "gamepad_mode_label"):
            return
        if self.gamepad_control_target == "Joint":
            self.gamepad_mode_display.set("Mode: Joint J{0}".format(self.gamepad_joint_index + 1))
            self.gamepad_mode_label.configure(background="#f97316", foreground="#ffffff")
        else:
            self.gamepad_mode_display.set("Mode: Cartesian")
            self.gamepad_mode_label.configure(background="#2563eb", foreground="#ffffff")

    def _safe_float(self, value, default, minimum, maximum):
        try:
            number = float(value)
        except ValueError:
            return default
        return max(minimum, min(maximum, number))

    def _update_gamepad_robot_position(self):
        now = monotonic()
        if now - self.gamepad_last_position_time < 0.5:
            return
        self.gamepad_last_position_time = now
        if self.robot is None or not self.robot.is_connected:
            self.gamepad_tcp_position.set("TCP: --")
            self.gamepad_joint_position.set("Joints: --")
            return
        try:
            tcp = self.robot.get_tcp_pose()
            joints = self.robot.get_joint_position()
            self.gamepad_tcp_position.set(
                "TCP: X {0:.2f}, Y {1:.2f}, Z {2:.2f}, RX {3:.2f}, RY {4:.2f}, RZ {5:.2f}".format(*tcp)
            )
            self.gamepad_joint_position.set(
                "Joints: J1 {0:.2f}, J2 {1:.2f}, J3 {2:.2f}, J4 {3:.2f}, J5 {4:.2f}, J6 {5:.2f}".format(*joints)
            )
        except Exception as error:
            self.gamepad_tcp_position.set("TCP: unavailable ({0})".format(error))
            self.gamepad_joint_position.set("Joints: unavailable")

    def _require_robot(self):
        if self.robot is None or not self.robot.is_connected:
            raise RuntimeError("Connect the robot first.")
        return self.robot

    def _set_status_button(self, button, status, text):
        if button is None:
            return
        colors = {
            "idle": ("#f2f4f7", "#101828"),
            "ok": ("#16a34a", "#ffffff"),
            "error": ("#dc2626", "#ffffff"),
        }
        background, foreground = colors.get(status, colors["idle"])
        button.configure(
            text=text,
            background=background,
            foreground=foreground,
            activebackground=background,
            activeforeground=foreground,
            relief="raised",
        )

    def _show_gamepad_help(self):
        viewer = tk.Toplevel(self)
        viewer.title("手柄操作说明")
        viewer.configure(background=self.app_bg)
        viewer.geometry("620x520")
        viewer.transient(self)

        container = ttk.Frame(viewer, padding=16, style="App.TFrame")
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)

        ttk.Label(container, text="手柄按键功能", font=("Segoe UI", 14, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 10))
        image_path = Path(r"C:\Users\Liyuan\AppData\Local\Temp\codex-clipboard-d9c9d4ad-b1bf-41ce-badb-40552b901666.png")
        if image_path.exists():
            try:
                from PIL import Image, ImageTk

                image = Image.open(image_path).convert("RGBA")
                image.thumbnail((360, 220))
                self.gamepad_help_photo = ImageTk.PhotoImage(image=image)
                ttk.Label(container, image=self.gamepad_help_photo).grid(row=1, column=0, sticky="n", pady=(0, 12))
            except Exception:
                pass
        help_text = (
            "笛卡尔模式：左摇杆控制 X/Y 方向小步移动。\n"
            "笛卡尔模式：右摇杆上下控制 Z 方向小步移动。\n"
            "关节模式：左摇杆左右控制当前选中的关节正反向运动。\n"
            "十字键上下：选择 J1-J6 关节。\n"
            "A 键：进入/确认关节模式。\n"
            "B 键：返回笛卡尔模式。\n"
            "Cartesian / Joint 按钮：也可以用鼠标手动切换模式。\n\n"
            "A/B buttons：根据 Raw input 中按 A/B 时出现的按钮编号进行选择。\n"
            "X/Y/Z axes：根据 Raw input 中摇杆变化的轴编号进行选择。\n"
            "Z axis：选择右摇杆上下变化对应的轴编号。\n"
            "Mapping preset：Xbox 手柄优先选 Xbox/XInput；北通手柄可先试 Beitong/DInput。\n\n"
            "正式接近硬件前，请先把 Jog speed 和 Jog step 调低。\n"
            "如果 Gamepad failed 显示红色，说明电脑没有识别到手柄，请检查 USB/蓝牙/电量。\n"
            "如果 Raw axes/buttons 完全不变化，请切换手柄模式或重新连接。"
        )
        ttk.Label(container, text=help_text, justify="left", wraplength=560).grid(row=2, column=0, sticky="w")
        ttk.Button(container, text="关闭", command=viewer.destroy).grid(row=3, column=0, sticky="e", pady=(16, 0))

    def _run_robot_action(self, success_message, action):
        try:
            action()
            self.robot_status.set(success_message)
        except Exception as error:
            messagebox.showerror("Robot command failed", str(error))

    def _close_application(self):
        self.camera.close()
        if self.gamepad.is_open:
            self.gamepad.close()
        self.destroy()


if __name__ == "__main__":
    MagneticRobotControlApp().mainloop()
