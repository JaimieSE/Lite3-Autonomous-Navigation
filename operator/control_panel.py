#!/usr/bin/env python3
"""Group 8 desktop controls for real Nav, Mission, scoring and RViz processes."""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
from urllib.parse import urlsplit

HOME = Path.home()
NAV = HOME / "lite3_ws"
MISSION = HOME / "live_mission"
SCENE = HOME / "scene_understanding"
STAIR = HOME / "stair_curtain"
PERCEPTION = {
    "Camera preview": ("run_direct_camera.sh", "direct_camera.py"),
    "YOLO camera": ("run_yolo_camera.sh", "direct_yolo.py"),
    "ROS scene detector": ("run.sh", "scene_node.py"),
    "Combined perception": ("run_perception.sh", "combined_perception.py"),
    "Hazard signs": ("run_signs.sh", "sign_vlm.py"),
    "Scene VLM": ("run_vlm.sh", "periodic_vlm.py"),
}
OPERATOR = NAV / "operator"
LOG_ROOT = NAV / "log" / "operator"
PANEL_LOG = LOG_ROOT / "control-panel"
STATE = LOG_ROOT / "state.env"
MODE_CODES = {
    "Auto": 0x21010C03,
    "Manual": 0x21010C02,
    "Enter AI": 0x21010528,
    "Exit AI": 0x2101052B,
}
CONTROLLER = "Controller mode"


def read_state():
    try:
        return dict(line.split("=", 1) for line in STATE.read_text().splitlines() if "=" in line)
    except OSError:
        return {}


def user_processes():
    for path in Path("/proc").glob("[0-9]*"):
        try:
            if path.stat().st_uid != os.getuid():
                continue
            stats = (path / "stat").read_text().rsplit(")", 1)[1].split()
            argv = (path / "cmdline").read_bytes().rstrip(b"\0").decode(errors="replace").split("\0")
            if stats[0] != "Z" and argv[0]:
                yield int(path.name), int(stats[2]), argv
        except (OSError, IndexError, ValueError):
            continue


def nav_snapshot(state, processes):
    try:
        pgid = int(state.get("PGID", "0"))
    except ValueError:
        pgid = 0
    launches = [(group, args) for _, group, args in processes
                if "navigation.launch.py" in args and "lite3_locomotion" in args and "launch" in args]
    if not launches:
        if pgid > 1 and any(group == pgid for _, group, _ in processes):
            return "unknown", "Check operator status · process group still exists"
        return "idle", "Stopped"
    if len(launches) != 1 or launches[0][0] != pgid:
        return "external", "Externally started · use operator terminal"
    mode = state.get("MODE", "unknown")
    arguments = launches[0][1]
    if any(arg.lower() == "allow_robot_motion:=true" for arg in arguments) or "mux_output:=/cmd_vel" in arguments:
        mode = "physical"
    if mode != "dry":
        return mode, f"{mode.capitalize()} · use operator terminal"
    try:
        verdict = json.loads((Path(state["RUN_DIR"]) / "ready.json").read_text())["verdict"]
    except (OSError, ValueError, KeyError):
        return mode, "Dry · checking readiness"
    return mode, f"Dry · {verdict}"


def mode_conflict(state, processes):
    if nav_snapshot(state, processes)[0] not in ("idle", "dry"):
        return "Stop physical or unrecognized Nav before changing controller mode."
    for _, _, args in processes:
        if "lite3_movement_tests.roam" in args and "run" in args:
            return "Stop roaming before changing controller mode."
        if "--armed" in args and any(Path(arg).name == "run_stair_curtain.sh" for arg in args):
            return "Stop the armed stair and curtain workflow first."
    return ""


def check_mode_graph(node, publisher, saw_velocity):
    if saw_velocity:
        raise RuntimeError("Observed /cmd_vel traffic; stop the active velocity source first.")
    readers = node.get_subscriptions_info_by_topic("/simple_cmd")
    if not any(reader.node_name == "motion_sender" and reader.node_namespace == "/"
               and reader.topic_type == "transfer_interfaces/msg/MotionSimpleCMD" for reader in readers):
        raise RuntimeError("The robot motion sender was not discovered on domain 64.")
    if not publisher.get_subscription_count():
        raise RuntimeError("The mode publisher did not match a subscriber.")
    writers = node.get_publishers_info_by_topic("/cmd_vel")
    if any(writer.node_name != "lite3_domain_bridge_64" for writer in writers):
        raise RuntimeError("Another /cmd_vel publisher is present; stop it before switching modes.")


def request_mode(mode):
    if mode not in MODE_CODES or os.environ.get("ROS_DOMAIN_ID") != "64":
        raise RuntimeError("Use a listed controller mode on the system ROS domain.")
    conflict = mode_conflict(read_state(), list(user_processes()))
    if conflict:
        raise RuntimeError(conflict)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from transfer_interfaces.msg import MotionSimpleCMD

    rclpy.init(args=[])
    node = rclpy.create_node("group8_controller_mode_request")
    saw_velocity = False

    def check_velocity(_):
        nonlocal saw_velocity
        saw_velocity = True

    try:
        subscriber = node.create_subscription(Twist, "/cmd_vel", check_velocity, qos_profile_sensor_data)
        publisher = node.create_publisher(MotionSimpleCMD, "/simple_cmd", 1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not saw_velocity:
            rclpy.spin_once(node, timeout_sec=0.1)
        check_mode_graph(node, publisher, saw_velocity)
        conflict = mode_conflict(read_state(), list(user_processes()))
        if conflict:
            raise RuntimeError(conflict)
        publisher.publish(MotionSimpleCMD(cmd_code=MODE_CODES[mode], type=0))
        if not publisher.wait_for_all_acked(Duration(seconds=2)):
            raise RuntimeError(f"{mode} request was sent, but delivery was not acknowledged. Check the controller before retrying.")
        print(f"{mode} request published. Verify the robot mode on the handheld controller.", flush=True)
        node.destroy_subscription(subscriber)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def latest_database():
    pointers = (MISSION / "data" / "runs").glob("*/database.path")
    for pointer in sorted(pointers, reverse=True):
        try:
            database = Path(pointer.read_text().strip())
            if database.is_file():
                return database
        except OSError:
            continue
    return None


def mission_command(profile, scene, wide_scout=False):
    args = [str(MISSION / "run_live.sh"), "--wide-search" if wide_scout else "--rgbd", "--competition", str(profile)]
    if scene:
        args.append("--scene-understanding")
    return args


def scorer_command(database, url):
    parsed = urlsplit(url)
    if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.query or parsed.fragment:
        raise ValueError("Enter the HTTP or HTTPS scorer base URL, without credentials.")
    return [str(MISSION / "competition.sh"), "--db", str(database), "submit", "--server", url, "--watch"]


class Panel:
    navigation_modules = ("Navigation", "Dry Roaming", "Stairs and Curtains", "Live Mission", "Scoring", "RViz")
    modules = (*navigation_modules, *PERCEPTION)
    external_signatures = {
        "Live Mission": str(MISSION / "live_perception.py"),
        "Scoring": str(MISSION / "competition.py"),
        "RViz": "/rviz2",
        "Dry Roaming": "lite3_movement_tests.roam",
        "Stairs and Curtains": "stair_curtain_advisory.py",
        **{name: process for name, (_, process) in PERCEPTION.items()},
    }

    def __init__(self, root):
        self.root = root
        self.jobs = {}
        self.stopping = set()
        self.files = {}
        self.external = set()
        self.nav_mode = "idle"
        self.last_log = (None, None)
        self.status = {name: tk.StringVar(value="Checking…") for name in self.modules}
        self.action = tk.StringVar(value="Navigation")
        options = sorted(MISSION.glob("competition*.json"))
        self.profile = tk.StringVar(value="competition-v6.3-practice.json")
        self.scene = tk.BooleanVar(value=False)
        self.wide_scout = tk.BooleanVar(value=True)
        self.scorer_url = tk.StringVar()
        self.feedback = tk.StringVar(value="Group 8 · waiting for status")
        self.database = tk.StringVar(value="No mission database")
        self.controls = {}
        root.title("Lite3 • Group 8")
        root.geometry("960x690")
        root.minsize(750, 560)
        header = ttk.Frame(root, padding=(18, 14))
        header.pack(fill="x")
        ttk.Label(header, text="Lite3 control panel", font=("TkDefaultFont", 19, "bold")).pack(anchor="w")
        ttk.Label(header, textvariable=self.feedback).pack(anchor="w", pady=(3, 0))
        tabs = ttk.Notebook(root)
        tabs.pack(fill="x", padx=18, pady=(0, 6))
        self.module_tab(tabs, "Navigation and Mission", self.navigation_modules)
        self.module_tab(tabs, "Standalone Perception", PERCEPTION)
        self.mode_tab(tabs)
        form = ttk.Frame(root, padding=(18, 3))
        form.pack(fill="x")
        ttk.Label(form, text="Mission profile").grid(row=0, column=0, sticky="w")
        ttk.Combobox(form, state="readonly", values=[p.name for p in options],
                     textvariable=self.profile, width=34).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Checkbutton(form, text="Scene understanding", variable=self.scene).grid(row=0, column=2, sticky="w")
        ttk.Checkbutton(form, text="Wide scout", variable=self.wide_scout).grid(row=0, column=3, sticky="w", padx=(8, 0))
        ttk.Label(form, text="Scorer base URL").grid(row=1, column=0, sticky="w", pady=8)
        ttk.Entry(form, textvariable=self.scorer_url).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Label(form, text="Practice uses a local mock scorer").grid(row=1, column=2, sticky="w")
        form.columnconfigure(1, weight=1)
        bar = ttk.Frame(root, padding=(18, 4))
        bar.pack(fill="x")
        for name, command in (("Preflight", self.preflight), ("Nav status", self.nav_status),
                              ("Mission results", self.results), ("Scoring reports", self.reports),
                              ("Stop dry Nav and panel jobs", self.stop_all)):
            ttk.Button(bar, text=name, command=command).pack(side="left", padx=(0, 7))
        ttk.Label(root, textvariable=self.database, padding=(18, 4)).pack(fill="x")
        logs = ttk.Frame(root, padding=(18, 4, 18, 14))
        logs.pack(fill="both", expand=True)
        view = ttk.Combobox(logs, state="readonly", textvariable=self.action,
                            values=(*self.modules, "Diagnostics", "Results", CONTROLLER), width=16)
        view.pack(anchor="w", pady=(0, 6))
        view.bind("<<ComboboxSelected>>", lambda _: self.show_log())
        self.console = tk.Text(logs, height=13, wrap="word", state="disabled", font=("TkFixedFont", 10))
        scroll = ttk.Scrollbar(logs, command=self.console.yview)
        self.console.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.console.pack(fill="both", expand=True)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()

    def module_tab(self, tabs, title, names):
        grid = ttk.Frame(tabs, padding=(8, 4))
        tabs.add(grid, text=title)
        grid.columnconfigure(1, weight=1)
        for row, name in enumerate(names):
            ttk.Label(grid, text=name, width=23).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Label(grid, textvariable=self.status[name]).grid(row=row, column=1, sticky="w")
            action = {
                "Navigation": "Start dry",
                "Dry Roaming": "Start dry",
                "Stairs and Curtains": "Start advisory",
            }.get(name, "Start")
            start = ttk.Button(grid, text=action, command=lambda m=name: self.start(m))
            stop = ttk.Button(grid, text="Stop", command=lambda m=name: self.stop(m))
            start.grid(row=row, column=2, padx=4)
            stop.grid(row=row, column=3, padx=4)
            self.controls[name] = start, stop

    def mode_tab(self, tabs):
        frame = ttk.Frame(tabs, padding=12)
        tabs.add(frame, text="Controller mode")
        ttk.Label(frame, text="Auto and Manual select the control source. Enter AI and Exit AI change AI state.",
                  wraplength=630).pack(anchor="w")
        buttons = ttk.Frame(frame)
        buttons.pack(anchor="w", pady=(9, 6))
        self.mode_buttons = []
        for mode in MODE_CODES:
            button = ttk.Button(buttons, text=mode, command=lambda selected=mode: self.select_mode(selected))
            button.pack(side="left", padx=(0, 8))
            self.mode_buttons.append(button)
        ttk.Label(frame, text="Confirm the resulting mode on the handheld controller.").pack(anchor="w")

    def select_mode(self, mode):
        if CONTROLLER in self.jobs or "Navigation" in self.jobs or "Dry Roaming" in self.jobs:
            self.feedback.set("Wait for the current controller or Nav operation to finish.")
            return
        conflict = mode_conflict(read_state(), list(user_processes()))
        if conflict:
            self.feedback.set(conflict)
            return
        if not messagebox.askyesno(
            f"Request {mode}",
            f"Request {mode} on the live Lite3? Confirm the robot is stationary, "
            "the area is clear, and the handheld controller is in reach.",
            default="no", parent=self.root,
        ):
            return
        self.spawn(CONTROLLER, [
            "bash", "-c",
            'source operator/env.sh && export ROS_DOMAIN_ID=64 && exec python3 operator/control_panel.py --request-mode "$1"',
            "controller-mode", mode,
        ], NAV)

    def camera_api_environment(self):
        env = os.environ.copy()
        if env.get("ANTHROPIC_API_KEY", "").strip():
            return env
        key = simpledialog.askstring("Scene understanding", "Anthropic API key", show="*", parent=self.root)
        if key and key.strip():
            env["ANTHROPIC_API_KEY"] = key.strip()
            return env
        return None

    def spawn(self, name, command, cwd, env=None):
        log = PANEL_LOG / f"{name.lower().replace(' ', '-')}.log"
        try:
            with log.open("wb") as stream:
                proc = subprocess.Popen(command, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as exc:
            self.feedback.set(f"{name}: {exc}")
            return
        self.jobs[name] = proc
        self.files[name] = log
        self.action.set(name)
        self.last_log = (None, None)
        self.feedback.set(f"{name} · started")
        self.show_log()

    def start(self, name):
        if name in self.jobs or name in self.external:
            return
        if name == "Navigation":
            if nav_snapshot(read_state(), list(user_processes()))[0] == "idle":
                self.spawn(name, [str(OPERATOR / "start_navigation.sh")], NAV)
        elif name == "Dry Roaming":
            state = read_state()
            if "Navigation" in self.jobs or nav_snapshot(state, list(user_processes()))[0] != "dry":
                self.feedback.set("Start dry Nav before dry roaming")
                return
            command = [str(OPERATOR / "roam.sh"), "start"]
            speed = state.get("MAX_SPEED", "")
            try:
                if 0 < float(speed) <= 1:
                    command.extend(("--max-speed", speed))
            except ValueError:
                pass
            self.spawn(name, command, NAV)
        elif name == "Stairs and Curtains":
            self.spawn(name, [str(STAIR / "run_stair_curtain.sh"), "--preview"], STAIR)
        elif name in PERCEPTION:
            launcher, _ = PERCEPTION[name]
            env = None
            if name == "Scene VLM":
                if not messagebox.askyesno("Start Scene VLM", "Send selected camera frames to the configured Claude API?",
                                           parent=self.root):
                    return
                env = self.camera_api_environment()
                if env is None:
                    return
            self.spawn(name, [str(SCENE / launcher)], SCENE, env)
        elif name == "Live Mission":
            if self.nav_mode != "dry" or "Navigation" in self.jobs:
                if not messagebox.askyesno("Dry Nav unavailable", "Map coordinates may be unavailable. Start Live Mission anyway?",
                                           parent=self.root):
                    return
            profile = MISSION / self.profile.get()
            if not profile.is_file():
                self.feedback.set("Select a mission profile")
                return
            env = self.camera_api_environment() if self.scene.get() else None
            if self.scene.get() and env is None:
                return
            self.spawn(name, mission_command(profile, self.scene.get(), self.wide_scout.get()), MISSION, env)
        elif name == "Scoring":
            database = latest_database()
            if database is None:
                self.feedback.set("No saved mission database to submit")
                return
            try:
                command = scorer_command(database, self.scorer_url.get().strip())
            except ValueError as exc:
                self.feedback.set(str(exc))
                return
            if messagebox.askyesno("Start scoring sender", f"Send staged reports for {database.parent.name} to the entered URL?",
                                   parent=self.root):
                self.spawn(name, command, MISSION)
        else:
            config = OPERATOR / "lite3_map.rviz"
            self.spawn(name, ["bash", "-c", 'source operator/env.sh && exec ros2 run rviz2 rviz2 -d "$1"',
                              "rviz", str(config)], NAV)

    def stop(self, name):
        if name == "Navigation":
            if "Navigation" not in self.jobs and nav_snapshot(read_state(), list(user_processes()))[0] == "dry":
                self.stop("Dry Roaming")
                self.spawn(name, [str(OPERATOR / "stop_navigation.sh")], NAV)
            return
        proc = self.jobs.get(name)
        if proc is not None and name not in self.stopping and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                self.stopping.add(name)
                self.feedback.set(f"{name} · waiting for clean shutdown")
            except ProcessLookupError:
                pass

    def stop_all(self):
        for name in reversed(self.modules):
            self.stop(name)

    def preflight(self):
        if self.nav_mode != "idle" or "Navigation" in self.jobs:
            self.feedback.set("Stop Nav before running a standalone preflight")
        elif "Diagnostics" not in self.jobs:
            self.spawn("Diagnostics", [str(OPERATOR / "preflight.sh"), "--direct-telemetry"], NAV)

    def nav_status(self):
        if "Diagnostics" not in self.jobs:
            self.spawn("Diagnostics", [str(OPERATOR / "status.sh")], NAV)

    def results(self):
        database = latest_database()
        if database is None:
            self.feedback.set("No saved mission results yet")
        elif "Results" not in self.jobs:
            self.spawn("Results", [str(MISSION / "status.sh"), str(database)], MISSION)

    def reports(self):
        database = latest_database()
        if database is None:
            self.feedback.set("No saved scoring reports yet")
        elif "Results" not in self.jobs:
            self.spawn("Results", [str(MISSION / "competition.sh"), "--db", str(database), "status"], MISSION)

    def refresh(self):
        for name, proc in list(self.jobs.items()):
            code = proc.poll()
            if code is not None:
                self.jobs.pop(name)
                self.stopping.discard(name)
                self.feedback.set(f"{name} · exited with code {code}. See its log.")
        processes = list(user_processes())
        state = read_state()
        self.nav_mode, description = nav_snapshot(state, processes)
        self.status["Navigation"].set("Operator command running…" if "Navigation" in self.jobs else description)
        self.external.clear()
        for name, signature in self.external_signatures.items():
            def matches(args):
                if name == "RViz":
                    return args[0] == "rviz2" or args[0].endswith(signature)
                if name == "Dry Roaming":
                    return signature in args and "run" in args
                found = any(arg == signature or arg.endswith("/" + signature) for arg in args)
                return found and (name != "Scoring" or "submit" in args)
            if name not in self.jobs and any(matches(args) for _, _, args in processes):
                self.external.add(name)
            self.status[name].set("Running" if name in self.jobs else
                                  "Running outside panel · close from original terminal" if name in self.external else "Stopped")
        for name, (start, stop) in self.controls.items():
            busy = name in self.jobs or name in self.external
            can_start = not busy and (self.nav_mode == "idle" if name == "Navigation" else True)
            if name == "Dry Roaming":
                can_start = can_start and self.nav_mode == "dry" and "Navigation" not in self.jobs
            can_stop = self.nav_mode == "dry" and not busy if name == "Navigation" else name in self.jobs
            start.configure(state="normal" if can_start else "disabled")
            stop.configure(state="normal" if can_stop and name not in self.stopping else "disabled")
        mode_busy = (self.nav_mode not in ("idle", "dry")
                     or any(name in self.jobs for name in (CONTROLLER, "Navigation", "Dry Roaming"))
                     or "Dry Roaming" in self.external)
        for button in self.mode_buttons:
            button.configure(state="disabled" if mode_busy else "normal")
        database = latest_database()
        self.database.set(f"Latest run · {database.parent.name}" if database else "No mission database")
        self.show_log()
        self.root.after(1000, self.refresh)

    def show_log(self):
        name = self.action.get()
        path = self.files.get(name, PANEL_LOG / f"{name.lower().replace(' ', '-')}.log")
        if name == "Navigation" and name not in self.jobs and self.nav_mode == "dry":
            run = read_state().get("RUN_DIR")
            if run:
                path = Path(run) / "launch.log"
        try:
            signature = (path, path.stat().st_mtime_ns)
            if signature == self.last_log:
                return
            with path.open("rb") as stream:
                stream.seek(max(0, stream.seek(0, os.SEEK_END) - 16000))
                value = stream.read().decode(errors="replace")
        except OSError:
            signature, value = (path, None), "No output recorded for this module."
        self.last_log = signature
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        self.console.insert("end", value)
        self.console.see("end")
        self.console.configure(state="disabled")

    def close(self):
        active = [name for name in self.modules if name != "Navigation" and name in self.jobs]
        if CONTROLLER in self.jobs:
            active.append(CONTROLLER)
        if active:
            messagebox.showinfo("Modules still running", "Stop the panel-started modules before closing.", parent=self.root)
        else:
            self.root.destroy()


def main():
    PANEL_LOG.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (PANEL_LOG / "panel.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("The Group 8 control panel is already open.")
        root = tk.Tk()
        Panel(root)
        root.mainloop()


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--request-mode":
        try:
            request_mode(sys.argv[2])
        except RuntimeError as exc:
            raise SystemExit(str(exc)) from None
    else:
        main()
