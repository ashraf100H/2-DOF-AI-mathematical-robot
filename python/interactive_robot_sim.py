"""Step 7: live Pygame controls for the existing frozen ANN robot.

Run: python python/interactive_robot_sim.py
Rendering/events stay on the main thread. Model loading and bounded planning
run on one worker, so a slow prediction never blocks the window's event loop.
"""

from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import traceback

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")

import numpy as np
import pygame

from trajectory_planner import (Scene, PlannerSettings, Workspace, JointTrajectory,
                                collision_reason, finite_pair, plan_motion, robot_points)


ROOT = Path(__file__).resolve().parents[1]
SCENE = Scene()  # Edit table, floor and box in Scene, in trajectory_planner.py.
SETTINGS = PlannerSettings()
START_ANGLES = np.array([90.0, 0.0])
FIXED_DT = 1.0 / 120.0
WINDOW_SIZE = (1160, 850)
WORLD = pygame.Rect(20, 104, 704, 704)
SCALE = 16.0
INK = (221, 230, 240)
MUTED = (151, 167, 187)
BLUE = (79, 163, 247)
GREEN = (85, 214, 170)
ORANGE = (248, 184, 91)
RED = (248, 115, 124)


def to_screen(point):
    return (round(WORLD.centerx + SCALE * point[0]),
            round(WORLD.centery - SCALE * point[1]))


def to_robot(point):
    return ((point[0] - WORLD.centerx) / SCALE,
            (WORLD.centery - point[1]) / SCALE)


def load_pipeline():
    # Import TensorFlow here, after the live window is open.
    from robot_pipeline import load_robot_pipeline
    robot, predict = load_robot_pipeline(ROOT)
    return robot, predict, Workspace.from_repository(ROOT)


class Simulator:
    def __init__(self):
        pygame.display.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode(WINDOW_SIZE)
        pygame.display.set_caption("2-DOF Robot | Step 7 - Frozen ANN Simulator")
        self.font = pygame.font.SysFont("segoeui", 19)
        self.small = pygame.font.SysFont("segoeui", 16)
        self.title_font = pygame.font.SysFont("segoeui", 29, bold=True)
        self.value_font = pygame.font.SysFont("consolas", 21)
        self.fields = [pygame.Rect(764, 145, 172, 40), pygame.Rect(950, 145, 172, 40)]
        self.move_button = pygame.Rect(764, 197, 358, 43)
        self.box_button = pygame.Rect(764, 250, 226, 37)
        self.reset_button = pygame.Rect(1000, 250, 122, 37)
        self.texts = ["10", "10"]
        self.focus = None
        self.select_all = False
        self.current = START_ANGLES.copy()
        self.target = None
        self.result = None
        self.trajectory = None
        self.elapsed = self.accumulator = 0.0
        self.status = "LOADING"
        self.message = "Loading the saved ANN and scalers..."
        self.pipeline = None
        self.preview = []
        self.running = True
        self.generation = 0
        self.job_generation = 0
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="robot-planner")
        self.future = self.worker.submit(load_pipeline)
        self.job_kind = "load"

    @property
    def busy(self):
        return self.future is not None or self.trajectory is not None

    def reset(self):
        # Explicit simulation reset; this is not a commanded robot trajectory.
        # Keep a running worker until it finishes, but discard its stale plan.
        self.generation += 1
        self.current = START_ANGLES.copy()
        self.target = self.result = self.trajectory = None
        self.preview = []
        self.elapsed = self.accumulator = 0.0
        self.status = "READY" if self.pipeline else "LOADING"
        self.message = "Scene reset to the vertical pose."
        if self.pipeline is None and self.future is None:
            self.future = self.worker.submit(load_pipeline)
            self.job_kind = "load"

    def submit(self):
        if self.busy or self.pipeline is None:
            return
        self.result = None
        self.preview = []
        try:
            target = finite_pair(self.texts)
        except ValueError as error:
            self.target = None
            self.status, self.message = "INVALID INPUT", str(error)
            return
        self.target = target
        self.status, self.message = "PLANNING", "ANN prediction -> endpoint checks -> collision-checked route."
        self.job_generation = self.generation
        self.job_kind = "plan"
        robot, predict, workspace = self.pipeline
        self.future = self.worker.submit(plan_motion, robot, predict, workspace,
                                         self.current.copy(), target, SCENE, SETTINGS)

    def poll_worker(self):
        if self.future is None or not self.future.done():
            return
        future, kind = self.future, self.job_kind
        self.future = None
        stale = kind == "plan" and self.job_generation != self.generation
        try:
            value = future.result()
            if stale:
                return
            if kind == "load":
                reason = collision_reason(value[0], START_ANGLES, SCENE, SETTINGS.clearance_cm)
                if reason:
                    raise ValueError("Starting pose is blocked by the configured scene: " + reason)
                self.pipeline = value
                self.status = "READY"
                self.message = "Enter X/Y and press Move, or click a target on the grid."
                return
            self.result = value
            self.message = value.message
            if not value.accepted:
                self.status = value.status.replace("_", " ").upper()
                return
            self.trajectory = JointTrajectory(value.nodes, SETTINGS.joint_speed_deg_s)
            self.elapsed = self.accumulator = 0.0
            self.status = "MOVING"
            robot = self.pipeline[0]
            # Preview the achieved FK route, not an assumed straight hand path.
            self.preview = [robot_points(robot, self.trajectory.sample(t))[2]
                            for t in np.linspace(0, self.trajectory.duration, 180)]
        except Exception as error:
            traceback.print_exc()
            if not stale:
                self.status = "ERROR"
                self.message = f"{error}. Check the terminal; install requirements/requirements-sim.txt if needed."

    def update(self, seconds):
        self.poll_worker()
        if self.trajectory is None:
            self.accumulator = 0.0
            return
        # Limit catch-up after a long OS pause. Slow machines slow the simulated
        # clock instead of skipping across an unchecked robot state.
        self.accumulator += min(max(seconds, 0.0), 0.1)
        while self.accumulator >= FIXED_DT and self.trajectory is not None:
            self.accumulator -= FIXED_DT
            self.elapsed = min(self.elapsed + FIXED_DT, self.trajectory.duration)
            next_angles = self.trajectory.sample(self.elapsed)
            reason = collision_reason(self.pipeline[0], next_angles, SCENE, SETTINGS.clearance_cm)
            if reason:
                self.trajectory = None
                self.status, self.message = "COLLISION", "Motion stopped at the previous safe pose: " + reason
                break
            self.current = next_angles
            if self.elapsed >= self.trajectory.duration:
                self.trajectory = None
                error = np.linalg.norm(robot_points(self.pipeline[0], self.current)[2] - self.target)
                self.status = "TARGET REACHED" if error <= SETTINGS.target_tolerance_cm else "ANN ERROR"
                self.message = f"Finished at the ANN pose; actual target error {error:.3f} cm."

    def choose_target(self, point):
        if self.busy or self.pipeline is None:
            return
        self.texts = [f"{point[0]:.3f}", f"{point[1]:.3f}"]
        self.focus = None
        self.submit()  # Typed, mouse and box controls all use the same pipeline.

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.focus = next((i for i, rect in enumerate(self.fields) if rect.collidepoint(event.pos)), None)
            self.select_all = self.focus is not None
            if self.reset_button.collidepoint(event.pos):
                self.reset()
            elif self.move_button.collidepoint(event.pos):
                self.submit()
            elif self.box_button.collidepoint(event.pos):
                self.choose_target(SCENE.box_xy)
            elif WORLD.collidepoint(event.pos):
                point = to_robot(event.pos)
                if np.max(np.abs(np.asarray(point) - SCENE.box_xy)) <= SCENE.box_side / 2:
                    point = SCENE.box_xy
                self.choose_target(point)
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.running = False
            elif event.key == pygame.K_TAB:
                self.focus = 0 if self.focus is None else 1 - self.focus
                self.select_all = True
            elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.submit()
            elif self.focus is not None:
                if event.key == pygame.K_a and event.mod & pygame.KMOD_CTRL:
                    self.select_all = True
                elif event.key in (pygame.K_BACKSPACE, pygame.K_DELETE):
                    self.texts[self.focus] = "" if self.select_all else self.texts[self.focus][:-1]
                    self.select_all = False
        elif event.type == pygame.TEXTINPUT and self.focus is not None:
            old = "" if self.select_all else self.texts[self.focus]
            self.texts[self.focus] = (old + event.text)[:20]
            self.select_all = False

    def text(self, value, position, color=INK, font=None):
        self.screen.blit((font or self.font).render(str(value), True, color), position)

    def wrapped(self, value, position, width, color=MUTED, max_lines=4):
        lines, line = [], ""
        for word in value.split():
            trial = f"{line} {word}".strip()
            if self.small.size(trial)[0] > width and line:
                lines.append(line)
                line = word
            else:
                line = trial
        lines.append(line)
        for index, line in enumerate(lines[:max_lines]):
            if index == max_lines - 1 and len(lines) > max_lines:
                line = line[:-3] + "..."
            self.text(line, (position[0], position[1] + 21 * index), color, self.small)

    def button(self, rect, label, enabled=True, primary=False):
        color = (38, 98, 154) if primary and enabled else (39, 51, 68)
        pygame.draw.rect(self.screen, color, rect, border_radius=6)
        surface = self.font.render(label, True, INK if enabled else MUTED)
        self.screen.blit(surface, surface.get_rect(center=rect.center))

    def draw_world(self):
        pygame.draw.rect(self.screen, (19, 30, 44), WORLD, border_radius=8)
        self.screen.set_clip(WORLD)
        for value in range(-20, 21, 2):
            color = (60, 76, 93) if value == 0 else (32, 45, 60)
            pygame.draw.line(self.screen, color, to_screen((value, -22)), to_screen((value, 22)))
            pygame.draw.line(self.screen, color, to_screen((-22, value)), to_screen((22, value)))
            if value % 10 == 0 and value != 0:
                self.text(value, (to_screen((value, 0))[0] + 3, WORLD.centery + 4), MUTED, self.small)
                self.text(value, (WORLD.centerx + 4, to_screen((0, value))[1]), MUTED, self.small)
        # Radial limits alone are not the full workspace: label them as guides.
        for radius in (10, 20):
            pygame.draw.circle(self.screen, (61, 74, 88), to_screen((0, 0)), int(radius * SCALE), 1)
        floor_y = to_screen((0, SCENE.floor_y))[1]
        pygame.draw.rect(self.screen, (31, 36, 44), (WORLD.left, floor_y, WORLD.width, WORLD.bottom - floor_y))
        pygame.draw.line(self.screen, MUTED, (WORLD.left, floor_y), (WORLD.right, floor_y), 2)
        self.text("FLOOR", (WORLD.left + 10, floor_y + 8), MUTED, self.small)
        table = SCENE.table
        top_left = to_screen((table.xmin, table.ymax))
        rect = pygame.Rect(*top_left, round((table.xmax-table.xmin)*SCALE), round((table.ymax-table.ymin)*SCALE))
        pygame.draw.rect(self.screen, (69, 76, 89), rect)
        pygame.draw.rect(self.screen, (134, 144, 159), rect, 2)
        self.text("TABLE", (rect.left + 26, rect.top + 32), INK, self.small)
        for a, b in zip(self.preview[:-1], self.preview[1:]):
            pygame.draw.line(self.screen, (57, 117, 105), to_screen(a), to_screen(b), 2)
        if self.result and self.result.accepted:
            for point in self.result.waypoints:
                pygame.draw.circle(self.screen, GREEN, to_screen(point), 4, 1)
        box = pygame.Rect(0, 0, round(SCENE.box_side*SCALE), round(SCENE.box_side*SCALE))
        box.center = to_screen(SCENE.box_xy)
        pygame.draw.rect(self.screen, ORANGE, box)
        if self.pipeline:
            points = [to_screen(p) for p in robot_points(self.pipeline[0], self.current)]
            pygame.draw.line(self.screen, BLUE, points[0], points[1], 5)
            pygame.draw.line(self.screen, GREEN, points[1], points[2], 5)
            pygame.draw.circle(self.screen, INK, points[0], 8)
            pygame.draw.circle(self.screen, INK, points[1], 6)
            pygame.draw.circle(self.screen, RED, points[2], 5)
        if self.target is not None and np.all(np.abs(self.target) <= 22):
            center = to_screen(self.target)
            pygame.draw.circle(self.screen, ORANGE, center, round(SETTINGS.target_tolerance_cm*SCALE), 1)
            pygame.draw.line(self.screen, ORANGE, (center[0]-10, center[1]), (center[0]+10, center[1]), 2)
            pygame.draw.line(self.screen, ORANGE, (center[0], center[1]-10), (center[0], center[1]+10), 2)
        self.text("+Y (cm)", (WORLD.centerx + 8, WORLD.top + 8), MUTED, self.small)
        self.text("+X (cm)", (WORLD.right - 72, WORLD.centery - 28), MUTED, self.small)
        self.text("10 / 20 cm radial guides | joint limits also apply", (WORLD.left + 12, WORLD.top + 10), MUTED, self.small)
        self.screen.set_clip(None)

    def draw(self):
        self.screen.fill((12, 19, 30))
        self.text("2-DOF / ANN ROBOT SIMULATOR", (22, 20), INK, self.title_font)
        self.text("STEP 7     Cartesian target  >  frozen ANN  >  checked trajectory  >  live robot", (24, 61), MUTED)
        self.draw_world()
        pygame.draw.rect(self.screen, (24, 34, 49), (744, 104, 398, 704), border_radius=8)
        self.text("TARGET X (cm)", (764, 115), MUTED, self.small)
        self.text("TARGET Y (cm)", (950, 115), MUTED, self.small)
        for index, rect in enumerate(self.fields):
            pygame.draw.rect(self.screen, (12, 22, 35), rect, border_radius=4)
            pygame.draw.rect(self.screen, BLUE if index == self.focus else (65, 80, 99), rect, 2, border_radius=4)
            self.screen.set_clip(rect.inflate(-10, -4))
            self.text(self.texts[index] + ("|" if index == self.focus else ""), (rect.x+9, rect.y+7),
                      BLUE if index == self.focus and self.select_all else INK, self.value_font)
            self.screen.set_clip(None)
        self.button(self.move_button, "MOVE TO TARGET", not self.busy and self.pipeline is not None, True)
        self.button(self.box_button, "TARGET BOX CENTER", not self.busy and self.pipeline is not None)
        self.button(self.reset_button, "RESET")
        target_text = "--" if self.target is None else f"({self.target[0]:.2f}, {self.target[1]:.2f}) cm"
        self.text("Submitted: " + target_text, (764, 299), ORANGE, self.small)
        prediction = None if self.result is None else self.result.prediction
        self.text("ANN PREDICTION / degrees", (764, 334), MUTED, self.small)
        self.text("--" if prediction is None else f"t1 {prediction[0]:8.2f}   t2 {prediction[1]:8.2f}", (764, 359), BLUE, self.value_font)
        predicted_error = None if self.result is None else self.result.error_cm
        if predicted_error is not None:
            self.text(f"Predicted endpoint error: {predicted_error:.3f} cm", (764, 385), MUTED, self.small)
        self.text("CURRENT JOINTS / degrees", (764, 404), MUTED, self.small)
        self.text(f"t1 {self.current[0]:8.2f}   t2 {self.current[1]:8.2f}", (764, 429), INK, self.value_font)
        hand = None if self.pipeline is None else robot_points(self.pipeline[0], self.current)[2]
        self.text("END EFFECTOR / cm", (764, 474), MUTED, self.small)
        self.text("--" if hand is None else f"x {hand[0]:9.2f}    y {hand[1]:9.2f}", (764, 499), GREEN, self.value_font)
        error = None if hand is None or self.target is None else np.hypot(*(hand-self.target))
        self.text("CURRENT TARGET ERROR", (764, 542), MUTED, self.small)
        self.text("--" if error is None else f"{error:.3f} cm   (limit {SETTINGS.target_tolerance_cm:.2f})", (764, 567), ORANGE, self.value_font)
        mode = "--" if self.result is None else self.result.mode
        self.text("Planner: " + mode, (764, 610), GREEN)
        good = self.status in ("READY", "MOVING", "TARGET REACHED", "LOADING", "PLANNING")
        self.text(self.status, (764, 645), GREEN if good else RED)
        self.wrapped(self.message, (764, 678), 358, max_lines=4)
        reason = "" if self.result is None else self.result.direct_reason
        self.wrapped("Direct check: " + reason if reason else "45 deg/s per joint | 120 Hz simulation | frozen ANN", (764, 768), 358, max_lines=1)
        self.text("Click grid to move  |  Tab: switch X/Y  |  Enter: move  |  Esc: exit  |  Reset: restart scene", (24, 820), MUTED, self.small)
        pygame.display.flip()

    def run(self):
        clock = pygame.time.Clock()
        pygame.key.start_text_input()
        try:
            while self.running:
                seconds = clock.tick(60) / 1000.0
                for event in pygame.event.get():
                    self.handle_event(event)
                self.update(seconds)
                self.draw()
        finally:
            self.close()

    def close(self):
        self.running = False
        self.worker.shutdown(wait=False, cancel_futures=True)
        pygame.quit()


if __name__ == "__main__":
    Simulator().run()
