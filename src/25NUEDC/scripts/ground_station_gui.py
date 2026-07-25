#!/usr/bin/env python3
"""HDMI touch-screen ground station GUI for the 9x7 patrol task."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


WIDTH = 9
HEIGHT = 7
CELL_COUNT = WIDTH * HEIGHT
START_CELL = 8  # A9B1
BLOCKED_COUNT = 3
MAX_EXTRA_WAYPOINTS = 18
BEAM_WIDTH = 30000
NO_DIRECTION = 8
CPP_PLANNER_TIMEOUT_SEC = 180

ANIMALS = ("象", "虎", "狼", "猴", "孔雀")


def cell_id(x: int, y: int) -> int:
    return (y - 1) * WIDTH + (x - 1)


def cell_x(index: int) -> int:
    return index % WIDTH + 1


def cell_y(index: int) -> int:
    return index // WIDTH + 1


def cell_code(index: int) -> str:
    return f"A{cell_x(index)}B{cell_y(index)}"


def parse_cell_code(code: str) -> int:
    normalized = code.strip().upper()
    try:
        x_text, y_text = normalized[1:].split("B", 1)
        x = int(x_text)
        y = int(y_text)
    except (IndexError, ValueError) as error:
        raise ValueError(f"非法方格代码: {code}") from error
    if not normalized.startswith("A") or x < 1 or x > WIDTH or y < 1 or y > HEIGHT:
        raise ValueError(f"方格超出 9x7 范围: {code}")
    return cell_id(x, y)


def iter_bits(bits: int) -> Iterable[int]:
    while bits:
        lowest = bits & -bits
        yield lowest.bit_length() - 1
        bits ^= lowest


def neighbors(index: int) -> tuple[int, ...]:
    x = cell_x(index)
    y = cell_y(index)
    values = (
        index - 1 if x > 1 else -1,
        index + 1 if x < WIDTH else -1,
        index - WIDTH if y > 1 else -1,
        index + WIDTH if y < HEIGHT else -1,
        index - WIDTH - 1 if x > 1 and y > 1 else -1,
        index - WIDTH + 1 if x < WIDTH and y > 1 else -1,
        index + WIDTH - 1 if x > 1 and y < HEIGHT else -1,
        index + WIDTH + 1 if x < WIDTH and y < HEIGHT else -1,
    )
    return tuple(value for value in values if value >= 0)


def validate_blocked_ids(blocked_ids: Iterable[int]) -> int:
    ids = sorted(set(blocked_ids))
    if len(ids) != BLOCKED_COUNT:
        raise ValueError("请恰好选择 3 个禁飞格")
    if START_CELL in ids:
        raise ValueError("固定起点 A9B1 不能设为禁飞区")

    xs = {cell_x(index) for index in ids}
    ys = {cell_y(index) for index in ids}
    if len(xs) == 1:
        ordered = sorted(ys)
        if ordered[-1] - ordered[0] == 2 and len(ordered) == 3:
            return sum(1 << index for index in ids)
    if len(ys) == 1:
        ordered = sorted(xs)
        if ordered[-1] - ordered[0] == 2 and len(ordered) == 3:
            return sum(1 << index for index in ids)
    raise ValueError("3 个禁飞格必须横向或纵向连续")


def move_cost(from_cell: int, to_cell: int) -> int:
    diagonal = cell_x(from_cell) != cell_x(to_cell) and cell_y(from_cell) != cell_y(to_cell)
    return 1414 if diagonal else 1000


def direction_index(from_cell: int, to_cell: int) -> int:
    delta_x = cell_x(to_cell) - cell_x(from_cell)
    delta_y = cell_y(to_cell) - cell_y(from_cell)
    directions = {
        (1, 0): 0,
        (1, 1): 1,
        (0, 1): 2,
        (-1, 1): 3,
        (-1, 0): 4,
        (-1, -1): 5,
        (0, -1): 6,
        (1, -1): 7,
    }
    try:
        return directions[(delta_x, delta_y)]
    except KeyError as error:
        raise ValueError("路径中出现非相邻航点") from error


def turn_penalty(previous: int, next_direction: int) -> int:
    if previous == NO_DIRECTION:
        return 0
    difference = abs(previous - next_direction)
    turn_steps = min(difference, 8 - difference)
    return (0, 200, 500, 1000, 1000)[turn_steps]


@dataclass(slots=True)
class PlannerState:
    visited: int
    path: tuple[int, ...]
    distance: int
    turn_cost: int
    score: int
    current: int
    direction: int
    covered: int

    @property
    def total_cost(self) -> int:
        return self.distance + self.turn_cost


def theoretical_minimum_distance(blocked: int, free_count: int) -> int:
    start_color = (cell_x(START_CELL) + cell_y(START_CELL)) % 2
    start_color_cells = 0
    other_color_cells = 0
    for index in range(CELL_COUNT):
        if blocked & (1 << index):
            continue
        if (cell_x(index) + cell_y(index)) % 2 == start_color:
            start_color_cells += 1
        else:
            other_color_cells += 1

    minimum_diagonals = max(
        0,
        start_color_cells - other_color_cells - 1,
        other_color_cells - start_color_cells,
    )
    segment_count = free_count - 1
    return (segment_count - minimum_diagonals) * 1000 + minimum_diagonals * 1414


def evaluate_state(state: PlannerState, blocked: int, free_count: int) -> int:
    free_mask = ((1 << CELL_COUNT) - 1) & ~blocked
    unvisited = free_mask & ~state.visited
    if not unvisited:
        return sys.maxsize

    components = 0
    pending_cells = unvisited
    while pending_cells:
        seed = next(iter_bits(pending_cells))
        components += 1
        queue = [seed]
        pending_cells &= ~(1 << seed)
        while queue:
            value = queue.pop(0)
            for next_cell in neighbors(value):
                if pending_cells & (1 << next_cell):
                    pending_cells &= ~(1 << next_cell)
                    queue.append(next_cell)

    isolated = 0
    leaves = 0
    for value in iter_bits(unvisited):
        available_degree = 0
        for next_cell in neighbors(value):
            if (unvisited & (1 << next_cell)) or next_cell == state.current:
                available_degree += 1
        isolated += 1 if available_degree == 0 else 0
        leaves += 1 if available_degree == 1 else 0

    onward_degree = sum(1 for next_cell in neighbors(state.current) if unvisited & (1 << next_cell))
    remaining = free_count - state.covered
    cost_lower_bound = state.total_cost + remaining * 1000
    return (
        -cost_lower_bound * 100
        - components * 5000
        - isolated * 20000
        - leaves * 300
        - onward_degree * 20
    )


def check_route(route: list[int], blocked: int) -> None:
    if not route or route[0] != START_CELL:
        raise RuntimeError("路径起点不是 A9B1")
    covered = 0
    for index, value in enumerate(route):
        if blocked & (1 << value):
            raise RuntimeError(f"路径经过禁飞格 {cell_code(value)}")
        covered |= 1 << value
        if index > 0:
            previous = route[index - 1]
            delta_x = abs(cell_x(value) - cell_x(previous))
            delta_y = abs(cell_y(value) - cell_y(previous))
            if max(delta_x, delta_y) != 1:
                raise RuntimeError("路径中出现非相邻航点")
    free_count = CELL_COUNT - blocked.bit_count()
    if covered.bit_count() != free_count:
        raise RuntimeError("路径没有覆盖全部可飞格")


def plan_low_repeat(blocked: int) -> list[int]:
    free_count = CELL_COUNT - blocked.bit_count()
    lower_bound = theoretical_minimum_distance(blocked, free_count)
    initial = PlannerState(
        visited=1 << START_CELL,
        path=(START_CELL,),
        distance=0,
        turn_cost=0,
        score=0,
        current=START_CELL,
        direction=NO_DIRECTION,
        covered=1,
    )
    initial.score = evaluate_state(initial, blocked, free_count)

    beam = [initial]
    best: PlannerState | None = None
    max_length = free_count + MAX_EXTRA_WAYPOINTS
    for _target_length in range(2, max_length + 1):
        candidates: list[PlannerState] = []
        candidate_index: dict[tuple[int, int, int], int] = {}

        for state in beam:
            for next_cell in neighbors(state.current):
                if blocked & (1 << next_cell):
                    continue
                next_direction = direction_index(state.current, next_cell)
                bit = 1 << next_cell
                is_new = (state.visited & bit) == 0
                candidate = PlannerState(
                    visited=state.visited | bit,
                    path=state.path + (next_cell,),
                    distance=state.distance + move_cost(state.current, next_cell),
                    turn_cost=state.turn_cost + turn_penalty(state.direction, next_direction),
                    score=0,
                    current=next_cell,
                    direction=next_direction,
                    covered=state.covered + (1 if is_new else 0),
                )

                if candidate.covered == free_count:
                    if candidate.total_cost == lower_bound:
                        route = list(candidate.path)
                        check_route(route, blocked)
                        return route
                    if (
                        best is None
                        or candidate.total_cost < best.total_cost
                        or (
                            candidate.total_cost == best.total_cost
                            and candidate.distance < best.distance
                        )
                        or (
                            candidate.total_cost == best.total_cost
                            and candidate.distance == best.distance
                            and len(candidate.path) < len(best.path)
                        )
                    ):
                        best = candidate
                    continue

                optimistic_cost = candidate.total_cost + (free_count - candidate.covered) * 1000
                if best is not None and optimistic_cost >= best.total_cost:
                    continue

                key = (candidate.visited, candidate.current, candidate.direction)
                existing_index = candidate_index.get(key)
                if existing_index is not None:
                    retained = candidates[existing_index]
                    if (
                        candidate.total_cost < retained.total_cost
                        or (
                            candidate.total_cost == retained.total_cost
                            and candidate.distance < retained.distance
                        )
                    ):
                        candidate.score = evaluate_state(candidate, blocked, free_count)
                        candidates[existing_index] = candidate
                    continue

                candidate.score = evaluate_state(candidate, blocked, free_count)
                candidate_index[key] = len(candidates)
                candidates.append(candidate)

        if not candidates:
            break
        if len(candidates) > BEAM_WIDTH:
            candidates.sort(key=lambda item: item.score, reverse=True)
            candidates = candidates[:BEAM_WIDTH]
        beam = candidates

    if best is None:
        raise RuntimeError("在搜索上限内没有找到覆盖路径")
    route = list(best.path)
    check_route(route, blocked)
    return route


def plan_spanning_walk(blocked: int) -> list[int]:
    free_mask = ((1 << CELL_COUNT) - 1) & ~blocked
    free_count = free_mask.bit_count()
    if not (free_mask & (1 << START_CELL)):
        raise RuntimeError("起点被禁飞区占用")

    def available_degree(index: int, seen: set[int]) -> int:
        return sum(
            1
            for next_cell in neighbors(index)
            if (free_mask & (1 << next_cell)) and next_cell not in seen
        )

    visited = {START_CELL}
    route = [START_CELL]

    def visit(index: int) -> None:
        ordered = sorted(
            (
                next_cell
                for next_cell in neighbors(index)
                if (free_mask & (1 << next_cell)) and next_cell not in visited
            ),
            key=lambda item: (available_degree(item, visited), cell_y(item), -cell_x(item)),
        )
        for next_cell in ordered:
            if next_cell in visited:
                continue
            visited.add(next_cell)
            route.append(next_cell)
            visit(next_cell)
            if len(visited) < free_count:
                route.append(index)

    visit(START_CELL)
    if len(visited) != free_count:
        raise RuntimeError("可飞区域不连通，无法生成覆盖航线")
    check_route(route, blocked)
    return route


def plan_route_from_blocked(blocked: int) -> list[int]:
    codes = [cell_code(index) for index in iter_bits(blocked)]
    executable = find_cpp_planner()
    try:
        completed = subprocess.run(
            [str(executable), "--plan", *codes],
            check=False,
            capture_output=True,
            text=True,
            timeout=CPP_PLANNER_TIMEOUT_SEC,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"C++路径规划超过 {CPP_PLANNER_TIMEOUT_SEC} 秒，请稍后重试或调整禁飞区"
        ) from error
    if completed.returncode != 0:
        error_text = completed.stderr.strip() or completed.stdout.strip() or "未知错误"
        raise RuntimeError(f"C++路径规划失败：{error_text}")
    route_text_output = completed.stdout.strip().splitlines()[-1]
    route = [parse_cell_code(code) for code in route_text_output.split(",") if code.strip()]
    check_route(route, blocked)
    return route


def find_cpp_planner() -> Path:
    script_path = Path(__file__).resolve()
    candidates = [
        Path(sys.argv[0]).resolve().with_name("best_path_planner_node"),
        script_path.with_name("best_path_planner_node"),
    ]
    for parent in script_path.parents:
        candidates.extend(
            [
                parent / "build" / "nuedc25_ground_station" / "best_path_planner_node",
                parent / "install" / "nuedc25_ground_station" / "lib" /
                "nuedc25_ground_station" / "best_path_planner_node",
            ]
        )
    path_candidate = shutil.which("best_path_planner_node")
    if path_candidate:
        candidates.append(Path(path_candidate))

    for candidate in candidates:
        if candidate.is_file() and candidate.exists():
            return candidate
    raise RuntimeError("找不到 C++ 路径规划器，请先 colcon build --packages-select nuedc25_ground_station")


def route_text(route: list[int]) -> str:
    return ",".join(cell_code(index) for index in route)


def run_self_test(test_all: bool) -> int:
    cases: list[list[int]] = [
        [cell_id(1, 1), cell_id(2, 1), cell_id(3, 1)],
        [cell_id(4, 3), cell_id(5, 3), cell_id(6, 3)],
        [cell_id(7, 5), cell_id(7, 6), cell_id(7, 7)],
    ]
    if test_all:
        cases = []
        for y in range(1, HEIGHT + 1):
            for x in range(1, WIDTH - 1):
                ids = [cell_id(x, y), cell_id(x + 1, y), cell_id(x + 2, y)]
                if START_CELL not in ids:
                    cases.append(ids)
        for x in range(1, WIDTH + 1):
            for y in range(1, HEIGHT - 1):
                ids = [cell_id(x, y), cell_id(x, y + 1), cell_id(x, y + 2)]
                if START_CELL not in ids:
                    cases.append(ids)

    for ids in cases:
        blocked = validate_blocked_ids(ids)
        route = plan_route_from_blocked(blocked)
        check_route(route, blocked)
        print(
            f"PASS blocked={','.join(cell_code(index) for index in ids)} "
            f"waypoints={len(route)}"
        )
    return 0


def run_gui(windowed: bool, ros_args: list[str] | None = None) -> int:
    try:
        from PySide6.QtCore import QLineF, QObject, QProcess, QRectF, Qt, QTimer
        from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
        from PySide6.QtWidgets import (
            QApplication,
            QAbstractItemView,
            QFrame,
            QGraphicsEllipseItem,
            QGraphicsLineItem,
            QGraphicsRectItem,
            QGraphicsScene,
            QGraphicsSimpleTextItem,
            QGraphicsView,
            QGridLayout,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QPushButton,
            QStackedWidget,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as error:
        print("PySide6 未安装，请先安装 python3-pyside6 或 pip install PySide6", file=sys.stderr)
        print(error, file=sys.stderr)
        return 2

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String, UInt8MultiArray
    except ImportError as error:
        print("ROS 2 Python 依赖不可用，请先 source /opt/ros/humble/setup.bash", file=sys.stderr)
        print(error, file=sys.stderr)
        return 2

    class GridCellItem(QGraphicsRectItem):
        def __init__(self, index: int, rect: QRectF, callback: QObject):
            super().__init__(rect)
            self.index = index
            self.callback = callback
            self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)

        def mousePressEvent(self, event):  # noqa: N802 - Qt override
            self.callback(self.index)
            event.accept()

    class GroundStationRosNode(Node):
        def __init__(self, window: "GroundStationWindow"):
            super().__init__("ground_station_gui")
            self.window = window
            self.route_publisher = self.create_publisher(
                UInt8MultiArray, "/planned_route", 10
            )
            path_qos = QoSProfile(depth=1)
            path_qos.reliability = ReliabilityPolicy.RELIABLE
            path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.grid_path_publisher = self.create_publisher(
                String, "patrol_path_grid", path_qos
            )
            self.start_publisher = self.create_publisher(Bool, "/mission_start", 10)
            self.create_subscription(String, "/animal_detection", self.on_detection, 10)
            self.create_subscription(Bool, "/mission_finished", self.on_finished, 10)

        def publish_route(self, route: list[int]) -> None:
            message = UInt8MultiArray()
            message.data = route
            self.route_publisher.publish(message)

        def publish_grid_path(self, route: list[int]) -> None:
            message = String()
            message.data = " -> ".join(cell_code(index) for index in route)
            self.grid_path_publisher.publish(message)

        def publish_start(self) -> None:
            message = Bool()
            message.data = True
            self.start_publisher.publish(message)

        def on_detection(self, message: String) -> None:
            self.window.handle_detection(message.data)

        def on_finished(self, message: Bool) -> None:
            if message.data:
                self.window.handle_mission_finished()

    class GroundStationWindow(QMainWindow):
        CELL = 64
        LEFT_LABEL = 52
        TOP_LABEL = 28
        RIGHT_PAD = 24
        BOTTOM_LABEL = 42

        def __init__(self):
            super().__init__()
            self.setWindowTitle("Quadrotor Ground Station")
            self.ros_node: GroundStationRosNode | None = None
            self.blocked = [False] * CELL_COUNT
            self.route: list[int] = []
            self.records: list[dict[str, object]] = []
            self.animal_totals = {animal: 0 for animal in ANIMALS}
            self.mission_started = False
            self.result_path = Path.home() / ".quadrotor_ground_station" / "last_result.json"
            self.cell_items: dict[int, QGraphicsRectItem] = {}
            self.route_items: list[object] = []
            self.total_labels: dict[str, QLabel] = {}
            self.generate_route_button = None
            self.route_planner_process = None
            self.route_planner_timeout = QTimer(self)
            self.route_planner_timeout.setSingleShot(True)
            self.route_planner_timeout.timeout.connect(self.on_route_planner_timeout)

            self.stack = QStackedWidget()
            self.setCentralWidget(self.stack)
            self.map_page = self.build_map_page()
            self.patrol_page = self.build_patrol_page()
            self.result_page = self.build_result_page()
            self.stack.addWidget(self.map_page)
            self.stack.addWidget(self.patrol_page)
            self.stack.addWidget(self.result_page)

            self.load_last_result()
            self.update_result_page()
            self.set_status("请选择 3 个连续禁飞格")
            self.resize(1024, 600)

        def set_ros_node(self, node: GroundStationRosNode) -> None:
            self.ros_node = node

        def build_map_page(self) -> QWidget:
            page = QWidget()
            page_layout = QVBoxLayout(page)
            page_layout.setContentsMargins(16, 14, 16, 16)
            page_layout.setSpacing(10)

            map_title = QLabel("9×7 方格地图")
            map_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            map_title.setObjectName("MapTitleLabel")
            page_layout.addWidget(map_title)

            content = QHBoxLayout()
            content.setSpacing(18)
            page_layout.addLayout(content, 1)

            self.scene = QGraphicsScene()
            self.view = QGraphicsView(self.scene)
            self.view.setRenderHint(QPainter.Antialiasing)
            self.view.setFrameShape(QFrame.NoFrame)
            self.view.setMinimumSize(660, 500)
            content.addWidget(self.view, 1)

            side_panel = QWidget()
            side_panel.setFixedWidth(300)
            side = QVBoxLayout(side_panel)
            side.setContentsMargins(0, 70, 0, 0)
            side.setSpacing(16)

            for text, slot in (
                ("清空", self.clear_map),
                ("生成航线", self.generate_route),
                ("开始巡查", self.start_mission),
                ("查看结果", self.show_result_page),
                ("下一页", self.show_patrol_page),
            ):
                button = QPushButton(text)
                button.setMinimumHeight(72)
                button.setObjectName("MapButton")
                button.clicked.connect(slot)
                if text == "生成航线":
                    self.generate_route_button = button
                side.addWidget(button)

            self.status_label = QLabel()
            self.status_label.setWordWrap(True)
            self.status_label.setMinimumHeight(48)
            self.status_label.setObjectName("StatusLabel")
            page_layout.addWidget(self.status_label)
            side.addStretch(1)
            content.addWidget(side_panel, 0)

            self.draw_grid()
            return page

        def build_patrol_page(self) -> QWidget:
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.setContentsMargins(60, 34, 60, 34)
            layout.setSpacing(14)
            self.patrol_state = QLabel("任务状态：正在巡查")
            self.latest_grid = QLabel("方格代码：--")
            self.latest_animal = QLabel("动物名称：--")
            self.latest_count = QLabel("动物数量：--")
            for label in (
                self.patrol_state,
                self.latest_grid,
                self.latest_animal,
                self.latest_count,
            ):
                label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                label.setObjectName("LargeLabel")
                layout.addWidget(label)

            nav = QHBoxLayout()
            nav.setSpacing(18)
            previous_button = QPushButton("上一页")
            previous_button.setMinimumHeight(54)
            previous_button.clicked.connect(self.show_map_page)
            next_button = QPushButton("下一页")
            next_button.setMinimumHeight(54)
            next_button.clicked.connect(self.show_result_page)
            nav.addWidget(previous_button)
            nav.addWidget(next_button)
            layout.addLayout(nav)
            return page

        def build_result_page(self) -> QWidget:
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.setContentsMargins(34, 28, 34, 28)
            layout.setSpacing(14)

            title = QLabel("动物总数")
            title.setObjectName("SectionLabel")
            layout.addWidget(title)

            totals_layout = QGridLayout()
            totals_layout.setHorizontalSpacing(24)
            totals_layout.setVerticalSpacing(12)
            for index, animal in enumerate(ANIMALS):
                label = QLabel()
                label.setObjectName("TotalLabel")
                self.total_labels[animal] = label
                totals_layout.addWidget(label, index // 3, index % 3)
            layout.addLayout(totals_layout)

            records_title = QLabel("识别记录")
            records_title.setObjectName("SectionLabel")
            layout.addWidget(records_title)

            self.record_table = QTableWidget(0, 4)
            self.record_table.setHorizontalHeaderLabels(("序号", "方格代码", "动物名称", "动物数量"))
            self.record_table.verticalHeader().setVisible(False)
            self.record_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
            self.record_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
            self.record_table.horizontalHeader().setStretchLastSection(True)
            layout.addWidget(self.record_table, 1)

            back = QPushButton("上一页")
            back.setMinimumHeight(58)
            back.clicked.connect(self.show_patrol_page)
            layout.addWidget(back)
            return page

        def draw_grid(self) -> None:
            self.scene.clear()
            self.cell_items.clear()
            self.route_items.clear()
            font = QFont()
            font.setPointSize(12)
            label_pen = QPen(QColor("#243447"))

            for col in range(WIDTH):
                text = QGraphicsSimpleTextItem(f"A{col + 1}")
                text.setFont(font)
                text.setBrush(QBrush(QColor("#243447")))
                text.setPos(
                    self.LEFT_LABEL + col * self.CELL + self.CELL * 0.28,
                    self.TOP_LABEL + HEIGHT * self.CELL + 8,
                )
                self.scene.addItem(text)
            for visual_row in range(HEIGHT):
                text = QGraphicsSimpleTextItem(f"B{HEIGHT - visual_row}")
                text.setFont(font)
                text.setBrush(QBrush(QColor("#243447")))
                text.setPos(8, self.TOP_LABEL + visual_row * self.CELL + self.CELL * 0.30)
                self.scene.addItem(text)

            for visual_row in range(HEIGHT):
                logical_y = HEIGHT - visual_row
                for col in range(WIDTH):
                    index = cell_id(col + 1, logical_y)
                    rect = QRectF(
                        self.LEFT_LABEL + col * self.CELL,
                        self.TOP_LABEL + visual_row * self.CELL,
                        self.CELL - 3,
                        self.CELL - 3,
                    )
                    item = GridCellItem(index, rect, self.on_cell_clicked)
                    item.setPen(QPen(QColor("#263238"), 1.4))
                    self.scene.addItem(item)
                    self.cell_items[index] = item
                    if index == START_CELL:
                        start_text = QGraphicsSimpleTextItem("A9B1")
                        start_text.setFont(font)
                        start_text.setBrush(QBrush(QColor("#0f3d1e")))
                        start_text.setPos(rect.x() + 12, rect.y() + 22)
                        self.scene.addItem(start_text)

            scene_width = self.LEFT_LABEL + WIDTH * self.CELL + self.RIGHT_PAD
            scene_height = self.TOP_LABEL + HEIGHT * self.CELL + self.BOTTOM_LABEL
            self.scene.setSceneRect(0, 0, scene_width, scene_height)
            self.update_grid_colors()

        def resizeEvent(self, event):  # noqa: N802 - Qt override
            super().resizeEvent(event)
            self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

        def update_grid_colors(self) -> None:
            for index, item in self.cell_items.items():
                if index == START_CELL:
                    item.setBrush(QBrush(QColor("#8fd694")))
                elif self.blocked[index]:
                    item.setBrush(QBrush(QColor("#8a949e")))
                else:
                    item.setBrush(QBrush(QColor("#f7fafc")))

        def center_of(self, index: int):
            row = HEIGHT - cell_y(index)
            col = index % WIDTH
            return (
                self.LEFT_LABEL + col * self.CELL + (self.CELL - 3) / 2,
                self.TOP_LABEL + row * self.CELL + (self.CELL - 3) / 2,
            )

        def clear_route_items(self) -> None:
            for item in self.route_items:
                self.scene.removeItem(item)
            self.route_items.clear()

        def draw_route(self) -> None:
            self.clear_route_items()
            if len(self.route) < 2:
                return
            line_pen = QPen(QColor("#1976d2"), 5)
            line_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            line_pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            point_pen = QPen(QColor("#0d47a1"), 1)
            point_brush = QBrush(QColor("#42a5f5"))
            for previous, current in zip(self.route, self.route[1:]):
                x1, y1 = self.center_of(previous)
                x2, y2 = self.center_of(current)
                line = QGraphicsLineItem(QLineF(x1, y1, x2, y2))
                line.setPen(line_pen)
                self.scene.addItem(line)
                self.route_items.append(line)
            for index in self.route:
                x, y = self.center_of(index)
                point = QGraphicsEllipseItem(x - 4, y - 4, 8, 8)
                point.setPen(point_pen)
                point.setBrush(point_brush)
                self.scene.addItem(point)
                self.route_items.append(point)

        def on_cell_clicked(self, index: int) -> None:
            if self.is_route_planning():
                self.set_status("航线正在生成，暂不能修改禁飞区")
                return
            if index == START_CELL:
                self.set_status("A9B1 是固定起点，不能设为禁飞区")
                return
            self.blocked[index] = not self.blocked[index]
            self.route = []
            self.mission_started = False
            self.clear_route_items()
            self.update_grid_colors()
            selected = [cell_code(i) for i, value in enumerate(self.blocked) if value]
            if selected:
                self.set_status(f"已选择：{', '.join(selected)}")
            else:
                self.set_status("请选择 3 个连续禁飞格")

        def clear_map(self) -> None:
            if self.is_route_planning():
                self.set_status("航线正在生成，暂不能清空")
                return
            self.blocked = [False] * CELL_COUNT
            self.route = []
            self.mission_started = False
            self.clear_route_items()
            self.update_grid_colors()
            self.set_status("已清空，请选择 3 个连续禁飞格")

        def generate_route(self) -> None:
            if self.is_route_planning():
                self.set_status("航线正在生成，请稍候")
                return
            blocked_ids = [index for index, value in enumerate(self.blocked) if value]
            try:
                blocked = validate_blocked_ids(blocked_ids)
                executable = find_cpp_planner()
            except Exception as error:  # noqa: BLE001 - show validation/planner failures to operator
                self.route = []
                self.clear_route_items()
                self.set_status(str(error))
                return

            self.mission_started = False
            self.route = []
            self.clear_route_items()
            process = QProcess(self)
            process.setProgram(str(executable))
            process.setArguments(["--plan", *[cell_code(index) for index in iter_bits(blocked)]])
            process.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
            process.finished.connect(
                lambda exit_code, exit_status: self.on_route_planner_finished(
                    process, blocked, exit_code, exit_status
                )
            )
            process.errorOccurred.connect(
                lambda error: self.on_route_planner_error(process, error)
            )
            self.route_planner_process = process
            if self.generate_route_button is not None:
                self.generate_route_button.setEnabled(False)
            self.set_status("正在调用 C++ 生成航线，请稍候")
            self.route_planner_timeout.start(CPP_PLANNER_TIMEOUT_SEC * 1000)
            process.start()

        def start_mission(self) -> None:
            if self.is_route_planning():
                self.set_status("航线正在生成，请稍候")
                return
            if not self.route:
                self.set_status("请先生成航线")
                return
            if self.mission_started:
                self.set_status("巡查已启动")
                return
            if self.ros_node is None:
                self.set_status("ROS 节点未就绪")
                return
            self.records = []
            self.animal_totals = {animal: 0 for animal in ANIMALS}
            self.update_result_page()
            self.ros_node.publish_route(self.route)
            self.ros_node.publish_grid_path(self.route)
            self.ros_node.publish_start()
            self.mission_started = True
            self.patrol_state.setText("任务状态：正在巡查")
            self.latest_grid.setText("方格代码：--")
            self.latest_animal.setText("动物名称：--")
            self.latest_count.setText("动物数量：--")
            self.stack.setCurrentWidget(self.patrol_page)

        def is_route_planning(self) -> bool:
            return (
                self.route_planner_process is not None
                and self.route_planner_process.state() != QProcess.ProcessState.NotRunning
            )

        def cleanup_route_planner(self, process) -> None:
            if process is self.route_planner_process:
                self.route_planner_process = None
                self.route_planner_timeout.stop()
                if self.generate_route_button is not None:
                    self.generate_route_button.setEnabled(True)
            process.deleteLater()

        def on_route_planner_timeout(self) -> None:
            process = self.route_planner_process
            if process is None:
                return
            process.kill()
            self.cleanup_route_planner(process)
            self.set_status(f"C++路径规划超过 {CPP_PLANNER_TIMEOUT_SEC} 秒，已停止")

        def on_route_planner_error(self, process, error) -> None:
            if process is not self.route_planner_process:
                return
            self.cleanup_route_planner(process)
            self.route = []
            self.clear_route_items()
            error_name = getattr(error, "name", str(error))
            self.set_status(f"C++路径规划启动失败：{error_name}")

        def on_route_planner_finished(self, process, blocked: int, exit_code: int, exit_status) -> None:
            if process is not self.route_planner_process:
                return
            stdout = bytes(process.readAllStandardOutput()).decode(errors="replace").strip()
            stderr = bytes(process.readAllStandardError()).decode(errors="replace").strip()
            self.cleanup_route_planner(process)
            if exit_status != QProcess.ExitStatus.NormalExit or exit_code != 0:
                self.route = []
                self.clear_route_items()
                self.set_status(f"C++路径规划失败：{stderr or stdout or '未知错误'}")
                return
            try:
                route_line = stdout.splitlines()[-1]
                route = [parse_cell_code(code) for code in route_line.split(",") if code.strip()]
                check_route(route, blocked)
            except Exception as error:  # noqa: BLE001 - show planner output failures to operator
                self.route = []
                self.clear_route_items()
                self.set_status(f"C++路径结果无效：{error}")
                return
            self.route = route
            self.mission_started = False
            self.draw_route()
            self.set_status(f"航线已生成，共 {len(self.route)} 个航点")

        def handle_detection(self, text: str) -> None:
            parts = [part.strip() for part in text.split(",")]
            if len(parts) != 3:
                self.set_status(f"识别消息格式错误：{text}")
                return
            grid_code, animal, count_text = parts
            try:
                count = int(count_text)
            except ValueError:
                self.set_status(f"识别数量不是整数：{text}")
                return
            record = {"grid": grid_code, "animal": animal, "count": count}
            self.records.append(record)
            if animal in self.animal_totals:
                self.animal_totals[animal] += count
            self.latest_grid.setText(f"方格代码：{grid_code}")
            self.latest_animal.setText(f"动物名称：{animal}")
            self.latest_count.setText(f"动物数量：{count}")

        def handle_mission_finished(self) -> None:
            self.patrol_state.setText("任务状态：巡查完成")
            self.save_last_result()
            self.update_result_page()
            self.stack.setCurrentWidget(self.result_page)
            self.set_status("巡查完成，结果已保存")

        def show_map_page(self) -> None:
            self.stack.setCurrentWidget(self.map_page)

        def show_patrol_page(self) -> None:
            self.stack.setCurrentWidget(self.patrol_page)

        def show_result_page(self) -> None:
            self.update_result_page()
            self.stack.setCurrentWidget(self.result_page)

        def set_status(self, text: str) -> None:
            self.status_label.setText(f"状态：{text}")

        def load_last_result(self) -> None:
            if not self.result_path.exists():
                return
            try:
                with self.result_path.open("r", encoding="utf-8") as stream:
                    data = json.load(stream)
            except (OSError, json.JSONDecodeError):
                return
            records = data.get("records", [])
            totals = data.get("animal_totals", {})
            if isinstance(records, list):
                self.records = [
                    {
                        "grid": str(item.get("grid", "")),
                        "animal": str(item.get("animal", "")),
                        "count": int(item.get("count", 0)),
                    }
                    for item in records
                    if isinstance(item, dict)
                ]
            if isinstance(totals, dict):
                self.animal_totals = {
                    animal: int(totals.get(animal, 0)) for animal in ANIMALS
                }

        def save_last_result(self) -> None:
            self.result_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "records": self.records,
                "animal_totals": self.animal_totals,
                "finished_at": datetime.now(timezone.utc).isoformat(),
            }
            with self.result_path.open("w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)

        def update_result_page(self) -> None:
            for animal in ANIMALS:
                self.total_labels[animal].setText(f"{animal}：{self.animal_totals.get(animal, 0)}")
            self.record_table.setRowCount(len(self.records))
            for row, record in enumerate(self.records):
                values = (
                    str(row + 1),
                    str(record.get("grid", "")),
                    str(record.get("animal", "")),
                    str(record.get("count", "")),
                )
                for column, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.record_table.setItem(row, column, item)
            self.record_table.resizeColumnsToContents()

    app = QApplication(sys.argv[:1])
    app.setStyleSheet(
        """
        QWidget {
            background: #eef3f7;
            color: #17212b;
            font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
            font-size: 20px;
        }
        QPushButton {
            background: #ffffff;
            border: 2px solid #8ea1b2;
            border-radius: 8px;
            padding: 10px 18px;
            font-size: 22px;
        }
        QPushButton:pressed {
            background: #d8e8f6;
            border-color: #1976d2;
        }
        QGraphicsView {
            background: #ffffff;
            border-radius: 8px;
        }
        QLabel#TitleLabel {
            font-size: 34px;
            font-weight: 700;
        }
        QLabel#MapTitleLabel {
            font-size: 32px;
            font-weight: 700;
            background: transparent;
        }
        QLabel#StatusLabel {
            color: #243447;
            font-size: 24px;
            font-weight: 700;
            background: transparent;
        }
        QPushButton#MapButton {
            font-size: 28px;
            font-weight: 700;
            padding: 12px 20px;
        }
        QLabel#LargeLabel {
            background: #ffffff;
            border: 2px solid #a7b6c2;
            border-radius: 8px;
            padding: 18px;
            font-size: 40px;
            font-weight: 700;
        }
        QLabel#SectionLabel {
            font-size: 28px;
            font-weight: 700;
            background: transparent;
        }
        QLabel#TotalLabel {
            background: #ffffff;
            border: 2px solid #a7b6c2;
            border-radius: 8px;
            padding: 14px;
            font-size: 24px;
        }
        QTableWidget {
            background: #ffffff;
            gridline-color: #b7c4ce;
            selection-background-color: #d8e8f6;
        }
        QHeaderView::section {
            background: #dbe5ec;
            padding: 8px;
            border: 1px solid #b7c4ce;
            font-weight: 700;
        }
        """
    )

    rclpy.init(args=ros_args)
    window = GroundStationWindow()
    node = GroundStationRosNode(window)
    window.set_ros_node(node)

    spin_timer = QTimer()

    def spin_ros_once() -> None:
        if not rclpy.ok():
            spin_timer.stop()
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except KeyboardInterrupt:
            spin_timer.stop()
            app.quit()
        except Exception as error:  # noqa: BLE001 - keep Qt event loop from flooding tracebacks
            if rclpy.ok():
                print(f"ROS spin error: {error}", file=sys.stderr)
            spin_timer.stop()
            app.quit()

    spin_timer.timeout.connect(spin_ros_once)
    spin_timer.start(20)

    if windowed:
        window.show()
    else:
        window.showFullScreen()

    try:
        return app.exec()
    except KeyboardInterrupt:
        return 130
    finally:
        spin_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--windowed", action="store_true", help="use a normal window instead of fullscreen")
    parser.add_argument("--self-test", action="store_true", help="run a few planner checks without Qt/ROS")
    parser.add_argument("--self-test-all", action="store_true", help="run planner checks for all legal 3-cell blocks")
    args, ros_args = parser.parse_known_args()
    if args.self_test or args.self_test_all:
        return run_self_test(args.self_test_all)
    return run_gui(args.windowed, ros_args)


if __name__ == "__main__":
    raise SystemExit(main())
