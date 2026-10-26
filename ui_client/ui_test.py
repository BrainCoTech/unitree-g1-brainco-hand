import sys
import time
import json
import asyncio
import threading
import math
import traceback
import ipaddress
import re
from functools import partial
from enum import Enum, auto
from pathlib import Path

import paramiko

try:
    import yaml
except ImportError:
    yaml = None

from PySide6.QtWidgets import (
    QApplication, QWidget, QTextEdit, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox, QLineEdit, QLabel,
    QSplitter, QTabWidget, QComboBox, QDialog, QDialogButtonBox, QFormLayout,
)
from PySide6.QtCore import QObject, Signal, Slot, QPoint, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygon, QTextCursor

from robot_agent.robot_client import RobotClient
from robot_agent.registry import CommandSpec
import gui_start as gui_start_module
from gui_start import (
    CONFIG_PATH as GUI_CONFIG_PATH,
    PASSWORD as SSH_PASSWORD,
    USERNAME as SSH_USERNAME,
    WIRED_HOST as SSH_WIRED_HOST,
    WIRELESS_HOST as SSH_WIRELESS_HOST,
)


SSH_TIMEOUT = 10


def _default_config_path() -> Path:
    # Prefer an external config next to the packaged exe.
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().with_name("client_config.yaml")
    return Path(__file__).resolve().with_name("client_config.yaml")


CONFIG_PATH = _default_config_path()
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT1 = 43210
DEFAULT_PORT2 = 46000
DEFAULT_THEME = {
    "window_bg": "#0b0f14",
    "panel_bg": "#11161d",
    "panel_alt_bg": "#151c24",
    "group_bg": "#121922",
    "border": "#273241",
    "border_strong": "#3a4758",
    "text_primary": "#edf2f7",
    "text_muted": "#91a0b3",
    "input_bg": "#0f141b",
    "input_border": "#2a3442",
    "input_focus_border": "#d7a64a",
    "button_bg": "#1a2330",
    "button_hover_bg": "#243142",
    "button_pressed_bg": "#121a24",
    "button_disabled_bg": "#0f141b",
    "button_disabled_text": "#5f6c7b",
    "button_reachable_text": "#11161d",
    "action_idle_bg": "#18212d",
    "action_active_bg": "#d7a64a",
    "action_inactive_bg": "#10161d",
    "action_active_text": "#0b0f14",
    "action_idle_text": "#edf2f7",
    "log_bg": "#0d1218",
    "log_border": "#23303d",
    "selection_bg": "#355070",
}
DEFAULT_STATE_GRAPH = {
    "initial_current_state": "init_state",
    "colors": {
        "active": "#d7a64a",
        "reachable": "#eeeeee",
        "inactive": "#10161d",
        "accent": "#edf2f7",
    },
    "transitions": {
        "init": {
            "config": "f",
        },
        "config": {
            "active": "a",
            "shutdown": "s",
        },
        "active": {
            "config": "d",
            "shutdown": "s",
        },
        "shutdown": {},
    },
    "nodes": {
        "init": {
            "title": "Init",
            "subtitle": "init_state",
            "states": ["init_state"],
            "allowed_next": ["config"],
            "position": [0, 0],
        },
        "config": {
            "title": "Config",
            "subtitle": "config_state",
            "states": ["config_state"],
            "allowed_next": ["active", "shutdown"],
            "position": [0, 1],
        },
        "active": {
            "title": "Active",
            "subtitle": "any Action state",
            "match": "unmapped",
            "allowed_next": ["config", "shutdown"],
            "position": [1, 1],
        },
        "shutdown": {
            "title": "Shutdown",
            "subtitle": "finalized",
            "states": ["finalized"],
            "allowed_next": [],
            "position": [1, 0],
        },
    },
}


def _parse_simple_yaml(text: str) -> dict:
    config = {}

    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue

        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip("'\"")

        if not key:
            continue

        if value.lower() in {"true", "false"}:
            config[key] = value.lower() == "true"
            continue

        try:
            config[key] = int(value)
            continue
        except ValueError:
            pass

        config[key] = value

    return config


def _load_gui_named_mapping(section_name: str) -> dict[str, str]:
    if not GUI_CONFIG_PATH.exists():
        return {}

    try:
        text = GUI_CONFIG_PATH.read_text(encoding="utf-8")
        if yaml is not None:
            config = yaml.safe_load(text) or {}
        else:
            config = {}
            in_section = False
            section_values = {}
            for raw_line in text.splitlines():
                stripped = raw_line.strip()
                if stripped == f"{section_name}:":
                    in_section = True
                    continue
                if in_section and stripped and not raw_line.startswith(" "):
                    break
                if not in_section:
                    continue

                match = re.match(r"^-\s*([^:]+)\s*:\s*(.*?)\s*$", stripped)
                if match:
                    key = match.group(1).strip().strip("'\"")
                    section_values[key] = match.group(2).strip().strip("'\"")
            return section_values
    except Exception:
        return {}

    if not isinstance(config, dict):
        return {}

    raw_values = config.get(section_name, {})
    if isinstance(raw_values, dict):
        items = raw_values.items()
    elif isinstance(raw_values, list):
        items = (
            pair
            for item in raw_values
            if isinstance(item, dict)
            for pair in item.items()
        )
    else:
        return {}

    section_values = {}
    for key, value in items:
        section_values[str(key).strip()] = str(value).strip()
    return section_values


def _load_gui_action_names() -> dict[int, str]:
    action_names = {}
    for key, value in _load_gui_named_mapping("action_name").items():
        try:
            action_names[int(key)] = value
        except ValueError:
            continue
    return action_names


def _load_gui_state_names() -> dict[str, str]:
    return _load_gui_named_mapping("state_name")


def _merge_dict(default: dict, override: dict) -> dict:
    result = dict(default)

    for key, value in override.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            result[key] = _merge_dict(result[key], value)
        else:
            result[key] = value

    return result


def load_client_config() -> tuple[str, int, int, dict, dict]:
    host = DEFAULT_HOST
    port1 = DEFAULT_PORT1
    port2 = DEFAULT_PORT2
    state_graph = DEFAULT_STATE_GRAPH
    theme = DEFAULT_THEME

    if not CONFIG_PATH.exists():
        return host, port1, port2, state_graph, theme

    try:
        text = CONFIG_PATH.read_text(encoding="utf-8")
        if yaml is not None:
            data = yaml.safe_load(text) or {}
        else:
            data = _parse_simple_yaml(text)
    except Exception:
        return host, port1, port2, state_graph, theme

    if not isinstance(data, dict):
        return host, port1, port2, state_graph, theme

    host = str(data.get("host", host)).strip() or host

    try:
        port1 = int(data.get("port1", port1))
    except (TypeError, ValueError):
        port1 = DEFAULT_PORT1

    try:
        port2 = int(data.get("port2", port2))
    except (TypeError, ValueError):
        port2 = DEFAULT_PORT2

    state_graph_cfg = data.get("state_graph", {})
    if isinstance(state_graph_cfg, dict):
        state_graph_cfg = {
            key: value for key, value in state_graph_cfg.items() if key != "colors"
        }
        state_graph = _merge_dict(DEFAULT_STATE_GRAPH, state_graph_cfg)

    return host, port1, port2, state_graph, theme


# =========================
# 日志类型
# =========================
class LogType(Enum):
    COMMAND_LIST = auto()
    EXECUTION = auto()


# =========================
# Qt 信号桥
# =========================
class UiBridge(QObject):
    log = Signal(int, LogType, str)
    startup_log = Signal(object, str)
    session_result = Signal(int, bool, str)
    robot_detection = Signal(int)
    enable_startup_connect = Signal(bool)
    enable_startup_disconnect = Signal(bool)
    enable_startup_safe = Signal(bool)
    enable_startup_run = Signal(bool)
    set_button = Signal(int, int, str)
    clear_buttons = Signal(int)
    set_current_state = Signal(str)
    set_port1_graph_mode = Signal(bool)
    refresh_action_buttons = Signal()


class StateGraphWidget(QWidget):

    def __init__(
        self,
        state_graph_config: dict,
        theme: dict,
        state_names: dict[str, str],
        on_node_clicked,
    ):

        super().__init__()

        self.state_graph_config = state_graph_config
        self.theme = theme
        self.state_names = state_names
        self.on_node_clicked = on_node_clicked
        self.node_buttons = {}
        self.node_aux_buttons = {}
        self.active_node = None
        self.allowed_next_set = set()

        self.setMinimumHeight(160)
        self.setAutoFillBackground(True)
        self._build_buttons()

    def _build_buttons(self):

        nodes = self.state_graph_config.get("nodes", {})

        for node_key, node_cfg in nodes.items():
            if not isinstance(node_cfg, dict):
                continue

            state_name_key = {
                "init": "Init",
                "config": "Ready",
                "active": "Active",
                "shutdown": "Shutdown",
            }.get(node_key)
            title = str(
                self.state_names.get(
                    state_name_key,
                    node_cfg.get("title", node_key),
                )
            )
            button = QPushButton(title, self)
            button.clicked.connect(
                lambda _checked=False, key=node_key: self.on_node_clicked(key)
            )
            button.setCheckable(False)
            self.node_buttons[node_key] = button

            if node_key == "config":
                table_safe_name = self.state_names.get("Table_Safe", "Table Safe")
                table_safe_name = table_safe_name.replace("安全回零", "安全\n回零")
                aux_button = QPushButton(table_safe_name, self)
                aux_button.clicked.connect(
                    lambda _checked=False, key=node_key: self.on_node_clicked(
                        key,
                        "safe_table",
                    )
                )
                aux_button.setCheckable(False)
                self.node_aux_buttons[node_key] = aux_button

    def set_graph_state(self, active_node: str | None, allowed_next_set: set[str]):

        self.active_node = active_node
        self.allowed_next_set = set(allowed_next_set)

        colors = self.state_graph_config.get("colors", {})
        active_color = str(colors.get("active", DEFAULT_STATE_GRAPH["colors"]["active"]))
        reachable_color = str(
            colors.get("reachable", DEFAULT_STATE_GRAPH["colors"]["reachable"])
        )
        inactive_color = str(
            colors.get("inactive", DEFAULT_STATE_GRAPH["colors"]["inactive"])
        )
        accent_color = str(colors.get("accent", DEFAULT_STATE_GRAPH["colors"]["accent"]))
        muted_text = str(self.theme.get("text_muted", DEFAULT_THEME["text_muted"]))
        primary_text = str(self.theme.get("text_primary", DEFAULT_THEME["text_primary"]))
        active_text = str(
            self.theme.get("action_active_text", DEFAULT_THEME["action_active_text"])
        )
        idle_border = str(self.theme.get("border", DEFAULT_THEME["border"]))
        disabled_bg = str(
            self.theme.get("button_disabled_bg", DEFAULT_THEME["button_disabled_bg"])
        )
        disabled_text = str(
            self.theme.get("button_disabled_text", DEFAULT_THEME["button_disabled_text"])
        )
        reachable_text = str(
            self.theme.get("button_reachable_text", DEFAULT_THEME["button_reachable_text"])
        )

        for node_key, button in self.node_buttons.items():
            if node_key == self.active_node:
                bg_color = active_color
                enabled = False
                border_color = accent_color
                text_color = active_text
            elif node_key in self.allowed_next_set:
                bg_color = reachable_color
                enabled = True
                border_color = idle_border
                text_color = reachable_text
            else:
                enabled = False
                bg_color = disabled_bg or inactive_color
                border_color = idle_border
                text_color = disabled_text or muted_text

            button_text_style = "text-align: center;"
            if node_key == "config":
                button_text_style = "text-align: left; padding-left: 30px;"

            button.setEnabled(enabled)
            button.setStyleSheet(
                f"border: 2px solid {border_color}; "
                "border-radius: 8px; "
                "padding: 3px 8px; "
                f"{button_text_style} "
                f"color: {text_color}; "
                f"background-color: {bg_color};"
            )

            aux_button = self.node_aux_buttons.get(node_key)
            if aux_button is not None:
                aux_button.setEnabled(enabled)
                aux_button.setStyleSheet(
                    f"border: 2px solid {border_color}; "
                    "border-radius: 8px; "
                    "padding: 2px 4px; "
                    "text-align: center; "
                    "font-size: 11px; "
                    f"color: {text_color}; "
                    f"background-color: {bg_color};"
                )

        self.update()

    def resizeEvent(self, event):

        super().resizeEvent(event)

        self._layout_buttons()

    def _layout_buttons(self):

        nodes = self.state_graph_config.get("nodes", {})
        if not nodes:
            return

        margin_x = 12
        margin_y = 10
        gap_x = 42
        gap_y = 34
        button_h = 48

        available_w = max(120, self.width() - (2 * margin_x) - gap_x)
        cell_w = max(72, available_w // 2)
        left_button_w = min(92, cell_w)
        right_total_w = min(124, cell_w)
        cell_h = button_h

        for node_key, node_cfg in nodes.items():
            if not isinstance(node_cfg, dict):
                continue

            position = node_cfg.get("position", [0, 0])
            try:
                row = int(position[0])
                col = int(position[1])
            except (TypeError, ValueError, IndexError):
                row = 0
                col = 0

            cell_x = margin_x + col * (cell_w + gap_x)
            cell_y = margin_y + row * (cell_h + gap_y)
            has_aux_button = node_key in self.node_aux_buttons
            is_right_column = col == 1
            current_button_w = right_total_w if is_right_column else left_button_w
            current_aux_w = 0
            aux_overlap = 0
            aux_height = button_h
            aux_y = y = cell_y
            if has_aux_button:
                current_aux_w = 40
                aux_overlap = current_aux_w + 4
                aux_height = 40

            total_w = current_button_w
            x = cell_x + max(0, (cell_w - total_w) // 2)
            x = max(margin_x, min(x, self.width() - margin_x - total_w))
            y = cell_y
            aux_y = y + max(0, (button_h - aux_height) // 2)

            button = self.node_buttons.get(node_key)
            if button is not None:
                button.setGeometry(x, y, current_button_w, button_h)

            aux_button = self.node_aux_buttons.get(node_key)
            if aux_button is not None:
                aux_button.setGeometry(
                    x + current_button_w - aux_overlap,
                    aux_y,
                    current_aux_w,
                    aux_height,
                )
                aux_button.raise_()

    def paintEvent(self, event):

        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        transitions = self.state_graph_config.get("transitions", {})

        for src_node, dst_map in transitions.items():
            if not isinstance(dst_map, dict):
                continue

            for dst_node in dst_map.keys():
                self._draw_transition(painter, str(src_node), str(dst_node))

    def _draw_transition(self, painter: QPainter, src_node: str, dst_node: str):

        src_rect = self._node_geometry(src_node)
        dst_rect = self._node_geometry(dst_node)
        if src_rect is None or dst_rect is None:
            return

        start, end = self._connection_points(src_rect, dst_rect)
        start, end = self._adjust_transition_endpoints(src_node, dst_node, start, end)
        if start == end:
            return

        is_active = src_node == self.active_node and dst_node in self.allowed_next_set
        colors = self.state_graph_config.get("colors", {})
        accent_color = str(colors.get("accent", DEFAULT_STATE_GRAPH["colors"]["accent"]))
        line_color = QColor(accent_color) if is_active else QColor(
            str(self.theme.get("border_strong", DEFAULT_THEME["border_strong"]))
        )
        pen = QPen(line_color, 2.4 if is_active else 1.8)
        painter.setPen(pen)
        painter.drawLine(start, end)
        self._draw_arrow_head(painter, start, end, line_color)

    def _node_geometry(self, node_key: str):

        button = self.node_buttons.get(node_key)
        if button is None:
            return None

        rect = button.geometry()
        aux_button = self.node_aux_buttons.get(node_key)
        if aux_button is not None:
            rect = rect.united(aux_button.geometry())

        return rect

    @staticmethod
    def _adjust_transition_endpoints(
        src_node: str,
        dst_node: str,
        start: QPoint,
        end: QPoint,
    ) -> tuple[QPoint, QPoint]:

        if src_node == "config" and dst_node == "shutdown":
            return (
                QPoint(start.x(), start.y() + 10),
                QPoint(end.x(), end.y() - 10),
            )

        if src_node == "config" and dst_node == "active":
            return (
                QPoint(start.x() - 10, start.y()),
                QPoint(end.x() - 10, end.y()),
            )

        if src_node == "active" and dst_node == "config":
            return (
                QPoint(start.x() + 10, start.y()),
                QPoint(end.x() + 10, end.y()),
            )

        return start, end

    @staticmethod
    def _connection_points(src_rect, dst_rect) -> tuple[QPoint, QPoint]:

        src_center = src_rect.center()
        dst_center = dst_rect.center()
        dx = dst_center.x() - src_center.x()
        dy = dst_center.y() - src_center.y()
        gap = 8

        if abs(dx) >= abs(dy):
            if dx >= 0:
                start = QPoint(src_rect.right() + gap, src_center.y())
                end = QPoint(dst_rect.left() - gap, dst_center.y())
            else:
                start = QPoint(src_rect.left() - gap, src_center.y())
                end = QPoint(dst_rect.right() + gap, dst_center.y())
        else:
            if dy >= 0:
                start = QPoint(src_center.x(), src_rect.bottom() + gap)
                end = QPoint(dst_center.x(), dst_rect.top() - gap)
            else:
                start = QPoint(src_center.x(), src_rect.top() - gap)
                end = QPoint(dst_center.x(), dst_rect.bottom() + gap)

        return start, end

    @staticmethod
    def _draw_arrow_head(painter: QPainter, start: QPoint, end: QPoint, color: QColor):

        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        head_len = 10
        head_angle = math.pi / 7

        left = QPoint(
            int(end.x() - head_len * math.cos(angle - head_angle)),
            int(end.y() - head_len * math.sin(angle - head_angle)),
        )
        right = QPoint(
            int(end.x() - head_len * math.cos(angle + head_angle)),
            int(end.y() - head_len * math.sin(angle + head_angle)),
        )

        painter.setBrush(color)
        painter.drawPolygon(QPolygon([end, left, right]))


# =========================
# 主窗口
# =========================
class MainWindow(QWidget):

    def __init__(self):

        super().__init__()

        self.setObjectName("MainWindow")
        self.setWindowTitle("Robot Command UI")
        self.resize(1000, 600)

        self.ssh_clients = [None, None]
        self.ssh_shells = [None, None]
        self.connected_sessions = [False, False]
        self.connection_results = [None, None]
        self.ssh_connecting = False
        self.connection_attempt_id = 0
        self.safe_started = False
        self.robot_running_left = False
        self.robot_running_right = False
        self.left_robot_ready = False
        self.right_robot_ready = False
        self.active_connection_mode = ""
        self.active_connection_host = ""
        self.connection_mode = self._load_connection_mode()
        self._ssh_lock = threading.Lock()

        # ===== 连接参数 =====
        self.host, self.port1, self.port2, self.state_graph_config, self.theme = (
            load_client_config()
        )
        self.host = self._selected_connection_host()
        self.action_names = _load_gui_action_names()
        self.state_names = _load_gui_state_names()
        self.current_state = str(
            self.state_graph_config.get("initial_current_state", "init_state")
        )

        # ===== 数据 =====
        self.commands = {
            self.port1: [],
            self.port2: [],
        }

        self.buttons = {
            self.port1: [],
            "action": [],
        }
        self.use_port1_state_graph = False
        self.action_buttons_by_name = {}
        self.action_side_buttons_by_name = {}

        # ===== 信号 =====
        self.bridge = UiBridge()
        self.bridge.log.connect(self._append_log)
        self.bridge.startup_log.connect(self._append_startup_log)
        self.bridge.session_result.connect(self._handle_session_result)
        self.bridge.robot_detection.connect(self._handle_robot_detection)
        self.bridge.set_button.connect(self._add_button)
        self.bridge.clear_buttons.connect(self._clear_buttons)
        self.bridge.set_current_state.connect(self._set_current_state)
        self.bridge.set_port1_graph_mode.connect(self._set_port1_graph_mode)
        self.bridge.refresh_action_buttons.connect(self._refresh_action_buttons)

        # ================= UI =================

        self.tabs = QTabWidget()
        startup_tab = QWidget()
        action_tab = QWidget()
        self.robot_status_label = QLabel("机器人状态:")
        self.robot_status_light = QLabel()
        self.robot_status_light.setFixedSize(16, 16)
        self.robot_status_text = QLabel("等待运行")
        self.stop_robot_button = QPushButton("停止")
        self.stop_robot_button.setFixedSize(100, 42)
        self.stop_robot_button.clicked.connect(self._handle_stop_button_clicked)

        # --- 启动控制 ---
        self.connection_ip_label = QLabel()
        self.connection_ip_label.setMinimumWidth(150)
        self.connection_config_button = QPushButton("配置")
        self.connect_button = QPushButton("1.连接")
        self.disconnect_button = QPushButton("断开连接")
        self.safe_startup_button = QPushButton("2.确认机器人已准备")
        self.run_robot_button = QPushButton("3.启动运行")
        self.disconnect_button.setEnabled(False)
        self.safe_startup_button.setEnabled(False)
        self.run_robot_button.setEnabled(False)

        self.connection_config_button.clicked.connect(self.open_connection_config)
        self.connect_button.clicked.connect(self.connect_selected_session)
        self.disconnect_button.clicked.connect(self.disconnect_sessions)
        self.safe_startup_button.clicked.connect(self.robot_safe_start)
        self.run_robot_button.clicked.connect(self.run_both_scripts)

        startup_controls = QHBoxLayout()
        startup_controls.addWidget(self.connection_ip_label)
        startup_controls.addWidget(self.disconnect_button)
        startup_controls.addStretch()
        startup_controls.addWidget(self.connect_button)
        startup_controls.addWidget(self.safe_startup_button)
        startup_controls.addWidget(self.run_robot_button)

        self.startup_output_left = QTextEdit(readOnly=True)
        self.startup_output_left.setPlaceholderText("左侧机器人日志")
        self.startup_output_right = QTextEdit(readOnly=True)
        self.startup_output_right.setPlaceholderText("右侧机器人日志")
        self.startup_extra_output = QTextEdit(readOnly=True)
        self.startup_extra_output.setPlaceholderText("启动状态日志")
        self.startup_extra_output.setMinimumHeight(150)

        startup_splitter = QSplitter(Qt.Horizontal)
        startup_splitter.addWidget(self.startup_output_left)
        startup_splitter.addWidget(self.startup_output_right)
        startup_splitter.setStretchFactor(0, 1)
        startup_splitter.setStretchFactor(1, 1)

        startup_layout = QVBoxLayout(startup_tab)
        startup_layout.addLayout(startup_controls)
        startup_layout.addWidget(startup_splitter, 1)
        startup_layout.addWidget(self.startup_extra_output)

        self.bridge.enable_startup_connect.connect(self.connect_button.setEnabled)
        self.bridge.enable_startup_connect.connect(
            self.connection_config_button.setEnabled
        )
        self.bridge.enable_startup_disconnect.connect(self.disconnect_button.setEnabled)
        self.bridge.enable_startup_safe.connect(self.safe_startup_button.setEnabled)
        self.bridge.enable_startup_run.connect(self.run_robot_button.setEnabled)

        # --- 顶部连接信息 ---
        self.action_connection_ip_label = QLabel()
        self.action_connection_ip_label.setMinimumWidth(150)

        self.rescan_button = QPushButton("重新扫描")
        self.rescan_button.clicked.connect(self.rescan_commands)

        host_layout = QHBoxLayout()
        host_layout.addWidget(self.action_connection_ip_label, 2)
        host_layout.addWidget(self.rescan_button)

        # --- 命令列表 ---
        self.log_cmd_43210 = QTextEdit(readOnly=True)
        self.log_cmd_46000 = QTextEdit(readOnly=True)

        self.log_cmd_43210.setPlaceholderText(f"{self.port1} command list")
        self.log_cmd_46000.setPlaceholderText(f"{self.port2} command list")

        # --- 执行日志 ---
        self.log_exec = QTextEdit(readOnly=True)
        self.log_exec.setPlaceholderText("Execution / response log")

        # --- 左侧按钮区 ---
        self.group_43210 = QGroupBox(f"状态控制")
        self.layout_43210 = QVBoxLayout(self.group_43210)
        active_state_color = self.theme.get(
            "action_active_bg", DEFAULT_THEME["action_active_bg"]
        )
        self.current_state_label = QLabel(
            f'current_state: <span style="color: {active_state_color};">'
            f'{self._state_display_name(self.current_state)}</span>'
        )
        self.current_state_label.setWordWrap(True)
        self.current_state_label.setTextFormat(Qt.TextFormat.RichText)
        self.current_state_label.setStyleSheet("padding-left: 20px;")
        self.layout_43210.addWidget(self.current_state_label)
        self._build_state_graph()

        # --- 右侧按钮区 ---
        self.group_46000 = QGroupBox(f"执行动作")
        self.layout_46000 = QVBoxLayout(self.group_46000)

        # Action 区
        self.group_action = QGroupBox("Action (Sleep)")
        self.layout_action = QGridLayout(self.group_action)
        self.layout_action.setContentsMargins(9, 9, 9, 9)
        self.layout_action.setHorizontalSpacing(30)
        self.layout_action.setVerticalSpacing(8)

        self.layout_46000.addWidget(self.group_action)

        # ===== 布局 =====

        # 左侧：两个命令列表 + 43210 按钮
        top_left = QVBoxLayout()
        top_left.addWidget(self.log_cmd_43210, 1)
        top_left.addWidget(self.log_cmd_46000, 2)
        top_left.addWidget(self.group_43210, 1)

        # 右侧：46000 按钮
        top_right = QVBoxLayout()
        top_right.addWidget(self.group_46000)

        top_layout = QHBoxLayout()
        top_layout.addLayout(top_left, 1)
        top_layout.addLayout(top_right, 2)

        main_layout = QVBoxLayout(self)
        action_layout = QVBoxLayout(action_tab)
        action_layout.addLayout(host_layout)
        action_layout.addLayout(top_layout, 2)
        action_layout.addWidget(self.log_exec, 1)
        self.log_exec.setMinimumHeight(150)
        self._apply_theme()

        startup_button_style = """
            QPushButton {
                background-color: #eeeeee;
                color: #11161d;
                border: 1px solid #aeb8c4;
                border-radius: 8px;
                padding: 8px 12px;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #ffffff;
            }
            QPushButton:pressed {
                background-color: #d9e0e8;
            }
            QPushButton:disabled {
                background-color: #222222;
                color: #5f6c7b;
                border: 1px solid #273241;
            }
        """
        for button in (
            self.connect_button,
            self.disconnect_button,
            self.safe_startup_button,
            self.run_robot_button,
        ):
            button.setStyleSheet(startup_button_style)

        self.disconnect_button.setStyleSheet(
            """
            QPushButton {
                background-color: #808080;
                color: #ffffff;
                border: 1px solid #aeb8c4;
                border-radius: 8px;
                padding: 8px 12px;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #909090;
            }
            QPushButton:pressed {
                background-color: #707070;
            }
            QPushButton:disabled {
                background-color: #222222;
                color: #5f6c7b;
                border: 1px solid #273241;
            }
            """
        )

        self.connection_config_button.setStyleSheet(
            """
            QPushButton {
                background-color: #808080;
                color: #ffffff;
                border: 1px solid #aeb8c4;
                border-radius: 8px;
                padding: 8px 12px;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #909090;
            }
            QPushButton:pressed {
                background-color: #707070;
            }
            QPushButton:disabled {
                background-color: #222222;
                color: #5f6c7b;
                border: 1px solid #273241;
            }
            """
        )

        self.stop_robot_button.setStyleSheet(
            """
            QPushButton {
                background-color: #b42318;
                color: #ffffff;
                border: 1px solid #8f1d16;
                border-radius: 6px;
                padding: 6px 14px;
                font-size: 15px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #991b1b;
            }
            QPushButton:pressed {
                background-color: #7f1d1d;
            }
            """
        )
        self.robot_status_label.setStyleSheet(
            "font-size: 13px; background-color: #252525;"
        )
        self.robot_status_text.setStyleSheet(
            "font-size: 13px; background-color: #252525;"
        )
        self.connection_ip_label.setStyleSheet("font-size: 13px;")
        self._update_connection_ip_label()
        self._set_robot_status(False)

        self.tabs.addTab(startup_tab, "启动")
        self.tabs.addTab(action_tab, "动作")
        self.tab_pages = (startup_tab, action_tab)
        self.tabs.currentChanged.connect(self._update_tab_page_background)
        top_bar = QHBoxLayout()
        top_bar.addWidget(self.connection_config_button)
        top_bar.addStretch()
        top_bar.addWidget(self.robot_status_label)
        top_bar.addWidget(self.robot_status_light)
        top_bar.addWidget(self.robot_status_text)
        top_bar.addSpacing(12)
        top_bar.addWidget(self.stop_robot_button)
        main_layout.addLayout(top_bar)
        main_layout.addWidget(self.tabs)
        self._update_tab_page_background(self.tabs.currentIndex())

        # ===== 加载命令 =====
        self.load_commands(self.port1)
        self.load_commands(self.port2)

    # ================= 启动页连接与日志 =================

    def _update_tab_page_background(self, current_index: int):
        for index, page in enumerate(self.tab_pages):
            page.setObjectName(
                "SelectedTabPage" if index == current_index else ""
            )
            page.style().unpolish(page)
            page.style().polish(page)
            page.update()

    def _load_connection_mode(self) -> str:
        if not GUI_CONFIG_PATH.exists():
            return "有线"

        try:
            config_text = GUI_CONFIG_PATH.read_text(encoding="utf-8")
            if yaml is not None:
                config = yaml.safe_load(config_text) or {}
            else:
                config = _parse_simple_yaml(config_text)
        except Exception:
            return "有线"

        if not isinstance(config, dict):
            return "有线"

        mode = str(config.get("connection_mode", "wired")).strip().lower()
        return "无线" if mode in {"wireless", "wifi", "无线"} else "有线"

    def _selected_connection_host(self) -> str:
        return SSH_WIRED_HOST if self.connection_mode == "有线" else SSH_WIRELESS_HOST

    def _save_connection_hosts(
        self,
        wired_host: str,
        wireless_host: str,
        connection_mode: str,
    ):
        config_text = (
            GUI_CONFIG_PATH.read_text(encoding="utf-8")
            if GUI_CONFIG_PATH.exists()
            else ""
        )

        if yaml is not None:
            config = yaml.safe_load(config_text) if config_text else {}
            if not isinstance(config, dict):
                config = {}
            config["wired_host"] = wired_host
            config["wireless_host"] = wireless_host
            config["connection_mode"] = (
                "wireless" if connection_mode == "无线" else "wired"
            )
            saved_text = yaml.safe_dump(
                config,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
            )
        else:
            values = {
                "wired_host": wired_host,
                "wireless_host": wireless_host,
                "connection_mode": (
                    "wireless" if connection_mode == "无线" else "wired"
                ),
            }
            lines = config_text.splitlines()
            updated_keys = set()
            for index, line in enumerate(lines):
                stripped = line.lstrip()
                if not stripped or stripped.startswith("#") or ":" not in stripped:
                    continue
                key = stripped.split(":", 1)[0].strip()
                if key not in values:
                    continue
                indent = line[: len(line) - len(stripped)]
                lines[index] = f"{indent}{key}: {values[key]}"
                updated_keys.add(key)

            for key, value in values.items():
                if key not in updated_keys:
                    lines.append(f"{key}: {value}")
            saved_text = "\n".join(lines).rstrip() + "\n"

        GUI_CONFIG_PATH.write_text(saved_text, encoding="utf-8")

    def open_connection_config(self):
        global SSH_WIRED_HOST, SSH_WIRELESS_HOST

        dialog = QDialog(self)
        dialog.setWindowTitle("连接配置")
        dialog.setModal(True)
        dialog.resize(420, 190)
        dialog.setStyleSheet(
            """
            QDialog {
                background-color: #2d2d2d;
                color: #ffffff;
            }
            QLabel {
                background-color: #2d2d2d;
                color: #ffffff;
            }
            QLineEdit, QComboBox {
                background-color: #3a3a3a;
                color: #ffffff;
                border: 1px solid #555555;
                padding: 8px 10px;
            }
            QComboBox QAbstractItemView {
                background-color: #3a3a3a;
                color: #ffffff;
                selection-background-color: #555555;
            }
            """
        )

        mode_switch = QComboBox()
        mode_switch.addItems(["有线", "无线"])
        mode_switch.setCurrentText(self.connection_mode)
        mode_switch.setFixedWidth(120)

        wired_input = QLineEdit(SSH_WIRED_HOST)
        wireless_input = QLineEdit(SSH_WIRELESS_HOST)
        form_layout = QFormLayout(dialog)
        form_layout.addRow("连接方式", mode_switch)
        form_layout.addRow("有线 IP", wired_input)
        form_layout.addRow("无线 IP", wireless_input)

        button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel
        )
        button_box.button(QDialogButtonBox.StandardButton.Save).setText("保存")
        button_box.button(QDialogButtonBox.StandardButton.Cancel).setText("取消")
        button_box.accepted.connect(dialog.accept)
        button_box.rejected.connect(dialog.reject)
        form_layout.addRow(button_box)

        if dialog.exec() != QDialog.DialogCode.Accepted:
            return

        wired_host = wired_input.text().strip()
        wireless_host = wireless_input.text().strip()
        connection_mode = mode_switch.currentText()
        for label, host in (("有线", wired_host), ("无线", wireless_host)):
            try:
                ipaddress.ip_address(host)
            except ValueError:
                QMessageBox.warning(self, "配置错误", f"{label} IP 地址无效")
                return

        try:
            self._save_connection_hosts(wired_host, wireless_host, connection_mode)
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", f"无法保存连接配置：{exc}")
            return

        SSH_WIRED_HOST = wired_host
        SSH_WIRELESS_HOST = wireless_host
        self.connection_mode = connection_mode
        gui_start_module.WIRED_HOST = wired_host
        gui_start_module.WIRELESS_HOST = wireless_host
        gui_start_module.CONNECTION_MODE = (
            "wireless" if connection_mode == "无线" else "wired"
        )
        self._update_connection_ip_label()
        self.bridge.startup_log.emit(
            self.startup_extra_output,
            f"连接配置已保存：有线 {wired_host}，无线 {wireless_host}",
        )

    def _update_connection_ip_label(self):
        mode = self.connection_mode
        host = self._selected_connection_host()
        self.host = host
        self.connection_ip_label.setText(f"{mode} IP: {host}")
        if hasattr(self, "action_connection_ip_label"):
            self.action_connection_ip_label.setText(f"{mode} IP: {host}")

    def connect_selected_session(self):
        mode = self.connection_mode
        host = self._selected_connection_host()
        self.connect_sessions(host, mode)

    def connect_sessions(self, host: str, mode: str):
        if self.ssh_connecting or all(self.connected_sessions):
            return

        self._close_ssh_sessions()
        self.connected_sessions = [False, False]
        self.connection_results = [None, None]
        self.ssh_connecting = True
        self.connection_attempt_id += 1
        attempt_id = self.connection_attempt_id
        self.safe_started = False
        self.active_connection_mode = mode
        self.active_connection_host = host
        self._reset_robot_status()
        self.bridge.enable_startup_connect.emit(False)
        self.bridge.enable_startup_disconnect.emit(False)
        self.bridge.enable_startup_safe.emit(False)
        self.bridge.enable_startup_run.emit(False)
        self.bridge.startup_log.emit(
            self.startup_extra_output,
            f"正在连接 {host}...",
        )
        threading.Thread(
            target=self._monitor_connection,
            args=(host, attempt_id),
            daemon=True,
        ).start()

        threading.Thread(
            target=self._connect_session,
            args=(1, host),
            daemon=True,
        ).start()
        threading.Thread(
            target=self._connect_session,
            args=(2, host),
            daemon=True,
        ).start()

    def _monitor_connection(self, host: str, attempt_id: int):
        for elapsed in range(1, SSH_TIMEOUT + 1):
            time.sleep(1)
            if (
                not self.ssh_connecting
                or self.connection_attempt_id != attempt_id
            ):
                return

            if elapsed < SSH_TIMEOUT:
                self.bridge.startup_log.emit(
                    self.startup_extra_output,
                    f"正在连接 {host}... ({elapsed}/{SSH_TIMEOUT}s)",
                )
            else:
                self.bridge.startup_log.emit(
                    self.startup_extra_output,
                    f"连接超时，请检查 {host}、用户名、密码和网络状况",
                )

    def _connect_session(self, session_id: int, host: str):
        index = session_id - 1
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            client.connect(
                host,
                username=SSH_USERNAME,
                password=SSH_PASSWORD,
                timeout=SSH_TIMEOUT,
            )
            transport = client.get_transport()
            if transport is None or not transport.is_active():
                raise RuntimeError("SSH transport is not active")

            shell = client.invoke_shell()
            shell.settimeout(0.2)
            with self._ssh_lock:
                self.ssh_clients[index] = client
                self.ssh_shells[index] = shell
            self.bridge.session_result.emit(session_id, True, host)
        except Exception:
            try:
                client.close()
            except Exception:
                pass
            self.bridge.session_result.emit(
                session_id,
                False,
                traceback.format_exc(),
            )

    @Slot(int, bool, str)
    def _handle_session_result(
        self,
        session_id: int,
        success: bool,
        message: str,
    ):
        index = session_id - 1
        output_area = (
            self.startup_output_left
            if index == 0
            else self.startup_output_right
        )
        if success:
            self.connected_sessions[index] = True
            self.bridge.startup_log.emit(
                output_area,
                f"Session {session_id} connected to {message}",
            )
        else:
            self.connected_sessions[index] = False
            self.bridge.startup_log.emit(
                output_area,
                f"Session {session_id} connection failed:\n{message}",
            )

        self.connection_results[index] = success
        if not all(result is not None for result in self.connection_results):
            return

        self.ssh_connecting = False
        if all(self.connection_results):
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                f"已成功连接 {message},\n请遥控机器人到正确状态,\n点击确认机器人已准备",
            )
            self.bridge.enable_startup_safe.emit(True)
            self.bridge.enable_startup_connect.emit(False)
            self.bridge.enable_startup_disconnect.emit(True)
            return

        self._close_ssh_sessions()
        self.bridge.startup_log.emit(
            self.startup_extra_output,
            "连接失败，请检查 IP、用户名、密码和网络状况",
        )
        self.bridge.enable_startup_connect.emit(True)
        self.bridge.enable_startup_disconnect.emit(False)
        self.bridge.enable_startup_safe.emit(False)
        self.bridge.enable_startup_run.emit(False)

    @Slot(object, str)
    def _append_startup_log(self, area, text: str):
        if area == self.startup_extra_output:
            timestamp = time.strftime("%H:%M:%S")
            area.append(f"[{timestamp}] {text}")
        else:
            area.append(text)
        area.moveCursor(QTextCursor.End)
        area.ensureCursorVisible()

    @Slot(int)
    def _handle_robot_detection(self, side: int):
        was_ready = self.left_robot_ready and self.right_robot_ready
        if side == 0:
            self.left_robot_ready = True
        elif side == 1:
            self.right_robot_ready = True
        ready = self.left_robot_ready and self.right_robot_ready
        self._set_robot_status(ready)
        if ready and not was_ready:
            self.rescan_commands()

    @Slot(bool)
    def _set_robot_status(self, ready: bool):
        color = "#22c55e" if ready else "#5f6975"
        status_text = "准备执行动作" if ready else "等待运行"
        self.robot_status_light.setStyleSheet(
            f"background-color: {color}; border: 1px solid #d7dde5; border-radius: 8px;"
        )
        self.robot_status_text.setText(status_text)

    def _reset_robot_status(self):
        self.left_robot_ready = False
        self.right_robot_ready = False
        self._set_robot_status(False)

    def disconnect_sessions(self):
        if not any(self.connected_sessions) and not self.ssh_connecting:
            return

        if self.robot_running_left or self.robot_running_right:
            self.stop_both_scripts(rescan=False)

        self.startup_output_left.clear()
        self.startup_output_right.clear()

        self._close_ssh_sessions()
        self.connection_results = [None, None]
        self.safe_started = False
        self._reset_robot_status()
        self.active_connection_mode = ""
        self.active_connection_host = ""
        self.bridge.enable_startup_connect.emit(True)
        self.bridge.enable_startup_disconnect.emit(False)
        self.bridge.enable_startup_safe.emit(False)
        self.bridge.enable_startup_run.emit(False)
        self.bridge.startup_log.emit(
            self.startup_extra_output,
            "SSH 连接已断开，请重新点击连接",
        )

    # ================= 安全启动 =================

    def robot_safe_start(self):
        if not all(self.connected_sessions):
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                "请先点击连接",
            )
            return

        self.safe_started = False
        self.bridge.enable_startup_safe.emit(False)
        t1 = threading.Thread(
            target=self._run_auto_commands,
            args=(self.ssh_shells[0], self.startup_output_left),
            daemon=True,
        )
        t2 = threading.Thread(
            target=self._run_auto_commands,
            args=(self.ssh_shells[1], self.startup_output_right),
            daemon=True,
        )
        t1.start()
        t2.start()

        def enable_after_done():
            t1.join()
            t2.join()
            self.safe_started = True
            self.bridge.enable_startup_run.emit(True)
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                "已确认机器人处于准备状态,\n可以开始运行机器人",
            )

        threading.Thread(target=enable_after_done, daemon=True).start()

    def _run_auto_commands(self, shell, area):
        try:
            if not shell:
                self.bridge.startup_log.emit(area, "shell 未就绪")
                return

            shell.send("conda activate g1brainco\n")
            time.sleep(1)
            self.bridge.startup_log.emit(area, "Activated conda: g1brainco")

            shell.send("cd ~/unitree-g1-brainco-hand/brainco_ws\n")
            time.sleep(0.5)
            self.bridge.startup_log.emit(
                area,
                "Changed dir to ~/unitree-g1-brainco-hand/brainco_ws",
            )
        except Exception:
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                f"_run_auto_commands exception:\n{traceback.format_exc()}",
            )

    # ================= 启动运行 =================

    def run_both_scripts(self):
        if not self.safe_started:
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                "请先点击确认机器人已准备",
            )
            return

        self.run_robot_button.setEnabled(False)
        if not self.robot_running_left:
            threading.Thread(target=self._run_left_script, daemon=True).start()
        if not self.robot_running_right:
            threading.Thread(target=self._run_right_script, daemon=True).start()

    def _run_left_script(self):
        self.robot_running_left = True
        try:
            shell = self.ssh_shells[0]
            if not shell:
                self.bridge.startup_log.emit(
                    self.startup_extra_output,
                    "左侧 shell 未就绪",
                )
                return

            try:
                shell.send("./launch/launch_robot.sh\n")
            except Exception:
                self.bridge.startup_log.emit(
                    self.startup_extra_output,
                    "左侧启动命令发送失败",
                )
                return

            buffer = ""
            configure_reported = False
            while self.robot_running_left:
                try:
                    if shell.recv_ready():
                        raw = shell.recv(4096)
                        if not raw:
                            self.bridge.startup_log.emit(
                                self.startup_extra_output,
                                "左侧 SSH 通道已关闭",
                            )
                            break
                        output = raw.decode("utf-8", errors="replace")
                        buffer += output
                        self.bridge.startup_log.emit(
                            self.startup_output_left,
                            output,
                        )
                        if not configure_reported and "Request 'configure' to start" in buffer:
                            configure_reported = True
                            self.bridge.robot_detection.emit(0)
                            self.bridge.startup_log.emit(
                                self.startup_extra_output,
                                "左侧检测到 Request 'configure' to start",
                            )
                        if "[sudo] password for unitree:" in buffer:
                            try:
                                shell.send("123\n")
                                self.bridge.startup_log.emit(
                                    self.startup_output_left,
                                    "自动输入 sudo 密码 123",
                                )
                            except Exception as exc:
                                self.bridge.startup_log.emit(
                                    self.startup_extra_output,
                                    f"sudo send failed: {exc}",
                                )
                            buffer = ""
                    else:
                        time.sleep(0.1)
                except Exception:
                    self.bridge.startup_log.emit(
                        self.startup_extra_output,
                        f"left loop error:\n{traceback.format_exc()}",
                    )
                    break
        except Exception:
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                f"_run_left_script exception:\n{traceback.format_exc()}",
            )
        finally:
            self.robot_running_left = False

    def _run_right_script(self):
        self.robot_running_right = True
        try:
            shell = self.ssh_shells[1]
            if not shell:
                self.bridge.startup_log.emit(
                    self.startup_extra_output,
                    "右侧 shell 未就绪",
                )
                return

            try:
                shell.send("./launch/launch_trans.sh agent\n")
            except Exception:
                self.bridge.startup_log.emit(
                    self.startup_extra_output,
                    "右侧启动命令发送失败",
                )
                return

            buffer = ""
            agents_reported = False
            while self.robot_running_right:
                try:
                    if shell.recv_ready():
                        raw = shell.recv(4096)
                        if not raw:
                            self.bridge.startup_log.emit(
                                self.startup_extra_output,
                                "右侧 SSH 通道已关闭",
                            )
                            break
                        output = raw.decode("utf-8", errors="replace")
                        buffer += output
                        self.bridge.startup_log.emit(
                            self.startup_output_right,
                            output,
                        )
                        if not agents_reported and "Agents started" in buffer:
                            agents_reported = True
                            self.bridge.robot_detection.emit(1)
                            self.bridge.startup_log.emit(
                                self.startup_extra_output,
                                "右侧检测到 Agents started",
                            )
                        if "[sudo] password for unitree:" in buffer:
                            try:
                                shell.send("123\n")
                                self.bridge.startup_log.emit(
                                    self.startup_output_right,
                                    "自动输入 sudo 密码 123",
                                )
                            except Exception as exc:
                                self.bridge.startup_log.emit(
                                    self.startup_extra_output,
                                    f"sudo send failed: {exc}",
                                )
                            buffer = ""
                    else:
                        time.sleep(0.1)
                except Exception:
                    self.bridge.startup_log.emit(
                        self.startup_extra_output,
                        f"right loop error:\n{traceback.format_exc()}",
                    )
                    break
        except Exception:
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                f"_run_right_script exception:\n{traceback.format_exc()}",
            )
        finally:
            self.robot_running_right = False

    def _send_ctrl_c_twice(self, shell):
        for _ in range(2):
            try:
                shell.send("\x03")
            except Exception:
                pass
            time.sleep(0.05)

    def stop_both_scripts(self, rescan=True, preserve_state=False):
        try:
            if self.robot_running_left and self.ssh_shells[0] is not None:
                self._send_ctrl_c_twice(self.ssh_shells[0])
                self.robot_running_left = False

            if self.robot_running_right and self.ssh_shells[1] is not None:
                self._send_ctrl_c_twice(self.ssh_shells[1])
                self.robot_running_right = False

            self.startup_output_left.clear()
            self.startup_output_right.clear()
            self.bridge.startup_log.emit(
                self.startup_output_left,
                "已停止左边脚本",
            )
            self.bridge.startup_log.emit(
                self.startup_output_right,
                "已停止右边脚本",
            )
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                "已停止脚本",
            )
            self._reset_robot_status()
            self.run_robot_button.setEnabled(
                all(self.connected_sessions) and self.safe_started
            )
            if rescan:
                self.rescan_commands(reset_state=not preserve_state)
        except Exception:
            self.bridge.startup_log.emit(
                self.startup_extra_output,
                f"stop error:\n{traceback.format_exc()}",
            )

    def _execute_shutdown_command(self) -> bool:
        for index, spec in enumerate(self.commands.get(self.port1, [])):
            if spec.name == "smach_s":
                self.exec_command(self.port1, index, "both")
                return True

        self.bridge.log.emit(
            self.port1,
            LogType.EXECUTION,
            f"[{self.port1}] Shutdown command not found",
        )
        return False

    def _handle_stop_button_clicked(self):
        shutdown_sent = self._execute_shutdown_command()
        if shutdown_sent:
            self._set_current_state("finalized")
        self.stop_both_scripts(preserve_state=shutdown_sent)

    def _stop_startup_scripts(self):
        for running, shell in (
            (self.robot_running_left, self.ssh_shells[0]),
            (self.robot_running_right, self.ssh_shells[1]),
        ):
            if running and shell is not None:
                self._send_ctrl_c_twice(shell)
        self.robot_running_left = False
        self.robot_running_right = False

    def _close_ssh_sessions(self):
        self._stop_startup_scripts()
        with self._ssh_lock:
            shells = list(self.ssh_shells)
            clients = list(self.ssh_clients)
            self.ssh_shells = [None, None]
            self.ssh_clients = [None, None]

        for shell in shells:
            if shell is not None:
                try:
                    shell.close()
                except Exception:
                    pass
        for client in clients:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass

        self.connected_sessions = [False, False]
        self.ssh_connecting = False

    def closeEvent(self, event):
        self.stop_both_scripts(rescan=False)
        self._close_ssh_sessions()
        super().closeEvent(event)

    # ================= 日志 =================

    @Slot(int, LogType, str)
    def _append_log(self, port: int, log_type: LogType, msg: str):

        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"

        if log_type == LogType.COMMAND_LIST:

            if port == self.port1:
                self.log_cmd_43210.append(line)

            elif port == self.port2:
                self.log_cmd_46000.append(line)

        else:
            self.log_exec.append(line)

    @Slot(str)
    def _set_current_state(self, current_state: str):

        state_text = current_state.strip() if isinstance(current_state, str) else ""
        self.current_state = state_text or self.state_graph_config.get(
            "initial_current_state", "init_state"
        )
        active_state_color = self.theme.get(
            "action_active_bg", DEFAULT_THEME["action_active_bg"]
        )
        self.current_state_label.setText(
            f'current_state: <span style="color: {active_state_color};">'
            f'{self._state_display_name(self.current_state)}</span>'
        )
        self._refresh_state_graph()
        self._refresh_action_buttons()

    def _state_display_name(self, current_state: str) -> str:
        node_key = self._resolve_state_node(current_state)
        state_name_key = {
            "init": "Init",
            "config": "Ready",
            "active": "Active",
            "shutdown": "Shutdown",
        }.get(node_key)
        return self.state_names.get(state_name_key, current_state)

    @Slot(bool)
    def _set_port1_graph_mode(self, enabled: bool):

        self.use_port1_state_graph = enabled
        self.state_graph_widget.setVisible(enabled)
        self._refresh_state_graph()

    def _build_state_graph(self):

        self.state_graph_widget = StateGraphWidget(
            self.state_graph_config,
            self.theme,
            self.state_names,
            self._handle_state_graph_click,
        )
        self.layout_43210.addWidget(self.state_graph_widget)
        self.state_graph_widget.hide()
        self._refresh_state_graph()

    def _apply_theme(self):

        theme = self.theme
        self.setStyleSheet(
            f"""
            QWidget {{
                background-color: #1e1e1e;
                color: #ffffff;
            }}
            QWidget#MainWindow {{
                background-color: #252525;
            }}
            QDialog {{
                background-color: #1e1e1e;
                color: #ffffff;
            }}
            QGroupBox {{
                background-color: #1e1e1e;
                border: 1px solid #555555;
                border-radius: 0;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #ffffff;
            }}
            QTextEdit, QLineEdit {{
                color: #ffffff;
                border: 1px solid #555555;
                border-radius: 0;
                selection-background-color: {theme["selection_bg"]};
            }}
            QTextEdit {{
                background-color: #2d2d2d;
                color: #ffffff;
                font-size: 12px;
            }}
            QLineEdit {{
                background-color: #1e1e1e;
                border: 1px solid #555555;
                padding: 8px 10px;
            }}
            QLineEdit:focus {{
                border: 1px solid #ffffff;
            }}
            QComboBox {{
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #555555;
                border-radius: 0;
                padding: 8px 10px;
            }}
            QComboBox QAbstractItemView {{
                background-color: #2d2d2d;
                color: #ffffff;
                selection-background-color: #555555;
            }}
            QTabWidget::pane {{
                background-color: #1e1e1e;
                border: none;
            }}
            QTabBar::tab {{
                background-color: #252525;
                color: #bdbdbd;
                border: 1px solid #2d2d2d;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                border-bottom-left-radius: 0;
                border-bottom-right-radius: 0;
                padding: 6px 16px;
            }}
            QTabBar::tab:selected {{
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #555555;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                border-bottom-left-radius: 0;
                border-bottom-right-radius: 0;
            }}
            QTabBar::tab:hover:!selected {{
                background-color: #555555;
                color: #ffffff;
            }}
            QLabel {{
                color: #ffffff;
            }}
            QPushButton {{
                background-color: {theme["button_bg"]};
                color: {theme["text_primary"]};
                border: 1px solid {theme["border"]};
                border-radius: 8px;
                padding: 8px 12px;
                font-size: 13px;
            }}
            QPushButton:hover {{
                background-color: {theme["button_hover_bg"]};
            }}
            QPushButton:pressed {{
                background-color: {theme["button_pressed_bg"]};
            }}
            QPushButton:disabled {{
                background-color: {theme["button_disabled_bg"]};
                color: {theme["button_disabled_text"]};
                border: 1px solid {theme["border"]};
            }}
            QWidget#SelectedTabPage {{
                background-color: #1e1e1e;
            }}
            """
        )

    def _resolve_state_node(self, current_state: str) -> str | None:

        nodes = self.state_graph_config.get("nodes", {})
        normalized_state = current_state.strip()

        for node_key, node_cfg in nodes.items():
            if not isinstance(node_cfg, dict):
                continue

            states = node_cfg.get("states", [])
            if isinstance(states, list) and normalized_state in {str(item) for item in states}:
                return node_key

        for node_key, node_cfg in nodes.items():
            if isinstance(node_cfg, dict) and node_cfg.get("match") == "unmapped":
                return node_key

        return None

    def _refresh_state_graph(self):

        transitions = self.state_graph_config.get("transitions", {})
        active_node = self._resolve_state_node(self.current_state)
        allowed_next = []

        if active_node and isinstance(transitions.get(active_node), dict):
            allowed_next = list(transitions.get(active_node, {}).keys())

        allowed_next_set = {str(item) for item in allowed_next}
        self.state_graph_widget.set_graph_state(active_node, allowed_next_set)

    def _handle_state_graph_click(self, target_node: str, extra: str = ""):

        active_node = self._resolve_state_node(self.current_state)
        if not active_node:
            return

        transitions = self.state_graph_config.get("transitions", {})
        next_map = transitions.get(active_node, {})
        if not isinstance(next_map, dict):
            return

        transition_id = next_map.get(target_node)
        if transition_id is None:
            return

        spec_name = f"smach_{transition_id}"
        for index, spec in enumerate(self.commands.get(self.port1, [])):
            if spec.name == spec_name:
                self.exec_command(self.port1, index, "both", extra=extra)
                if target_node == "shutdown":
                    self._set_current_state("finalized")
                    self.stop_both_scripts(preserve_state=True)
                return

        QMessageBox.warning(
            self,
            "Error",
            f"State transition command not found: {spec_name}"
        )

    @staticmethod
    def _should_use_port1_state_graph(cmds: list[CommandSpec]) -> bool:

        required_names = {"smach_f", "smach_a", "smach_d", "smach_s"}
        command_names = {spec.name for spec in cmds}

        return len(cmds) == len(required_names) and command_names == required_names

    # ================= 按钮 =================

    @Slot(int)
    def _clear_buttons(self, port: int):

        if port == self.port1:

            for widget in self.buttons[self.port1]:
                self.layout_43210.removeWidget(widget)
                widget.deleteLater()

            self.buttons[self.port1].clear()

        elif port == self.port2:

            for widget in self.buttons["action"]:
                self.layout_action.removeWidget(widget)
                widget.deleteLater()

            self.buttons["action"].clear()
            self.action_buttons_by_name.clear()
            self.action_side_buttons_by_name.clear()

    @Slot(int, int, str)
    def _add_button(self, port: int, index: int, text: str):

        spec = self.commands.get(port, [])[index]
        row_widget = QWidget()
        row_widget.setStyleSheet("background: transparent;")
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(6)

        btn = QPushButton(text)
        btn.setStyleSheet("text-align: left; padding-left: 40px;")
        btn.clicked.connect(partial(self.exec_command, port, index, "both"))
        row_layout.addWidget(btn, 1)
        side_buttons = []

        if self._requires_handside(spec):
            left_btn = QPushButton("L")
            left_btn.setFixedWidth(36)
            left_btn.clicked.connect(partial(self.exec_command, port, index, "left"))
            row_layout.addWidget(left_btn)
            side_buttons.append(left_btn)

            right_btn = QPushButton("R")
            right_btn.setFixedWidth(36)
            right_btn.clicked.connect(partial(self.exec_command, port, index, "right"))
            row_layout.addWidget(right_btn)
            side_buttons.append(right_btn)

        if port == self.port1:

            insert_index = 1
            self.layout_43210.insertWidget(insert_index, row_widget)
            self.buttons[self.port1].append(row_widget)

        elif port == self.port2:

            name = spec.name

            if name.startswith("a"):
                row_widget.setMaximumWidth(380)
                self.buttons["action"].append(row_widget)
                self.action_buttons_by_name[spec.name] = btn
                self.action_side_buttons_by_name[spec.name] = side_buttons
                self._relayout_action_buttons()
                self._refresh_action_buttons()

            elif name.startswith("h"):
                return

            else:
                row_widget.setMaximumWidth(400)
                self.buttons["action"].append(row_widget)
                self._relayout_action_buttons()

    @staticmethod
    def _requires_handside(spec: CommandSpec) -> bool:

        handside_arg = (spec.args or {}).get("handside", {})

        return (
            handside_arg.get("desc") == "both/left/right"
            and handside_arg.get("required") is True
        )

    @staticmethod
    def _extract_action_state_name(spec_name: str) -> str | None:

        if not spec_name.startswith("a"):
            return None

        parts = spec_name.split("_", 1)
        if len(parts) != 2:
            return None

        return parts[1]

    @staticmethod
    def _extract_action_idx(spec_name: str) -> str | None:

        if not spec_name.startswith("a"):
            return None

        parts = spec_name.split("_", 1)
        if len(parts) != 2:
            return None

        action_prefix = parts[0][1:]
        return action_prefix if action_prefix.isdigit() else None

    @staticmethod
    def _extract_current_active_idx(current_state: str) -> str | None:

        if not isinstance(current_state, str):
            return None

        normalized = current_state.strip()
        for prefix in ("active_", "active."):
            if normalized.startswith(prefix):
                action_idx = normalized.split(prefix, 1)[1]
                return action_idx if action_idx.isdigit() else None

        return None

    @staticmethod
    def _format_sleep_text(spec: CommandSpec) -> str:

        sleep = (spec.safety or {}).get("sleep")
        if sleep is None:
            return ""

        if isinstance(sleep, str):
            normalized = sleep.strip()
            return normalized if normalized else ""

        if isinstance(sleep, float) and sleep.is_integer():
            return f"{int(sleep)}s"

        return f"{sleep}s"

    def _format_command_button_text(self, index: int, spec: CommandSpec) -> str:

        text = spec.name
        match = re.match(r"^a(\d+)(?:_|$)", spec.name)
        if match:
            action_name = self.action_names.get(int(match.group(1)))
            if action_name:
                text = action_name

        sleep_text = self._format_sleep_text(spec)
        if sleep_text:
            seconds = sleep_text[:-1] if sleep_text.endswith("s") else sleep_text
            return f"{text} ({seconds}秒)"
        return text

    @Slot()
    def _refresh_action_buttons(self):

        active_node = self._resolve_state_node(self.current_state)
        in_active_state = active_node == "active"
        theme = self.theme
        reachable_bg = str(
            self.state_graph_config.get("colors", {}).get(
                "reachable", DEFAULT_STATE_GRAPH["colors"]["reachable"]
            )
        )
        inactive_bg = str(
            theme.get("button_disabled_bg", DEFAULT_THEME["button_disabled_bg"])
        ) or str(
            self.state_graph_config.get("colors", {}).get(
                "inactive", DEFAULT_STATE_GRAPH["colors"]["inactive"]
            )
        )
        reachable_text = str(
            theme.get("button_reachable_text", DEFAULT_THEME["button_reachable_text"])
        )
        inactive_text = str(
            theme.get("button_disabled_text", DEFAULT_THEME["button_disabled_text"])
        ) or str(theme.get("text_muted", DEFAULT_THEME["text_muted"]))
        active_border = str(
            self.state_graph_config.get("colors", {}).get(
                "accent", DEFAULT_STATE_GRAPH["colors"]["accent"]
            )
        )

        for spec_name, button in self.action_buttons_by_name.items():
            side_buttons = self.action_side_buttons_by_name.get(spec_name, [])
            action_idx = self._extract_action_idx(spec_name)
            current_active_idx = self._extract_current_active_idx(self.current_state)
            is_current_action = in_active_state and action_idx == current_active_idx

            if not in_active_state:
                main_style = self._build_action_button_style(
                    inactive_bg,
                    inactive_text,
                    theme["border"],
                    align_left=True,
                    border_width=2,
                )
                side_style = self._build_action_button_style(
                    inactive_bg,
                    inactive_text,
                    theme["border"],
                    border_width=2,
                )
            elif is_current_action:
                main_style = self._build_action_button_style(
                    theme["action_active_bg"],
                    theme["action_active_text"],
                    active_border,
                    align_left=True,
                    border_width=2,
                )
                side_style = self._build_action_button_style(
                    theme["action_active_bg"],
                    theme["action_active_text"],
                    active_border,
                    border_width=2,
                )
            else:
                main_style = self._build_action_button_style(
                    reachable_bg,
                    reachable_text,
                    theme["border"],
                    align_left=True,
                    border_width=2,
                )
                side_style = self._build_action_button_style(
                    reachable_bg,
                    reachable_text,
                    theme["border"],
                    border_width=2,
                )

            button.setStyleSheet(main_style)
            for side_button in side_buttons:
                side_button.setStyleSheet(side_style)

    @staticmethod
    def _build_action_button_style(
        background_color: str,
        text_color: str,
        border_color: str,
        *,
        align_left: bool = False,
        left_padding: int = 20,
        font_size: int = 13,
        border_width: int = 1,
    ) -> str:

        alignment = (
            f"text-align: left; padding-left: {left_padding}px;"
            if align_left
            else "text-align: center;"
        )
        return (
            f"{alignment} "
            f"background-color: {background_color}; "
            f"color: {text_color}; "
            f"border: {border_width}px solid {border_color}; "
            f"font-size: {font_size}px;"
        )

    def _relayout_action_buttons(self) -> None:

        button_count = len(self.buttons["action"])
        if button_count == 0:
            return

        for index, widget in enumerate(self.buttons["action"]):
            self.layout_action.addWidget(widget, index, 0)

    def rescan_commands(self, reset_state=True):

        host = self._selected_connection_host()
        if not host:
            QMessageBox.warning(self, "Error", "Host IP cannot be empty")
            return

        self.host = host
        self.log_cmd_43210.clear()
        self.log_cmd_46000.clear()
        self.bridge.clear_buttons.emit(self.port1)
        self.bridge.clear_buttons.emit(self.port2)

        self.commands = {
            self.port1: [],
            self.port2: [],
        }
        self.buttons = {
            self.port1: [],
            "action": [],
        }
        self.group_43210.setTitle(f"状态控制")
        self.group_46000.setTitle(f"执行动作")
        self.log_cmd_43210.setPlaceholderText(f"{self.port1} command list")
        self.log_cmd_46000.setPlaceholderText(f"{self.port2} command list")
        if reset_state:
            initial_state = str(
                self.state_graph_config.get("initial_current_state", "init_state")
            )
            self.bridge.set_current_state.emit(initial_state)
        self.bridge.refresh_action_buttons.emit()

        self.load_commands(self.port1)
        self.load_commands(self.port2)

    # ================= 业务逻辑 =================

    def load_commands(self, port: int):

        def worker():
            try:
                asyncio.run(self._load_async(port))
            except Exception as e:
                self.bridge.log.emit(
                    port,
                    LogType.COMMAND_LIST,
                    f"[{port}] Load failed: {e}"
                )

        threading.Thread(target=worker, daemon=True).start()

    async def _load_async(self, port: int):

        self.bridge.log.emit(
            port,
            LogType.COMMAND_LIST,
            f"[{port}] Fetching command list..."
        )

        self.bridge.clear_buttons.emit(port)

        async with RobotClient(self.host, port) as client:
            cmds = await client.get_commands()

        self.commands[port] = cmds

        use_state_graph = False
        if port == self.port1:
            use_state_graph = self._should_use_port1_state_graph(cmds)
            self.bridge.set_port1_graph_mode.emit(use_state_graph)

        for i, cmd in enumerate(cmds):
            self.bridge.log.emit(
                port,
                LogType.COMMAND_LIST,
                f"[{port}:{i}] {cmd.name}: {cmd.desc}"
            )

            if not (port == self.port1 and use_state_graph):
                self.bridge.set_button.emit(
                    port,
                    i,
                    self._format_command_button_text(i, cmd)
                )

    def exec_command(
        self,
        port: int,
        index: int,
        handside: str = "both",
        extra: str = "",
    ):

        if index >= len(self.commands[port]):
            QMessageBox.warning(self, "Error", "Invalid command")
            return

        spec = self.commands[port][index]
        args = {}

        if self._requires_handside(spec):
            args["handside"] = handside

        if extra:
            args["extra"] = extra

        self.bridge.log.emit(
            port,
            LogType.EXECUTION,
            f"[{port}] Executing {spec.name} handside={handside} extra={extra!r}"
        )

        def worker():
            try:
                asyncio.run(self._exec_async(port, spec, args))
            except Exception as e:
                self.bridge.log.emit(
                    port,
                    LogType.EXECUTION,
                    f"[{port}] Execution error: {e}"
                )

        threading.Thread(target=worker, daemon=True).start()

    async def _exec_async(self, port: int, spec: CommandSpec, args: dict):

        async with RobotClient(self.host, port) as client:

            resp = await client.exec_command(spec.name, args=args)

            # self.bridge.log.emit(
            #     port,
            #     LogType.EXECUTION,
            #     f"[{port}] Response: {json.dumps(resp, ensure_ascii=False)}\n"
            # )
            type_ = resp.get("type")
            req_id = resp.get("req_id")
            ok = resp.get("ok")
            data = resp.get("data")
            err = resp.get("err")

            msg = (
                f"[{port}] Response: "
                f'type: "{type_}", req_id: "{req_id}", ok: {ok}, err: {json.dumps(err, ensure_ascii=False)}\n'
                f'data: {json.dumps(data, ensure_ascii=False)}\n'
            )

            should_update_current_state = (
                port == self.port1
                or (port == self.port2 and spec.name.startswith("a"))
            )

            if should_update_current_state and isinstance(data, dict):
                current_state = data.get("current_state")
                if current_state is not None:
                    self.bridge.set_current_state.emit(str(current_state))

            self.bridge.log.emit(
                port,
                LogType.EXECUTION,
                msg
            )


# =========================
# 程序入口
# =========================

if __name__ == "__main__":

    app = QApplication(sys.argv)

    win = MainWindow()
    win.show()

    sys.exit(app.exec())
