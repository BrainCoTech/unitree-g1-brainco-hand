import sys
import time
import json
import asyncio
import threading
import math
from functools import partial
from enum import Enum, auto
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

from PySide6.QtWidgets import (
    QApplication, QWidget, QTextEdit, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox, QLineEdit, QLabel
)
from PySide6.QtCore import QObject, Signal, Slot, QPoint, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygon

from robot_agent.robot_client import RobotClient
from robot_agent.registry import CommandSpec


CONFIG_PATH = Path(__file__).with_name("client_config.yaml")
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
    "highcmd_bg": "#eeeeee",
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
    set_button = Signal(int, int, str)
    clear_buttons = Signal(int)
    set_current_state = Signal(str)
    set_port1_graph_mode = Signal(bool)
    refresh_action_buttons = Signal()


class StateGraphWidget(QWidget):

    def __init__(self, state_graph_config: dict, theme: dict, on_node_clicked):

        super().__init__()

        self.state_graph_config = state_graph_config
        self.theme = theme
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

            title = str(node_cfg.get("title", node_key))
            button = QPushButton(title, self)
            button.clicked.connect(
                lambda _checked=False, key=node_key: self.on_node_clicked(key)
            )
            button.setCheckable(False)
            self.node_buttons[node_key] = button

            if node_key == "config":
                aux_button = QPushButton("Table\nSafe", self)
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

        self.setWindowTitle("Robot Command UI")
        self.resize(1000, 600)

        # ===== 连接参数 =====
        self.host, self.port1, self.port2, self.state_graph_config, self.theme = (
            load_client_config()
        )
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
            "highcmd": []
        }
        self.use_port1_state_graph = False
        self.action_buttons_by_name = {}
        self.action_side_buttons_by_name = {}

        # ===== 信号 =====
        self.bridge = UiBridge()
        self.bridge.log.connect(self._append_log)
        self.bridge.set_button.connect(self._add_button)
        self.bridge.clear_buttons.connect(self._clear_buttons)
        self.bridge.set_current_state.connect(self._set_current_state)
        self.bridge.set_port1_graph_mode.connect(self._set_port1_graph_mode)
        self.bridge.refresh_action_buttons.connect(self._refresh_action_buttons)

        # ================= UI =================

        # --- 顶部 Host 输入 ---
        self.host_input = QLineEdit(self.host)
        self.host_input.setPlaceholderText("Host IP")

        self.port1_input = QLineEdit(str(self.port1))
        self.port1_input.setPlaceholderText("Port 1")

        self.port2_input = QLineEdit(str(self.port2))
        self.port2_input.setPlaceholderText("Port 2")

        self.rescan_button = QPushButton("Rescan")
        self.rescan_button.clicked.connect(self.rescan_commands)

        host_layout = QHBoxLayout()
        host_layout.addWidget(self.host_input, 2)
        host_layout.addWidget(self.port1_input, 1)
        host_layout.addWidget(self.port2_input, 1)
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
        self.group_43210 = QGroupBox(f"Port {self.port1}")
        self.layout_43210 = QVBoxLayout(self.group_43210)
        active_state_color = self.theme.get(
            "action_active_bg", DEFAULT_THEME["action_active_bg"]
        )
        self.current_state_label = QLabel(
            f'current_state: <span style="color: {active_state_color};">{self.current_state}</span>'
        )
        self.current_state_label.setWordWrap(True)
        self.current_state_label.setTextFormat(Qt.TextFormat.RichText)
        self.current_state_label.setStyleSheet("padding-left: 20px;")
        self.layout_43210.addWidget(self.current_state_label)
        self._build_state_graph()

        # --- 右侧按钮区 ---
        self.group_46000 = QGroupBox(f"Port {self.port2}")
        self.layout_46000 = QVBoxLayout(self.group_46000)

        # Action 区
        self.group_action = QGroupBox("Action (Sleep)")
        self.layout_action = QGridLayout(self.group_action)
        self.layout_action.setContentsMargins(9, 9, 9, 9)
        self.layout_action.setHorizontalSpacing(30)
        self.layout_action.setVerticalSpacing(8)

        # HighCmd 区
        self.group_highcmd = QGroupBox("HighCmd")
        self.group_highcmd.setFixedHeight(130)
        self.layout_highcmd = QGridLayout(self.group_highcmd)
        self.layout_highcmd.setContentsMargins(9, 9, 9, 9)
        self.layout_highcmd.setHorizontalSpacing(30)
        self.layout_highcmd.setVerticalSpacing(8)

        self.layout_46000.addWidget(self.group_action)
        self.layout_46000.addWidget(self.group_highcmd)

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
        main_layout.addLayout(host_layout)
        main_layout.addLayout(top_layout, 2)
        main_layout.addWidget(self.log_exec, 1)
        self.log_exec.setMinimumHeight(150)
        self._apply_theme()

        # ===== 加载命令 =====
        self.load_commands(self.port1)
        self.load_commands(self.port2)

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
            f'current_state: <span style="color: {active_state_color};">{self.current_state}</span>'
        )
        self._refresh_state_graph()
        self._refresh_action_buttons()

    @Slot(bool)
    def _set_port1_graph_mode(self, enabled: bool):

        self.use_port1_state_graph = enabled
        self.state_graph_widget.setVisible(enabled)
        self._refresh_state_graph()

    def _build_state_graph(self):

        self.state_graph_widget = StateGraphWidget(
            self.state_graph_config,
            self.theme,
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
                background-color: {theme["window_bg"]};
                color: {theme["text_primary"]};
            }}
            QGroupBox {{
                background-color: {theme["group_bg"]};
                border: 1px solid {theme["border"]};
                border-radius: 10px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: {theme["text_primary"]};
            }}
            QTextEdit, QLineEdit {{
                background-color: {theme["log_bg"]};
                color: {theme["text_primary"]};
                border: 1px solid {theme["log_border"]};
                border-radius: 8px;
                selection-background-color: {theme["selection_bg"]};
            }}
            QTextEdit {{
                font-size: 12px;
            }}
            QLineEdit {{
                background-color: {theme["input_bg"]};
                border: 1px solid {theme["input_border"]};
                padding: 8px 10px;
            }}
            QLineEdit:focus {{
                border: 1px solid {theme["input_focus_border"]};
            }}
            QLabel {{
                color: {theme["text_muted"]};
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

            for widget in self.buttons["highcmd"]:
                self.layout_highcmd.removeWidget(widget)
                widget.deleteLater()

            self.buttons["action"].clear()
            self.buttons["highcmd"].clear()
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
                row_widget.setMaximumWidth(280)
                self.buttons["action"].append(row_widget)
                self.action_buttons_by_name[spec.name] = btn
                self.action_side_buttons_by_name[spec.name] = side_buttons
                self._relayout_action_buttons()
                self._refresh_action_buttons()

            elif name.startswith("h"):
                row_widget.setMaximumWidth(180)
                self.buttons["highcmd"].append(row_widget)
                btn.setStyleSheet(self._build_highcmd_button_style())
                self._relayout_highcmd_buttons()

            else:
                row_widget.setMaximumWidth(300)
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
        sleep_text = self._format_sleep_text(spec)
        if sleep_text:
            return f"{text} ({sleep_text})"
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

    def _build_highcmd_button_style(self) -> str:

        return self._build_action_button_style(
            self.theme.get("highcmd_bg", DEFAULT_THEME["highcmd_bg"]),
            self.theme.get("button_reachable_text", DEFAULT_THEME["button_reachable_text"]),
            self.theme.get("border", DEFAULT_THEME["border"]),
            align_left=True,
            left_padding=18,
            border_width=2,
        )

    def _relayout_highcmd_buttons(self) -> None:

        button_count = len(self.buttons["highcmd"])
        if button_count == 0:
            return

        num_columns = 3
        num_rows = max(1, (button_count + num_columns - 1) // num_columns)

        for index, widget in enumerate(self.buttons["highcmd"]):
            row = index % num_rows
            col = index // num_rows
            self.layout_highcmd.addWidget(widget, row, col)

    def _relayout_action_buttons(self) -> None:

        button_count = len(self.buttons["action"])
        if button_count == 0:
            return

        num_columns = 2
        num_rows = max(1, (button_count + num_columns - 1) // num_columns)

        for index, widget in enumerate(self.buttons["action"]):
            row = index % num_rows
            col = index // num_rows
            self.layout_action.addWidget(widget, row, col)

    def rescan_commands(self):

        host = self.host_input.text().strip()
        if not host:
            QMessageBox.warning(self, "Error", "Host IP cannot be empty")
            return

        port1_text = self.port1_input.text().strip()
        port2_text = self.port2_input.text().strip()
        if not port1_text or not port2_text:
            QMessageBox.warning(self, "Error", "Port cannot be empty")
            return

        try:
            port1 = int(port1_text)
            port2 = int(port2_text)
        except ValueError:
            QMessageBox.warning(self, "Error", "Port must be an integer")
            return

        if port1 <= 0 or port2 <= 0:
            QMessageBox.warning(self, "Error", "Port must be greater than 0")
            return

        if port1 == port2:
            QMessageBox.warning(self, "Error", "The two ports must be different")
            return

        self.host = host
        self.log_cmd_43210.clear()
        self.log_cmd_46000.clear()
        self.bridge.clear_buttons.emit(self.port1)
        self.bridge.clear_buttons.emit(self.port2)

        self.port1 = port1
        self.port2 = port2
        self.commands = {
            self.port1: [],
            self.port2: [],
        }
        self.buttons = {
            self.port1: [],
            "action": [],
            "highcmd": []
        }
        self.group_43210.setTitle(f"Port {self.port1}")
        self.group_46000.setTitle(f"Port {self.port2}")
        self.log_cmd_43210.setPlaceholderText(f"{self.port1} command list")
        self.log_cmd_46000.setPlaceholderText(f"{self.port2} command list")
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
