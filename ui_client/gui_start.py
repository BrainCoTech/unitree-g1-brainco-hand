import sys
import re
import threading
import time
import unicodedata
from pathlib import Path

import paramiko
try:
    import yaml
except ImportError:
    yaml = None

from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


CONFIG_PATH = Path(__file__).with_name("gui_config.yaml")
DEFAULT_WIRED_HOST = "192.168.123.164"
DEFAULT_WIRELESS_HOST = "192.168.110.105"
DEFAULT_PORT1 = 43210
DEFAULT_PORT2 = 46000
DEFAULT_GUIDE_COLUMNS = ["编号", "按键", "模式/动作", "备注"]
DEFAULT_GUIDE_ROWS = [
    {"id": "①", "keys": "L2 + B", "action": "阻尼模式", "note": ""},
    {"id": "②", "keys": "L2 + ↑", "action": "预备模式", "note": "机器人不能站立"},
    {"id": "③", "keys": "R1 + X", "action": "常规运控模式", "note": "机器人可以站立"},
    {"id": "④", "keys": "L2 + START", "action": "启动程序", "note": ""},
    {"id": "⑤", "keys": "L2 + B", "action": "结束程序", "note": "软急停"},
    {"id": "⑥", "keys": "L2 + B 长按5s以上", "action": "急停", "note": "机器人不再能站立"},
    {"id": "⑪", "keys": "F1 + Y", "action": "打招呼", "note": ""},
    {"id": "⑫", "keys": "F1 + B", "action": "点赞", "note": ""},
    {"id": "⑬", "keys": "F1 + A", "action": "握手", "note": ""},
    {"id": "⑭", "keys": "F1 + X", "action": "石头剪刀布", "note": ""},
    {"id": "㉑", "keys": "F3 + ↑", "action": "抬起手臂（端托盘姿势）", "note": ""},
    {"id": "㉒", "keys": "F3 + ←", "action": "握紧（托盘）", "note": ""},
    {"id": "㉓", "keys": "F3 + →", "action": "松开（托盘）", "note": ""},
    {"id": "㉔", "keys": "F3 + ↓", "action": "放下手臂", "note": ""},
]
DEFAULT_STARTUP_SEQUENCE = ["开机", "1", "2", "3", "启动软件程序", "4"]
DEFAULT_SHUTDOWN_SEQUENCE = ["5", "结束软件程序", "挂上吊架", "6", "关机"]
WIRED_HOST = DEFAULT_WIRED_HOST
WIRELESS_HOST = DEFAULT_WIRELESS_HOST
PORT1 = DEFAULT_PORT1
PORT2 = DEFAULT_PORT2
GUIDE_COLUMNS = DEFAULT_GUIDE_COLUMNS
GUIDE_ROWS = DEFAULT_GUIDE_ROWS
STARTUP_SEQUENCE = DEFAULT_STARTUP_SEQUENCE
SHUTDOWN_SEQUENCE = DEFAULT_SHUTDOWN_SEQUENCE
USERNAME = "unitree"
PASSWORD = "123"
REMOTE_DIR = "~/unitree_sdk2/build/bin"
RUN_COMMAND = "./g1_remote_monitor_control eth0"
ANSI_ESCAPE_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


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

        try:
            config[key] = int(value)
            continue
        except ValueError:
            pass

        config[key] = value

    return config


def _normalize_guide_columns(data) -> list[str]:
    if not isinstance(data, list):
        return list(DEFAULT_GUIDE_COLUMNS)

    columns = [str(item).strip() for item in data if str(item).strip()]
    return columns if columns else list(DEFAULT_GUIDE_COLUMNS)


def _normalize_guide_rows(data) -> list[dict[str, str]]:
    if not isinstance(data, list):
        return [dict(row) for row in DEFAULT_GUIDE_ROWS]

    rows = []
    for item in data:
        if not isinstance(item, dict):
            continue
        rows.append(
            {
                "id": str(item.get("id", "")).strip(),
                "keys": str(item.get("keys", "")).strip(),
                "action": str(item.get("action", "")).strip(),
                "note": str(item.get("note", "")).strip(),
            }
        )

    return rows if rows else [dict(row) for row in DEFAULT_GUIDE_ROWS]


def _normalize_sequence(data, default_sequence: list[str]) -> list[str]:
    if not isinstance(data, list):
        return list(default_sequence)

    sequence = [str(item).strip() for item in data if str(item).strip()]
    return sequence if sequence else list(default_sequence)


def load_gui_config() -> tuple[str, str, int, int, list[str], list[dict[str, str]], list[str], list[str]]:
    wired_host = DEFAULT_WIRED_HOST
    wireless_host = DEFAULT_WIRELESS_HOST
    port1 = DEFAULT_PORT1
    port2 = DEFAULT_PORT2
    guide_columns = list(DEFAULT_GUIDE_COLUMNS)
    guide_rows = [dict(row) for row in DEFAULT_GUIDE_ROWS]
    startup_sequence = list(DEFAULT_STARTUP_SEQUENCE)
    shutdown_sequence = list(DEFAULT_SHUTDOWN_SEQUENCE)

    if not CONFIG_PATH.exists():
        return (
            wired_host,
            wireless_host,
            port1,
            port2,
            guide_columns,
            guide_rows,
            startup_sequence,
            shutdown_sequence,
        )

    try:
        text = CONFIG_PATH.read_text(encoding="utf-8")
        if yaml is not None:
            data = yaml.safe_load(text) or {}
        else:
            data = _parse_simple_yaml(text)
    except Exception:
        return (
            wired_host,
            wireless_host,
            port1,
            port2,
            guide_columns,
            guide_rows,
            startup_sequence,
            shutdown_sequence,
        )

    if not isinstance(data, dict):
        return (
            wired_host,
            wireless_host,
            port1,
            port2,
            guide_columns,
            guide_rows,
            startup_sequence,
            shutdown_sequence,
        )

    legacy_host = str(data.get("host", wired_host)).strip()
    wired_host = str(data.get("wired_host", legacy_host)).strip() or wired_host
    wireless_host = (
        str(data.get("wireless_host", wireless_host)).strip() or wireless_host
    )

    try:
        port1 = int(data.get("port1", port1))
    except (TypeError, ValueError):
        port1 = DEFAULT_PORT1

    try:
        port2 = int(data.get("port2", port2))
    except (TypeError, ValueError):
        port2 = DEFAULT_PORT2

    guide_table = data.get("guide_table", {})
    if isinstance(guide_table, dict):
        guide_columns = _normalize_guide_columns(guide_table.get("columns"))
        guide_rows = _normalize_guide_rows(guide_table.get("rows"))

    startup_sequence = _normalize_sequence(
        data.get("startup_sequence"), DEFAULT_STARTUP_SEQUENCE
    )
    shutdown_sequence = _normalize_sequence(
        data.get("shutdown_sequence"), DEFAULT_SHUTDOWN_SEQUENCE
    )

    return (
        wired_host,
        wireless_host,
        port1,
        port2,
        guide_columns,
        guide_rows,
        startup_sequence,
        shutdown_sequence,
    )


(
    WIRED_HOST,
    WIRELESS_HOST,
    PORT1,
    PORT2,
    GUIDE_COLUMNS,
    GUIDE_ROWS,
    STARTUP_SEQUENCE,
    SHUTDOWN_SEQUENCE,
) = load_gui_config()


class UiBridge(QObject):
    append_log = Signal(str)
    set_connected = Signal(bool)
    set_running = Signal(bool)
    show_error = Signal(str)


class RemoteMonitorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.client = None
        self.shell = None
        self.reader_thread = None
        self.connecting = False
        self.connected = False
        self.command_running = False
        self.stop_reader = False
        self.active_host = ""
        self._connect_lock = threading.Lock()

        self.bridge = UiBridge()
        self.bridge.append_log.connect(self.append_log)
        self.bridge.set_connected.connect(self.on_connected_changed)
        self.bridge.set_running.connect(self.on_running_changed)
        self.bridge.show_error.connect(self.show_error)

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Unitree Remote Monitor")
        self.resize(1380, 760)

        self.wired_label = QLabel(f"有线 IP  {WIRED_HOST}")
        self.wired_connect_button = QPushButton("连接")
        self.wireless_label = QLabel(f"无线 IP  {WIRELESS_HOST}")
        self.wireless_connect_button = QPushButton("连接")
        self.run_button = QPushButton("启动")
        self.stop_button = QPushButton("结束")
        self.output_edit = QTextEdit()
        self.output_edit.setReadOnly(True)
        self.output_edit.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.guide_table_group = self.create_guide_table_group()
        self.startup_group = self.create_sequence_group(
            "启动顺序", " -> ".join(STARTUP_SEQUENCE)
        )
        self.shutdown_group = self.create_sequence_group(
            "结束顺序", " -> ".join(SHUTDOWN_SEQUENCE)
        )

        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        wired_row = QHBoxLayout()
        wired_row.addWidget(self.wired_label)
        wired_row.addWidget(self.wired_connect_button)
        wired_row.addSpacing(28)
        wired_row.addWidget(self.run_button)
        wired_row.addStretch()

        wireless_row = QHBoxLayout()
        wireless_row.addWidget(self.wireless_label)
        wireless_row.addWidget(self.wireless_connect_button)
        wireless_row.addSpacing(28)
        wireless_row.addWidget(self.stop_button)
        wireless_row.addStretch()

        left_layout = QVBoxLayout()
        left_layout.addLayout(wired_row)
        left_layout.addLayout(wireless_row)
        left_layout.addWidget(self.output_edit)

        guide_layout = QVBoxLayout()
        guide_layout.addWidget(self.guide_table_group, 1)
        guide_layout.addWidget(self.startup_group)
        guide_layout.addWidget(self.shutdown_group)

        content_layout = QHBoxLayout()
        content_layout.addLayout(left_layout, 1)
        content_layout.addLayout(guide_layout, 1)

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(content_layout)

        self.setStyleSheet(
            """
            QWidget {
                background-color: #ffffff;
                color: #222222;
            }
            QLabel {
                font-size: 18px;
                font-weight: 600;
                padding-right: 10px;
                color: #222222;
            }
            QPushButton {
                background-color: #e4f0fb;
                border: 2px solid #93b4d4;
                border-radius: 6px;
                min-height: 54px;
                min-width: 120px;
                padding: 12px 18px;
                font-size: 20px;
                font-weight: 600;
                color: #0f1720;
            }
            QPushButton:hover {
                background-color: #d2e5f8;
            }
            QPushButton:disabled {
                background-color: #c7cdd4;
                color: #5f6975;
                border: 2px solid #a9b1ba;
            }
            QTextEdit {
                background-color: #fafbfc;
                border: 1px solid #d7dde5;
                color: #222222;
                font-family: "DejaVu Sans Mono";
                font-size: 13px;
                selection-background-color: #dbeafe;
            }
            QGroupBox {
                border: 1px solid #c7d2df;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 14px;
                font-size: 16px;
                font-weight: 600;
                color: #1f2937;
                background-color: #f8fafc;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }
            QTableWidget {
                background-color: #fafbfc;
                border: 1px solid #d7dde5;
                gridline-color: #d7dde5;
                font-size: 14px;
                color: #222222;
            }
            QHeaderView::section {
                background-color: #e8eef5;
                color: #1f2937;
                padding: 8px;
                border: 0;
                border-right: 1px solid #d7dde5;
                border-bottom: 1px solid #d7dde5;
                font-size: 14px;
                font-weight: 600;
            }
            """
        )

        self.wired_connect_button.clicked.connect(
            lambda: self.connect_remote(WIRED_HOST)
        )
        self.wireless_connect_button.clicked.connect(
            lambda: self.connect_remote(WIRELESS_HOST)
        )
        self.run_button.clicked.connect(self.run_remote_command)
        self.stop_button.clicked.connect(self.stop_remote_command)

    def parse_guide_id(self, value: str) -> int | None:
        text = str(value).strip()
        if not text:
            return None

        if re.fullmatch(r"\d+", text):
            return int(text)

        if len(text) == 1:
            try:
                numeric_value = unicodedata.numeric(text)
                if float(numeric_value).is_integer():
                    return int(numeric_value)
            except (TypeError, ValueError):
                pass

        digit_matches = re.findall(r"\d+", text)
        if digit_matches:
            return int(digit_matches[0])

        return None

    def build_guide_table(self, rows: list[dict[str, str]]) -> QTableWidget:
        table = QTableWidget(len(rows), len(GUIDE_COLUMNS))
        table.setHorizontalHeaderLabels(GUIDE_COLUMNS)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        table.setAlternatingRowColors(True)
        table.setWordWrap(True)
        table.setMinimumWidth(560)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)

        for row_index, row in enumerate(rows):
            values = [
                row.get("id", ""),
                row.get("keys", ""),
                row.get("action", ""),
                row.get("note", ""),
            ]
            for col_index, value in enumerate(values):
                item = QTableWidgetItem(value)
                table.setItem(row_index, col_index, item)

        table.resizeRowsToContents()
        return table

    def create_guide_table_group(self) -> QGroupBox:
        group = QGroupBox("按键说明")
        layout = QVBoxLayout(group)
        layout.setSpacing(12)

        grouped_rows = {0: [], 1: []}
        for row in GUIDE_ROWS:
            guide_id = self.parse_guide_id(row.get("id", ""))
            if guide_id is None:
                grouped_rows[1].append(row)
                continue
            if 1 <= guide_id <= 10:
                grouped_rows[0].append(row)
            else:
                grouped_rows[1].append(row)

        for index, rows in grouped_rows.items():
            if rows:
                layout.addWidget(self.build_guide_table(rows))
            if index < 1:
                layout.addSpacing(14)

        return group

    def create_sequence_group(self, title: str, text: str) -> QGroupBox:
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        label = QLabel(text)
        label.setWordWrap(True)
        label.setStyleSheet(
            "padding: 8px 10px; font-size: 15px; font-weight: 500; color: #1f2937;"
        )
        layout.addWidget(label)
        return group

    def append_log(self, text: str):
        self.output_edit.moveCursor(QTextCursor.End)
        self.output_edit.insertPlainText(text)
        self.output_edit.moveCursor(QTextCursor.End)

    def sanitize_output(self, text: str) -> str:
        text = text.replace("\x00", "")
        text = ANSI_ESCAPE_RE.sub("", text)
        text = text.replace("\r\n", "\n").replace("\r", "\n")
        return "".join(
            ch for ch in text if ch == "\n" or ch == "\t" or ch >= " "
        )

    def on_connected_changed(self, connected: bool):
        self.connected = connected
        self.connecting = False
        self.wired_connect_button.setEnabled(not connected)
        self.wireless_connect_button.setEnabled(not connected)
        self.run_button.setEnabled(connected and not self.command_running)
        self.stop_button.setEnabled(connected and self.command_running)

    def on_running_changed(self, running: bool):
        self.command_running = running
        self.run_button.setEnabled(self.connected and not running)
        self.stop_button.setEnabled(self.connected and running)

    def show_error(self, message: str):
        QMessageBox.critical(self, "Error", message)

    def connect_remote(self, host: str):
        if self.connected or self.connecting:
            return

        self.active_host = host
        self.connecting = True
        self.wired_connect_button.setEnabled(False)
        self.wireless_connect_button.setEnabled(False)
        self.bridge.append_log.emit(
            f"[INFO] Connecting to {USERNAME}@{host}...\n"
        )
        threading.Thread(
            target=self._connect_remote_worker,
            args=(host,),
            daemon=True,
        ).start()

    def _connect_remote_worker(self, host: str):
        with self._connect_lock:
            try:
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(
                    host,
                    username=USERNAME,
                    password=PASSWORD,
                    timeout=10,
                )
                shell = client.invoke_shell()
                shell.settimeout(0.2)
                time.sleep(0.5)

                self.client = client
                self.shell = shell
                self.stop_reader = False

                self._drain_initial_output()
                self.shell.send(f"cd {REMOTE_DIR}\n")
                time.sleep(0.3)
                self._drain_initial_output()
                self.bridge.append_log.emit(
                    f"[INFO] Connected. Current directory set to {REMOTE_DIR}\n"
                )

                self.reader_thread = threading.Thread(
                    target=self._read_shell_output,
                    daemon=True,
                )
                self.reader_thread.start()
                self.bridge.set_connected.emit(True)
            except Exception as exc:
                self._close_connection()
                self.connecting = False
                self.bridge.set_connected.emit(False)
                self.bridge.show_error.emit(f"SSH connection failed: {exc}")

    def _drain_initial_output(self):
        if not self.shell:
            return
        for _ in range(10):
            if not self.shell.recv_ready():
                break
            data = self.shell.recv(4096)
            if not data:
                break
            text = data.decode("utf-8", errors="replace")
            cleaned = self.sanitize_output(text)
            if cleaned:
                self.bridge.append_log.emit(cleaned)
            time.sleep(0.05)

    def _read_shell_output(self):
        while not self.stop_reader and self.shell is not None:
            try:
                if self.shell.recv_ready():
                    data = self.shell.recv(4096)
                    if not data:
                        self.bridge.append_log.emit("\n[INFO] Remote shell closed.\n")
                        break

                    text = data.decode("utf-8", errors="replace")
                    cleaned = self.sanitize_output(text)
                    if cleaned:
                        self.bridge.append_log.emit(cleaned)

                    if "[sudo] password for" in cleaned:
                        self.shell.send(f"{PASSWORD}\n")
                        self.bridge.append_log.emit("[INFO] Sent sudo password automatically.\n")
                else:
                    time.sleep(0.1)
            except Exception as exc:
                if not self.stop_reader:
                    self.bridge.append_log.emit(f"\n[ERROR] Read failed: {exc}\n")
                break

        self._close_connection()
        self.bridge.set_running.emit(False)
        self.bridge.set_connected.emit(False)

    def run_remote_command(self):
        if not self.connected or self.shell is None:
            self.bridge.show_error.emit("SSH is not connected yet.")
            return
        if self.command_running:
            return

        self.output_edit.clear()
        self.bridge.append_log.emit(f"$ cd {REMOTE_DIR}\n")
        self.bridge.append_log.emit(f"$ {RUN_COMMAND}\n")
        self.bridge.set_running.emit(True)

        try:
            self.shell.send(f"cd {REMOTE_DIR}\n")
            time.sleep(0.1)
            self.shell.send(f"{RUN_COMMAND}\n")
        except Exception as exc:
            self.bridge.set_running.emit(False)
            self.bridge.show_error.emit(f"Failed to start remote command: {exc}")

    def stop_remote_command(self):
        if not self.connected or self.shell is None or not self.command_running:
            return

        self._stop_remote_command_internal(show_error=True, log_message=True)

    def _stop_remote_command_internal(self, show_error: bool, log_message: bool):
        try:
            self.shell.send("\x03")
            self.shell.send("\n")
            time.sleep(0.2)
            if log_message:
                self.bridge.append_log.emit("\n[INFO] Sent Ctrl+C to stop remote command.\n")
        except Exception as exc:
            if show_error:
                self.bridge.show_error.emit(f"Failed to stop remote command: {exc}")
        finally:
            self.bridge.set_running.emit(False)

    def _close_connection(self):
        self.stop_reader = True

        if self.shell is not None:
            try:
                self.shell.close()
            except Exception:
                pass
            self.shell = None

        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None

    def closeEvent(self, event):
        if self.command_running and self.shell is not None:
            self._stop_remote_command_internal(show_error=False, log_message=False)

        self._close_connection()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = RemoteMonitorWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
