import sys
import re
import threading
import time

import paramiko
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


HOST = "192.168.110.105"
USERNAME = "unitree"
PASSWORD = "123"
REMOTE_DIR = "~/unitree_sdk2/build/bin"
RUN_COMMAND = "./g1_remote_monitor_control eth0"
ANSI_ESCAPE_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")


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
        self._connect_lock = threading.Lock()

        self.bridge = UiBridge()
        self.bridge.append_log.connect(self.append_log)
        self.bridge.set_connected.connect(self.on_connected_changed)
        self.bridge.set_running.connect(self.on_running_changed)
        self.bridge.show_error.connect(self.show_error)

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Unitree Remote Monitor")
        self.resize(900, 600)

        self.ip_label = QLabel(f"{HOST}")
        self.connect_button = QPushButton("连接")
        self.run_button = QPushButton("启动")
        self.stop_button = QPushButton("结束")
        self.output_edit = QTextEdit()
        self.output_edit.setReadOnly(True)

        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        button_row = QHBoxLayout()
        button_row.addWidget(self.ip_label)
        button_row.addWidget(self.connect_button)
        button_row.addWidget(self.run_button)
        button_row.addWidget(self.stop_button)

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(button_row)
        main_layout.addWidget(self.output_edit)

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
            """
        )

        self.connect_button.clicked.connect(self.connect_remote)
        self.run_button.clicked.connect(self.run_remote_command)
        self.stop_button.clicked.connect(self.stop_remote_command)

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
        self.connect_button.setEnabled(not connected)
        self.run_button.setEnabled(connected and not self.command_running)
        self.stop_button.setEnabled(connected and self.command_running)

    def on_running_changed(self, running: bool):
        self.command_running = running
        self.run_button.setEnabled(self.connected and not running)
        self.stop_button.setEnabled(self.connected and running)

    def show_error(self, message: str):
        QMessageBox.critical(self, "Error", message)

    def connect_remote(self):
        if self.connected or self.connecting:
            return

        self.connecting = True
        self.connect_button.setEnabled(False)
        self.bridge.append_log.emit(
            f"[INFO] Connecting to {USERNAME}@{HOST}...\n"
        )
        threading.Thread(target=self._connect_remote_worker, daemon=True).start()

    def _connect_remote_worker(self):
        with self._connect_lock:
            try:
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(
                    HOST,
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
