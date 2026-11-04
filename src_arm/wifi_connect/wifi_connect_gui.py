"""
WiFi connecter — Windows 端 GUI 发送器
桌面 / 摄像头 → MJPEG → TCP 发送
"""

import sys
import time
import socket
import threading
import logging
import json
import struct
from typing import Optional

import cv2
import numpy as np
from mss import MSS as mss_backend

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSlider, QSpinBox, QGroupBox,
    QStatusBar, QCheckBox, QLineEdit, QListWidget, QMessageBox, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap

from stream_server import StreamServer

logging.basicConfig(level=logging.INFO,
                    format="[%(asctime)s] %(levelname)s %(message)s")
logger = logging.getLogger("SenderGUI")


class CaptureSource:
    """抓取接口: 桌面 / 摄像头"""

    MODE_DESKTOP = "desktop"
    MODE_CAMERA = "camera"

    def __init__(self):
        self._mode = self.MODE_DESKTOP
        self._sct: Optional[mss_backend] = None
        self._cap: Optional[cv2.VideoCapture] = None
        self._monitor_index = 1
        self._camera_index = 0

    def set_desktop(self, monitor_index: int = 1):
        self._mode = self.MODE_DESKTOP
        self._monitor_index = monitor_index

    def set_camera(self, index: int = 0):
        self._mode = self.MODE_CAMERA
        self._camera_index = index

    def start(self):
        if self._mode == self.MODE_CAMERA:
            self._cap = cv2.VideoCapture(self._camera_index)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            return self._cap.isOpened()
        self._sct = mss_backend()
        return True

    def stop(self):
        if self._sct:
            self._sct.close()
            self._sct = None
        if self._cap:
            self._cap.release()
            self._cap = None

    def capture(self) -> Optional[np.ndarray]:
        try:
            if self._mode == self.MODE_CAMERA:
                if self._cap is None:
                    return None
                ret, frame = self._cap.read()
                return frame if ret else None
            else:
                if self._sct is None:
                    return None
                monitor = self._sct.monitors[self._monitor_index]
                img = self._sct.grab(monitor)
                frame = np.array(img, dtype=np.uint8)
                return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        except Exception as e:
            logger.error(f"抓取失败 [{self._mode}]: {e}")
            return None


def enum_cameras(max_test: int = 4) -> list[str]:
    cams = []
    for i in range(max_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            cams.append(f"摄像头 {i}")
            cap.release()
    return cams


class SenderWindow(QMainWindow):

    client_connected_signal = pyqtSignal(str)
    client_disconnected_signal = pyqtSignal()
    control_discovered_signal = pyqtSignal(str)
    recognition_result_signal = pyqtSignal(object, str)
    plate_image_signal = pyqtSignal(object, bytes, str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("WiFi connecter - RK3568 全功能控制")
        self.setMinimumSize(680, 760)

        self.capture = CaptureSource()
        self.server = StreamServer(tcp_port=8080, udp_port=12345,
                                   width=1280, height=720)
        self.client_connected_signal.connect(self._on_client_connected)
        self.client_disconnected_signal.connect(self._on_client_disconnected)
        self.server.on_client_connected = self.client_connected_signal.emit
        self.server.on_client_disconnected = self.client_disconnected_signal.emit

        self._streaming = False
        self._stream_thread: Optional[threading.Thread] = None
        self._target_w = 1280
        self._target_h = 720
        self._jpeg_quality = 80
        self._target_fps = 25
        self._adaptive = True
        self._cur_quality = 60
        self._rk_ip = ""
        self._control_probe_running = False
        self._result_port = 12347
        self._result_running = True
        self._result_socket = None
        self._latest_results = {}
        self._image_port = 12348
        self._image_running = True
        self._image_server_socket = None
        self._image_client_socket = None
        self._latest_plate_jpeg = None

        self._setup_ui()
        self._populate_monitors()
        self._refresh_cameras()

        self._local_ip = self._get_local_ip()
        self.ip_label.setText(f"本机IP: {self._local_ip}")

        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._update_stats)
        self._fps_timer.start(500)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")

        self.control_discovered_signal.connect(self._on_control_discovered)
        self.recognition_result_signal.connect(self._on_recognition_result)
        self.plate_image_signal.connect(self._on_plate_image)
        self._control_discovery_timer = QTimer(self)
        self._control_discovery_timer.timeout.connect(self._start_control_probe)
        self._control_discovery_timer.start(2000)
        self._result_thread = threading.Thread(
            target=self._result_receive_loop, daemon=True)
        self._result_thread.start()
        self._image_thread = threading.Thread(
            target=self._image_receive_loop, daemon=True)
        self._image_thread.start()
        self._start_control_probe()

    def _setup_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        central = QWidget()
        scroll.setWidget(central)
        self.setCentralWidget(scroll)
        layout = QVBoxLayout(central)

        # -- 输入源 --
        src_group = QGroupBox("输入源")
        src_layout = QVBoxLayout(src_group)

        btn_row = QHBoxLayout()
        self.btn_desktop = QPushButton("桌面")
        self.btn_desktop.setCheckable(True)
        self.btn_desktop.setChecked(True)
        self.btn_desktop.clicked.connect(lambda: self._switch_mode("desktop"))
        btn_row.addWidget(self.btn_desktop)

        self.btn_camera = QPushButton("摄像头")
        self.btn_camera.setCheckable(True)
        self.btn_camera.clicked.connect(lambda: self._switch_mode("camera"))
        btn_row.addWidget(self.btn_camera)
        src_layout.addLayout(btn_row)

        self.row_desktop = QHBoxLayout()
        self.row_desktop.addWidget(QLabel("显示器:"))
        self.cb_monitor = QComboBox()
        self.row_desktop.addWidget(self.cb_monitor)
        src_layout.addLayout(self.row_desktop)

        self.row_camera = QHBoxLayout()
        self.row_camera.addWidget(QLabel("摄像头:"))
        self.cb_camera = QComboBox()
        self.row_camera.addWidget(self.cb_camera)
        self.row_camera.setEnabled(False)
        src_layout.addLayout(self.row_camera)

        res_row = QHBoxLayout()
        res_row.addWidget(QLabel("输出分辨率:"))
        self.spin_w = QSpinBox()
        self.spin_w.setRange(320, 3840)
        self.spin_w.setValue(self._target_w)
        self.spin_w.valueChanged.connect(self._on_resolution_changed)
        res_row.addWidget(self.spin_w)
        res_row.addWidget(QLabel("×"))
        self.spin_h = QSpinBox()
        self.spin_h.setRange(240, 2160)
        self.spin_h.setValue(self._target_h)
        self.spin_h.valueChanged.connect(self._on_resolution_changed)
        res_row.addWidget(self.spin_h)
        src_layout.addLayout(res_row)

        layout.addWidget(src_group)

        # -- 编码 --
        enc_group = QGroupBox("编码")
        enc_layout = QVBoxLayout(enc_group)
        qual_row = QHBoxLayout()
        qual_row.addWidget(QLabel("JPEG 质量:"))
        self.qual_slider = QSlider(Qt.Horizontal)
        self.qual_slider.setRange(10, 100)
        self.qual_slider.setValue(self._cur_quality)
        self.qual_slider.valueChanged.connect(self._on_quality_changed)
        qual_row.addWidget(self.qual_slider)
        self.qual_label = QLabel(f"{self._cur_quality}%")
        qual_row.addWidget(self.qual_label)
        enc_layout.addLayout(qual_row)

        self.adaptive_cb = QCheckBox("自适应码率")
        self.adaptive_cb.setChecked(True)
        self.adaptive_cb.setToolTip("根据网络状况自动调整 JPEG 质量")
        enc_layout.addWidget(self.adaptive_cb)
        layout.addWidget(enc_group)

        # -- 网络 --
        net_group = QGroupBox("网络")
        net_layout = QHBoxLayout(net_group)
        net_layout.addWidget(QLabel("TCP 端口:"))
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1024, 65535)
        self.port_spin.setValue(8080)
        self.port_spin.valueChanged.connect(self._on_port_changed)
        net_layout.addWidget(self.port_spin)
        net_layout.addStretch()
        self.ip_label = QLabel("本机IP: ...")
        self.ip_label.setStyleSheet("color: #08c; font-weight: bold;")
        net_layout.addWidget(self.ip_label)
        layout.addWidget(net_group)

        # -- 远程控制 --
        ctrl_group = QGroupBox("远程控制 RK3568")
        ctrl_layout = QVBoxLayout(ctrl_group)

        target_row = QHBoxLayout()
        target_row.addWidget(QLabel("RK3568 IP："))
        self.rk_ip_edit = QLineEdit()
        self.rk_ip_edit.setPlaceholderText("自动获取；未连接时使用 UDP 广播")
        target_row.addWidget(self.rk_ip_edit)
        ctrl_layout.addLayout(target_row)

        # 放在远程控制区顶部；保留的按钮与板端对应按键走相同路径。
        key_group = QGroupBox("RK3568 键盘等价控制")
        key_layout = QVBoxLayout(key_group)
        key_rows = (
            (("KEY_R", "[R] 恢复 PCIe 链路"),
             ("KEY_A", "[A] 车牌识别"),
             ("KEY_E", "[E] 增强车牌识别")),
            (("KEY_1", "[1] SCI"), ("KEY_2", "[2] FFDNet"),
             ("KEY_3", "[3] Deblur"), ("KEY_4", "[4] 安全帽检测")),
            (("KEY_5", "[5] 闯红灯检测"),
             ("KEY_6", "[6] 自动驾驶辅助"),
             ("KEY_7", "[7] 斑马线礼让")),
        )
        for row_index, controls in enumerate(key_rows):
            row = QHBoxLayout()
            for command, label in controls:
                button = QPushButton(label)
                button.clicked.connect(
                    lambda _checked=False, cmd=command: self._send_cmd(cmd))
                row.addWidget(button)
            if row_index == len(key_rows) - 1:
                self.btn_remote_quit = QPushButton("[Esc] 退出 RK 程序")
                self.btn_remote_quit.setToolTip("远程结束 RK3568 当前应用")
                self.btn_remote_quit.clicked.connect(self._confirm_remote_quit)
                row.addWidget(self.btn_remote_quit)
            key_layout.addLayout(row)
        ctrl_layout.addWidget(key_group)

        r2 = QHBoxLayout()
        self.btn_src_pcie = QPushButton("切到 PCIe")
        self.btn_src_pcie.clicked.connect(lambda: self._send_cmd("SRC_PCIE"))
        r2.addWidget(self.btn_src_pcie)
        self.btn_src_wifi = QPushButton("切到 WiFi")
        self.btn_src_wifi.clicked.connect(lambda: self._send_cmd("SRC_WIFI"))
        r2.addWidget(self.btn_src_wifi)
        self.btn_src_ipcam = QPushButton("切到 IP Cam")
        self.btn_src_ipcam.clicked.connect(lambda: self._send_cmd("SRC_IPCAM"))
        r2.addWidget(self.btn_src_ipcam)
        ctrl_layout.addLayout(r2)

        ipcam_row = QHBoxLayout()
        ipcam_row.addWidget(QLabel("手机 MJPEG："))
        self.ipcam_url_edit = QLineEdit()
        self.ipcam_url_edit.setPlaceholderText(
            "例如 http://192.168.2.102:8080/video")
        self.ipcam_url_edit.returnPressed.connect(self._apply_ip_camera_url)
        ipcam_row.addWidget(self.ipcam_url_edit)
        self.btn_apply_ipcam = QPushButton("应用地址")
        self.btn_apply_ipcam.clicked.connect(self._apply_ip_camera_url)
        ipcam_row.addWidget(self.btn_apply_ipcam)
        ctrl_layout.addLayout(ipcam_row)

        r3 = QHBoxLayout()
        self.btn_fpga_fmc = QPushButton("FPGA 输入：FMC")
        self.btn_fpga_fmc.clicked.connect(lambda: self._send_cmd("FPGA_FMC"))
        r3.addWidget(self.btn_fpga_fmc)
        self.btn_fpga_hdmi = QPushButton("FPGA 输入：HDMI")
        self.btn_fpga_hdmi.clicked.connect(lambda: self._send_cmd("FPGA_HDMI"))
        r3.addWidget(self.btn_fpga_hdmi)
        self.btn_fpga_fiber = QPushButton("FPGA 输入：光纤")
        self.btn_fpga_fiber.clicked.connect(lambda: self._send_cmd("FPGA_FIBER"))
        r3.addWidget(self.btn_fpga_fiber)
        ctrl_layout.addLayout(r3)

        r4 = QHBoxLayout()
        self.btn_brightness_on = QPushButton("自适应亮度：开")
        self.btn_brightness_on.clicked.connect(
            lambda: self._send_cmd("BRIGHTNESS_AUTO_ON"))
        r4.addWidget(self.btn_brightness_on)
        self.btn_brightness_off = QPushButton("自适应亮度：关")
        self.btn_brightness_off.clicked.connect(
            lambda: self._send_cmd("BRIGHTNESS_AUTO_OFF"))
        r4.addWidget(self.btn_brightness_off)
        self.btn_save_frame = QPushButton("保存 RK3568 当前帧")
        self.btn_save_frame.setToolTip("保存到 RK3568 的 /userdata/captures")
        self.btn_save_frame.clicked.connect(lambda: self._send_cmd("SAVE_FRAME"))
        r4.addWidget(self.btn_save_frame)
        ctrl_layout.addLayout(r4)

        layout.addWidget(ctrl_group)

        # -- RK3568 recognition results --
        result_group = QGroupBox("RK3568 识别结果")
        result_layout = QVBoxLayout(result_group)
        result_header = QHBoxLayout()
        self.result_status_label = QLabel(
            f"等待结果（UDP {self._result_port}）")
        self.result_status_label.setStyleSheet(
            "color: #0af; font-weight: bold;")
        result_header.addWidget(self.result_status_label)
        result_header.addStretch()
        self.btn_clear_results = QPushButton("清空")
        self.btn_clear_results.clicked.connect(self._clear_recognition_results)
        result_header.addWidget(self.btn_clear_results)
        result_layout.addLayout(result_header)
        self.result_list = QListWidget()
        self.result_list.setMinimumHeight(140)
        result_layout.addWidget(self.result_list)
        self.plate_image_info = QLabel(
            f"等待已确认车牌图像（TCP {self._image_port}）")
        self.plate_image_info.setStyleSheet("color: #ccc;")
        result_layout.addWidget(self.plate_image_info)
        self.plate_image_label = QLabel("车牌识别图像")
        self.plate_image_label.setAlignment(Qt.AlignCenter)
        self.plate_image_label.setMinimumHeight(80)
        self.plate_image_label.setStyleSheet(
            "background-color: #111; border: 1px solid #555; color: #777;")
        result_layout.addWidget(self.plate_image_label)
        layout.addWidget(result_group)

        # -- 开始/停止 --
        ctrl_row = QHBoxLayout()
        self.btn_start = QPushButton("▶ 开始发送")
        self.btn_start.setStyleSheet(
            "QPushButton { background-color: #0a0; color: white; font-size: 16px; "
            "font-weight: bold; padding: 8px 20px; border-radius: 6px; }"
            "QPushButton:hover { background-color: #0c0; }")
        self.btn_start.clicked.connect(self._toggle_stream)
        ctrl_row.addWidget(self.btn_start)

        self.btn_stop = QPushButton("■ 停止")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(
            "QPushButton { background-color: #d00; color: white; font-size: 16px; "
            "font-weight: bold; padding: 8px 20px; border-radius: 6px; }"
            "QPushButton:hover { background-color: #f00; }")
        self.btn_stop.clicked.connect(self._toggle_stream)
        ctrl_row.addWidget(self.btn_stop)
        layout.addLayout(ctrl_row)

        self.conn_label = QLabel("● 等待连接")
        self.conn_label.setStyleSheet("color: #888; font-weight: bold; font-size: 14px;")
        layout.addWidget(self.conn_label)

        self.fps_label = QLabel("0 FPS | 0 MB")
        self.fps_label.setStyleSheet("color: #08c; font-weight: bold; font-size: 14px;")
        layout.addWidget(self.fps_label)

    # ---- 模式 ----

    def _switch_mode(self, mode: str):
        self.btn_desktop.setChecked(mode == "desktop")
        self.btn_camera.setChecked(mode == "camera")
        self.row_desktop.setEnabled(mode == "desktop")
        self.row_camera.setEnabled(mode == "camera")
        if mode == "desktop":
            self.capture.set_desktop(self._monitor_list[self.cb_monitor.currentIndex()]
                                     if self.cb_monitor.currentIndex() >= 0 else 1)
        else:
            self.capture.set_camera(self.cb_camera.currentIndex())

    # ---- 枚举 ----

    def _populate_monitors(self):
        try:
            sct = mss_backend()
            monitors = sct.monitors
            sct.close()
            self._monitor_list = []
            self.cb_monitor.blockSignals(True)
            self.cb_monitor.clear()
            for i in range(1, len(monitors)):
                m = monitors[i]
                self._monitor_list.append(i)
                self.cb_monitor.addItem(
                    f"显示器 {i}: {m['width']}×{m['height']}"
                    f"{' [主]' if m.get('is_primary') else ''}")
            self.cb_monitor.blockSignals(False)
            if self.cb_monitor.count() > 0:
                self.cb_monitor.setCurrentIndex(0)
        except Exception as e:
            logger.warning(f"显示器枚举失败: {e}")

    def _refresh_cameras(self):
        cams = enum_cameras()
        self.cb_camera.blockSignals(True)
        self.cb_camera.clear()
        for c in cams:
            self.cb_camera.addItem(c)
        self.cb_camera.blockSignals(False)

    # ---- 参数 ----

    def _on_resolution_changed(self):
        self._target_w = self.spin_w.value()
        self._target_h = self.spin_h.value()
        self.server.width = self._target_w
        self.server.height = self._target_h

    def _on_quality_changed(self, val: int):
        self._cur_quality = val
        self.qual_label.setText(f"{val}%")

    def _on_port_changed(self, val: int):
        self.server.tcp_port = val

    def _start_control_probe(self):
        """Discover RK3568 control service without starting the video stream."""
        if self._control_probe_running:
            return
        manual_ip = self.rk_ip_edit.text().strip()
        self._control_probe_running = True
        threading.Thread(target=self._control_probe_loop,
                         args=(manual_ip,), daemon=True).start()

    def _control_probe_loop(self, manual_ip: str = ""):
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(0.7)

            targets = [manual_ip] if manual_ip else []
            if not manual_ip:
                if self._rk_ip:
                    targets.append(self._rk_ip)
                targets.append("255.255.255.255")
                parts = self._local_ip.split(".")
                if len(parts) == 4 and self._local_ip != "127.0.0.1":
                    directed = ".".join(parts[:3] + ["255"])
                    if directed not in targets:
                        targets.append(directed)

            discover_msg = (
                f"SAI_CTRL_DISCOVER {self._result_port} {self._image_port}"
            ).encode("ascii")
            for target in targets:
                sock.sendto(discover_msg, (target, 12346))

            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                data, peer = sock.recvfrom(256)
                if data.strip() == b"SAI_CTRL_HERE":
                    self.control_discovered_signal.emit(peer[0])
                    break
        except socket.timeout:
            pass
        except OSError as e:
            logger.debug(f"RK3568 control discovery failed: {e}")
        finally:
            if sock:
                sock.close()
            self._control_probe_running = False

    def _on_control_discovered(self, addr: str):
        changed = addr and addr != self._rk_ip
        if addr:
            self._rk_ip = addr
            self.rk_ip_edit.setPlaceholderText(f"控制端已发现：{addr}")
        if changed:
            self.status_bar.showMessage(
                f"已发现 RK3568 常驻控制端：{addr}", 3000)
            logger.info(f"发现 RK3568 常驻控制端: {addr}")

    def _result_receive_loop(self):
        """Receive JSON recognition results independently from video TCP."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self._result_port))
            sock.settimeout(1.0)
            self._result_socket = sock
            logger.info(f"RK3568 result listener: UDP 0.0.0.0:{self._result_port}")
        except OSError as e:
            logger.error(f"RK3568 result listener failed: {e}")
            return

        prefix = b"SAI_RESULT "
        while self._result_running:
            try:
                data, peer = sock.recvfrom(65535)
                if not data.startswith(prefix):
                    continue
                payload = json.loads(data[len(prefix):].decode("utf-8"))
                if isinstance(payload, dict):
                    self.recognition_result_signal.emit(payload, peer[0])
            except socket.timeout:
                continue
            except (OSError, UnicodeDecodeError, json.JSONDecodeError) as e:
                if self._result_running:
                    logger.warning(f"Invalid RK3568 result packet: {e}")

    @staticmethod
    def _recv_exact(sock: socket.socket, size: int):
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def _image_receive_loop(self):
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", self._image_port))
            server.listen(1)
            server.settimeout(1.0)
            self._image_server_socket = server
            logger.info(f"RK3568 plate image listener: TCP 0.0.0.0:{self._image_port}")
        except OSError as e:
            logger.error(f"RK3568 plate image listener failed: {e}")
            return

        while self._image_running:
            try:
                client, peer = server.accept()
                client.settimeout(1.0)
                self._image_client_socket = client
                while self._image_running:
                    try:
                        magic = self._recv_exact(client, 4)
                    except socket.timeout:
                        # Confirmed plates can arrive many seconds apart. Keep
                        # the RK connection alive while no packet is pending.
                        continue
                    if magic is None:
                        break
                    if magic != b"SPIM":
                        raise ValueError("bad plate image magic")
                    lengths = self._recv_exact(client, 8)
                    if lengths is None:
                        break
                    metadata_size, jpeg_size = struct.unpack("<II", lengths)
                    if metadata_size > 65536 or jpeg_size > 4 * 1024 * 1024:
                        raise ValueError("plate image packet too large")
                    metadata_bytes = self._recv_exact(client, metadata_size)
                    jpeg_bytes = self._recv_exact(client, jpeg_size)
                    if metadata_bytes is None or jpeg_bytes is None:
                        break
                    metadata = json.loads(metadata_bytes.decode("utf-8"))
                    self.plate_image_signal.emit(metadata, jpeg_bytes, peer[0])
            except socket.timeout:
                continue
            except (OSError, ValueError, UnicodeDecodeError,
                    json.JSONDecodeError) as e:
                if self._image_running:
                    logger.warning(f"Invalid RK3568 plate image stream: {e}")
            finally:
                client = self._image_client_socket
                self._image_client_socket = None
                if client:
                    try:
                        client.close()
                    except OSError:
                        pass

    def _on_plate_image(self, metadata: dict, jpeg_bytes: bytes, peer_ip: str):
        pixmap = QPixmap()
        if not pixmap.loadFromData(jpeg_bytes, "JPEG"):
            logger.warning("Unable to decode RK3568 plate JPEG")
            return
        self._latest_plate_jpeg = jpeg_bytes
        self.plate_image_label.setPixmap(
            pixmap.scaled(320, 100, Qt.KeepAspectRatio,
                          Qt.SmoothTransformation))
        plate = metadata.get("plate", "")
        track_id = metadata.get("track_id", "-")
        frame_seq = metadata.get("frame_seq", "-")
        self.plate_image_info.setText(
            f"来自 {peer_ip} | 轨迹 #{track_id} | 帧 {frame_seq} | 车牌 {plate}")
        logger.info(f"收到已确认车牌图像: track={track_id} plate={plate}")

    def _on_recognition_result(self, result: dict, peer_ip: str):
        result_type = str(result.get("type", "unknown"))
        self._latest_results[result_type] = result
        if peer_ip:
            self._rk_ip = peer_ip

        self.result_list.clear()
        total_boxes = 0
        for key in ("obb_lpr", "yolo"):
            current = self._latest_results.get(key)
            if not current:
                continue
            boxes = current.get("boxes", [])
            total_boxes += len(boxes)
            frame_seq = current.get("frame_seq", "-")
            source = current.get("source", "-")
            if key == "obb_lpr":
                latency = float(current.get("latency_ms", 0.0))
                self.result_list.addItem(
                    f"【车牌 OBB/LPR】帧 {frame_seq} | {source} | "
                    f"{latency:.1f} ms | {len(boxes)} 个")
                for box in boxes[:50]:
                    name = box.get("class_name", box.get("class_id", "?"))
                    confidence = float(box.get("confidence", 0.0)) * 100.0
                    plate = box.get("plate") or "识别中"
                    vehicle = box.get("vehicle_label") or ""
                    suffix = f" {vehicle}" if vehicle else ""
                    self.result_list.addItem(
                        f"  #{box.get('track_id', '-')} {name} "
                        f"{confidence:.1f}% | {plate}{suffix} | "
                        f"中心({float(box.get('cx', 0)):.0f},"
                        f"{float(box.get('cy', 0)):.0f})")
            else:
                self.result_list.addItem(
                    f"【普通 YOLO】帧 {frame_seq} | {source} | {len(boxes)} 个")
                for box in boxes[:50]:
                    name = box.get("class_name", box.get("class_id", "?"))
                    confidence = float(box.get("confidence", 0.0)) * 100.0
                    self.result_list.addItem(
                        f"  {name} {confidence:.1f}% | "
                        f"({box.get('left', 0)},{box.get('top', 0)})-"
                        f"({box.get('right', 0)},{box.get('bottom', 0)})")

        self.result_status_label.setText(
            f"来自 {peer_ip} | 当前共 {total_boxes} 个目标")

    def _clear_recognition_results(self):
        self._latest_results.clear()
        self.result_list.clear()
        self.result_status_label.setText(
            f"等待结果（UDP {self._result_port}）")
        self._latest_plate_jpeg = None
        self.plate_image_label.clear()
        self.plate_image_label.setText("相同文字确认 3 次后显示")
        self.plate_image_info.setText(
            f"等待已确认车牌图像（TCP {self._image_port}）")

    def _send_cmd(self, cmd: str):
        """发送 UDP 控制命令到 RK3568"""
        manual_ip = self.rk_ip_edit.text().strip()
        target = manual_ip or self._rk_ip or "255.255.255.255"
        try:
            msg = f"SAI_CTRL {cmd}".encode("utf-8")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.4)
            if target == "255.255.255.255":
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(msg, (target, 12346))
            logger.info(f"发送命令: {cmd} → {target}")
            try:
                ack, peer = sock.recvfrom(2048)
                ack_text = ack.decode("utf-8", errors="replace").strip()
                expected = f"SAI_CTRL_ACK {cmd}"
                if ack_text == expected:
                    self._rk_ip = peer[0]
                    if not manual_ip:
                        self.rk_ip_edit.setPlaceholderText(
                            f"已自动识别：{self._rk_ip}")
                    self.status_bar.showMessage(
                        f"RK3568 已收到：{cmd} ← {peer[0]}", 3000)
                    logger.info(f"RK3568 确认: {cmd} ← {peer[0]}")
                else:
                    self.status_bar.showMessage(
                        f"收到未知控制响应：{ack_text}", 5000)
            except socket.timeout:
                self.status_bar.showMessage(
                    f"命令已发往 {target}，但 RK3568 未确认；请检查目标 IP 和板端程序",
                    5000)
                logger.warning(f"RK3568 未确认命令: {cmd} → {target}")
            finally:
                sock.close()
        except Exception as e:
            logger.error(f"命令发送失败: {e}")
            self.status_bar.showMessage(f"RK3568 命令发送失败：{e}", 5000)

    def _confirm_remote_quit(self):
        """Confirm the remote equivalent of the board's Esc key."""
        answer = QMessageBox.question(
            self, "退出 RK3568 程序",
            "确定要远程结束 RK3568 上的当前应用吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer == QMessageBox.Yes:
            self._send_cmd("KEY_ESC")

    def _apply_ip_camera_url(self):
        """Validate and send the phone HTTP MJPEG URL to RK3568."""
        url = self.ipcam_url_edit.text().strip()
        if not url:
            self.status_bar.showMessage("请填写手机 MJPEG 地址", 5000)
            return
        if not url.lower().startswith("http://"):
            self.status_bar.showMessage(
                "初版 IP Cam 仅支持 http:// MJPEG 地址", 5000)
            return
        if any(ch.isspace() for ch in url):
            self.status_bar.showMessage("手机 MJPEG 地址中不能包含空格", 5000)
            return
        self._send_cmd(f"IPCAM_URL {url}")

    @staticmethod
    def _get_local_ip() -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    # ---- 流 ----

    def _toggle_stream(self):
        if self._streaming:
            self._stop_stream()
        else:
            self._start_stream()

    def _start_stream(self):
        if not self.capture.start():
            self.status_bar.showMessage("启动失败")
            return
        self.server.start()
        self._streaming = True
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_desktop.setEnabled(False)
        self.btn_camera.setEnabled(False)
        self.status_bar.showMessage("发送中...")

    def _stop_stream(self):
        self._streaming = False
        if self._stream_thread:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
        self.server.stop()
        self.capture.stop()
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_desktop.setEnabled(True)
        self.btn_camera.setEnabled(True)
        self.conn_label.setText("● 已停止")
        self.conn_label.setStyleSheet("color: #888; font-weight: bold; font-size: 14px;")
        self.status_bar.showMessage("就绪")

    def _stream_loop(self):
        frame_interval = 1.0 / self._target_fps
        last_frame_time = time.monotonic()

        while self._streaming:
            now = time.monotonic()
            elapsed = now - last_frame_time

            # 跳帧: 如果已经落后超过 1 帧, 直接跳到最新
            if elapsed < frame_interval:
                time.sleep(max(frame_interval - elapsed - 0.002, 0.0))
                continue

            # 落后超过 2 帧, 重置时钟避免追赶积压
            if elapsed > frame_interval * 2:
                last_frame_time = now
            else:
                last_frame_time += frame_interval

            # 捕获
            t0 = time.monotonic()
            frame = self.capture.capture()
            if frame is None:
                continue

            # 缩放
            h, w = frame.shape[:2]
            if w != self._target_w or h != self._target_h:
                frame = cv2.resize(frame, (self._target_w, self._target_h),
                                   interpolation=cv2.INTER_LINEAR)

            # 自适应质量
            q = self._cur_quality
            if self.adaptive_cb.isChecked():
                q = self._cur_quality
                self.qual_slider.blockSignals(True)
                self.qual_slider.setValue(q)
                self.qual_slider.blockSignals(False)
                self.qual_label.setText(f"{q}%")

            # 编码
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, q]
            success, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not success:
                continue

            # 发送 + 测速
            send_start = time.monotonic()
            ok = self.server.send_frame(jpeg.tobytes())
            send_ms = (time.monotonic() - send_start) * 1000

            # 发送失败 (客户端断开), 短暂等待后继续
            if not ok:
                time.sleep(0.1)
                continue

            # 自适应: 根据实际耗时调整质量
            if self.adaptive_cb.isChecked():
                target_ms = frame_interval * 1000 * 0.7
                if send_ms > target_ms * 1.5:
                    self._cur_quality = max(20, q - 5)
                elif send_ms < target_ms * 0.5 and q < 85:
                    self._cur_quality = min(85, q + 3)

    def _on_client_connected(self, addr: str = ""):
        if addr:
            self._rk_ip = addr
            self.rk_ip_edit.setPlaceholderText(f"已自动识别：{addr}")
        self.conn_label.setText(f"● 已连接 {addr}")
        self.conn_label.setStyleSheet("color: #0a0; font-weight: bold; font-size: 14px;")

    def _on_client_disconnected(self):
        self.conn_label.setText("● 等待连接")
        self.conn_label.setStyleSheet("color: #f80; font-weight: bold; font-size: 14px;")
        self._start_control_probe()

    def _update_stats(self):
        if self._streaming:
            fps = self.server.fps
            mb = self.server.bytes_sent / (1024 * 1024)
            self.fps_label.setText(
                f"{fps:.1f} FPS | {mb:.1f} MB ({self.server.frames_sent} 帧)")

    def closeEvent(self, event):
        self._result_running = False
        if self._result_socket:
            try:
                self._result_socket.close()
            except OSError:
                pass
            self._result_socket = None
        self._image_running = False
        for image_socket in (self._image_client_socket, self._image_server_socket):
            if image_socket:
                try:
                    image_socket.close()
                except OSError:
                    pass
        self._image_client_socket = None
        self._image_server_socket = None
        self._stop_stream()
        super().closeEvent(event)


def main():
    try:
        import ctypes
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet("""
        QMainWindow, QWidget { background-color: #1e1e1e; color: #ddd; }
        QGroupBox { border: 1px solid #444; border-radius: 4px;
            margin-top: 8px; padding-top: 16px; font-weight: bold; }
        QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
        QPushButton { background-color: #333; border: 1px solid #555;
            border-radius: 4px; padding: 6px 12px; }
        QPushButton:hover { background-color: #444; }
        QPushButton:checked { background-color: #0078d4; color: white; }
        QPushButton:disabled { background-color: #222; color: #666; }
        QComboBox { background-color: #2a2a2a; border: 1px solid #555;
            border-radius: 3px; padding: 3px 6px; color: #ddd; }
        QSpinBox, QSlider { background-color: #2a2a2a; border: 1px solid #555;
            border-radius: 3px; padding: 3px; }
        QLabel { color: #ccc; }
        QStatusBar { background-color: #111; color: #888; }
    """)

    window = SenderWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
