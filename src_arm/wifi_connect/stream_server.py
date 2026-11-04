"""
SAI Stream Server — TCP 帧发送 + UDP 广播

协议:
  TCP帧: [4字节 uint32 LE = jpeg长度] [JPEG数据]
  UDP广播: "SAI_DISCOVER <tcp_port> <width> <height>" 每1秒, 端口12345
"""

import socket
import struct
import threading
import time
import logging

logger = logging.getLogger("StreamServer")


class StreamServer:
    """TCP MJPEG 帧服务器 + UDP 自动发现广播"""

    def __init__(self, tcp_port: int = 8080, udp_port: int = 12345,
                 width: int = 1280, height: int = 720):
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.width = width
        self.height = height

        self._server_sock: socket.socket | None = None
        self._client_sock: socket.socket | None = None
        self._udp_sock: socket.socket | None = None
        self._running = False
        self._broadcast_running = False
        self._lock = threading.Lock()

        # 统计
        self.frames_sent = 0
        self.bytes_sent = 0
        self.fps = 0.0
        self._fps_frames = 0
        self._fps_timer = time.monotonic()

        # 客户端连接事件
        self.on_client_connected = None   # callable(addr)
        self.on_client_disconnected = None
        self.client_addr = None           # (ip, port) of connected client

    def start(self):
        """启动 TCP 监听 + UDP 广播"""
        if self._running:
            return

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._server_sock.bind(("0.0.0.0", self.tcp_port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(1.0)

        self._running = True
        logger.info(f"TCP 服务器启动: 0.0.0.0:{self.tcp_port}")

        # 启动 UDP 广播线程
        self._start_broadcast()

        # 启动 accept 线程
        accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        accept_thread.start()

    def stop(self):
        """停止所有服务"""
        self._running = False
        self._stop_broadcast()

        with self._lock:
            if self._client_sock:
                try:
                    self._client_sock.close()
                except Exception:
                    pass
                self._client_sock = None

        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
            self._server_sock = None

        logger.info("TCP 服务器已停止")

    def send_frame(self, jpeg_data: bytes) -> bool:
        """发送一帧 JPEG 数据, 返回是否成功"""
        with self._lock:
            sock = self._client_sock

        if sock is None:
            return False

        try:
            header = struct.pack("<I", len(jpeg_data))
            sock.sendall(header + jpeg_data)
            self.frames_sent += 1
            self.bytes_sent += len(jpeg_data) + 4

            # FPS 统计
            self._fps_frames += 1
            now = time.monotonic()
            elapsed = now - self._fps_timer
            if elapsed >= 1.0:
                self.fps = self._fps_frames / elapsed
                self._fps_frames = 0
                self._fps_timer = now

            return True
        except (BrokenPipeError, ConnectionResetError, OSError):
            self._disconnect()
            return False

    def is_connected(self) -> bool:
        with self._lock:
            return self._client_sock is not None

    # ---- 内部 ----

    def _accept_loop(self):
        while self._running:
            try:
                client, addr = self._server_sock.accept()
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._lock:
                    if self._client_sock:
                        self._client_sock.close()
                    self._client_sock = client
                self.client_addr = addr
                logger.info(f"客户端连接: {addr[0]}:{addr[1]}")
                if self.on_client_connected:
                    self.on_client_connected(addr[0])
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error(f"accept 错误: {e}")
                break

    def _disconnect(self):
        with self._lock:
            if self._client_sock:
                try:
                    self._client_sock.close()
                except Exception:
                    pass
                self._client_sock = None
        logger.info("客户端断开")
        if self.on_client_disconnected:
            self.on_client_disconnected()

    # ---- UDP 广播 ----

    def _start_broadcast(self):
        self._broadcast_running = True
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        t = threading.Thread(target=self._broadcast_loop, daemon=True)
        t.start()
        logger.info(f"UDP 广播启动: 255.255.255.255:{self.udp_port}")

    def _stop_broadcast(self):
        self._broadcast_running = False
        if self._udp_sock:
            try:
                self._udp_sock.close()
            except Exception:
                pass
            self._udp_sock = None

    def _broadcast_loop(self):
        msg = f"SAI_DISCOVER {self.tcp_port} {self.width} {self.height}"
        data = msg.encode("utf-8")
        while self._broadcast_running:
            try:
                self._udp_sock.sendto(data, ("255.255.255.255", self.udp_port))
            except Exception:
                pass
            time.sleep(1.0)
