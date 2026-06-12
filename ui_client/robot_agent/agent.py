from __future__ import annotations
import asyncio
import socket
import time
from typing import Any, Dict, Optional

from .protocol import json_dumps, json_loads, commands_hash
from .registry import CommandRegistry

DISCOVERY_PORT = 37020

def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()
     

class RobotAgent(asyncio.DatagramProtocol):
    """
    可嵌入模块：robot 业务侧只需要
      - 初始化 RobotAgent(...)
      - registry.register(...) 注册指令集
      - await agent.start()
    """
    def __init__(
        self,
        *,
        robot_id: str,
        name: str,
        registry: CommandRegistry,
        tcp_port: int = 46000,
        discovery_port: int = DISCOVERY_PORT,
        host: Optional[str] = None,
    ) -> None:
        self.robot_id = robot_id
        self.name = name
        self.registry = registry
        self.tcp_port = tcp_port
        self.discovery_port = discovery_port
        self.host = host or get_local_ip()

        self._tcp_server: Optional[asyncio.AbstractServer] = None
        self._udp_task: Optional[asyncio.Task] = None

    def _announce_payload(self, nonce: Any) -> Dict[str, Any]:
        cmds = self.registry.list_specs()
        return {
            "type": "announce",
            "id": self.robot_id,
            "name": self.name,
            "host": self.host,
            "tcp_port": self.tcp_port,
            "commands_hash": commands_hash(cmds),
            "schema_version": "1.0",
            "ts": int(time.time() * 1000),
            "nonce": nonce,
        }

    # implementation of asyncio.DatagramProtocol
    def connection_made(self, transport):
        self.transport = transport
        sockname = transport.get_extra_info("sockname")
        print(f"[server] UDP discovery listening on {sockname}")

    def datagram_received(self, data, addr):
        print(f"[agent] UDP packet from {addr}")
        try:
            msg = json_loads(data.decode("utf-8"))
        except Exception:
            return

        if msg.get("type") != "discover":
            return

        reply_port = int(msg.get("reply_port", 0))
        if reply_port <= 0 or reply_port > 65535:
            return
        
        resp = self._announce_payload(nonce=msg.get("nonce"))
        out = json_dumps(resp).encode("utf-8")
        self.transport.sendto(out, addr)
        print(f"[server] Replied to {addr}")   
    # end of implementation of asyncio.DatagramProtocol

    async def start(self) -> None:
        # UDP discovery
        self._udp_task = asyncio.create_task(self._udp_discovery_loop())

        # TCP control
        self._tcp_server = await asyncio.start_server(
            self._handle_tcp_client, host="0.0.0.0", port=self.tcp_port
        )

        addrs = ", ".join(str(s.getsockname()) for s in self._tcp_server.sockets or [])
        print(f"[agent] TCP listening on {addrs}")
        print(f"[agent] UDP discovery listening on 0.0.0.0:{self.discovery_port}")

    async def serve_forever(self) -> None:
        if not self._tcp_server:
            raise RuntimeError("agent not started")
        async with self._tcp_server:
            await self._tcp_server.serve_forever()

    async def stop(self) -> None:
        if self._udp_task:
            self._udp_task.cancel()
        if self._tcp_server:
            self._tcp_server.close()
            await self._tcp_server.wait_closed()

    async def _udp_discovery_loop(self) -> None:
        loop = asyncio.get_running_loop()

        transport, protocol = await loop.create_datagram_endpoint(
            lambda: self,
            local_addr=("0.0.0.0", self.discovery_port),
            allow_broadcast=True,
        )

        try:
            # UDP 服务通常一直跑
            await asyncio.Future()
        finally:
            transport.close()

    async def _handle_tcp_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        print(f"[agent] client connected: {peer}")
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break

                try:
                    req = json_loads(line.decode("utf-8"))
                except Exception:
                    await self._send(writer, {"type": "error", "ok": False, "err": "bad_json"})
                    continue

                typ = req.get("type")
                req_id = req.get("req_id")

                if typ == "ping":
                    await self._send(writer, {"type": "pong", "req_id": req_id, "ok": True})
                    continue

                if typ == "get_commands":
                    await self._send(writer, {
                        "type": "commands",
                        "req_id": req_id,
                        "ok": True,
                        "schema_version": "1.0",
                        "commands": self.registry.list_specs(),
                    })
                    continue

                if typ == "exec":
                    cmd = req.get("cmd")
                    args = req.get("args", {}) or {}

                    handler = self.registry.get_handler(cmd)
                    if not handler:
                        await self._send(writer, {"type": "result", "req_id": req_id, "ok": False, "err": "unknown_cmd"})
                        continue

                    try:
                        data = await handler(args)  # 业务侧处理
                        await self._send(writer, {"type": "result", "req_id": req_id, "ok": True, "data": data, "err": None})
                    except Exception as e:
                        await self._send(writer, {"type": "result", "req_id": req_id, "ok": False, "data": None, "err": f"exception:{e}"})
                    continue

                await self._send(writer, {"type": "error", "req_id": req_id, "ok": False, "err": "unknown_type"})
        finally:
            print(f"[agent] client disconnected: {peer}")
            writer.close()
            await writer.wait_closed()

    async def _send(self, writer: asyncio.StreamWriter, obj: Dict[str, Any]) -> None:
        writer.write((json_dumps(obj) + "\n").encode("utf-8"))
        await writer.drain()