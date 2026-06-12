import asyncio
import json
import socket
import time
import uuid
from typing import Dict, Any, List
from .registry import CommandSpec

def json_dumps(obj): 
    return json.dumps(obj, ensure_ascii=False)

async def probe_tcp(host: str, port: int = 46000, timeout_s: float = 0.3):
    try:
        reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), timeout=timeout_s)
        try:
            req_id = uuid.uuid4().hex
            writer.write((json_dumps({"type":"ping","req_id":req_id}) + "\n").encode("utf-8"))
            await writer.drain()
            line = await asyncio.wait_for(reader.readline(), timeout=timeout_s)
            if not line:
                return None
            resp = json.loads(line.decode("utf-8"))
            if resp.get("type") != "pong":
                return None
            return host
        finally:
            writer.close()
            await writer.wait_closed()
    except Exception:
        return None

def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

DISCOVERY_PORT = 37020
REPLY_PORT = 37021
TIMEOUT = 3

def json_dumps(obj) -> str:
    return json.dumps(obj, ensure_ascii=False)

async def discover(timeout_s: float = 1.2) -> List[Dict[str, Any]]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(TIMEOUT)

        nonce = uuid.uuid4().hex[:8]
        local_ip = get_local_ip()
        msg = {"type": "discover", "nonce": nonce, "reply_ip": local_ip, "reply_port": REPLY_PORT}
        data = json_dumps(msg).encode("utf-8")
        
        print(f"[Discovery] Broadcasting from {local_ip}:{REPLY_PORT} with nonce {nonce}")

        # 广播：同步发送即可
        sock.sendto(data, ("255.255.255.255", DISCOVERY_PORT))

        found: Dict[str, Dict[str, Any]] = {}
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            try:
                # 在线程里 recvfrom，避免阻塞 asyncio loop
                pkt, addr = await asyncio.to_thread(sock.recvfrom, 65535)
            except socket.timeout:
                continue

            try:
                resp = json.loads(pkt.decode("utf-8"))
            except Exception:
                continue

            if resp.get("type") != "announce":
                continue
            if resp.get("nonce") != nonce:
                continue

            rid = resp.get("id")
            if rid:
                found[rid] = resp

        return list(found.values())
    finally:
        sock.close()

async def tcp_request(writer, reader, req: Dict[str, Any]) -> Dict[str, Any]:
    writer.write((json_dumps(req) + "\n").encode("utf-8"))
    await writer.drain()
    line = await reader.readline()
    if not line:
        return {"ok": False, "err": "no_response"}
    return json.loads(line.decode("utf-8"))

class RobotClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None

    async def connect(self):
        """建立 TCP 连接"""
        self.reader, self.writer = await asyncio.open_connection(self.host, self.port)

    async def disconnect(self):
        """断开 TCP 连接"""
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()

    async def request(self, req_type: str, **kwargs) -> Dict[str, Any]:
        """发送请求"""
        req = {
            "type": req_type,
            "req_id": uuid.uuid4().hex,
            **kwargs
        }
        return await tcp_request(self.writer, self.reader, req)

    async def get_commands(self) -> List[CommandSpec]:
        """获取命令列表"""
        resp = await self.request("get_commands")
        commands: List[CommandSpec] = []
        if resp.get("ok") and "commands" in resp:
            for cmd_data in resp["commands"]:
                spec = CommandSpec(
                    name=cmd_data.get("name", ""),
                    desc=cmd_data.get("desc", ""),
                    args=cmd_data.get("args"),
                    returns=cmd_data.get("returns"),
                    safety=cmd_data.get("safety"),
                )
                commands.append(spec)
        return commands

    async def exec_command(self, cmd_name: str, args: Dict[str, Any] = None) -> Dict[str, Any]:
        """执行命令"""
        return await self.request("exec", cmd=cmd_name, args=args or {})

    async def __aenter__(self):
        """异步上下文管理器入口"""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """异步上下文管理器出口"""
        await self.disconnect()