#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
sensors_service_multi.py
------------------------

Multi-drone version of your telemetry service. Runs one MAVSDK client per drone,
each with its own UDP link and gRPC port (e.g., 14540->50051, 14541->50052).

Endpoints:
  /sensors            -> snapshots for all drones (dict keyed by drone id)
  /sensors?drone=ID   -> snapshot for a single drone (compat mode)
  /sensors/{drone_id} -> snapshot for a single drone

Usage examples (two drones):

  python sensors_service_multi.py \
      --vehicle id=1,url=udp://:14540,grpc=50051 \
      --vehicle id=2,url=udp://:14541,grpc=50052 \
      --hz 1.0 --host 0.0.0.0 --port 8001

If you omit --vehicle flags, it defaults to:
  id=1,url=udp://:14540,grpc=50051
  id=2,url=udp://:14541,grpc=50052
"""

import argparse
import asyncio
import json
import math
import re
import signal
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

try:
    from mavsdk import System
    from mavsdk.telemetry import FlightMode, FixType  # noqa: F401  (kept for parity)
except ImportError:
    System = None  # type: ignore

import uvicorn
from fastapi import FastAPI, Query

# ------------------------------ helpers ------------------------------

def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")

def fmt(value: Optional[Any], nd: int = 2, unit: str = "", percent: bool = False) -> str:
    if value is None:
        return "N/A"
    try:
        v = float(value)
    except Exception:
        return str(value)
    if percent:
        return f"{v:.{nd}f}%"
    return f"{v:.{nd}f}{unit}"

def quat_to_euler_deg(w: float, x: float, y: float, z: float) -> Dict[str, float]:
    t0 = 2.0 * (w * x + y * z)
    t1 = w * w - x * x - y * y - z * z
    roll = math.degrees(math.atan2(t0, t1))
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else (-1.0 if t2 < -1.0 else t2)
    pitch = math.degrees(math.asin(t2))
    t3 = 2.0 * (w * z + x * y)
    t4 = w * w - x * x - y * y - z * z
    yaw = math.degrees(math.atan2(t3, t4))
    return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw}

# --------------------------- per-drone monitor ------------------------

class TelemetryMonitor:
    def __init__(self, drone_id: str, url: str, grpc_port: int, hz: float, json_path: Path) -> None:
        if System is None:
            raise RuntimeError("mavsdk is not installed; please install mavsdk>=1.1.0")
        self.drone_id = str(drone_id)
        self.url = url
        self.grpc_port = int(grpc_port)
        self.hz = max(0.2, float(hz))
        self.json_path = json_path
        self.drone = System(port=self.grpc_port)  # <-- unique gRPC per drone
        self._stop = asyncio.Event()
        self._reading_count = 0
        self.snap: Dict[str, Any] = {
            "drone_id": self.drone_id,
            "timestamp": now_iso(),
            "battery": {},
            "gps": {},
            "position": {},
            "velocity_ned": {},
            "attitude": {},
            "health": {},
            "status": {},
            "rc": {},
            "heading_deg": None,
            "wind": {},
        }

    async def connect(self) -> None:
        await self.drone.connect(system_address=self.url)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

    async def _set_rates(self) -> None:
        t = self.drone.telemetry
        async def try_rate(name: str, val: float) -> None:
            fn = getattr(t, name, None)
            if fn:
                try:
                    await fn(val)
                except Exception:
                    pass
        await try_rate("set_rate_battery", 5.0)
        await try_rate("set_rate_gps_info", 2.0)
        await try_rate("set_rate_position", 5.0)
        await try_rate("set_rate_velocity_ned", 5.0)
        await try_rate("set_rate_attitude_quaternion", 10.0)
        await try_rate("set_rate_health", 1.0)
        await try_rate("set_rate_rc_status", 1.0)
        await try_rate("set_rate_heading", 2.0)
        await try_rate("set_rate_wind", 1.0)

    # --- telemetry loops ---
    async def _battery(self) -> None:
        async for b in self.drone.telemetry.battery():
            self.snap["battery"] = {
                "voltage_v": round(getattr(b, "voltage_v", 0.0) or 0.0, 2),
                "current_a": (round(getattr(b, "current_a", 0.0), 2)
                              if getattr(b, "current_a", None) is not None else None),
                "remaining": getattr(b, "remaining_percent", None),
            }

    async def _gps(self) -> None:
        async for gi in self.drone.telemetry.gps_info():
            self.snap["gps"] = {
                "num_satellites": getattr(gi, "num_satellites", None),
                "fix_type": (getattr(gi, "fix_type", None).name
                             if getattr(gi, "fix_type", None) is not None else None),
            }

    async def _position(self) -> None:
        async for pos in self.drone.telemetry.position():
            self.snap["position"] = {
                "lat_deg": getattr(pos, "latitude_deg", None),
                "lon_deg": getattr(pos, "longitude_deg", None),
                "abs_alt_m": getattr(pos, "absolute_altitude_m", None),
                "rel_alt_m": getattr(pos, "relative_altitude_m", None),
            }

    async def _velocity(self) -> None:
        async for v in self.drone.telemetry.velocity_ned():
            self.snap["velocity_ned"] = {
                "north_m_s": getattr(v, "north_m_s", None),
                "east_m_s": getattr(v, "east_m_s", None),
                "down_m_s": getattr(v, "down_m_s", None),
            }

    async def _attitude(self) -> None:
        async for q in self.drone.telemetry.attitude_quaternion():
            w, x, y, z = getattr(q, "w", 1.0), getattr(q, "x", 0.0), getattr(q, "y", 0.0), getattr(q, "z", 0.0)
            self.snap["attitude"] = {"quaternion": {"w": w, "x": x, "y": y, "z": z},
                                     "euler_deg": quat_to_euler_deg(w, x, y, z)}

    async def _health(self) -> None:
        async for h in self.drone.telemetry.health():
            self.snap["health"] = {
                "local_position_ok": getattr(h, "is_local_position_ok", None),
                "global_position_ok": getattr(h, "is_global_position_ok", None),
                "home_position_ok": getattr(h, "is_home_position_ok", None),
            }

    async def _status(self) -> None:
        async def armed_loop() -> None:
            async for a in self.drone.telemetry.armed():
                self.snap.setdefault("status", {})["armed"] = bool(a)
        async def in_air_loop() -> None:
            async for ia in self.drone.telemetry.in_air():
                self.snap.setdefault("status", {})["in_air"] = bool(ia)
        async def mode_loop() -> None:
            async for fm in self.drone.telemetry.flight_mode():
                self.snap.setdefault("status", {})["flight_mode"] = (fm.name if fm else None)
        await asyncio.gather(armed_loop(), in_air_loop(), mode_loop())

    async def _rc(self) -> None:
        try:
            async for rc in self.drone.telemetry.rc_status():
                self.snap["rc"] = {
                    "available": getattr(rc, "is_available", None),
                    "signal_strength_percent": getattr(rc, "signal_strength_percent", None),
                }
        except Exception:
            pass

    async def _heading(self) -> None:
        try:
            async for hd in self.drone.telemetry.heading():
                self.snap["heading_deg"] = getattr(hd, "heading_deg", None)
        except Exception:
            pass

    async def _wind(self) -> None:
        try:
            async for w in self.drone.telemetry.wind():
                self.snap["wind"] = {
                    "speed_m_s": getattr(w, "speed_m_s", None),
                    "direction_deg": getattr(w, "direction_deg", None),
                }
        except Exception:
            pass

    def _render_summary(self) -> str:
        snap = self.snap
        try:
            ts_dt = datetime.fromisoformat(snap.get("timestamp").replace("Z", "+00:00")).astimezone(timezone.utc)
            time_str = ts_dt.strftime("%H:%M.%S")
        except Exception:
            time_str = datetime.now(timezone.utc).strftime("%H:%M.%S")
        count_str = f"{self._reading_count:03d}" if self._reading_count < 1000 else str(self._reading_count)

        b = snap.get("battery", {})
        batt_rem = b.get("remaining")
        battery_charge = fmt(batt_rem * 100.0 if isinstance(batt_rem, (int, float)) else None, nd=1, unit="%")
        lines = [
            f"[Drone {self.drone_id}] Sensor Reading {count_str}",
            f"Time: {time_str}",
            f"Battery: {battery_charge}  {fmt(b.get('voltage_v'),2,' V')}  {fmt(b.get('current_a'),2,' A')}",
        ]
        return "\n".join(lines)

    async def _printer(self) -> None:
        interval = max(1.0 / self.hz, 0.1)
        while not self._stop.is_set():
            self.snap["timestamp"] = now_iso()
            self._reading_count += 1
            try:
                safe = json.loads(json.dumps(self.snap, default=lambda o: None))
                with open(self.json_path, "w", encoding="utf-8") as f:
                    json.dump(safe, f, indent=2)
            except Exception as exc:
                print(f"[WARN][{self.drone_id}] JSON write failed: {exc}")
            print(self._render_summary(), flush=True)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=interval)
            except asyncio.TimeoutError:
                pass

    async def run(self) -> None:
        await self.connect()
        await self._set_rates()
        tasks = [
            asyncio.create_task(self._battery()),
            asyncio.create_task(self._gps()),
            asyncio.create_task(self._position()),
            asyncio.create_task(self._velocity()),
            asyncio.create_task(self._attitude()),
            asyncio.create_task(self._health()),
            asyncio.create_task(self._status()),
            asyncio.create_task(self._rc()),
            asyncio.create_task(self._heading()),
            asyncio.create_task(self._wind()),
            asyncio.create_task(self._printer()),
        ]
        await self._stop.wait()
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    def stop(self) -> None:
        self._stop.set()

# ------------------------------ app wiring ----------------------------

@dataclass
class VehicleSpec:
    id: str
    url: str
    grpc: int

VEHICLE_RE = re.compile(
    r"""^\s*id=(?P<id>[^,]+)\s*,\s*url=(?P<url>[^,]+)\s*,\s*grpc=(?P<grpc>\d+)\s*$"""
)

def parse_vehicle(arg: str) -> VehicleSpec:
    m = VEHICLE_RE.match(arg)
    if not m:
        raise argparse.ArgumentTypeError(
            "Vehicle must be like: id=1,url=udp://:14540,grpc=50051"
        )
    return VehicleSpec(id=str(m.group("id")), url=m.group("url"), grpc=int(m.group("grpc")))

def create_app(monitors: Dict[str, TelemetryMonitor]) -> FastAPI:
    app = FastAPI()

    @app.get("/sensors")
    async def sensors_all(drone: Optional[str] = Query(None)) -> Dict[str, Any]:
        if drone:
            mon = monitors.get(str(drone))
            if not mon:
                return {"error": f"unknown drone id '{drone}'"}
            return json.loads(json.dumps(mon.snap, default=lambda o: None))
        # all drones
        return {
            drone_id: json.loads(json.dumps(mon.snap, default=lambda o: None))
            for drone_id, mon in monitors.items()
        }

    @app.get("/sensors/{drone_id}")
    async def sensors_one(drone_id: str) -> Dict[str, Any]:
        mon = monitors.get(str(drone_id))
        if not mon:
            return {"error": f"unknown drone id '{drone_id}'"}
        return json.loads(json.dumps(mon.snap, default=lambda o: None))

    return app

async def run_service(vehicles: Dict[str, VehicleSpec], hz: float, json_dir: Path, host: str, port: int) -> None:
    monitors: Dict[str, TelemetryMonitor] = {}
    for vid, spec in vehicles.items():
        json_path = json_dir / f"mavsdk_sensor_snapshot_{vid}.json"
        monitors[vid] = TelemetryMonitor(drone_id=vid, url=spec.url, grpc_port=spec.grpc, hz=hz, json_path=json_path)

    app = create_app(monitors)

    # start telemetry tasks
    tasks = [asyncio.create_task(mon.run()) for mon in monitors.values()]

    # start web server
    server = uvicorn.Server(uvicorn.Config(app=app, host=host, port=port, log_level="info", lifespan="on"))
    server_task = asyncio.create_task(server.serve())

    # stop handling
    loop = asyncio.get_event_loop()
    stop_evt = asyncio.Event()
    def _stop_all() -> None:
        for m in monitors.values():
            m.stop()
        stop_evt.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _stop_all)
        except NotImplementedError:
            pass

    await stop_evt.wait()
    await asyncio.gather(*tasks, server_task, return_exceptions=True)

def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-drone PX4 telemetry service with FastAPI")
    parser.add_argument("--vehicle", action="append", type=parse_vehicle,
                        help="Repeatable: id=1,url=udp://:14540,grpc=50051")
    parser.add_argument("--hz", type=float, default=1.0, help="Refresh rate in Hz")
    parser.add_argument("--json_dir", type=Path, default=Path("."), help="Directory for snapshots")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP host")
    parser.add_argument("--port", type=int, default=8001, help="HTTP port")
    args = parser.parse_args()

    if System is None:
        print("mavsdk is not installed; please `pip install mavsdk fastapi uvicorn`")
        return

    # default to two vehicles if none provided
    vehicles = args.vehicle or [
        VehicleSpec(id="1", url="udp://:14540", grpc=50051),
        VehicleSpec(id="2", url="udp://:14541", grpc=50052),
    ]
    vehicle_map = {v.id: v for v in vehicles}

    try:
        asyncio.run(run_service(vehicle_map, hz=args.hz, json_dir=args.json_dir, host=args.host, port=args.port))
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()

