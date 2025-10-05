#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Agentic Controller (PX4 + MAVSDK) — Single-Drone Edition
Preconfigured for Drone 1:
  - MAVSDK gRPC: localhost:50051
  - PX4 UDP:     udp://:14540

How to switch to Drone 2 later:
  - Set GRPC_PORT   = 50052
  - Set PX4_UDP_PORT = "14541"

Notes:
- Requires mavsdk >= 1.x (`pip install mavsdk`).
- If you run a separate mavsdk_server, start it first and ensure it listens on GRPC_PORT.
- This script will instruct that server to connect to PX4 at PX4_UDP_PORT.
"""

import asyncio
import math
from typing import Optional

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import FixType

# --------- HARD-CODED TARGET: DRONE 1 (edit these two for Drone 2) ----------
GRPC_ADDRESS = "127.0.0.1"  # or "localhost"
GRPC_PORT = 50052           # Drone 1 => 50051   (Drone 2 => 50052)
PX4_UDP_PORT = "14541"      # Drone 1 => "14540" (Drone 2 => "14541")
# ---------------------------------------------------------------------------

LINEAR_SPEED_MPS = 1.0
YAW_RATE_DEG_S = 30.0
ORBIT_SPEED_MPS = 1.5
SETTLE_SEC = 0.6
DEFAULT_TAKEOFF_M = 3.0

def _haversine_m(lat1, lon1, lat2, lon2):
    """Rough distance in meters between two lat/lon points."""
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def _unwrap_deg(prev, curr):
    """Shortest signed delta from prev->curr in degrees, handling wrap."""
    d = curr - prev
    while d <= -180.0:
        d += 360.0
    while d > 180.0:
        d -= 360.0
    return d

class AgenticController:
    def __init__(self) -> None:
        self.drone: Optional[System] = None
        self.offboard_active: bool = False
        self.stream_task: Optional[asyncio.Task] = None

    # --------------------------- Utilities ---------------------------

    async def _await_first(self, agen):
        async for item in agen:
            return item
        raise RuntimeError("Empty stream")

    async def _start_offboard_if_needed(self) -> None:
        if self.offboard_active:
            return
        assert self.drone is not None
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.start()
        self.offboard_active = True

    async def _stop_offboard_if_active(self) -> None:
        if not self.offboard_active:
            return
        assert self.drone is not None
        try:
            await self.drone.offboard.stop()
        except OffboardError:
            pass
        self.offboard_active = False

    async def _stream_zero_hold(self, seconds: float) -> None:
        assert self.drone is not None
        await self._start_offboard_if_needed()
        t_end = asyncio.get_event_loop().time() + seconds
        while asyncio.get_event_loop().time() < t_end:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.05)

    async def _cancel_stream_task(self) -> None:
        if self.stream_task and not self.stream_task.done():
            self.stream_task.cancel()
            try:
                await self.stream_task
            except asyncio.CancelledError:
                pass
        self.stream_task = None

    async def _ensure_connected(self) -> None:
        """Connect to the MAVSDK server (gRPC) and tell it to connect to PX4 UDP."""
        if self.drone is not None:
            return

        # Connect to an already-running mavsdk_server (gRPC)
        # If you are not running an external server, remove port/address and
        # mavsdk-python will auto-spawn one; but here we respect the user's fixed ports.
        self.drone = System(mavsdk_server_address=GRPC_ADDRESS, port=GRPC_PORT)

        system_address = f"udp://:{PX4_UDP_PORT}"
        print(f"▶ Connecting to mavsdk_server at {GRPC_ADDRESS}:{GRPC_PORT}")
        print(f"▶ Instructing server to open PX4 link: {system_address}")
        await self.drone.connect(system_address=system_address)

        # Wait until the vehicle is discovered
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("  ✓ Vehicle discovered")
                break

        # Quick health gate (non-blocking style)
        print("  …sensors/health check")
        try:
            for _ in range(50):  # ~5s max
                h = await self._await_first(self.drone.telemetry.health())
                if (h.is_accelerometer_calibration_ok and
                    h.is_gyrometer_calibration_ok):
                    break
                await asyncio.sleep(0.1)
        except Exception:
            pass
        print("  ✓ Ready")

    async def _ensure_armed(self) -> None:
        assert self.drone is not None
        is_armed = await self._await_first(self.drone.telemetry.armed())
        if not is_armed:
            print("▶ Arm")
            try:
                await self.drone.action.arm()
                print("  ✓ Armed")
            except ActionError as e:
                print(f"  ✗ Arm failed: {e._result.result}")
                raise

    # --------------------------- Core Commands ---------------------------

    async def status(self) -> None:
        await self._ensure_connected()
        pos = await self._await_first(self.drone.telemetry.position())
        batt = await self._await_first(self.drone.telemetry.battery())
        in_air = await self._await_first(self.drone.telemetry.in_air())
        armed = await self._await_first(self.drone.telemetry.armed())
        flight_mode = await self._await_first(self.drone.telemetry.flight_mode())
        gps_info = await self._await_first(self.drone.telemetry.gps_info())

        print("---- STATUS ----")
        print(f"Armed: {armed}")
        print(f"In air: {in_air}")
        print(f"Flight mode: {flight_mode}")
        print(f"Rel Alt: {pos.relative_altitude_m:.2f} m | Abs Alt: {pos.absolute_altitude_m:.2f} m")
        print(f"Lat/Lon: {pos.latitude_deg:.6f}, {pos.longitude_deg:.6f}")
        print(f"Battery: {batt.remaining_percent*100:.0f}% ({batt.voltage_v:.1f} V)")
        print(f"GPS: {FixType(gps_info.fix_type).name}")
        print("----------------")

    async def battery(self) -> None:
        await self._ensure_connected()
        batt = await self._await_first(self.drone.telemetry.battery())
        print(f"Battery: {batt.remaining_percent*100:.0f}% ({batt.voltage_v:.1f} V)")

    async def takeoff(self, alt_m: Optional[float]) -> None:
        await self._ensure_connected()
        await self._ensure_armed()
        assert self.drone is not None
        target = alt_m if (alt_m and alt_m > 0.3) else DEFAULT_TAKEOFF_M
        print(f"▶ Takeoff to {target:.1f} m")
        try:
            await self.drone.action.set_takeoff_altitude(float(target))
            await self.drone.action.takeoff()
            while True:
                pos = await self._await_first(self.drone.telemetry.position())
                if pos.relative_altitude_m >= 0.92 * target:
                    break
                await asyncio.sleep(0.1)
            print(f"  ✓ Takeoff complete: {target:.1f} m")
        except ActionError as e:
            print(f"  ✗ Takeoff failed: {e._result.result}")

    async def land(self) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        print("▶ Land")
        try:
            await self._cancel_stream_task()
            await self._stop_offboard_if_active()
            await self.drone.action.land()
            while True:
                in_air = await self._await_first(self.drone.telemetry.in_air())
                if not in_air:
                    break
                await asyncio.sleep(0.3)
            try:
                await self.drone.action.disarm()
            except ActionError:
                pass
            print("  ✓ Landed and disarmed")
        except ActionError as e:
            print(f"  ✗ Land failed: {e._result.result}")

    async def disarm(self) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        print("▶ Disarm")
        try:
            await self.drone.action.disarm()
            print("  ✓ Disarmed")
        except ActionError as e:
            print(f"  ✗ Disarm failed: {e._result.result}")

    async def rtl(self) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        print("▶ RTL")
        try:
            await self._cancel_stream_task()
            await self._stop_offboard_if_active()
            await self.drone.action.return_to_launch()
            print("  ✓ RTL initiated")
        except ActionError as e:
            print(f"  ✗ RTL failed: {e._result.result}")

    async def stop(self) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        print("▶ Stop/Hold")
        await self._cancel_stream_task()
        if self.offboard_active:
            await self._stream_zero_hold(SETTLE_SEC)
            await self._stop_offboard_if_active()
        print("  ✓ Holding")

    # --------------------------- Movement ---------------------------

    async def _move_body(self, vx: float, vy: float, vz: float, duration_s: float, label: str) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        await self._cancel_stream_task()
        await self._start_offboard_if_needed()
        print(f"▶ Move {label} for {duration_s:.2f}s")

        async def _runner():
            end_t = asyncio.get_event_loop().time() + duration_s
            while asyncio.get_event_loop().time() < end_t:
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
                await asyncio.sleep(0.05)
            await self._stream_zero_hold(SETTLE_SEC)
            await self._stop_offboard_if_active()

        self.stream_task = asyncio.create_task(_runner())
        await self.stream_task
        self.stream_task = None
        print("  ✓ Move done")

    async def forward(self, meters: float) -> None:
        meters = max(0.0, float(meters))
        if meters == 0.0:
            print("Distance must be > 0.")
            return
        duration = meters / LINEAR_SPEED_MPS
        await self._move_body(LINEAR_SPEED_MPS, 0.0, 0.0, duration, f"forward {meters:.2f}m")

    async def backward(self, meters: float) -> None:
        meters = max(0.0, float(meters))
        duration = meters / LINEAR_SPEED_MPS
        await self._move_body(-LINEAR_SPEED_MPS, 0.0, 0.0, duration, f"backward {meters:.2f}m")

    async def left(self, meters: float) -> None:
        meters = max(0.0, float(meters))
        duration = meters / LINEAR_SPEED_MPS
        await self._move_body(0.0, -LINEAR_SPEED_MPS, 0.0, duration, f"left {meters:.2f}m")

    async def right(self, meters: float) -> None:
        meters = max(0.0, float(meters))
        duration = meters / LINEAR_SPEED_MPS
        await self._move_body(0.0, LINEAR_SPEED_MPS, 0.0, duration, f"right {meters:.2f}m")

    async def up(self, meters: float) -> None:
        meters = max(0.0, float(meters))
        duration = meters / LINEAR_SPEED_MPS
        await self._move_body(0.0, 0.0, -LINEAR_SPEED_MPS, duration, f"up {meters:.2f}m")  # NED up => -z

    async def down(self, meters: float) -> None:
        meters = max(0.0, float(meters))
        duration = meters / LINEAR_SPEED_MPS
        await self._move_body(0.0, 0.0, LINEAR_SPEED_MPS, duration, f"down {meters:.2f}m")

    async def _yaw_by(self, degrees: float, yaw_rate_deg_s: float = YAW_RATE_DEG_S) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        await self._cancel_stream_task()
        await self._start_offboard_if_needed()
        yaw_rate = abs(yaw_rate_deg_s) * (1.0 if degrees >= 0 else -1.0)
        duration = abs(float(degrees)) / max(1e-3, abs(yaw_rate))
        print(f"▶ Yaw {'CCW' if degrees>=0 else 'CW'} {abs(degrees):.1f}°")

        async def _runner():
            end_t = asyncio.get_event_loop().time() + duration
            while asyncio.get_event_loop().time() < end_t:
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate))
                await asyncio.sleep(0.05)
            await self._stream_zero_hold(SETTLE_SEC)
            await self._stop_offboard_if_active()

        self.stream_task = asyncio.create_task(_runner())
        await self.stream_task
        self.stream_task = None
        print("  ✓ Yaw done")

    async def yaw_left(self, degrees: float) -> None:
        await self._yaw_by(abs(float(degrees)))

    async def yaw_right(self, degrees: float) -> None:
        await self._yaw_by(-abs(float(degrees)))

    async def look_left(self, degrees: float) -> None:
        await self.yaw_left(degrees)

    async def look_right(self, degrees: float) -> None:
        await self.yaw_right(degrees)

    # --------------------------- Navigation ---------------------------

    async def goto(self, lat: float, lon: float, alt_abs_m: Optional[float]) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        await self._cancel_stream_task()
        await self._stop_offboard_if_active()
        try:
            pos = await self._await_first(self.drone.telemetry.position())
            abs_alt = float(alt_abs_m) if alt_abs_m is not None else float(pos.absolute_altitude_m)
            att = await self._await_first(self.drone.telemetry.attitude_euler())
            print(f"▶ Goto {lat:.6f}, {lon:.6f} @ {abs_alt:.1f}m (abs)")
            await self.drone.action.goto_location(float(lat), float(lon), abs_alt, float(att.yaw_deg))
            print("  ↳ enroute")
        except ActionError as e:
            print(f"  ✗ Goto failed: {e._result.result}")

    # --------------------------- Orbit ---------------------------

    async def orbit(self, radius_m: float, direction: str = "cw",
                    speed_mps: float = ORBIT_SPEED_MPS, return_to_center: bool = False) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        r = max(0.5, float(radius_m))
        v = max(0.2, float(speed_mps))
        dir_is_ccw = (direction.lower() == "ccw")

        # Center snapshot
        center = await self._await_first(self.drone.telemetry.position())
        center_lat, center_lon, center_abs = center.latitude_deg, center.longitude_deg, center.absolute_altitude_m

        # 1) Move to perimeter (left for CW, right for CCW)
        print(f"▶ Orbit r={r:.1f}m dir={'CCW' if dir_is_ccw else 'CW'} v={v:.2f}m/s")
        if dir_is_ccw:
            await self.right(r)
        else:
            await self.left(r)

        # Record exact start-perimeter fix
        start_fix = await self._await_first(self.drone.telemetry.position())
        start_lat, start_lon, start_abs = start_fix.latitude_deg, start_fix.longitude_deg, start_fix.absolute_altitude_m

        # 2) Align tangent
        if dir_is_ccw:
            await self.yaw_left(90.0)
        else:
            await self.yaw_right(90.0)

        # 3) Full 360° using actual yaw integration (with progress logs)
        omega_rad_s = v / r
        yaw_rate_deg_s = math.degrees(omega_rad_s)
        if dir_is_ccw:
            yaw_rate_deg_s = -yaw_rate_deg_s

        await self._cancel_stream_task()
        await self._start_offboard_if_needed()

        att = await self._await_first(self.drone.telemetry.attitude_euler())
        yaw_prev = att.yaw_deg
        accum = 0.0
        expected_T = (2.0 * math.pi * r) / v
        next_progress = 0.25  # 25%, then 50, 75

        print(f"  ↳ circle: target 360°, expected ~{expected_T:.1f}s")
        start_time = asyncio.get_event_loop().time()
        try:
            while accum < 360.0 and (asyncio.get_event_loop().time() - start_time) < expected_T * 1.35:
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(v, 0.0, 0.0, yaw_rate_deg_s))
                await asyncio.sleep(0.05)
                att = await self._await_first(self.drone.telemetry.attitude_euler())
                d = abs(_unwrap_deg(yaw_prev, att.yaw_deg))
                accum += d
                yaw_prev = att.yaw_deg
                if accum / 360.0 >= next_progress:
                    print(f"    …{int(next_progress*100)}%")
                    next_progress += 0.25
        finally:
            await self._stream_zero_hold(SETTLE_SEC)
            await self._stop_offboard_if_active()

        # 4) Snap back exactly to start perimeter using goto (in case of drift)
        print("  ↳ correcting to start-perimeter")
        await self.goto(start_lat, start_lon, start_abs)
        # wait until close or timeout
        t0 = asyncio.get_event_loop().time()
        while True:
            pos = await self._await_first(self.drone.telemetry.position())
            d = _haversine_m(pos.latitude_deg, pos.longitude_deg, start_lat, start_lon)
            if d < max(0.6, r * 0.12):
                break
            if asyncio.get_event_loop().time() - t0 > 12.0:
                print(f"    (warn) couldn't converge closer than {d:.1f} m")
                break
            await asyncio.sleep(0.3)

        # Optional 5) Return to center if requested
        if return_to_center:
            print("  ↳ returning to center")
            await self.goto(center_lat, center_lon, center_abs)
        print("  ✓ Orbit complete (back at start-perimeter)")

    # --------------------------- Gimbal / Look down ---------------------------

    async def look_down(self, degrees: float = 90.0) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        pos = await self._await_first(self.drone.telemetry.position())
        if pos.relative_altitude_m < 3.8:
            print(f"[warn] Alt {pos.relative_altitude_m:.1f} m; consider >4 m for ground-looking.")

        print(f"▶ Look down {degrees:.1f}°")
        # Try angle setpoint
        try:
            if hasattr(self.drone.gimbal, "set_pitch_and_yaw"):
                att = await self._await_first(self.drone.telemetry.attitude_euler())
                await self.drone.gimbal.set_pitch_and_yaw(pitch_deg=-abs(float(degrees)), yaw_deg=float(att.yaw_deg))
                print("  ✓ Gimbal angle set")
                return
        except Exception as e:
            print(f"  (gimbal-angle unsupported) {e}")

        # Try rate setpoint
        try:
            if hasattr(self.drone.gimbal, "set_pitch_rate_and_yaw_rate"):
                rate = -60.0
                dur = abs(float(degrees)) / abs(rate)
                await self.drone.gimbal.set_pitch_rate_and_yaw_rate(pitch_rate_deg_s=rate, yaw_rate_deg_s=0.0)
                await asyncio.sleep(dur)
                await self.drone.gimbal.set_pitch_rate_and_yaw_rate(pitch_rate_deg_s=0.0, yaw_rate_deg_s=0.0)
                print("  ✓ Gimbal rate applied")
                return
        except Exception as e:
            print(f"  (gimbal-rate unsupported) {e}")

        print("  ✱ No gimbal API on this vehicle. Use look_left/right (yaw), or add a MAVLink gimbal/servo.")

    async def look_forward(self) -> None:
        await self._ensure_connected()
        assert self.drone is not None
        print("▶ Look forward (0°)")
        try:
            if hasattr(self.drone.gimbal, "set_pitch_and_yaw"):
                att = await self._await_first(self.drone.telemetry.attitude_euler())
                await self.drone.gimbal.set_pitch_and_yaw(pitch_deg=0.0, yaw_deg=float(att.yaw_deg))
                print("  ✓ Gimbal angle set to 0°")
                return
            elif hasattr(self.drone.gimbal, "set_pitch_rate_and_yaw_rate"):
                await self.drone.gimbal.set_pitch_rate_and_yaw_rate(pitch_rate_deg_s=60.0, yaw_rate_deg_s=0.0)
                await asyncio.sleep(1.0)
                await self.drone.gimbal.set_pitch_rate_and_yaw_rate(pitch_rate_deg_s=0.0, yaw_rate_deg_s=0.0)
                print("  ✓ Gimbal nudged toward 0°")
                return
        except Exception as e:
            print(f"  (gimbal control error) {e}")
        print("  ✱ No gimbal API on this vehicle.")

    # --------------------------- CLI ---------------------------

    @staticmethod
    def _print_help() -> None:
        print("""
Basic Commands:
  help                     - Show this help menu
  arm                      - Arm the drone
  disarm                   - Disarm the drone
  takeoff [altitude]       - Take off to specified altitude (default: 3m)
  land                     - Land the drone
  rtl                      - Return to launch position
  stop                     - Stop current movement (hover)
  status                   - Show drone status
  exit                     - Exit the program
  battery                  - Show current battery % and voltage

Movement Commands:
  forward <meters>         - Move forward by meters
  backward <meters>        - Move backward by meters
  left <meters>            - Move left by meters
  right <meters>           - Move right by meters
  up <meters>              - Move up by meters
  down <meters>            - Move down by meters
  yaw_left <deg>           - Rotate CCW by degrees
  yaw_right <deg>          - Rotate CW by degrees
  turn_cw <deg>            - Alias for yaw_right
  turn_ccw <deg>           - Alias for yaw_left
  look_left <deg>          - Alias for yaw_left
  look_right <deg>         - Alias for yaw_right

Navigation:
  goto <lat> <lon> [alt]   - Go to GPS coordinate (alt is absolute/AMSL)

Orbit:
  orbit <r> [ccw|cw] [v] [return]
      Full 360° around current position; ends at same perimeter start point.
      Add 'return' to go back to center.
""")

    async def repl(self) -> None:
        self._print_help()
        while True:
            try:
                raw = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                raw = "exit"
            if not raw:
                continue
            parts = raw.split()
            cmd = parts[0].lower()
            args = parts[1:]

            try:
                if cmd == "help":
                    self._print_help()

                elif cmd == "status":
                    await self.status()
                elif cmd == "battery":
                    await self.battery()

                elif cmd == "arm":
                    await self._ensure_connected()
                    await self._ensure_armed()
                elif cmd == "disarm":
                    await self.disarm()
                elif cmd == "takeoff":
                    alt = float(args[0]) if args else None
                    await self.takeoff(alt)
                elif cmd == "land":
                    await self.land()
                elif cmd == "rtl":
                    await self.rtl()
                elif cmd == "stop":
                    await self.stop()

                elif cmd == "forward":
                    await self.forward(float(args[0]))
                elif cmd == "backward":
                    await self.backward(float(args[0]))
                elif cmd == "left":
                    await self.left(float(args[0]))
                elif cmd == "right":
                    await self.right(float(args[0]))
                elif cmd == "up":
                    await self.up(float(args[0]))
                elif cmd == "down":
                    await self.down(float(args[0]))

                elif cmd == "yaw_left":
                    await self.yaw_left(float(args[0]))
                elif cmd == "yaw_right":
                    await self.yaw_right(float(args[0]))
                elif cmd == "turn_cw":
                    await self.yaw_right(float(args[0]))
                elif cmd == "turn_ccw":
                    await self.yaw_left(float(args[0]))
                elif cmd == "look_left":
                    await self.look_left(float(args[0]))
                elif cmd == "look_right":
                    await self.look_right(float(args[0]))

                elif cmd == "goto":
                    if len(args) < 2:
                        print("Usage: goto <lat> <lon> [alt_abs_m]")
                    else:
                        lat = float(args[0]); lon = float(args[1])
                        alt = float(args[2]) if len(args) >= 3 else None
                        await self.goto(lat, lon, alt)

                elif cmd == "orbit":
                    if not args:
                        print("Usage: orbit <radius_m> [ccw|cw] [speed_mps] [return]")
                    else:
                        r = float(args[0])
                        direction = "cw"
                        speed = ORBIT_SPEED_MPS
                        ret_center = False
                        if len(args) >= 2 and args[1].lower() in ("ccw", "cw"):
                            direction = args[1].lower()
                        if len(args) >= 3:
                            try:
                                speed = float(args[2])
                            except ValueError:
                                pass
                        if args and args[-1].lower() == "return":
                            ret_center = True
                        await self.orbit(r, direction, speed, ret_center)

                elif cmd == "look_down":
                    deg = float(args[0]) if args else 90.0
                    await self.look_down(deg)
                elif cmd == "look_forward":
                    await self.look_forward()

                elif cmd == "exit":
                    print("Exiting…")
                    try:
                        await self._cancel_stream_task()
                        await self._stop_offboard_if_active()
                    except Exception:
                        pass
                    break
                else:
                    print("Unknown command. Type 'help'.")

            except (IndexError, ValueError):
                print("Invalid arguments. Type 'help' for usage.")
            except Exception as e:
                print(f"[error] {e}")

async def main():
    ctrl = AgenticController()
    try:
        await ctrl._ensure_connected()
    except Exception as e:
        print(f"[connect] Failed: {e}")
        return
    await ctrl.repl()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")

