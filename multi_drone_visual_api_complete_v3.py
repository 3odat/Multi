#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, collections, json, math, os, threading, time
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

import cv2  # type: ignore
import numpy as np  # type: ignore
import requests

# -------- ROS 2 (fallback-friendly) --------
try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from sensor_msgs.msg import Image, CameraInfo  # type: ignore
    try:
        from cv_bridge import CvBridge  # type: ignore
        CV_BRIDGE_AVAILABLE = True
    except Exception:
        CV_BRIDGE_AVAILABLE = False
    try:
        from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore
        MF_AVAILABLE = True
    except Exception:
        MF_AVAILABLE = False
except Exception:
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    Image = CameraInfo = object  # type: ignore
    CV_BRIDGE_AVAILABLE = False
    MF_AVAILABLE = False

# -------- Tunables --------
SCENE_WINDOW_SEC   = 2.0
HISTORY_WINDOW_SEC = 4.0
CLUSTER_RADIUS_M   = 0.6
PER_FRAME_IOU      = 0.5

# -------- Config/State --------
@dataclass
class DroneCfg:
    name: str
    rgb: str
    depth: str
    info: str
    sensors_url: str  # http://localhost:8001/sensors/N

class DroneState:
    def __init__(self, cfg: DroneCfg) -> None:
        self.cfg = cfg
        self.fx = self.fy = self.cx = self.cy = None
        self.frames: List[Dict[str, Any]] = []   # rolling frames: {"timestamp","detections":[...]}
        self.stream_frame: Optional[np.ndarray] = None
        self.latest_frame_for_photo: Optional[np.ndarray] = None

DRONES: Dict[str, DroneState] = {}
lock = threading.RLock()

# -------- Utils --------
def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")

def json_safe(x: Any) -> Any:
    if isinstance(x, float):
        return None if (math.isnan(x) or math.isinf(x)) else x
    if isinstance(x, (np.floating,)):
        xf = float(x); return None if (math.isnan(xf) or math.isinf(xf)) else xf
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, dict):
        return {k: json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [json_safe(v) for v in x]
    return x

def get_telemetry(url: str) -> Dict[str, Any]:
    """Pulls timestamp, GPS (lat_deg, lon_deg, abs_alt_m, rel_alt_m), and yaw_deg from your sensors API."""
    def fetch(u: str) -> Optional[Dict[str, Any]]:
        try:
            r = requests.get(u, timeout=0.5); return r.json()
        except Exception:
            return None

    data = fetch(url)
    if data is None:
        # fallback between /sensors/N and /droneN/sensors
        if "/sensors/" in url:
            head, tail = url.split("/sensors/", 1)
            if tail and tail[0].isdigit():
                alt = f"{head}/drone{tail}/sensors"
                data = fetch(alt)
        else:
            # /droneN/sensors -> /sensors/N
            if "/drone" in url and url.endswith("/sensors"):
                base, suf = url.rsplit("/drone", 1)  # e.g. '1/sensors'
                num = suf.split("/")[0]
                if num.isdigit():
                    data = fetch(f"{base}/sensors/{num}")

    # default empty structure if anything fails
    if not isinstance(data, dict):
        return {
            "timestamp": None,
            "gps": {"lat_deg": None, "lon_deg": None, "abs_alt_m": None, "rel_alt_m": None},
            "yaw_deg": None
        }

    pos = data.get("position", {})
    att = data.get("attitude", {}).get("euler_deg", {})
    return {
        "timestamp": data.get("timestamp"),
        "gps": {
            "lat_deg": pos.get("lat_deg"),
            "lon_deg": pos.get("lon_deg"),
            "abs_alt_m": pos.get("absolute_altitude_m", pos.get("abs_alt_m")),
            "rel_alt_m": pos.get("relative_altitude_m", pos.get("rel_alt_m")),
        },
        "yaw_deg": att.get("yaw_deg"),
    }

def iou(a: List[int], b: List[int]) -> float:
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2-ix1), max(0, iy2-iy1)
    inter = iw*ih
    ua = max(0, ax2-ax1)*max(0, ay2-ay1) + max(0, bx2-bx1)*max(0, by2-by1) - inter
    return inter/ua if ua>0 else 0.0

def merge_boxes(dets: List[Dict[str, Any]], thr: float = PER_FRAME_IOU) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for d in dets:
        merged = False
        for o in out:
            if d["label"] == o["label"] and iou(d["bbox"], o["bbox"]) >= thr:
                da = max(0, d["bbox"][2]-d["bbox"][0]) * max(0, d["bbox"][3]-d["bbox"][1])
                oa = max(0, o["bbox"][2]-o["bbox"][0]) * max(0, o["bbox"][3]-o["bbox"][1])
                if da > oa: o["bbox"] = d["bbox"]
                if d.get("score",0.0) > o.get("score",0.0): o["score"] = d["score"]
                z1 = d.get("position",{}).get("z"); z2 = o.get("position",{}).get("z")
                if isinstance(z1,float) and isinstance(z2,float) and not (math.isnan(z1) or math.isnan(z2)):
                    o["position"]["z"] = 0.5*(z1+z2)
                merged = True; break
        if not merged: out.append(d.copy())
    return out

def cluster_detections(dets: List[Dict[str, Any]], radius: float = CLUSTER_RADIUS_M) -> List[Dict[str, Any]]:
    clusters: List[Dict[str, Any]] = []
    for d in dets:
        lab = d.get("label"); pos = d.get("position", {})
        x,y,z = pos.get("x"), pos.get("y"), pos.get("z")
        if lab is None or any(v is None for v in (x,y,z)) or (isinstance(z,float) and math.isnan(z)):
            continue
        target = None
        for c in clusters:
            if c["label"] != lab: continue
            if math.dist((x,y,z), (c["position"]["x"], c["position"]["y"], c["position"]["z"])) <= radius:
                target = c; break
        if target is None:
            clusters.append({
                "label": lab,
                "position": {"x": x, "y": y, "z": z},
                "score": d.get("score",0.0),
                "sources": set([d.get("source","")]),
            })
        else:
            if d.get("score",0.0) > target.get("score",0.0): target["score"] = d["score"]
            for k in ("x","y","z"):
                target["position"][k] = (target["position"][k] + d["position"][k]) / 2.0
            target["sources"].add(d.get("source",""))
    for c in clusters:
        c["sources"] = sorted([s for s in c["sources"] if s])
    return clusters

def estimate_global_from_local(local_xyz: Dict[str, Any],
                               gps: Optional[Dict[str, Any]],
                               yaw_deg: Optional[float]) -> Optional[Dict[str, float]]:
    """Accepts keys x/y/z or x_m/y_m/z_m; returns {lat,lon,alt} or None."""
    try:
        if not gps or yaw_deg is None: return None
        lat1, lon1 = gps.get("lat_deg"), gps.get("lon_deg")
        alt1 = gps.get("abs_alt_m")
        X = local_xyz.get("x",  local_xyz.get("x_m"))
        Y = local_xyz.get("y",  local_xyz.get("y_m"))
        Z = local_xyz.get("z",  local_xyz.get("z_m"))
        if None in (lat1, lon1, alt1, X, Y, Z): return None
        if isinstance(Z,float) and (math.isnan(Z) or math.isinf(Z)): return None

        yaw = math.radians(float(yaw_deg))
        north = Z*math.cos(yaw) - X*math.sin(yaw)
        east  = Z*math.sin(yaw) + X*math.cos(yaw)
        up    = Y

        R = 6378137.0
        dlat = (north / R) * (180.0 / math.pi)
        dlon = (east / (R * math.cos(math.radians(lat1)))) * (180.0 / math.pi)
        return {"lat": lat1 + dlat, "lon": lon1 + dlon, "alt": alt1 + up}
    except Exception:
        return None

# -------- HTTP server --------
class API(BaseHTTPRequestHandler):
    def log_message(self, *_): return

    def do_GET(self):
        try:
            parts = [p for p in self.path.split("/") if p]
            if not parts: return self._json(404, {"error":"not found"})
            if parts[0] == "fusion" and len(parts)>1 and parts[1]=="scene":
                return self._fusion_scene()
            name = parts[0]
            if name not in DRONES:
                return self._json(404, {"error":"unknown drone", "available": list(DRONES)})
            cmd = parts[1] if len(parts)>1 else ""
            if   cmd == "scene":       return self._scene(name)
            elif cmd == "history":     return self._history(name)
            elif cmd == "video.mjpg":  return self._video(name)
            elif cmd == "take_photo":  return self._photo(name)
            else: return self._json(404, {"error":"not found"})
        except Exception as e:
            return self._json(500, {"error": str(e)})

    def _scene(self, name:str):
        now = time.time(); cutoff = now - SCENE_WINDOW_SEC
        with lock:
            frames = [f for f in DRONES[name].frames if f["timestamp"] >= cutoff]
        dets = [{**d, "source": name} for f in frames for d in f["detections"]]
        fused = cluster_detections(dets, CLUSTER_RADIUS_M)
        telem = get_telemetry(DRONES[name].cfg.sensors_url)
        summary = dict(collections.Counter(d["label"] for d in fused))
        payload = {
            "drone": name,
            "global": {
                "timestamp": telem.get("timestamp"),
                "gps": telem.get("gps"),
                "yaw_deg": telem.get("yaw_deg")
            },
            "count": len(fused),
            "summary_by_object": summary,
            "detections": [self._det_payload_min(d, telem) for d in fused],
            "window_sec": SCENE_WINDOW_SEC,
        }
        return self._json(200, payload)

    def _history(self, name:str):
        now = time.time()
        t0 = now - (SCENE_WINDOW_SEC + HISTORY_WINDOW_SEC)
        t1 = now - SCENE_WINDOW_SEC
        with lock:
            frames = [f for f in DRONES[name].frames if t0 <= f["timestamp"] < t1]
        dets = [{**d, "source": name} for f in frames for d in f["detections"]]
        fused = cluster_detections(dets, CLUSTER_RADIUS_M)
        telem = get_telemetry(DRONES[name].cfg.sensors_url)
        summary = dict(collections.Counter(d["label"] for d in fused))
        payload = {
            "drone": name,
            "global": {
                "timestamp": telem.get("timestamp"),
                "gps": telem.get("gps"),
                "yaw_deg": telem.get("yaw_deg")
            },
            "count": len(fused),
            "summary_by_object": summary,
            "detections": [self._det_payload_min(d, telem) for d in fused],
            "window_sec": HISTORY_WINDOW_SEC,
        }
        return self._json(200, payload)

    def _fusion_scene(self):
        now = time.time(); cutoff = now - SCENE_WINDOW_SEC
        all_dets: List[Dict[str, Any]] = []
        for nm, st in DRONES.items():
            with lock:
                frames = [f for f in st.frames if f["timestamp"] >= cutoff]
            for f in frames:
                for d in f["detections"]:
                    all_dets.append({**d, "source": nm})
        fused = cluster_detections(all_dets, CLUSTER_RADIUS_M)
        telem = {nm: get_telemetry(st.cfg.sensors_url) for nm, st in DRONES.items()}
        summary = dict(collections.Counter(d["label"] for d in fused))
        payload = {
            "drones": list(DRONES.keys()),
            "count": len(fused),
            "summary_by_object": summary,
            "detections": [
                self._det_payload_min(
                    d,
                    telem.get(d["sources"][0], {}) if d.get("sources") else {"gps":None,"yaw_deg":None}
                ) for d in fused
            ],
            "window_sec": SCENE_WINDOW_SEC,
        }
        return self._json(200, payload)

    def _det_payload_min(self, d: Dict[str, Any], telem: Dict[str, Any]) -> Dict[str, Any]:
        center = {"x_m": d["position"]["x"], "y_m": d["position"]["y"], "z_m": d["position"]["z"]}
        est = estimate_global_from_local(d["position"], telem.get("gps"), telem.get("yaw_deg"))
        return {
            "Object Name": d["label"],
            "Confidence": d.get("score"),
            "Center": center,
            "estimated_global": est,
            "sources": d.get("sources", []),
        }

    def _photo(self, name:str):
        with lock:
            frame = DRONES[name].latest_frame_for_photo
            frame = None if frame is None else frame.copy()
        if frame is None: return self._json(200, {"status":"error","message":"no frame"})
        outdir = os.path.join(os.path.dirname(__file__), "images"); os.makedirs(outdir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S"); path = os.path.join(outdir, f"{name}_{ts}.jpg")
        ok = cv2.imwrite(path, frame)
        return self._json(200, {"status":"success" if ok else "error","file": os.path.relpath(path, os.path.dirname(__file__))})

    def _video(self, name:str):
        try:
            self.send_response(200)
            self.send_header("Content-Type","multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with lock:
                    fr = DRONES[name].stream_frame
                    fr = None if fr is None else fr.copy()
                if fr is None:
                    fr = np.zeros((360,640,3), np.uint8)
                    cv2.putText(fr, f"{name}: no frame", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                ok, jpg = cv2.imencode(".jpg", fr)
                if not ok: continue
                b = jpg.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(b)}\r\n\r\n".encode())
                self.wfile.write(b); self.wfile.write(b"\r\n"); self.wfile.flush()
                time.sleep(0.05)
        except Exception:
            return

    def _json(self, code:int, payload:Dict[str,Any]):
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.end_headers()
        self.wfile.write(json.dumps(json_safe(payload), indent=2, allow_nan=False).encode("utf-8"))

# -------- ROS Node --------
class Vision(Node):
    def __init__(self, cfgs: List[DroneCfg], http_port:int=8088) -> None:
        super().__init__("multi_drone_visual_min")
        for cfg in cfgs: DRONES[cfg.name] = DroneState(cfg)
        if MF_AVAILABLE:
            self._setup_synced(cfgs)
        else:
            self._setup_unsynced(cfgs)
        threading.Thread(target=self._http, args=(http_port,), daemon=True).start()
        self.get_logger().info(f"HTTP on 0.0.0.0:{http_port} (/{{drone}}/scene, /fusion/scene)")

    def _setup_synced(self, cfgs: List[DroneCfg]) -> None:
        from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore
        self.syncs = []
        for cfg in cfgs:
            cam = Subscriber(self, Image, cfg.rgb)   # type: ignore
            dep = Subscriber(self, Image, cfg.depth) # type: ignore
            inf = Subscriber(self, CameraInfo, cfg.info)  # type: ignore
            sync = ApproximateTimeSynchronizer([cam,dep,inf], queue_size=10, slop=0.1)  # type: ignore
            sync.registerCallback(lambda rgb, d, info, n=cfg.name: self._cb(n, rgb, d, info))  # type: ignore
            self.syncs.append(sync)

    def _setup_unsynced(self, cfgs: List[DroneCfg]) -> None:
        self.latest = {c.name: {"rgb":None, "depth":None, "info":None} for c in cfgs}
        for cfg in cfgs:
            self.create_subscription(Image, cfg.rgb,   lambda m,n=cfg.name:self._rgb(n,m), 10)
            self.create_subscription(Image, cfg.depth, lambda m,n=cfg.name:self._depth(n,m), 10)
            self.create_subscription(CameraInfo, cfg.info, lambda m,n=cfg.name:self._info(n,m), 10)
        self.create_timer(0.066, self._tick)  # ~15Hz

    def _rgb(self, n,m):   self.latest[n]["rgb"] = m
    def _depth(self, n,m): self.latest[n]["depth"] = m
    def _info(self, n,m):  self.latest[n]["info"] = m

    def _tick(self):
        for n,v in self.latest.items():
            if v["rgb"] is not None and v["depth"] is not None:
                info = v["info"] if v["info"] is not None else CameraInfo()  # type: ignore
                self._cb(n, v["rgb"], v["depth"], info)
                v["rgb"] = v["depth"] = None

    def _cb(self, name:str, rgb_msg:Image, depth_msg:Image, info_msg:CameraInfo):
        st = DRONES[name]
        rgb = self._to_bgr(rgb_msg)
        depth, dw, dh = self._to_depth(depth_msg)
        if rgb is None or depth is None: return

        # intrinsics (once)
        try:
            k = getattr(info_msg, "k", None)
            if st.fx is None and k is not None:
                k = list(k)
                if len(k) >= 6:
                    st.fx, st.fy = float(k[0]), float(k[4])
                    st.cx, st.cy = float(k[2]), float(k[5])
        except Exception:
            pass

        # detector: YOLO if available; else HOG(person)
        dets: List[Dict[str, Any]] = []
        try:
            from ultralytics import YOLO  # type: ignore
            if not hasattr(self, "_yolo"):
                self._yolo = YOLO("yolov8n.pt")
            for r in self._yolo(rgb):
                for b in r.boxes:
                    x1,y1,x2,y2 = b.xyxy[0].tolist()
                    cls = int(b.cls[0]); conf = float(b.conf[0])
                    label = self._yolo.model.names[cls]  # type: ignore
                    dets.append({"label": label, "score": conf, "bbox":[int(x1),int(y1),int(x2),int(y2)]})
        except Exception:
            hog = cv2.HOGDescriptor(); hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            rects, weights = hog.detectMultiScale(rgb, winStride=(8,8))
            for (x,y,w,h), s in zip(rects, weights):
                dets.append({"label":"person","score":float(s),"bbox":[int(x),int(y),int(x+w),int(y+h)]})

        dets = merge_boxes(dets, PER_FRAME_IOU)

        processed: List[Dict[str, Any]] = []
        for d in dets:
            x1,y1,x2,y2 = d["bbox"]
            u,v = int((x1+x2)/2), int((y1+y2)/2)
            du,dv = self._map(u,v, rgb.shape[1],rgb.shape[0], dw,dh)
            Z = self._depth_at(depth, du, dv)
            X = Y = None
            if Z is not None and st.fx and st.fy and st.cx is not None and st.cy is not None:
                X = (float(du)-st.cx) * Z / st.fx
                Y = (float(dv)-st.cy) * Z / st.fy
            processed.append({"label": d["label"], "score": d.get("score"),
                              "position": {"x": X, "y": Y, "z": Z}})

        ts = time.time()
        with lock:
            st.frames.append({"timestamp": ts, "detections": processed})
            cutoff = ts - (SCENE_WINDOW_SEC + HISTORY_WINDOW_SEC + 1.0)
            st.frames = [fr for fr in st.frames if fr["timestamp"] >= cutoff]
            # light overlay for video
            ann = rgb.copy()
            for d in processed:
                # draw small dot at the center only (minimal overlay)
                txt = d["label"]
                z = d["position"]["z"]
                if isinstance(z,float) and not (math.isnan(z) or math.isinf(z)):
                    txt += f" Z={z:.2f}m"
                cv2.putText(ann, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            st.stream_frame = ann
            st.latest_frame_for_photo = ann

    def _to_bgr(self, msg:Image) -> Optional[np.ndarray]:
        try:
            if CV_BRIDGE_AVAILABLE:
                return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")  # type: ignore
            arr = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)  # type: ignore
            return arr.reshape((msg.height, msg.width, 3))
        except Exception:
            return None

    def _to_depth(self, msg:Image) -> Tuple[Optional[np.ndarray],int,int]:
        try:
            if msg.encoding == "32FC1":
                d = np.ndarray(shape=(msg.height, msg.width), dtype=np.float32, buffer=msg.data)  # type: ignore
            else:
                d16 = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)  # type: ignore
                d = d16.astype(np.float32)/1000.0
            return d, msg.width, msg.height  # type: ignore
        except Exception:
            return None, 0, 0

    def _map(self, u:int, v:int, rw:int, rh:int, dw:int, dh:int) -> Tuple[int,int]:
        du = min(max(int(u*dw/rw), 0), dw-1); dv = min(max(int(v*dh/rh), 0), dh-1)
        return du, dv

    def _depth_at(self, depth: np.ndarray, u:int, v:int, k:int=5) -> Optional[float]:
        h,w = depth.shape[:2]; u0,v0 = max(0,u-k//2), max(0,v-k//2)
        u1,v1 = min(w,u+k//2+1), min(h,v+k//2+1)
        P = depth[v0:v1, u0:u1]
        if P.size==0: return None
        vals = P[np.isfinite(P)]
        vals = vals[(vals>0.1) & (vals<60.0)]
        return float(np.median(vals)) if vals.size else None

    def _http(self, port:int):
        srv = ThreadingHTTPServer(("0.0.0.0", port), API)
        try: srv.serve_forever()
        except Exception: pass

# -------- CLI --------
def parse_args():
    p = argparse.ArgumentParser(description="Multi-drone visual perception API (minimal fields + GPS in /scene)")
    p.add_argument("--port", type=int, default=8088)
    p.add_argument("--drone", action="append",
                   help="name;rgb;depth;info;SENSORS_URL  (repeat for multiple drones)")
    return p.parse_args()

def default_cfgs() -> List[DroneCfg]:
    return [
        DroneCfg("drone1","/drone1/camera","/drone1/depth_camera","/drone1/camera_info","http://localhost:8001/sensors/1"),
        DroneCfg("drone2","/drone2/camera","/drone2/depth_camera","/drone2/camera_info","http://localhost:8001/sensors/2"),
    ]

def main():
    args = parse_args()
    cfgs: List[DroneCfg] = []
    if args.drone:
        for spec in args.drone:
            parts = spec.split(";")
            if len(parts) != 5:
                raise SystemExit(f"--drone must be name;rgb;depth;info;SENSORS_URL (got: {spec})")
            cfgs.append(DroneCfg(*parts))
    else:
        cfgs = default_cfgs()

    if rclpy is None or Node is object:
        print("Error: ROS 2 not found. Source your ROS 2 environment and retry.")
        return

    rclpy.init()
    node = Vision(cfgs, http_port=args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

