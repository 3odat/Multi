#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Two live maps with identical features per drone, tied directly to your per-drone APIs.

Pages
-----
/map1                 : drone 1 (uses /api/sensors/1 and /api/scene/1)
/map2                 : drone 2
/map?id=<id>          : generic single-drone page
/fleet?ids=1,2        : combined view

Proxies
-------
/api/sensors/<id>     -> <base>/sensors/<id>     (default base=http://localhost:8001)
/api/scene/<id>       -> <scene>/drone<id>/scene (default scene=http://localhost:8088)
/favicon.ico          : tiny empty icon (avoid 404 spam)

Run
---
python3 multi_drone_map_server.py --port 8002 --base http://localhost:8001 --scene http://localhost:8088 --hz 1.0 --trail 6000 --smooth 0.35
"""

from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import urllib.request, urllib.error
import json, argparse

# ----------------------- CLI -----------------------
def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8002)
    ap.add_argument("--base", default="http://localhost:8001",
                    help="Base URL of the sensors service")
    ap.add_argument("--scene", default="http://localhost:8088",
                    help="Base URL of the vision /scene service")
    ap.add_argument("--hz", type=float, default=1.0,
                    help="Browser polling rate (Hz)")
    ap.add_argument("--trail", type=int, default=6000,
                    help="Trail ring buffer length")
    ap.add_argument("--smooth", type=float, default=0.35,
                    help="EMA smoothing alpha in [0,1]; 0=off (0.3–0.5 recommended)")
    return ap.parse_args()

ARGS = parse_args()

LEAFLET_CSS = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
LEAFLET_JS  = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"

# ----------------------- HTML (placeholders, no f-strings in markup) -----------------------
def html_map_single(drone_id: int) -> bytes:
    tpl = """<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<title>Drone __DRONE_ID__ Map</title>
<link rel="stylesheet" href="__LEAFLET_CSS__"/>
<style>
  html,body,#map { height:100%; margin:0; background:#0b0f14; }
  .hud {
    position:absolute; top:10px; left:10px; padding:10px 12px; background:rgba(0,0,0,0.55);
    color:#fff; font:14px/1.35 system-ui, -apple-system, Segoe UI, Roboto, Arial;
    border-radius:10px; min-width: 280px; box-shadow:0 6px 20px rgba(0,0,0,0.35);
  }
  .hud .row { display:flex; justify-content:space-between; gap:10px; }
  .toolbar { position:absolute; top:10px; right:10px; display:flex; gap:8px; flex-wrap:wrap; }
  .toolbar button {
    padding:6px 12px; border-radius:10px; border:1px solid #718096; background:#111827; color:#e5e7eb; cursor:pointer;
  }
  .toolbar button:hover { background:#1f2937; }
  .badge { display:inline-block; padding:2px 6px; border-radius:999px; background:#1e88e5; color:#fff; font-size:12px; }
  .ok { background:#2e7d32; } .warn { background:#ef6c00; } .err { background:#c62828; }
  .arrow .wrap { width:40px; height:40px; transform-origin:20px 20px; }
  .legend { position:absolute; bottom:10px; left:10px; background:rgba(0,0,0,.5); color:#fff; padding:6px 8px; border-radius:8px; font:12px system-ui; }
</style>
</head>
<body>
<div id="map"></div>
<div class="hud" id="hud">
  <div><b>Drone</b>: <span id="hud-drone">__DRONE_ID__</span> &nbsp;
      <span id="hud-status" class="badge warn">connecting...</span></div>
  <div class="row"><div>Time</div><div id="hud-ts">—</div></div>
  <div class="row"><div>Lat</div><div id="hud-lat">—</div></div>
  <div class="row"><div>Lon</div><div id="hud-lon">—</div></div>
  <div class="row"><div>Alt abs</div><div id="hud-alt-abs">— m</div></div>
  <div class="row"><div>Alt rel</div><div id="hud-alt-rel">— m</div></div>
  <div class="row"><div>Heading</div><div id="hud-yaw">— °</div></div>
  <div class="row"><div>Satellites</div><div id="hud-sats">—</div></div>
  <div class="row"><div>Battery</div><div id="hud-batt">— %</div></div>
  <div class="row"><div>Speed</div><div id="hud-speed">— m/s</div></div>
  <div class="row"><div>Trail</div><div id="hud-pts">0</div></div>
  <div class="row"><div>Fields</div><div id="hud-src">pos:?, head:?</div></div>
</div>
<div class="toolbar">
  <button id="btn-follow" title="Follow UAV">Follow: ON</button>
  <button id="btn-recenter">Recenter</button>
  <button id="btn-clear">Clear Trail</button>
  <button id="btn-export">Export GeoJSON</button>
  <button id="btn-objects">Objects: ON</button>
  <button id="btn-smooth">Smoothing: __SMOOTH_LABEL__</button>
</div>
<div class="legend">Arrow = heading • Red line = path • Red dots = detections (from /scene)</div>

<script src="__LEAFLET_JS__"></script>
<script>
// -------- constants from server --------
const DRONE_ID = __DRONE_ID__;
const POLL_MS = Math.max(100, Math.floor(1000/__HZ__));
const TRAIL_MAX = __TRAIL__;
let   SMOOTH_A = __SMOOTH__;

// -------- map + layers --------
const map = L.map('map').setView([0,0], 18);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 20 }).addTo(map);

// Heading arrow marker (SVG) + red trail + heading ray
const arrowHTML = '<div class="wrap">' +
  '<svg width="40" height="40" viewBox="-20 -20 40 40" xmlns="http://www.w3.org/2000/svg">' +
  '<polygon points="0,-14 10,8 0,3 -10,8" fill="#ff1744" stroke="#b71c1c" stroke-width="2"/>' +
  '</svg></div>';
const arrowIcon = L.divIcon({ className: 'arrow', html: arrowHTML, iconSize:[40,40], iconAnchor:[20,20] });

let arrow = null;
let headingRay = L.polyline([], { color:'#ff1744', weight: 3, opacity: 0.9 }).addTo(map);
let trail = L.polyline([], { color:'#ff1744', weight: 3, opacity: 0.85 }).addTo(map);

// Object overlay group
let objectsOn = true;
let objectLayer = L.layerGroup().addTo(map);

// UI toggles
let follow = true;
document.getElementById('btn-follow').onclick = () => {
  follow = !follow;
  document.getElementById('btn-follow').innerText = 'Follow: ' + (follow ? 'ON' : 'OFF');
};
document.getElementById('btn-recenter').onclick = () => { if (arrow) map.panTo(arrow.getLatLng()); };
document.getElementById('btn-clear').onclick = () => { trail.setLatLngs([]); updateHUDPts(); };
document.getElementById('btn-export').onclick = () => exportGeoJSON();
document.getElementById('btn-objects').onclick = () => {
  objectsOn = !objectsOn;
  document.getElementById('btn-objects').innerText = 'Objects: ' + (objectsOn ? 'ON' : 'OFF');
  if(!objectsOn) objectLayer.clearLayers();
};
document.getElementById('btn-smooth').onclick = () => {
  SMOOTH_A = (SMOOTH_A > 0 ? 0 : __SMOOTH__);
  document.getElementById('btn-smooth').innerText = 'Smoothing: ' + (SMOOTH_A>0 ? 'ON' : 'OFF');
};

// -------- helpers --------
let posSrc='?', headSrc='?';
function pick(obj, paths) {
  for (const p of paths) {
    try {
      const v = p.split('.').reduce((o,k)=>o&&o[k], obj);
      if (v !== undefined && v !== null) return v;
    } catch(e){}
  }
  return null;
}
function haversine(a,b) {
  const R=6378137.0;
  const toRad = d=>d*Math.PI/180;
  const dLat = toRad(b.lat-a.lat), dLon = toRad(b.lon-a.lon);
  const s = Math.sin(dLat/2)**2 + Math.cos(toRad(a.lat))*Math.cos(toRad(b.lat))*Math.sin(dLon/2)**2;
  return 2*R*Math.asin(Math.sqrt(s));
}
function destPoint(lat, lon, brgDeg, distM) {
  const R=6378137.0;
  const brg = brgDeg*Math.PI/180;
  const φ1 = lat*Math.PI/180, λ1 = lon*Math.PI/180, δ = distM / R;
  const φ2 = Math.asin( Math.sin(φ1)*Math.cos(δ) + Math.cos(φ1)*Math.sin(δ)*Math.cos(brg) );
  const λ2 = λ1 + Math.atan2(Math.sin(brg)*Math.sin(δ)*Math.cos(φ1), Math.cos(δ)-Math.sin(φ1)*Math.sin(φ2));
  return { lat: φ2*180/Math.PI, lon: ((λ2*180/Math.PI)+540)%360-180 };
}
// unwrap degrees so 359 -> 0 becomes +1 not -359
function unwrapDeg(prev, cur) {
  if (prev == null || cur == null) return cur;
  let d = cur - prev;
  d = ((d + 540) % 360) - 180; // [-180, 180)
  return prev + d;
}

// -------- polling: sensors (no GPS math; use your API as-is) --------
let prevFix = null;
let filtLat = null, filtLon = null;
let yawUnwrapped = null, yawFilt = null;

function updateHUD(o) {
  const set = (id, v)=>document.getElementById(id).innerText = v;
  set('hud-ts', o.ts || '—');
  set('hud-lat', o.lat != null ? o.lat.toFixed(7) : '—');
  set('hud-lon', o.lon != null ? o.lon.toFixed(7) : '—');
  set('hud-alt-abs', o.absAlt != null ? o.absAlt.toFixed(2)+' m' : '— m');
  set('hud-alt-rel', o.relAlt != null ? o.relAlt.toFixed(2)+' m' : '— m');
  set('hud-yaw', o.yaw != null ? o.yaw.toFixed(1)+'°' : '— °');
  set('hud-sats', o.sats != null ? o.sats : '—');
  set('hud-batt', o.batt != null ? (o.batt*100).toFixed(0)+' %' : '— %');
  set('hud-speed', o.speed != null ? o.speed.toFixed(2)+' m/s' : '— m/s');
  set('hud-src', 'pos:'+posSrc+', head:'+headSrc);
}
function updateHUDPts(){ document.getElementById('hud-pts').innerText = trail.getLatLngs().length; }
function setStatus(cls, text){
  const el = document.getElementById('hud-status');
  el.classList.remove('ok','warn','err'); el.classList.add(cls); el.innerText = text;
}

async function pollSensors(){
  try {
    const r = await fetch(`/api/sensors/${DRONE_ID}`, { cache:'no-store' });
    if(!r.ok) throw new Error('HTTP '+r.status);
    const j = await r.json();
    setStatus('ok','live');

    // tolerant extraction (position or gps, heading or attitude.yaw)
    let lat = pick(j, ['position.lat_deg','gps.lat_deg']);
    let lon = pick(j, ['position.lon_deg','gps.lon_deg']);
    let absAlt = pick(j, ['position.absolute_altitude_m','position.abs_alt_m','gps.abs_alt_m']);
    let relAlt = pick(j, ['position.relative_altitude_m','position.rel_alt_m','gps.rel_alt_m']);
    let yaw = pick(j, ['heading_deg','attitude.euler_deg.yaw_deg']);
    let sats = pick(j, ['gps.num_satellites','num_satellites','sats']);
    let batt = pick(j, ['battery.remaining','battery_remaining']);
    let ts = j.timestamp || null;

    posSrc = (lat!=null && lon!=null) ? (j.position?'position':'gps') : '?';
    headSrc = (yaw!=null) ? (j.heading_deg?'heading_deg':'attitude.euler_deg.yaw_deg') : '?';
    if(lat==null || lon==null) return;

    // EMA smoothing (visual only)
    if(__SMOOTH__ > 0) {
      if(filtLat==null || filtLon==null){ filtLat=lat; filtLon=lon; }
      else { filtLat = __SMOOTH__*lat + (1-__SMOOTH__)*filtLat; filtLon = __SMOOTH__*lon + (1-__SMOOTH__)*filtLon; }
      lat = filtLat; lon = filtLon;
    }

    const p = L.latLng(lat, lon);
    if(!arrow){
      arrow = L.marker(p, {icon: arrowIcon, interactive:false}).addTo(map);
      map.setView(p, 19);
    } else {
      arrow.setLatLng(p);
    }

    // unwrap + smooth yaw for smooth rotation
    if(yaw != null){
      yawUnwrapped = unwrapDeg(yawUnwrapped, yaw);
      if(__SMOOTH__ > 0){
        yawFilt = (yawFilt==null) ? yawUnwrapped : __SMOOTH__*yawUnwrapped + (1-__SMOOTH__)*yawFilt;
      } else {
        yawFilt = yawUnwrapped;
      }
      // map filter back into [0,360) for display
      let yawDisp = ((yawFilt % 360) + 360) % 360;
      const el = arrow.getElement(); if(el){ const w = el.querySelector('.wrap'); if(w) w.style.transform = 'rotate('+yawDisp+'deg)'; }
      const tip = destPoint(lat, lon, yawDisp, 12.0);
      headingRay.setLatLngs([p, [tip.lat, tip.lon]]);
    }

    // red path
    const pts = trail.getLatLngs(); pts.push(p);
    if(pts.length > TRAIL_MAX) pts.splice(0, pts.length - TRAIL_MAX);
    trail.setLatLngs(pts); updateHUDPts();
    if(follow) map.panTo(p);

    // speed from consecutive fixes
    let speed = null;
    const now = Date.now()/1000;
    if(prevFix){
      const dt = Math.max(0.001, now - prevFix.ts);
      const d = haversine(prevFix, {lat, lon});
      speed = d / dt;
    }
    prevFix = {lat, lon, ts: now};

    updateHUD({ ts, lat, lon, absAlt, relAlt, yaw: (yaw!=null?(((yawFilt%360)+360)%360):null), sats, batt, speed });
  } catch(e){
    setStatus('err','lost');
  } finally {
    setTimeout(pollSensors, POLL_MS);
  }
}

// -------- polling: scene (objects) --------
async function pollScene(){
  try{
    if(!objectsOn){ objectLayer.clearLayers(); return setTimeout(pollScene, POLL_MS*2); }
    const r = await fetch(`/api/scene/${DRONE_ID}`, { cache:'no-store' });
    if(!r.ok) throw new Error('HTTP '+r.status);
    const j = await r.json();
    objectLayer.clearLayers();
    const dets = Array.isArray(j.detections) ? j.detections : [];
    for(const d of dets){
      const est = d.estimated_global;
      if(!est || est.lat==null || est.lon==null) continue;
      const name = d["Object Name"] || 'object';
      const conf = d["Confidence"]!=null ? d["Confidence"].toFixed(2) : '—';
      const dot = L.circleMarker([est.lat, est.lon], { radius:6, color:'#ff1744', weight:2, fillOpacity:0.6 });
      dot.bindTooltip(name+' ('+conf+')', {permanent:false});
      objectLayer.addLayer(dot);
    }
  }catch(e){
    // ignore errors quietly
  }finally{
    setTimeout(pollScene, POLL_MS*2);
  }
}

// export trail
function exportGeoJSON(){
  const pts = trail.getLatLngs();
  const gj = { type: "FeatureCollection", features: [{
    type: "Feature", properties: { drone: DRONE_ID },
    geometry: { type: "LineString", coordinates: pts.map(p => [p.lng, p.lat]) }
  }]};
  const blob = new Blob([JSON.stringify(gj,null,2)], {type: 'application/json'});
  const url = URL.createObjectURL(blob); const a = document.createElement('a');
  a.href = url; a.download = 'drone_'+DRONE_ID+'_trail.geojson'; a.click(); URL.revokeObjectURL(url);
}

pollSensors();
pollScene();
</script>
</body>
</html>
"""
    html = (tpl
            .replace("__DRONE_ID__", str(drone_id))
            .replace("__LEAFLET_CSS__", LEAFLET_CSS)
            .replace("__LEAFLET_JS__", LEAFLET_JS)
            .replace("__HZ__", f"{ARGS.hz:.6f}")
            .replace("__TRAIL__", str(ARGS.trail))
            .replace("__SMOOTH__", f"{max(0.0, min(1.0, ARGS.smooth)):.4f}")
            .replace("__SMOOTH_LABEL__", "ON" if ARGS.smooth > 0 else "OFF"))
    return html.encode("utf-8")

def html_map_fleet(ids: list[int]) -> bytes:
    tpl = """<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<title>Fleet Map</title>
<link rel="stylesheet" href="__LEAFLET_CSS__"/>
<style>
  html,body,#map { height:100%; margin:0; background:#0b0f14; }
  .legend { position:absolute; top:10px; left:10px; background:rgba(0,0,0,.55); color:#fff; padding:8px 10px; border-radius:8px; font:12px system-ui; }
  .row { display:flex; gap:8px; align-items:center; margin-bottom:4px; }
  .dot { width:12px; height:12px; border-radius:999px; }
</style>
</head>
<body>
<div id="map"></div>
<div class="legend" id="legend"></div>
<script src="__LEAFLET_JS__"></script>
<script>
const IDS = __IDS__;
const COLORS = ["#ff1744","#1e88e5","#43a047","#fdd835","#8e24aa","#00897b"];
const map = L.map('map').setView([0,0], 17);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 20 }).addTo(map);

const tracks = {};
const markers = {};
const rays = {};

function destPoint(lat, lon, brgDeg, distM) {
  const R=6378137.0;
  const brg = brgDeg*Math.PI/180;
  const φ1 = lat*Math.PI/180, λ1 = lon*Math.PI/180, δ = distM / R;
  const φ2 = Math.asin( Math.sin(φ1)*Math.cos(δ) + Math.cos(φ1)*Math.sin(δ)*Math.cos(brg) );
  const λ2 = λ1 + Math.atan2(Math.sin(brg)*Math.sin(δ)*Math.cos(φ1), Math.cos(δ)-Math.sin(φ1)*Math.sin(φ2));
  return { lat: φ2*180/Math.PI, lon: ((λ2*180/Math.PI)+540)%360-180 };
}
function pick(obj, paths) {
  for (const p of paths) {
    try {
      const v = p.split('.').reduce((o,k)=>o&&o[k], obj);
      if (v !== undefined && v !== null) return v;
    } catch(e){}
  }
  return null;
}
async function tick(id, color){
  try {
    const r = await fetch(`/api/sensors/${id}`, { cache:'no-store' });
    if(!r.ok) throw new Error('http '+r.status);
    const j = await r.json();
    let lat = pick(j, ['position.lat_deg','gps.lat_deg']);
    let lon = pick(j, ['position.lon_deg','gps.lon_deg']);
    let yaw = pick(j, ['heading_deg','attitude.euler_deg.yaw_deg']);
    if(lat==null || lon==null) return;

    const p = L.latLng(lat, lon);
    if(!tracks[id]) {
      tracks[id] = L.polyline([p], { color, weight:3 }).addTo(map);
      markers[id] = L.circleMarker(p, { radius:7, color, weight:3, fillOpacity:0.9 }).addTo(map);
      rays[id] = L.polyline([], { color, weight:3, dashArray:'4 4' }).addTo(map);
      map.panTo(p);
    } else {
      const pts = tracks[id].getLatLngs(); pts.push(p);
      if(pts.length > __TRAIL__) pts.splice(0, pts.length - __TRAIL__);
      tracks[id].setLatLngs(pts); markers[id].setLatLng(p);
    }
    if(yaw!=null){
      const tip = destPoint(lat, lon, yaw, 12.0);
      rays[id].setLatLngs([p, [tip.lat, tip.lon]]);
    }
  } catch(e){}
}
function loop(){
  IDS.forEach((id, idx)=>tick(id, COLORS[idx % COLORS.length]));
  setTimeout(loop, __POLL__);
}
loop();

const lg = document.getElementById('legend');
IDS.forEach((id, idx) => {
  const row = document.createElement('div'); row.className='row';
  const dot = document.createElement('div'); dot.className='dot'; dot.style.background = COLORS[idx % COLORS.length];
  const txt = document.createElement('div'); txt.textContent = 'drone '+id;
  row.appendChild(dot); row.appendChild(txt); lg.appendChild(row);
});
</script>
</body>
</html>
"""
    ids_js = "[" + ",".join(str(i) for i in ids) + "]"
    html = (tpl
            .replace("__LEAFLET_CSS__", LEAFLET_CSS)
            .replace("__LEAFLET_JS__", LEAFLET_JS)
            .replace("__IDS__", ids_js)
            .replace("__TRAIL__", str(ARGS.trail))
            .replace("__POLL__", str(int(1000/ARGS.hz))))
    return html.encode("utf-8")

# ----------------------- HTTP handler (broken-pipe safe) -----------------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet logs
        pass

    # Write helpers that swallow BrokenPipeError
    def _safe_end_headers(self):
        try:
            self.end_headers()
        except BrokenPipeError:
            self.close_connection = True

    def _safe_write(self, data: bytes):
        try:
            self.wfile.write(data)
        except BrokenPipeError:
            self.close_connection = True

    def _send_json(self, code:int, obj):
        body = json.dumps(obj).encode("utf-8")
        try:
            self.send_response(code)
            self.send_header("Content-Type","application/json")
            self.send_header("Access-Control-Allow-Origin","*")
            self.send_header("Content-Length", str(len(body)))
            self._safe_end_headers()
            self._safe_write(body)
        except BrokenPipeError:
            self.close_connection = True

    def _send_html(self, code:int, body:bytes):
        try:
            self.send_response(code)
            self.send_header("Content-Type","text/html; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin","*")
            self.send_header("Content-Length", str(len(body)))
            self._safe_end_headers()
            self._safe_write(body)
        except BrokenPipeError:
            self.close_connection = True

    def do_GET(self):
        try:
            u = urlparse(self.path)
            path = u.path
            qs = parse_qs(u.query)

            # Favicon (avoid 404 + broken pipe spam)
            if path == "/favicon.ico":
                try:
                    self.send_response(204)  # no content
                    self.send_header("Content-Type","image/x-icon")
                    self.send_header("Access-Control-Allow-Origin","*")
                    self._safe_end_headers()
                except BrokenPipeError:
                    pass
                return

            # Pages
            if path in ("/", "/map1"):
                return self._send_html(200, html_map_single(1))
            if path == "/map2":
                return self._send_html(200, html_map_single(2))
            if path == "/map":
                did = int(qs.get("id", ["1"])[0])
                return self._send_html(200, html_map_single(did))
            if path == "/fleet":
                ids = [int(x) for x in (qs.get("ids", ["1,2"])[0].split(",")) if x.strip().isdigit()]
                ids = ids or [1,2]
                return self._send_html(200, html_map_fleet(ids))

            # Proxies
            if path.startswith("/api/sensors/"):
                drone_id = path.split("/")[-1]
                if not drone_id.isdigit():
                    return self._send_json(400, {"error":"bad drone id"})
                url = f"{ARGS.base.rstrip('/')}/sensors/{drone_id}"
                try:
                    req = urllib.request.Request(url, headers={"Accept":"application/json"})
                    with urllib.request.urlopen(req, timeout=1.2) as r:
                        data = r.read()
                        try:
                            self.send_response(200)
                            self.send_header("Content-Type","application/json")
                            self.send_header("Access-Control-Allow-Origin","*")
                            self.send_header("Content-Length", str(len(data)))
                            self._safe_end_headers()
                            self._safe_write(data)
                        except BrokenPipeError:
                            pass
                        return
                except urllib.error.HTTPError as e:
                    return self._send_json(e.code, {"error":"upstream", "status": e.code})
                except Exception as e:
                    return self._send_json(502, {"error":"upstream unavailable", "detail": str(e)})

            if path.startswith("/api/scene/"):
                drone_id = path.split("/")[-1]
                if not drone_id.isdigit():
                    return self._send_json(400, {"error":"bad drone id"})
                url = f"{ARGS.scene.rstrip('/')}/drone{drone_id}/scene"
                try:
                    req = urllib.request.Request(url, headers={"Accept":"application/json"})
                    with urllib.request.urlopen(req, timeout=1.2) as r:
                        data = r.read()
                        try:
                            self.send_response(200)
                            self.send_header("Content-Type","application/json")
                            self.send_header("Access-Control-Allow-Origin","*")
                            self.send_header("Content-Length", str(len(data)))
                            self._safe_end_headers()
                            self._safe_write(data)
                        except BrokenPipeError:
                            pass
                        return
                except urllib.error.HTTPError as e:
                    return self._send_json(e.code, {"error":"upstream", "status": e.code})
                except Exception as e:
                    return self._send_json(502, {"error":"upstream unavailable", "detail": str(e)})

            # Unknown path
            return self._send_json(404, {"error":"not found"})
        except BrokenPipeError:
            # Client disappeared; nothing to do.
            self.close_connection = True
        except Exception as e:
            # Try to report an error once; swallow broken pipes.
            try:
                return self._send_json(500, {"error": str(e)})
            except BrokenPipeError:
                self.close_connection = True

# ----------------------- Main -----------------------
def main():
    srv = ThreadingHTTPServer(("0.0.0.0", ARGS.port), Handler)
    print(f"[multi-map] http://0.0.0.0:{ARGS.port}  "
          f"pages: /map1 /map2 /map?id=1|2 /fleet?ids=1,2  "
          f"proxy: /api/sensors/<id> -> {ARGS.base}/sensors/<id>, /api/scene/<id> -> {ARGS.scene}/drone<id>/scene")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()

