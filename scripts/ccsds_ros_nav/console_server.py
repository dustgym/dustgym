"""Human-in-the-loop operator console (host-side FastAPI web UI).

The Earth side of the loop. Point-and-click a goal on the real Haworth hillshade; the console plans a
slope-aware route, commands it over the CCSDS link to the (containerized) rover, and shows the rover
driving back via delayed telemetry. The console OWNS the adjustable latency model (so it is live-tunable
and the operator genuinely experiences the lag) and the mission clock (so accelerating time sweeps the
Sun's azimuth and walks terrain shadows across the patch). A second page shows all rover cameras — the
imagery feed COLMAP consumes — rendered on the host GPU at the rover's pose under the mission Sun.

Runs on the HOST (it needs the GPU + Godot for the camera render); the ROS rover stack stays in the
container. Latency is injected here, on the ground side of the link.

    python scripts/ccsds_ros_nav/console_server.py --bridge-host 127.0.0.1 --port 8080
"""
from __future__ import annotations

import argparse
import heapq
import io
import os
import sys
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", ".."))
for _p in (_HERE, os.path.join(_HERE, "render"), _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import camera_render
import messages
import mission_clock as mc
from flight import load_crop
from link import UdpLink
from route import plan_route, slope_deg, snap_to_navigable


class Console:
    """Operator-side state: CCSDS link + latency model + mission clock + route queue + telemetry."""

    def __init__(self, *, haworth: str, r0: int, c0: int, win: int, bridge_host: str, bridge_port: int,
                 local_port: int, round_trip_s: float, time_factor: float, sun_el: float,
                 v_max: float, max_slope_deg: float, cam_size: str = "320x240",
                 cam_pitch_deg: float = 10.0, chase_size: str = "960x720") -> None:
        self.crop = load_crop(haworth, r0, c0, win, win)
        self.haworth = haworth
        self.win = win
        self.cell_m = self.crop.cell_m
        self.v_max = v_max
        self.max_slope_deg = max_slope_deg
        self.capture_size = cam_size
        # Front-stereo down-pitch (camera_rig.gd --cam-pitch): the rover cameras sit ~8 cm up, so a
        # level gaze is half black sky. A modest downward tilt centers the lit terrain band.
        self.cam_pitch_deg = float(cam_pitch_deg)
        self.chase_size = str(chase_size)
        self._slope = slope_deg(self.crop.heightmap, self.cell_m)

        # latency model (round-trip, in mission seconds); wall delay per direction scales with time_factor.
        self.round_trip_s = float(round_trip_s)
        self.time_factor = max(1e-3, float(time_factor))
        self.sun_el = float(sun_el)
        az0, lit0 = mc.find_illuminated_start(self.crop.heightmap, self.cell_m, el_deg=self.sun_el)
        self.clock = mc.MissionClock(az0_deg=az0, el_deg=self.sun_el, time_factor=self.time_factor)
        self._start_lit = lit0

        self.link = UdpLink(("0.0.0.0", local_port), (bridge_host, bridge_port),
                            light_time_s=self._one_way_wall())

        self._lock = threading.Lock()
        self._inbound: list[tuple[float, int, object]] = []   # heap (release_wall, seq, SpacePacket)
        self._inseq = 0
        self._pose: dict | None = None
        self._last_leg: dict | None = None
        self._queue: list[tuple[float, float]] = []           # remaining waypoints (row,col)
        self._leg_id = 0
        self._last_capture: list[dict] = []
        self._capture_seq = 0                                 # bumps only on a completed render (UI gates reload on it)
        self._chase_path: str | None = None                   # third-person 'coach' view (opt-in, higher res)
        self._chase_seq = 0
        # rover pose belief for route planning / capture: starts at the executive's snapped boot pose.
        sr, sc = snap_to_navigable(self._slope, (win // 3, win // 3), max_slope_deg)
        self._belief_rc = (float(sr), float(sc))
        self._belief_yaw = 0.0
        # the lander/deployment point: a FIXED world landmark the rover drives away from (so the camera
        # feed visibly moves with the rover instead of being pinned to a glued-ahead lander).
        self._lander_rc = (int(sr), int(sc))

        self._hillshade = self._compute_hillshade()
        self._stop = threading.Event()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

    # --- latency ---------------------------------------------------------------------------------
    def _one_way_wall(self) -> float:
        return max(0.0, (self.round_trip_s * 0.5) / self.time_factor)

    def set_config(self, *, round_trip_s=None, time_factor=None, sun_el=None, cam_pitch_deg=None) -> None:
        with self._lock:
            if cam_pitch_deg is not None:
                self.cam_pitch_deg = max(0.0, float(cam_pitch_deg))
            if round_trip_s is not None:
                self.round_trip_s = max(0.0, float(round_trip_s))
            if time_factor is not None:
                self.time_factor = max(1e-3, float(time_factor))
                self.clock.set_time_factor(self.time_factor)
                self.link.send(messages.encode(messages.SetSim(self.time_factor),
                                               met=self.clock.mission_time()))   # retime the rover too
            if sun_el is not None:
                self.sun_el = float(sun_el)
                self.clock.el_deg = self.sun_el
            self.link.light_time_s = self._one_way_wall()

    def set_cam_size(self, cam_size: str) -> None:
        with self._lock:
            self.capture_size = str(cam_size)

    # --- telemetry release loop (applies the downlink half of the latency) ------------------------
    def _telemetry_loop(self) -> None:
        while not self._stop.is_set():
            pkt = self.link.recv(timeout=0.1)
            now = time.monotonic()
            with self._lock:
                if pkt is not None:
                    heapq.heappush(self._inbound, (now + self._one_way_wall(), self._inseq, pkt))
                    self._inseq += 1
                while self._inbound and self._inbound[0][0] <= now:
                    self._process(heapq.heappop(self._inbound)[2])

    def _process(self, pkt) -> None:
        msg = messages.decode(pkt)
        if isinstance(msg, messages.Pose):
            self._pose = vars(msg)
            self._belief_rc = (msg.row, msg.col)
            self._belief_yaw = msg.yaw_rad
        elif isinstance(msg, messages.Leg):
            self._last_leg = vars(msg)
            if msg.status == messages.LEG_REACHED and self._queue:
                self._send_next_locked()
            elif msg.status != messages.LEG_REACHED:
                self._queue.clear()                            # non-nominal leg ends the route

    # --- commanding ------------------------------------------------------------------------------
    def _send_next_locked(self) -> None:
        if not self._queue:
            return
        row, col = self._queue.pop(0)
        self._leg_id += 1
        self.link.send(messages.encode(
            messages.GoTo(leg_id=self._leg_id, goal_row=row, goal_col=col, v_max_mps=self.v_max,
                          goal_radius_cells=1.0), seq_count=self._leg_id, met=self.clock.mission_time()))

    def on_goal(self, u: float, v: float) -> dict:
        """Plan a slope-aware route from the rover belief to the clicked (normalized) point + start it."""
        with self._lock:
            goal = snap_to_navigable(self._slope,
                                     (int(round(v * (self.win - 1))), int(round(u * (self.win - 1)))),
                                     self.max_slope_deg)
            start = (int(round(self._belief_rc[0])), int(round(self._belief_rc[1])))
            wps = plan_route(self.crop.heightmap, self.cell_m, start, goal,
                             max_slope_deg=self.max_slope_deg, n_waypoints=6)
            self._queue = wps
            if wps:
                self._send_next_locked()
            return {"ok": bool(wps), "waypoints": wps, "goal": list(goal)}

    def on_stop(self) -> None:
        with self._lock:
            self._queue.clear()
            self.link.send(messages.encode(messages.Safe(reason=1), met=self.clock.mission_time()))

    # --- camera capture (host GPU) ---------------------------------------------------------------
    def capture(self, size: str | None = None) -> dict:
        with self._lock:
            rc = (int(round(self._belief_rc[0])), int(round(self._belief_rc[1])))
            yaw = self._belief_yaw
            lander = self._lander_rc
            sz = size or self.capture_size
            pitch = self.cam_pitch_deg
            az, el = self.clock.sun()
        # single-frame at the rover's ACTUAL pose + heading, with the lander pinned to its world cell, so
        # the view tracks the rover. One shot per capture; the UI gates its reload on _capture_seq.
        cap = camera_render.render_single(_REPO, self.crop, rc, yaw_flight=yaw, sun_az=az, sun_el=el,
                                          scene_name="ccsds_nav_live", haworth_dir=self.haworth,
                                          lander_rc=lander, size=sz, cam_pitch_deg=pitch)
        if not cap["ok"]:
            return {"ok": False}
        latest: dict[str, dict] = {}                           # one frame per camera = the live view
        for fr in cap["frames"]:
            latest[fr["camera"]] = fr
        with self._lock:
            self._last_capture = [{"camera": k, "path": v["path"], "frame": v["frame"]}
                                  for k, v in sorted(latest.items())]
            self._capture_seq += 1
            seq = self._capture_seq
        return {"ok": True, "cameras": [c["camera"] for c in self._last_capture],
                "sun": [round(az, 1), round(el, 2)], "capture_seq": seq, "size": sz}

    def capture_chase(self, size: str | None = None) -> dict:
        """Render the third-person 'coach' view (sidecar --chase-cam): an EXTERNAL trailing camera
        looking at the rover under the same faithful lighting as the rover cams (it casts its shadow on
        the lit regolith). 'Cheating' for a real operator (the rover can't see itself), so opt-in + higher res."""
        with self._lock:
            rc = (int(round(self._belief_rc[0])), int(round(self._belief_rc[1])))
            yaw = self._belief_yaw
            lander = self._lander_rc
            sz = size or self.chase_size
            az, el = self.clock.sun()
        cap = camera_render.render_chase(_REPO, self.crop, rc, yaw_flight=yaw, sun_az=az, sun_el=el,
                                         scene_name="ccsds_nav_chase", haworth_dir=self.haworth,
                                         lander_rc=lander, size=sz)
        if not cap["ok"]:
            return {"ok": False}
        with self._lock:
            self._chase_path = cap["path"]
            self._chase_seq += 1
            seq = self._chase_seq
        return {"ok": True, "chase_seq": seq, "size": sz}

    # --- views -----------------------------------------------------------------------------------
    def _compute_hillshade(self) -> np.ndarray:
        hm = self.crop.heightmap
        gy, gx = np.gradient(hm, self.cell_m)
        az, el = np.radians(315.0), np.radians(35.0)
        aspect = np.arctan2(-gx, gy)
        slope = np.arctan(np.hypot(gx, gy))
        sh = np.sin(el) * np.cos(slope) + np.cos(el) * np.sin(slope) * np.cos(az - aspect)
        return np.clip(sh, 0, 1)

    def map_png(self) -> bytes:
        """Hillshade with the current-Sun cast-shadow overlay (shadows move as mission time advances)."""
        from PIL import Image
        az, el = self.clock.sun()
        lit = mc.illumination.horizon_clip(self.crop.heightmap, self.cell_m, az, el)
        base = self._hillshade.copy()
        shaded = np.where(lit, 0.4 + 0.6 * base, 0.12 * base)   # darken terrain-shadowed cells
        img = (np.clip(shaded, 0, 1) * 255).astype(np.uint8)
        rgb = np.stack([img, img, np.clip(img.astype(np.int16) + 8, 0, 255).astype(np.uint8)], axis=-1)
        buf = io.BytesIO()
        Image.fromarray(rgb, "RGB").save(buf, format="PNG")
        return buf.getvalue()

    def state(self) -> dict:
        with self._lock:
            az, el = self.clock.sun()
            return {
                "win": self.win, "cell_m": self.cell_m,
                "pose": self._pose, "last_leg": self._last_leg,
                "belief_rc": list(self._belief_rc), "belief_yaw": round(self._belief_yaw, 4),
                "queue_len": len(self._queue),
                "round_trip_s": self.round_trip_s, "time_factor": self.time_factor,
                "one_way_wall_s": round(self._one_way_wall(), 3),
                "mission_time_s": round(self.clock.mission_time(), 1),
                "lunar_day_frac": round(self.clock.lunar_day_fraction(), 4),
                "sun_az_deg": round(az, 1), "sun_el_deg": round(el, 2),
                "lit_fraction": round(mc.lit_fraction(self.crop.heightmap, self.cell_m, az, el), 3),
                "cameras": [c["camera"] for c in self._last_capture],
                "capture_seq": self._capture_seq, "capture_size": self.capture_size,
                "cam_pitch_deg": round(self.cam_pitch_deg, 1), "chase_seq": self._chase_seq,
            }


# ============================================================================================
# Web app
# ============================================================================================
from fastapi import FastAPI, Request                                          # noqa: E402
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response  # noqa: E402

app = FastAPI(title="dustgym HITL console")
CONSOLE: "Console | None" = None


_MAP_HTML = """<!doctype html><html><head><meta charset=utf-8><title>HITL console — map</title>
<style>body{background:#111;color:#ddd;font:13px monospace;margin:0;display:flex}
#left{padding:10px}#stage{position:relative;width:600px;height:600px}
#bg,#ov{position:absolute;left:0;top:0;width:600px;height:600px}
#hud{white-space:pre;padding:10px;min-width:260px}
button,input{font:12px monospace;margin:2px 0}.row{margin:6px 0}a{color:#6cf}</style></head>
<body><div id=left><div id=stage>
<img id=bg src="/map.png"><canvas id=ov width=600 height=600></canvas></div>
<div class=row>click the map to send a goal &nbsp; <button onclick="stop()">STOP (safe)</button>
&nbsp; <a href="/cameras">camera feed »</a></div></div>
<div id=hud>connecting…
<div class=row>round-trip latency <input id=rt type=range min=0 max=20 step=0.5 oninput="cfg()"> <span id=rtv></span>s</div>
<div class=row>time factor <input id=tf type=range min=1 max=2000 step=1 oninput="cfg()"> <span id=tfv></span>x</div>
<div class=row>sun elevation <input id=el type=range min=0.5 max=20 step=0.5 oninput="cfg()"> <span id=elv></span>°</div>
</div>
<script>
const bg=document.getElementById('bg'),ov=document.getElementById('ov'),ctx=ov.getContext('2d');
let win=160,inited=false;
ov.addEventListener('click',e=>{const r=ov.getBoundingClientRect();
 fetch('/goal',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({u:(e.clientX-r.left)/600,v:(e.clientY-r.top)/600})});});
function stop(){fetch('/stop',{method:'POST'});}
function cfg(){fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({round_trip_s:+rt.value,time_factor:+tf.value,sun_el:+el.value})});}
function P(rc){return [rc[1]/win*600, rc[0]/win*600];}
function arrow(x,y,ang){            // heading arrow: canvas angle == flight yaw (forward = cos,sin)
 const L=13,B=6,W=7,ca=Math.cos(ang),sa=Math.sin(ang),px=-sa,py=ca;
 ctx.fillStyle='#3f6';ctx.strokeStyle='#0c4';ctx.lineWidth=1.5;
 ctx.beginPath();
 ctx.moveTo(x+ca*L, y+sa*L);                          // tip (points along heading)
 ctx.lineTo(x-ca*B+px*W, y-sa*B+py*W);                // rear-left
 ctx.lineTo(x-ca*B-px*W, y-sa*B-py*W);                // rear-right
 ctx.closePath();ctx.fill();ctx.stroke();
}
async function tick(){
 const s=await (await fetch('/state')).json(); win=s.win;
 if(!inited){rt.value=s.round_trip_s;tf.value=s.time_factor;el.value=s.sun_el_deg;inited=true;}
 rtv.textContent=s.round_trip_s.toFixed(1);tfv.textContent=s.time_factor;elv.textContent=s.sun_el_deg;
 ctx.clearRect(0,0,600,600);
 if(s.belief_rc){const[x,y]=P(s.belief_rc);arrow(x,y,s.belief_yaw||0);}
 const soc=s.pose?(s.pose.soc*100).toFixed(0):'—',slip=s.pose?s.pose.slip.toFixed(3):'—';
 const day=(s.lunar_day_frac*100).toFixed(1);
 document.getElementById('hud').firstChild.textContent=
  `MISSION  t=${(s.mission_time_s/3600).toFixed(2)} h  (lunar day ${day}%)\n`+
  `SUN      az ${s.sun_az_deg}°  el ${s.sun_el_deg}°   lit ${(s.lit_fraction*100).toFixed(0)}%\n`+
  `LINK     RT ${s.round_trip_s}s  (one-way wall ${s.one_way_wall_s}s)  ${s.round_trip_s==0?'[TRAINING: no delay]':''}\n`+
  `ROVER    soc ${soc}%  slip ${slip}  queue ${s.queue_len}\n`+
  `LASTLEG  ${s.last_leg?('leg '+s.last_leg.leg_id+' status '+s.last_leg.status):'—'}`;
}
setInterval(tick,500); setInterval(()=>{bg.src='/map.png?'+Date.now();},4000);
</script></body></html>"""

_CAM_HTML = """<!doctype html><html><head><meta charset=utf-8><title>HITL console — cameras</title>
<style>body{background:#111;color:#ddd;font:13px monospace;padding:10px}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;max-width:1100px}
.cam{border:1px solid #333}.cam img{width:100%;display:block}.cam div{padding:2px 4px;color:#9cf}
#chasewrap{display:none;margin:8px 0;max-width:720px}#chasewrap img{width:100%;display:block;border:1px solid #640}
#chasewrap .cap{color:#fb6;padding:2px 4px}a{color:#6cf}button{font:12px monospace}
input[type=range]{vertical-align:middle}</style></head>
<body><div><button onclick="cap()">capture now</button>
&nbsp;<label><input type=checkbox id=auto checked> auto-capture (track rover)</label>
&nbsp;<label title="External trailing view of the rover (the rover cannot see itself). 'Cheating' for a
flight operator — a sim-manager / coach aid. Same faithful lighting as the rover cams: the rover casts
its shadow on the lit regolith."><input type=checkbox id=chaseon> third-person coach view ⚠</label>
&nbsp;cam-pitch <input id=pitch type=range min=0 max=30 step=1 oninput="setpitch()"> <span id=pv></span>°
&nbsp;<span id=info></span> &nbsp; <a href="/">« map</a></div>
<p>The rover's 8 cameras at its CURRENT pose + heading under the mission Sun — the imagery feed COLMAP
consumes (front/rear stereo, side mono, drum). The lander is a fixed landmark the rover drives away from.
Forward cameras sit ~8 cm up; raise <i>cam-pitch</i> to aim the stereo pair down at the terrain. Under
the grazing polar Sun parts of the near field fall in cast shadow (faithful) — raise the Sun elevation
(map page) for more fill.</p>
<div id=chasewrap><div class=cap>third-person coach view (external — not a rover sensor)</div><img id=chaseimg alt=chase></div>
<div class=grid id=grid></div>
<script>
let busy=false,cbusy=false,lastSeq=-1,lastChase=-1,imgs={},pinited=false;
async function cap(){if(busy)return;busy=true;
 try{await fetch('/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});}
 catch(e){}finally{busy=false;}}
async function capChase(){if(cbusy)return;cbusy=true;
 try{await fetch('/capture_chase',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});}
 catch(e){}finally{cbusy=false;}}
function setpitch(){fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({cam_pitch_deg:+pitch.value})});pv.textContent=pitch.value;}
function ensureGrid(cams){
 if(Object.keys(imgs).length)return;
 cams.forEach(c=>{const d=document.createElement('div');d.className='cam';
  const im=document.createElement('img');im.alt=c;const cp=document.createElement('div');cp.textContent=c;
  d.appendChild(im);d.appendChild(cp);grid.appendChild(d);imgs[c]=im;});}
function pre1(el,src){const p=new Image();p.onload=()=>{el.src=p.src;};p.src=src;}  // swap only once decoded
function swap(cams,seq){cams.forEach(c=>pre1(imgs[c],'/cam/'+c+'?seq='+seq));}
async function poll(){let s;try{s=await (await fetch('/state')).json();}catch(e){return;}
 if(!pinited){pitch.value=s.cam_pitch_deg;pv.textContent=s.cam_pitch_deg;pinited=true;}
 info.textContent='rover ('+s.belief_rc.map(x=>x.toFixed(0)).join(',')+')  sun az '+s.sun_az_deg+
  '° el '+s.sun_el_deg+'°  lit '+(s.lit_fraction*100).toFixed(0)+'%  res '+s.capture_size+
  '  cap#'+s.capture_seq;
 if((s.cameras||[]).length){ensureGrid(s.cameras);
  if(s.capture_seq!==lastSeq){lastSeq=s.capture_seq;swap(s.cameras,s.capture_seq);}}
 const on=chaseon.checked;chasewrap.style.display=on?'block':'none';
 if(on&&s.chase_seq>0&&s.chase_seq!==lastChase){lastChase=s.chase_seq;pre1(chaseimg,'/chase?seq='+s.chase_seq);}}
setInterval(poll,1000);                       // HUD/state at 1 Hz (cheap)
setInterval(()=>{if(document.getElementById('auto').checked)cap();},4000);   // re-render once / 4 s
setInterval(()=>{if(chaseon.checked)capChase();},4000);                      // coach view (opt-in) / 4 s
poll();
</script></body></html>"""


@app.get("/", response_class=HTMLResponse)
def _map_page() -> str:
    return _MAP_HTML


@app.get("/cameras", response_class=HTMLResponse)
def _cam_page() -> str:
    return _CAM_HTML


@app.get("/map.png")
def _map_png() -> Response:
    return Response(CONSOLE.map_png(), media_type="image/png")


@app.get("/state")
def _state() -> JSONResponse:
    return JSONResponse(CONSOLE.state())


@app.post("/goal")
async def _goal(req: Request) -> JSONResponse:
    b = await req.json()
    return JSONResponse(CONSOLE.on_goal(float(b["u"]), float(b["v"])))


@app.post("/stop")
def _stop() -> JSONResponse:
    CONSOLE.on_stop()
    return JSONResponse({"ok": True})


@app.post("/config")
async def _config(req: Request) -> JSONResponse:
    b = await req.json()
    CONSOLE.set_config(round_trip_s=b.get("round_trip_s"), time_factor=b.get("time_factor"),
                       sun_el=b.get("sun_el"), cam_pitch_deg=b.get("cam_pitch_deg"))
    if b.get("cam_size"):
        CONSOLE.set_cam_size(b["cam_size"])
    return JSONResponse(CONSOLE.state())


@app.post("/capture")
async def _capture(req: Request) -> JSONResponse:
    b = await req.json()
    return JSONResponse(CONSOLE.capture(size=b.get("size")))


@app.post("/capture_chase")
async def _capture_chase(req: Request) -> JSONResponse:
    b = await req.json()
    return JSONResponse(CONSOLE.capture_chase(size=b.get("size")))


@app.get("/cam/{camera}")
def _cam(camera: str) -> Response:
    for c in CONSOLE._last_capture:
        if c["camera"] == camera:
            return FileResponse(c["path"], media_type="image/png")
    return Response(status_code=404)


@app.get("/chase")
def _chase() -> Response:
    if CONSOLE._chase_path and os.path.exists(CONSOLE._chase_path):
        return FileResponse(CONSOLE._chase_path, media_type="image/png")
    return Response(status_code=404)


def main() -> int:
    import uvicorn
    ap = argparse.ArgumentParser(description="HITL operator console (web UI) for the rover nav stack")
    ap.add_argument("--scene", default="samples/lunar_dem/haworth_10km_5m")
    ap.add_argument("--r0", type=int, default=720)
    ap.add_argument("--c0", type=int, default=1800)
    ap.add_argument("--win", type=int, default=160)
    ap.add_argument("--bridge-host", default="127.0.0.1")
    ap.add_argument("--bridge-port", type=int, default=52001)
    ap.add_argument("--local-port", type=int, default=52000)
    ap.add_argument("--round-trip-s", type=float, default=8.0)
    ap.add_argument("--time-factor", type=float, default=1.0)
    ap.add_argument("--sun-el", type=float, default=3.0)
    ap.add_argument("--v-max", type=float, default=0.3)
    ap.add_argument("--max-slope-deg", type=float, default=18.0)
    ap.add_argument("--cam-size", default="320x240", help="rover camera render resolution (WxH)")
    ap.add_argument("--cam-pitch", type=float, default=10.0,
                    help="front-stereo downward pitch (deg) so the low rover cams aim at terrain, not sky")
    ap.add_argument("--chase-size", default="960x720", help="third-person coach view resolution (WxH)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    scene = args.scene if os.path.isabs(args.scene) else os.path.join(_REPO, args.scene)
    global CONSOLE
    CONSOLE = Console(haworth=scene, r0=args.r0, c0=args.c0, win=args.win,
                      bridge_host=args.bridge_host, bridge_port=args.bridge_port,
                      local_port=args.local_port, round_trip_s=args.round_trip_s,
                      time_factor=args.time_factor, sun_el=args.sun_el, v_max=args.v_max,
                      max_slope_deg=args.max_slope_deg, cam_size=args.cam_size,
                      cam_pitch_deg=args.cam_pitch, chase_size=args.chase_size)
    print(f"[console] http://{args.host}:{args.port}  bridge {args.bridge_host}:{args.bridge_port}  "
          f"RT={args.round_trip_s}s  tf={args.time_factor}x  start-lit={CONSOLE._start_lit*100:.0f}%")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
