"""Live OEE-style web dashboard for the colour-sorting cell.

Subscribes to /cell/telemetry and serves a self-contained web dashboard (no
external dependencies) showing the state, parts sorted, throughput, cycle time,
per-colour counts, and alarms, the kind of line-monitoring view a production
cell exposes. Open http://localhost:8080 in a browser.

    ros2 run armik_moveit dashboard
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PORT = 8080
STATE = {"tele": {}}

# Live 3D view of the robot for the /control room page. Points at the running
# cell's robot: a URSim/UR noVNC canvas by default, overridable for real
# hardware or a different host.
ROBOT_VNC_URL = os.environ.get(
    "ROBOT_VNC_URL",
    "http://172.17.0.2:6080/vnc.html?host=172.17.0.2&port=6080"
    "&autoconnect=true&resize=scale&reconnect=true&view_only=1",
)

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>Colour Sorting Cell</title>
<style>
:root{--bg:#0e1116;--card:#161b22;--line:#232a33;--fg:#e6edf3;--mut:#8b949e;--ok:#2ea043;--warn:#d29922;--red:#d92626;--green:#2ca02c;--blue:#2659d9}
*{box-sizing:border-box;font-family:'Segoe UI',system-ui,sans-serif}
body{margin:0;background:var(--bg);color:var(--fg)}
header{padding:18px 28px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:14px}
header h1{font-size:18px;margin:0;font-weight:600;letter-spacing:.3px}
#state{margin-left:auto;padding:6px 14px;border-radius:20px;background:#21262d;font-size:13px;font-weight:600}
.wrap{padding:24px 28px;max-width:900px;margin:0 auto}
.kpis{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin-bottom:22px}
.kpi{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px}
.kpi .v{font-size:30px;font-weight:700}.kpi .l{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.6px;margin-top:4px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px;margin-bottom:16px}
.card h2{font-size:13px;color:var(--mut);text-transform:uppercase;letter-spacing:.6px;margin:0 0 14px}
.bar{display:flex;align-items:center;gap:12px;margin:9px 0}
.bar .name{width:60px;font-size:14px}
.bar .track{flex:1;height:14px;background:#21262d;border-radius:7px;overflow:hidden}
.bar .fill{height:100%;border-radius:7px;transition:width .4s}
.bar .n{width:34px;text-align:right;font-variant-numeric:tabular-nums;font-weight:600}
#alarm{display:none;background:#3d1418;border:1px solid var(--red);color:#ffb3b3;padding:12px 16px;border-radius:10px;margin-bottom:16px;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block}
</style></head><body>
<header><span class="dot" id="live" style="background:var(--ok)"></span>
<h1>Colour Sorting Cell — Live Monitor</h1><span id="state">--</span></header>
<div class="wrap">
<div id="safety" style="display:flex;align-items:center;gap:14px;padding:12px 16px;border-radius:10px;margin-bottom:16px;border:1px solid var(--line);background:var(--card);font-weight:600">
 <span class="dot" id="sdot" style="background:var(--mut)"></span>
 <span>SAFETY: <span id="sstate">--</span></span>
 <span id="sreason" style="color:var(--mut);font-weight:400"></span>
 <span style="margin-left:auto;color:var(--mut);font-weight:400">Speed <span id="sspeed">--</span></span>
</div>
<div id="alarm"></div>
<div class="kpis">
 <div class="kpi"><div class="v" id="parts">0</div><div class="l">Parts sorted</div></div>
 <div class="kpi"><div class="v" id="tput">0</div><div class="l">Throughput /min</div></div>
 <div class="kpi"><div class="v" id="cycle">0</div><div class="l">Cycle time (s)</div></div>
 <div class="kpi"><div class="v" id="uptime">0</div><div class="l">Uptime (s)</div></div>
</div>
<div class="card"><h2>Sorted by colour</h2>
 <div class="bar"><div class="name">Red</div><div class="track"><div class="fill" id="fr" style="background:var(--red);width:0"></div></div><div class="n" id="nr">0</div></div>
 <div class="bar"><div class="name">Green</div><div class="track"><div class="fill" id="fg" style="background:var(--green);width:0"></div></div><div class="n" id="ng">0</div></div>
 <div class="bar"><div class="name">Blue</div><div class="track"><div class="fill" id="fb" style="background:var(--blue);width:0"></div></div><div class="n" id="nb">0</div></div>
</div>
<div class="card" style="color:var(--mut);font-size:13px">OPC UA endpoint: <code>opc.tcp://&lt;host&gt;:4840/cell/</code> &nbsp;·&nbsp; a PLC writes CellController/TargetColour to command a sort.</div>
</div>
<script>
async function tick(){
 try{
  const t=await (await fetch('/metrics')).json();
  document.getElementById('state').textContent=(t.state||'--').toUpperCase()+(t.current_color?(' · '+t.current_color):'');
  document.getElementById('parts').textContent=t.parts_sorted||0;
  document.getElementById('tput').textContent=(t.throughput_ppm||0).toFixed(1);
  document.getElementById('cycle').textContent=(t.last_cycle_s||0).toFixed(1);
  document.getElementById('uptime').textContent=Math.round(t.uptime_s||0);
  const c=t.counts||{}, mx=Math.max(1,c.red||0,c.green||0,c.blue||0);
  const set=(f,n,v)=>{document.getElementById(f).style.width=(100*(v||0)/mx)+'%';document.getElementById(n).textContent=v||0};
  set('fr','nr',c.red);set('fg','ng',c.green);set('fb','nb',c.blue);
  const ss=t.safety_state||'--';
  document.getElementById('sstate').textContent=ss;
  document.getElementById('sreason').textContent=t.safety_reason?('— '+t.safety_reason):'';
  document.getElementById('sspeed').textContent=Math.round(100*(t.speed_scale!=null?t.speed_scale:1))+'%';
  const col={RUN:'var(--ok)',REDUCED:'var(--warn)',ESTOP:'var(--red)',GUARD_STOP:'var(--red)',FAULT:'var(--red)',INIT:'var(--mut)'}[ss]||'var(--mut)';
  document.getElementById('sdot').style.background=col;
  document.getElementById('safety').style.borderColor=(ss==='RUN'||ss==='INIT')?'var(--line)':col;
  const a=document.getElementById('alarm');
  if(t.alarm){a.style.display='block';a.textContent='ALARM: '+(t.alarm_msg||'fault')}else{a.style.display='none'}
  document.getElementById('live').style.background='var(--ok)';
 }catch(e){document.getElementById('live').style.background='var(--warn)'}
}
setInterval(tick,1000);tick();
</script></body></html>"""


# Control-room HMI: the live robot (URSim/UR noVNC) alongside live process and
# safety telemetry in one full-screen view, the way a cell is watched on the
# floor. Served at /control; the robot view is __ROBOT_URL__ (see ROBOT_VNC_URL).
CONTROL_PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>Cell Control Room</title>
<style>
:root{--bg:#0b0e13;--card:#161b22;--line:#232a33;--fg:#e6edf3;--mut:#8b949e;--ok:#2ea043;--warn:#d29922;--red:#d92626;--green:#2ca02c;--blue:#2659d9}
*{box-sizing:border-box;font-family:'Segoe UI',system-ui,sans-serif;margin:0}
html,body{height:100%}
body{background:var(--bg);color:var(--fg);display:flex;flex-direction:column;overflow:hidden}
header{padding:12px 22px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:14px;flex:0 0 auto}
header h1{font-size:16px;font-weight:600;letter-spacing:.3px}
.pill{margin-left:auto;padding:6px 16px;border-radius:20px;background:#21262d;font-size:13px;font-weight:700}
main{flex:1;display:grid;grid-template-columns:1.7fr 1fr;gap:0;min-height:0}
.stage{position:relative;border-right:1px solid var(--line);background:#05070a}
.stage iframe{width:100%;height:100%;border:0;display:block}
.stage .tag{position:absolute;left:14px;top:12px;background:rgba(0,0,0,.55);padding:5px 12px;border-radius:16px;font-size:12px;color:#cfe;letter-spacing:.4px}
aside{padding:18px 20px;overflow:auto;display:flex;flex-direction:column;gap:14px}
.safe{display:flex;align-items:center;gap:12px;padding:12px 14px;border-radius:10px;border:1px solid var(--line);background:var(--card);font-weight:700}
.kpis{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.kpi{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
.kpi .v{font-size:26px;font-weight:800}.kpi .l{color:var(--mut);font-size:11px;text-transform:uppercase;letter-spacing:.6px;margin-top:3px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px}
.card h2{font-size:12px;color:var(--mut);text-transform:uppercase;letter-spacing:.6px;margin-bottom:12px}
.bar{display:flex;align-items:center;gap:10px;margin:8px 0}
.bar .name{width:52px;font-size:13px}.bar .track{flex:1;height:13px;background:#21262d;border-radius:7px;overflow:hidden}
.bar .fill{height:100%;border-radius:7px;transition:width .4s}.bar .n{width:26px;text-align:right;font-weight:700;font-variant-numeric:tabular-nums}
#log{flex:1;min-height:120px;font-family:ui-monospace,Menlo,monospace;font-size:12px;line-height:1.7;overflow:auto}
#log div{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#log .t{color:var(--mut)}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block}
</style></head><body>
<header><span class="dot" id="live" style="background:var(--ok)"></span>
<h1>Colour Sorting Cell — Control Room</h1><span class="pill" id="state">--</span></header>
<main>
 <div class="stage"><span class="tag">● LIVE ROBOT — UR5e (URSim, RTDE)</span>
  <iframe src="__ROBOT_URL__" allow="fullscreen"></iframe></div>
 <aside>
  <div class="safe"><span class="dot" id="sdot" style="background:var(--mut)"></span>
   <span>SAFETY: <span id="sstate">--</span></span>
   <span style="margin-left:auto;color:var(--mut);font-weight:400">Speed <span id="sspeed">--</span></span></div>
  <div class="kpis">
   <div class="kpi"><div class="v" id="parts">0</div><div class="l">Parts sorted</div></div>
   <div class="kpi"><div class="v" id="tput">0</div><div class="l">Throughput /min</div></div>
   <div class="kpi"><div class="v" id="cycle">0</div><div class="l">Cycle (s)</div></div>
   <div class="kpi"><div class="v" id="avail">0</div><div class="l">On board</div></div>
  </div>
  <div class="card"><h2>Sorted by colour</h2>
   <div class="bar"><div class="name">Red</div><div class="track"><div class="fill" id="fr" style="background:var(--red);width:0"></div></div><div class="n" id="nr">0</div></div>
   <div class="bar"><div class="name">Green</div><div class="track"><div class="fill" id="fg" style="background:var(--green);width:0"></div></div><div class="n" id="ng">0</div></div>
   <div class="bar"><div class="name">Blue</div><div class="track"><div class="fill" id="fb" style="background:var(--blue);width:0"></div></div><div class="n" id="nb">0</div></div>
  </div>
  <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:0"><h2>Event log</h2><div id="log"></div></div>
 </aside>
</main>
<script>
let lastState="",lastParts=-1,lastSafe="";
function log(msg,cls){const l=document.getElementById('log');const d=document.createElement('div');
 const ts=new Date().toLocaleTimeString();d.innerHTML='<span class="t">'+ts+'</span>  '+msg;
 if(cls)d.style.color=cls;l.insertBefore(d,l.firstChild);while(l.children.length>40)l.removeChild(l.lastChild);}
async function tick(){
 try{
  const t=await (await fetch('/metrics')).json();
  const st=(t.state||'--').toUpperCase();
  document.getElementById('state').textContent=st+(t.current_color?(' · '+t.current_color.toUpperCase()):'');
  document.getElementById('parts').textContent=t.parts_sorted||0;
  document.getElementById('tput').textContent=(t.throughput_ppm||0).toFixed(1);
  document.getElementById('cycle').textContent=(t.last_cycle_s||0).toFixed(1);
  document.getElementById('avail').textContent=(t.available||[]).length;
  const c=t.counts||{},mx=Math.max(1,c.red||0,c.green||0,c.blue||0);
  const set=(f,n,v)=>{document.getElementById(f).style.width=(100*(v||0)/mx)+'%';document.getElementById(n).textContent=v||0};
  set('fr','nr',c.red);set('fg','ng',c.green);set('fb','nb',c.blue);
  const ss=t.safety_state||'--';
  document.getElementById('sstate').textContent=ss;
  document.getElementById('sspeed').textContent=Math.round(100*(t.speed_scale!=null?t.speed_scale:1))+'%';
  const col={RUN:'var(--ok)',REDUCED:'var(--warn)',ESTOP:'var(--red)',GUARD_STOP:'var(--red)',FAULT:'var(--red)',INIT:'var(--mut)'}[ss]||'var(--mut)';
  document.getElementById('sdot').style.background=col;
  if(t.current_color&&t.state==='sorting'&&lastState!=='sorting')log('SORT '+t.current_color.toUpperCase()+' commanded','#cfe');
  if((t.parts_sorted||0)>lastParts&&lastParts>=0)log('part placed  ·  total '+t.parts_sorted,'var(--ok)');
  if(ss!==lastSafe&&lastSafe)log('SAFETY '+lastSafe+' -> '+ss,col);
  lastState=t.state;lastParts=t.parts_sorted||0;lastSafe=ss;
  document.getElementById('live').style.background='var(--ok)';
 }catch(e){document.getElementById('live').style.background='var(--warn)'}
}
log('control room online','var(--mut)');setInterval(tick,700);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/metrics"):
            body = json.dumps(STATE["tele"]).encode()
            ctype = "application/json"
        elif self.path.startswith("/control"):
            body = CONTROL_PAGE.replace("__ROBOT_URL__", ROBOT_VNC_URL).encode()
            ctype = "text/html"
        else:
            body = PAGE.encode()
            ctype = "text/html"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


class Dashboard(Node):
    def __init__(self):
        super().__init__("dashboard")
        self.create_subscription(String, "/cell/telemetry", self._on, 10)

    def _on(self, msg):
        try:
            STATE["tele"] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass


def main():
    rclpy.init()
    node = Dashboard()
    srv = HTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"dashboard at http://localhost:{PORT}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
