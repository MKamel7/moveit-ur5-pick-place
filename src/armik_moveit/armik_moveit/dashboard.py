"""Live OEE-style web dashboard for the colour-sorting cell.

Subscribes to /cell/telemetry and serves a self-contained web dashboard (no
external dependencies) showing the state, parts sorted, throughput, cycle time,
per-colour counts, and alarms, the kind of line-monitoring view a production
cell exposes. Open http://localhost:8080 in a browser.

    ros2 run armik_moveit dashboard
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PORT = 8080
STATE = {"tele": {}}

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


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/metrics"):
            body = json.dumps(STATE["tele"]).encode()
            ctype = "application/json"
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
