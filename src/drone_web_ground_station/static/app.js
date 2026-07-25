"use strict";

const $ = (id) => document.getElementById(id);
const mapCanvas = $("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");
const chartIds = ["positionChart", "errorChart", "rpmChart", "progressChart"];
let snapshot = null;
let waypoints = [];
let interactionMode = "goal";
let altitudeTimer = null;
let preview = null;
let viewMode = "top";
let zoom = 1;
let panX = 0;
let panY = 0;
let dragIndex = -1;
let dragging = false;
let dragOriginal = null;
let panning = false;
let lastMouse = null;
let eventSource = null;

function log(message, kind = "") {
  const box = $("eventLog");
  const line = document.createElement("div");
  line.className = `event-line ${kind}`;
  line.textContent = `${new Date().toLocaleTimeString()}  ${message}`;
  box.appendChild(line);
  while (box.children.length > 120) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

async function api(path, body = null) {
  const options = body === null ? {} : {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  };
  const response = await fetch(path, options);
  const payload = await response.json();
  if (!response.ok || !payload.ok) throw new Error(payload.message || `HTTP ${response.status}`);
  return payload.data;
}

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function fmt(value, digits = 2, suffix = "") {
  return Number.isFinite(Number(value)) ? `${Number(value).toFixed(digits)}${suffix}` : "--";
}

function setBadge(element, text, state) {
  element.textContent = text;
  element.className = `badge ${state}`;
}

function resizeCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return {width: rect.width, height: rect.height, ctx};
}

function baseBounds() {
  const b = snapshot?.map?.bounds || [-1, 6, -4, 4, 0, 3.5];
  return {xMin:b[0], xMax:b[1], yMin:b[2], yMax:b[3], zMin:b[4], zMax:b[5]};
}

function visibleBounds() {
  const b = baseBounds();
  const cx = (b.xMin + b.xMax) / 2 + panX;
  const cy = (b.yMin + b.yMax) / 2 + panY;
  const hx = (b.xMax - b.xMin) / (2 * zoom);
  const hy = (b.yMax - b.yMin) / (2 * zoom);
  return {xMin:cx-hx, xMax:cx+hx, yMin:cy-hy, yMax:cy+hy, zMin:b.zMin, zMax:b.zMax};
}

function projectTop(x, y, width, height) {
  const b = visibleBounds();
  return [(x-b.xMin)/(b.xMax-b.xMin)*width, (b.yMax-y)/(b.yMax-b.yMin)*height];
}

function unprojectTop(px, py, width, height) {
  const b = visibleBounds();
  return [b.xMin + px/width*(b.xMax-b.xMin), b.yMax - py/height*(b.yMax-b.yMin)];
}

function projectIso(x, y, z, width, height) {
  const b = baseBounds();
  const sx = width / Math.max(10, (b.xMax-b.xMin)+(b.yMax-b.yMin));
  const sy = sx * 0.55;
  const sz = height / Math.max(5, (b.zMax-b.zMin)*2.5);
  const ox = width * 0.49;
  const oy = height * 0.72;
  const cx = (b.xMin+b.xMax)/2;
  const cy = (b.yMin+b.yMax)/2;
  return [ox + (x-cx-(y-cy))*sx, oy - (x-cx+y-cy)*sy - (z-b.zMin)*sz];
}

function drawGrid(ctx, width, height) {
  const b = visibleBounds();
  ctx.strokeStyle = "rgba(74,103,137,.25)";
  ctx.lineWidth = 1;
  ctx.font = "10px Segoe UI";
  ctx.fillStyle = "#66809d";
  for (let x=Math.ceil(b.xMin); x<=b.xMax; x++) {
    const [px] = projectTop(x, 0, width, height);
    ctx.beginPath(); ctx.moveTo(px,0); ctx.lineTo(px,height); ctx.stroke();
    ctx.fillText(String(x), px+3, height-5);
  }
  for (let y=Math.ceil(b.yMin); y<=b.yMax; y++) {
    const [,py] = projectTop(0, y, width, height);
    ctx.beginPath(); ctx.moveTo(0,py); ctx.lineTo(width,py); ctx.stroke();
    ctx.fillText(String(y), 4, py-3);
  }
}

function drawTopBox(ctx, box, width, height, fill, stroke) {
  const [x1,y1] = projectTop(box.x-box.sx/2, box.y+box.sy/2, width, height);
  const [x2,y2] = projectTop(box.x+box.sx/2, box.y-box.sy/2, width, height);
  ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 1;
  ctx.fillRect(x1,y1,x2-x1,y2-y1); ctx.strokeRect(x1,y1,x2-x1,y2-y1);
}

function drawPath(ctx, points, width, height, color, lineWidth=2) {
  if (!points || points.length < 2) return;
  ctx.beginPath();
  points.forEach((p,i) => {
    const q = viewMode === "top" ? projectTop(p[0],p[1],width,height) : projectIso(p[0],p[1],p[2],width,height);
    if (i===0) ctx.moveTo(q[0],q[1]); else ctx.lineTo(q[0],q[1]);
  });
  ctx.strokeStyle=color; ctx.lineWidth=lineWidth; ctx.stroke();
}

function isoBoxCorners(box) {
  const xs=[box.x-box.sx/2, box.x+box.sx/2], ys=[box.y-box.sy/2,box.y+box.sy/2], zs=[box.z-box.sz/2,box.z+box.sz/2];
  return [[xs[0],ys[0],zs[0]],[xs[1],ys[0],zs[0]],[xs[1],ys[1],zs[0]],[xs[0],ys[1],zs[0]],[xs[0],ys[0],zs[1]],[xs[1],ys[0],zs[1]],[xs[1],ys[1],zs[1]],[xs[0],ys[1],zs[1]]];
}

function drawIsoBox(ctx, box, width, height, fill, stroke) {
  const c=isoBoxCorners(box).map(p=>projectIso(...p,width,height));
  const faces=[[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]];
  faces.forEach(face=>{ctx.beginPath();face.forEach((i,n)=>n?ctx.lineTo(...c[i]):ctx.moveTo(...c[i]));ctx.closePath();ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.stroke();});
}

function drawVehicle(ctx, vehicle, width, height) {
  if (!Number.isFinite(vehicle?.x)) return;
  const p = viewMode === "top" ? projectTop(vehicle.x,vehicle.y,width,height) : projectIso(vehicle.x,vehicle.y,vehicle.z,width,height);
  const yaw=finite(vehicle.yaw); ctx.save();ctx.translate(p[0],p[1]);ctx.rotate(viewMode==="top"?-yaw:0);ctx.beginPath();ctx.moveTo(10,0);ctx.lineTo(-7,-6);ctx.lineTo(-4,0);ctx.lineTo(-7,6);ctx.closePath();ctx.fillStyle="#38bdf8";ctx.strokeStyle="#d8f4ff";ctx.lineWidth=1.5;ctx.fill();ctx.stroke();ctx.restore();
}

function drawMap() {
  const {width,height,ctx}=resizeCanvas(mapCanvas);
  ctx.clearRect(0,0,width,height); ctx.fillStyle="#07111d"; ctx.fillRect(0,0,width,height);
  if (!snapshot) return;
  if (viewMode === "top") drawGrid(ctx,width,height);
  const map=snapshot.map||{};
  (map.inflated||[]).forEach(box=>viewMode==="top"?drawTopBox(ctx,box,width,height,"rgba(47,125,246,.16)","rgba(68,139,255,.45)"):drawIsoBox(ctx,box,width,height,"rgba(47,125,246,.08)","rgba(68,139,255,.25)"));
  (map.obstacles||[]).forEach(box=>viewMode==="top"?drawTopBox(ctx,box,width,height,"rgba(239,68,68,.58)","rgba(255,152,152,.9)"):drawIsoBox(ctx,box,width,height,"rgba(239,68,68,.35)","rgba(255,152,152,.75)"));
  drawPath(ctx,snapshot.paths?.planned,width,height,"#f59e0b",2.7);
  drawPath(ctx,snapshot.paths?.actual,width,height,"#22c55e",2.4);
  waypoints.forEach((p,i)=>{
    const q=viewMode==="top"?projectTop(p[0],p[1],width,height):projectIso(p[0],p[1],p[2],width,height);
    ctx.beginPath();ctx.arc(q[0],q[1],i===Number(snapshot.patrol?.waypoint_index)?7:5,0,Math.PI*2);ctx.fillStyle=p.valid===false?"#ef4444":(i===Number(snapshot.patrol?.waypoint_index)?"#f59e0b":"#a78bfa");ctx.fill();ctx.strokeStyle="#fff";ctx.stroke();ctx.fillStyle="#fff";ctx.font="10px Segoe UI";ctx.fillText(String(i+1),q[0]+7,q[1]-6);
  });
  if (preview) { const q=viewMode==="top"?projectTop(preview.x,preview.y,width,height):projectIso(preview.x,preview.y,preview.z,width,height);ctx.beginPath();ctx.arc(q[0],q[1],8,0,Math.PI*2);ctx.strokeStyle=preview.valid?"#facc15":"#ef4444";ctx.lineWidth=2;ctx.stroke();ctx.beginPath();ctx.moveTo(q[0]-11,q[1]);ctx.lineTo(q[0]+11,q[1]);ctx.moveTo(q[0],q[1]-11);ctx.lineTo(q[0],q[1]+11);ctx.stroke(); }
  const goal=snapshot.paths?.mission_goal;if(goal&&Number.isFinite(goal.x)){const q=viewMode==="top"?projectTop(goal.x,goal.y,width,height):projectIso(goal.x,goal.y,goal.z,width,height);ctx.strokeStyle="#facc15";ctx.lineWidth=2;ctx.strokeRect(q[0]-6,q[1]-6,12,12);}
  drawVehicle(ctx,snapshot.vehicle,width,height);
}

function nearestWaypoint(px,py,maxDistance=15) {
  if (viewMode!=="top") return -1;
  const {width,height}=mapCanvas.getBoundingClientRect();
  let best=-1,bestD=maxDistance;
  waypoints.forEach((p,i)=>{const q=projectTop(p[0],p[1],width,height);const d=Math.hypot(q[0]-px,q[1]-py);if(d<bestD){best=i;bestD=d;}});
  return best;
}

function setInteractionMode(mode) {
  interactionMode = mode === "waypoint" ? "waypoint" : "goal";
  $("goalModeBtn").classList.toggle("active", interactionMode === "goal");
  $("waypointModeBtn").classList.toggle("active", interactionMode === "waypoint");
  $("previewText").textContent = interactionMode === "goal"
    ? "目标飞行模式：单击预览，双击直接飞行"
    : "巡航航点模式：单击地图依次添加航点";
  mapCanvas.style.cursor = interactionMode === "goal" ? "crosshair" : "copy";
}

function publishRvizAltitude() {
  clearTimeout(altitudeTimer);
  altitudeTimer = setTimeout(() => {
    api("/api/rviz/altitude", {
      z: finite($("targetZNumber").value, 1.5),
    }).catch(() => {});
  }, 120);
}

async function previewAt(x,y) {
  const z=finite($("targetZNumber").value,1.5);
  preview={x,y,z,yaw:0,valid:false,reason:"CHECKING"}; drawMap();
  try { const data=await api("/api/goal/preview",{goal:{x,y,z,yaw:0}}); preview={x,y,z,yaw:0,valid:data.valid,reason:data.reason}; $("sendPreviewBtn").disabled=!data.valid; $("previewText").textContent=`X ${x.toFixed(2)}  Y ${y.toFixed(2)}  Z ${z.toFixed(2)} — ${data.reason}`; }
  catch(error){preview={x,y,z,yaw:0,valid:false,reason:error.message};$("sendPreviewBtn").disabled=true;$("previewText").textContent=`目标非法：${error.message}`;}
  drawMap();
}

async function addWaypoint(x,y) { const z=finite($("targetZNumber").value,1.5);try{await api("/api/goal/preview",{goal:{x,y,z,yaw:0}});const point=[x,y,z,0];point.valid=true;waypoints.push(point);log(`已添加巡航点 ${waypoints.length}`,"good");}catch(error){log(`航点非法：${error.message}`,"bad");}renderWaypoints();drawMap(); }
async function validateWaypointIndex(index){const p=waypoints[index];if(!p)return;try{await api("/api/goal/preview",{goal:p});p.valid=true;}catch(error){p.valid=false;log(`航点 ${index+1} 非法：${error.message}`,"bad");}renderWaypoints();drawMap();}
function deleteWaypoint(index) { if(index>=0){waypoints.splice(index,1);renderWaypoints();drawMap();} }
function renderWaypoints() { const list=$("waypointList"); $("waypointCount").textContent=String(waypoints.length); list.innerHTML=""; if(!waypoints.length){list.innerHTML='<div class="empty">尚未添加航点</div>';return;} waypoints.forEach((p,i)=>{const row=document.createElement("div");const active=i===Number(snapshot?.patrol?.waypoint_index);row.className=`waypoint-row ${active?"active":""} ${p.valid===false?"invalid":""}`;row.innerHTML=`<b>${i+1}</b><span>${p[0].toFixed(2)}, ${p[1].toFixed(2)}, ${p[2].toFixed(2)}</span><button title="删除">×</button>`;row.querySelector("button").onclick=()=>deleteWaypoint(i);list.appendChild(row);}); }

mapCanvas.addEventListener("mousedown",e=>{const r=mapCanvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;if(e.button===1||e.altKey){panning=true;lastMouse=[x,y];return;}dragIndex=nearestWaypoint(x,y);dragging=dragIndex>=0;dragOriginal=dragging?[...waypoints[dragIndex]]:null;});
mapCanvas.addEventListener("mousemove",e=>{const r=mapCanvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;if(panning&&lastMouse){const b=visibleBounds();panX-= (x-lastMouse[0])/r.width*(b.xMax-b.xMin);panY+= (y-lastMouse[1])/r.height*(b.yMax-b.yMin);lastMouse=[x,y];drawMap();return;}if(dragging&&dragIndex>=0){const p=unprojectTop(x,y,r.width,r.height);waypoints[dragIndex][0]=p[0];waypoints[dragIndex][1]=p[1];renderWaypoints();drawMap();}});
window.addEventListener("mouseup",()=>{const completedDrag=dragging?dragIndex:-1;dragging=false;panning=false;dragIndex=-1;lastMouse=null;dragOriginal=null;if(completedDrag>=0)validateWaypointIndex(completedDrag);});
mapCanvas.addEventListener("click",e=>{if(viewMode!=="top"||dragging)return;const r=mapCanvas.getBoundingClientRect(),p=unprojectTop(e.clientX-r.left,e.clientY-r.top,r.width,r.height);if(e.shiftKey||interactionMode==="waypoint")addWaypoint(...p);else previewAt(...p);});
mapCanvas.addEventListener("dblclick",async e=>{e.preventDefault();if(viewMode!=="top"||e.shiftKey||interactionMode!=="goal")return;const r=mapCanvas.getBoundingClientRect(),point=unprojectTop(e.clientX-r.left,e.clientY-r.top,r.width,r.height);await previewAt(...point);if(preview?.valid)await sendPreview();});
mapCanvas.addEventListener("contextmenu",e=>{e.preventDefault();const r=mapCanvas.getBoundingClientRect();deleteWaypoint(nearestWaypoint(e.clientX-r.left,e.clientY-r.top,24));});
mapCanvas.addEventListener("wheel",e=>{e.preventDefault();zoom=Math.max(.7,Math.min(4,zoom*(e.deltaY<0?1.12:.89)));drawMap();},{passive:false});

function drawLineChart(canvasId, series, yLabel="") {
  const canvas=$(canvasId), {width,height,ctx}=resizeCanvas(canvas);ctx.clearRect(0,0,width,height);ctx.fillStyle="#08131f";ctx.fillRect(0,0,width,height);const valid=series.filter(s=>s.values?.length);if(!valid.length){ctx.fillStyle="#526b87";ctx.font="11px Segoe UI";ctx.fillText("等待任务数据",12,22);return;}const n=Math.max(...valid.map(s=>s.values.length));let values=valid.flatMap(s=>s.values.filter(Number.isFinite));if(!values.length)return;let min=Math.min(...values),max=Math.max(...values);if(Math.abs(max-min)<1e-9){max+=1;min-=1;}const pad=25;ctx.strokeStyle="#203650";ctx.lineWidth=1;for(let i=0;i<4;i++){const y=pad+(height-2*pad)*i/3;ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(width-5,y);ctx.stroke();}valid.forEach(s=>{ctx.beginPath();s.values.forEach((v,i)=>{if(!Number.isFinite(v))return;const x=pad+(width-pad-7)*i/Math.max(1,n-1),y=height-pad-(v-min)/(max-min)*(height-2*pad);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.strokeStyle=s.color;ctx.lineWidth=1.5;ctx.stroke();});ctx.fillStyle="#748aa4";ctx.font="9px Segoe UI";ctx.fillText(max.toFixed(2),2,10);ctx.fillText(min.toFixed(2),2,height-7);ctx.fillText(yLabel,width-45,12);valid.forEach((s,i)=>{ctx.fillStyle=s.color;ctx.fillRect(pad+i*58,4,10,3);ctx.fillStyle="#9db0c6";ctx.fillText(s.label,pad+13+i*58,9);});
}

function drawCharts() { const rec=snapshot?.recording, rows=rec?.samples||[];const target=rec?.meta?.target || (rec?.meta?.waypoints||[]).slice(-1)[0];const errors=rows.map(r=>target&&Number.isFinite(r.x)?Math.hypot(r.x-target[0],r.y-target[1],r.z-target[2]):NaN);drawLineChart("positionChart",[{label:"x",color:"#38bdf8",values:rows.map(r=>r.x)},{label:"y",color:"#f59e0b",values:rows.map(r=>r.y)},{label:"z",color:"#22c55e",values:rows.map(r=>r.z)}],"m");drawLineChart("errorChart",[{label:"z",color:"#a78bfa",values:rows.map(r=>r.z)},{label:"error",color:"#ef4444",values:errors}],"m");drawLineChart("rpmChart",[1,2,3,4].map((i)=>({label:`M${i}`,color:["#38bdf8","#f59e0b","#22c55e","#f43f5e"][i-1],values:rows.map(r=>r[`rpm${i}`])})),"RPM");drawLineChart("progressChart",[{label:"clear",color:"#38bdf8",values:rows.map(r=>r.minimum_clearance)},{label:"progress",color:"#22c55e",values:rows.map(r=>r.patrol_progress)}],""); }

function updateUI(data) { snapshot=data;const c=data.connection||{},v=data.vehicle||{},p=data.planner||{},pat=data.patrol||{},m=data.metrics||{};setBadge($("rosBadge"),c.ros?"ROS CONNECTED":"ROS OFFLINE",c.ros?"good":"bad");setBadge($("simBadge"),c.simulation_running?"SIM RUNNING":"SIM STOPPED",c.simulation_running?"good":"neutral");setBadge($("plannerBadge"),`PLANNER ${p.state||"IDLE"}`,p.state==="FAILED"?"bad":p.state==="COMPLETED"?"good":"neutral");const patrolState=Number(pat.waypoint_count)>0?(pat.state||"IDLE"):"IDLE";setBadge($("patrolBadge"),`PATROL ${patrolState}`,patrolState==="FAILED"?"bad":patrolState==="COMPLETED"?"good":"neutral");setBadge($("waypointBadge"),`WP ${(finite(pat.waypoint_index)+1)||"--"}/${pat.waypoint_count||"--"}`,"neutral");$("duplicateWarning").classList.toggle("hidden",!c.duplicate);const blocked=c.duplicate||!c.ros;$("sendPreviewBtn").disabled=blocked||!preview?.valid;$("addPreviewWaypointBtn").disabled=blocked||!preview?.valid;$("startPatrolBtn").disabled=blocked||waypoints.length<2||waypoints.some(point=>point.valid===false);$("startPatrolBtn").title=waypoints.length<2?"至少添加两个合法巡航航点":"";$("startSimBtn").disabled=c.simulation_running;[["xValue",v.x,2," m"],["yValue",v.y,2," m"],["zValue",v.z,2," m"],["speedValue",v.speed,2," m/s"],["rollValue",finite(v.roll)*180/Math.PI,1,"°"],["pitchValue",finite(v.pitch)*180/Math.PI,1,"°"],["yawValue",finite(v.yaw)*180/Math.PI,1,"°"]].forEach(a=>$(a[0]).textContent=fmt(a[1],a[2],a[3]));for(let i=1;i<=4;i++)$(`rpm${i}Value`).textContent=fmt(v[`rpm${i}`],0);$("clearanceValue").textContent=fmt(m.minimum_clearance,3," m");$("planningTimeValue").textContent=fmt(m.planning_time,3," s");$("expandedValue").textContent=Number.isFinite(Number(m.expanded_nodes))?String(m.expanded_nodes):"--";$("plannedLengthValue").textContent=fmt(m.planned_path_length,3," m");$("actualLengthValue").textContent=fmt(m.actual_path_length,3," m");$("finalErrorValue").textContent=fmt(m.final_position_error,4," m");$("collisionValue").textContent=String(m.collision_count??"--");const progress=Math.max(0,Math.min(1,finite(pat.progress)));$("progressFill").style.width=`${progress*100}%`;$("lapValue").textContent=`Lap ${pat.lap||"--"}`;$("patrolElapsed").textContent=fmt(pat.elapsed_sec,1," s");renderWaypoints();drawMap();drawCharts(); }

async function sendPreview(){if(!preview?.valid)return;try{await api("/api/goal/send",{goal:preview});log(`目标已发送 (${preview.x.toFixed(2)}, ${preview.y.toFixed(2)}, ${preview.z.toFixed(2)})`,"good");}catch(e){log(e.message,"bad");}}
async function command(path,body={}){try{const data=await api(path,body);log(`${path}: success`,"good");return data;}catch(e){log(`${path}: ${e.message}`,"bad");throw e;}}

$("targetZ").oninput=e=>{$("targetZNumber").value=e.target.value;if(preview)previewAt(preview.x,preview.y);publishRvizAltitude()};$("targetZNumber").oninput=e=>{$("targetZ").value=e.target.value;if(preview)previewAt(preview.x,preview.y);publishRvizAltitude()};
$("goalModeBtn").onclick=()=>setInteractionMode("goal");$("waypointModeBtn").onclick=()=>setInteractionMode("waypoint");$("sendPreviewBtn").onclick=sendPreview;$("addPreviewWaypointBtn").onclick=()=>{if(preview?.valid)addWaypoint(preview.x,preview.y)};$("returnHomeBtn").onclick=()=>command("/api/return-home");$("holdBtn").onclick=()=>command("/api/hold");$("pauseBtn").onclick=()=>command("/api/patrol/pause");$("resumeBtn").onclick=()=>command("/api/patrol/resume");$("skipBtn").onclick=()=>command("/api/patrol/skip");$("stopPatrolBtn").onclick=()=>command("/api/patrol/stop");$("clearBtn").onclick=()=>{waypoints=[];renderWaypoints();drawMap();command("/api/clear")};
$("startPatrolBtn").onclick=async()=>{if(waypoints.length<2){log("巡航至少需要两个航点","bad");setInteractionMode("waypoint");return;}try{await command("/api/patrol/start",{waypoints,config:{mode:$("patrolMode").value,laps:Number($("patrolLaps").value),duration_sec:Number($("patrolDuration").value),dwell_sec:Number($("dwellSec").value),final_action:$("finalAction").value,home:[0,0,1.5,0]}});log(`巡航已启动：${waypoints.length} 个航点`,"good");}catch(e){setInteractionMode("waypoint")}};
$("startSimBtn").onclick=()=>command("/api/simulation/start",{map_file:$("mapSelect").value,rviz:false});$("stopSimBtn").onclick=()=>command("/api/simulation/stop");$("openRvizBtn").onclick=()=>command("/api/rviz/start");$("closeRvizBtn").onclick=()=>command("/api/rviz/stop");$("exportBtn").onclick=async()=>{const d=await command("/api/export");$("exportPath").textContent=d.directory};$("saveMapBtn").onclick=()=>{const a=document.createElement("a");a.download=`quadrotor_map_${Date.now()}.png`;a.href=mapCanvas.toDataURL("image/png");a.click()};$("clearLogBtn").onclick=()=>$("eventLog").innerHTML="";$("resetViewBtn").onclick=()=>{zoom=1;panX=0;panY=0;drawMap()};document.querySelectorAll("[data-view]").forEach(btn=>btn.onclick=()=>{viewMode=btn.dataset.view;document.querySelectorAll("[data-view]").forEach(b=>b.classList.toggle("active",b===btn));drawMap()});

async function loadMaps(){try{const maps=await api("/api/maps");$("mapSelect").innerHTML=maps.map(m=>`<option value="${m.path}" ${m.label.includes("Six")?"selected":""}>${m.label}</option>`).join("");}catch(e){log(`地图列表失败: ${e.message}`,"bad")}}
function connectSSE(){if(eventSource)eventSource.close();eventSource=new EventSource("/api/sse");eventSource.addEventListener("snapshot",e=>{try{updateUI(JSON.parse(e.data));}catch(err){log(`状态解析失败: ${err.message}`,"bad")}});eventSource.onopen=()=>log("实时数据流已连接","good");eventSource.onerror=()=>log("实时数据流断开，浏览器将自动重连","bad");}
window.addEventListener("resize",()=>{drawMap();drawCharts()});
loadMaps();connectSSE();renderWaypoints();setInteractionMode("goal");publishRvizAltitude();log("Web 地面站已加载");
