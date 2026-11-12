(function(){
"use strict";

/* =========================================================
   本文件所有场景参数与指标值，逐项取自方案文档，不做自拟：
   · 厂房 60m×40m×8m；12 组金属货架；4 台协作机器人；
     2 处堆垛区（纸箱 + 集装箱）；20 台 AGV @1.2m/s；
     无人叉车；无人机送急件
     ——《逐词大白话解析》仿真平台段
   · 4 个分布式接入点（pRRU）；智能超表面 12—16 块；
     每块 0.6m×0.6m、256 单元、1-bit 相位（PIN）
     ——《逐词大白话解析》部署清单表
   · 控制指令周期 5ms；视觉上行 4 路 1080p@30；仿真折算 72h
   · 五步管线：建图→传播计算→遮挡预测→相位决策→执行补盲
     （孪生 50ms 周期，预测提前 300—500ms，慢环 1Hz / 快环 20Hz）
   ========================================================= */
const SCENE={L:60,D:40,H:8};

const MAT={
  steel:    {name:'钢结构',    rho:0.92, loss:28, color:0xA39C90, edge:0x6F685D},
  rack:     {name:'金属货架',  rho:0.86, loss:26, color:0xE6DCC2, edge:0xB09668},
  arm:      {name:'协作机械臂',rho:0.78, loss:24, color:0xCFC7B7, edge:0x8C8375},
  forklift: {name:'无人叉车',  rho:0.82, loss:25, color:0xDFC98E, edge:0xA8853F},
  carton:   {name:'纸箱',      rho:0.08, loss:11, color:0xDCC9A4, edge:0xB2946A},
  container:{name:'集装箱',    rho:0.90, loss:30, color:0xADA79B, edge:0x756E63}
};

const STATE={agvCount:20, speed:1.2, forklifts:2, shadow:100,
             ris:12, risOn:true, showBeam:true, bs:4, predWin:400, predOn:true,
             sensing:true, slicing:'dynamic', showMat:true};
const PRESETS={
  baseline:{agvCount:20,speed:1.2,forklifts:2,shadow:100,ris:0, bs:4,predWin:400,predOn:false,sensing:false,slicing:'cloud'},
  optimal :{agvCount:20,speed:1.2,forklifts:2,shadow:100,ris:12,bs:4,predWin:400,predOn:true, sensing:true, slicing:'dynamic'}
};

/* ---------- 静态场景（米） ---------- */
const RACKS=[];
[3.0,7.0,14.0,18.0].forEach(z=>{ [4,18,32].forEach(x=> RACKS.push({x,z,w:12,d:1.2})); });
const COLUMNS=[];
[17,31].forEach(x=>{ [5.6,9.6,20.2].forEach(z=> COLUMNS.push({x,z,r:0.45})); });
const ARMS=[{x:10,z:29.5},{x:18.5,z:29.5},{x:27,z:29.5},{x:35.5,z:29.5}];
const STACKS=[
  {type:'carton',    x:45, z:25.0, w:5.0,  d:3.6},
  {type:'container', x:45, z:30.2, w:10.5, d:2.4}
];
const BS_POOL=[{x:3,z:2.5},{x:57,z:2.5},{x:3,z:37.5},{x:57,z:37.5},{x:30,z:12.0},{x:30,z:37.8}];
const RIS_POOL=[
  {x:10,z:8.9},{x:24,z:8.9},{x:38,z:8.9},
  {x:10,z:12.6},{x:24,z:12.6},{x:38,z:12.6},
  {x:10,z:16.8},{x:24,z:16.8},{x:38,z:16.8},
  {x:45.6,z:27.0},{x:45.6,z:32.0},
  {x:17,z:5.6},{x:31,z:5.6},
  {x:17,z:20.2},{x:31,z:20.2},
  {x:55,z:21.0}
];
const COL_SET=COLUMNS.map(c=>c.x+','+c.z);
RIS_POOL.forEach((p,i)=>{
  p.id=i+1; p.on=true; p.fix=0; p.tgt=null; p.eg=0; p.ax=0; p.az=0; p.an=0;
  p.onCol=COL_SET.indexOf(p.x+','+p.z)>=0;
});
const THRESH=-85;

/* ---------- 遮挡栅格 ---------- */
const CELL=0.5;
const GW=Math.ceil(SCENE.L/CELL), GH=Math.ceil(SCENE.D/CELL);
const S_MASK=new Int16Array(GW*GH).fill(-1);
const D_MASK=new Int16Array(GW*GH).fill(-1);
const STATIC=[];
RACKS.forEach(r=> STATIC.push({mat:MAT.rack, r}));
COLUMNS.forEach(c=> STATIC.push({mat:MAT.steel, r:{x:c.x-c.r,z:c.z-c.r,w:c.r*2,d:c.r*2}}));
STACKS.forEach(s=> STATIC.push({mat: s.type==='carton'?MAT.carton:MAT.container, r:s}));
STATIC.forEach((o,i)=>o.id=i);
function fillMask(mask,obj){
  const r=obj.r;
  const x0=Math.max(0,Math.floor(r.x/CELL)), x1=Math.min(GW-1,Math.floor((r.x+r.w)/CELL));
  const z0=Math.max(0,Math.floor(r.z/CELL)), z1=Math.min(GH-1,Math.floor((r.z+r.d)/CELL));
  for(let z=z0;z<=z1;z++) for(let x=x0;x<=x1;x++) mask[z*GW+x]=obj.id;
}
STATIC.forEach(o=>fillMask(S_MASK,o));
const DYN=[];

/* ---------- 运动对象 ---------- */
const LOOP_M=[{x:2.5,z:21.5},{x:57.5,z:21.5},{x:57.5,z:38.2},{x:2.5,z:38.2}];
const SEGS=(function(){
  const s=[]; let total=0;
  for(let i=0;i<LOOP_M.length;i++){
    const a=LOOP_M[i], b=LOOP_M[(i+1)%LOOP_M.length];
    const len=Math.hypot(b.x-a.x,b.z-a.z);
    s.push({a,b,len,start:total}); total+=len;
  }
  return {s,total};
})();
function loopPoint(d){
  const t=((d%SEGS.total)+SEGS.total)%SEGS.total;
  for(const g of SEGS.s){
    if(t<=g.start+g.len){
      const k=(t-g.start)/g.len;
      return {x:g.a.x+(g.b.x-g.a.x)*k, z:g.a.z+(g.b.z-g.a.z)*k,
              ang:Math.atan2(g.b.z-g.a.z,g.b.x-g.a.x)};
    }
  }
  return {x:LOOP_M[0].x,z:LOOP_M[0].z,ang:0};
}

const AGVS=[], FORKLIFTS=[];
function buildAGVs(){
  AGVS.length=0;
  const n=STATE.agvCount;
  for(let i=0;i<n;i++){
    AGVS.push({id:i+1, lane:(i%6===5)?'mid':'loop', s:(SEGS.total/n)*i,
      x0:4+((i*97)%52), mdir:i%2?1:-1,
      x:0,z:0,ang:0, rssi:-70, futLost:false, warned:false,
      halt:0, lost:false, wasLost:false, _st:''});
  }
}
function buildForklifts(){
  FORKLIFTS.length=0;
  const lanesZ=[11.5,16.6,12.4,15.8];
  for(let i=0;i<STATE.forklifts;i++){
    FORKLIFTS.push({id:i+1, z:lanesZ[i%4], x:6+i*7,
      dir:i%2?1:-1, ang:i%2?Math.PI:0, rssi:-70, halt:0, lost:false, wasLost:false, _st:''});
  }
}
buildAGVs(); buildForklifts();

/* ---------- 信号：材质感知 ---------- */
const activeBS=()=>BS_POOL.slice(0,STATE.bs);
const activeRIS=()=>RIS_POOL.slice(0,STATE.ris);

function pathLoss(x1,z1,x2,z2){
  const steps=20;
  let lastS=-1,lastD=-1,loss=0,rho=0,layers=0;
  const kShadow=0.50+STATE.shadow*0.0042;
  for(let i=1;i<steps;i++){
    const k=i/steps;
    const px=x1+(x2-x1)*k, pz=z1+(z2-z1)*k;
    const gx=px/CELL|0, gz=pz/CELL|0;
    if(gx<0||gx>=GW||gz<0||gz>=GH) continue;
    const idx=gz*GW+gx;
    const s=S_MASK[idx];
    if(s>=0 && s!==lastS){
      if(++layers>2) break;
      loss+=STATIC[s].mat.loss*kShadow; rho=STATIC[s].mat.rho; lastS=s;
    }
    const d=D_MASK[idx];
    if(d>=0 && d!==lastD){
      if(++layers>2) break;
      loss+=DYN[d].mat.loss*kShadow; rho=DYN[d].mat.rho; lastD=d;
    }
  }
  return {loss,rho,layers};
}

/* RIS 阵列：面板 0.6m×0.6m、256 单元、1-bit 相位（文档部署清单）
   补盲增益上限取文档仿真值 +12.4dB，不做放大 */
const RIS_R=15, RIS_MAXD=17, RIS_MAXG=8.0, RIS_CAP=12.4;
const RIS_H=3.6;   // 面板挂高 m（柱装 6m）——贴装于立柱/货架侧板/天花板

function baseRSSI(x,z){
  let best=-999;
  for(const b of activeBS()){
    const d=Math.max(Math.hypot(x-b.x,z-b.z),0.6);
    const fspl=-30-22*Math.log10(d);
    const pl=pathLoss(b.x,b.z,x,z);
    const refl = pl.layers>0 ? pl.rho*3.2 : 0;
    const v=fspl-pl.loss+refl;
    if(v>best) best=v;
  }
  return best;
}

function risGain(x,z,pick){
  if(!STATE.risOn) return 0;
  let g1=0,g2=0,g3=0,p1=null;
  for(const p of activeRIS()){
    if(!p.on) continue;
    const d=Math.hypot(x-p.x,z-p.z);
    if(d>=RIS_R) continue;
    const g=RIS_MAXG*(1-d/RIS_MAXD);
    if(g>g1){ g3=g2; g2=g1; g1=g; p1=p; }
    else if(g>g2){ g3=g2; g2=g; }
    else if(g>g3){ g3=g; }
  }
  if(pick) pick.p=p1;
  return Math.min(g1+g2*0.34+g3*0.34, RIS_CAP);
}

function computeRSSI(x,z){ return baseRSSI(x,z)+risGain(x,z); }

/* ---------- 热力图（只算车间内部 60×40） ---------- */
const HC=1.5;
const HW=Math.ceil(SCENE.L/HC), HH=Math.ceil(SCENE.D/HC);
const HEAT=new Float32Array(HW*HH);
const INFO={blind:0,blindNo:0,avg:-75};
let heatDirty=true, heatAt=0;

function rebuildHeat(){
  let blind=0,tot=0,sum=0,blindNo=0;
  const AC=HC*HC;
  const live=(STATE.risOn? activeRIS().filter(p=>p.on) : []);
  live.forEach(p=>{p.fix=0;p.ax=0;p.az=0;p.an=0;});
  const pick={p:null};
  for(let gz=0;gz<HH;gz++)for(let gx=0;gx<HW;gx++){
    const x=(gx+.5)*HC, z=(gz+.5)*HC;
    const base=baseRSSI(x,z);
    pick.p=null;
    const comp=risGain(x,z,pick);
    let v=base+comp;
    const inObj = S_MASK[(z/CELL|0)*GW+(x/CELL|0)]>=0;
    if(inObj) v-=6;
    HEAT[gz*HW+gx]=v; sum+=v;
    if(!inObj){
      tot++;
      if(v<THRESH) blind++;
      if(base<THRESH) blindNo++;
      if(base<THRESH && v>=THRESH && pick.p){
        pick.p.fix+=AC; pick.p.ax+=x; pick.p.az+=z; pick.p.an++;
      }
    }
  }
  INFO.blind= tot? blind/tot : 0;
  INFO.blindNo= tot? blindNo/tot : 0;
  INFO.avg=sum/(HW*HH);
  live.forEach(p=>{ p.tgt = p.an? {x:p.ax/p.an, z:p.az/p.an} : null; });
  heatDirty=false; heatAt=performance.now();
  refreshHeatTexture();
}

/* =========================================================
   KPI —— 全部取自方案文档，基线/方案双锚点线性标定
   来源：《逐词大白话解析》仿真对照表 + 八项指标表
   基线 = 5G-A 现状；方案 = 6G 联合优化；target = 文档目标值
   ========================================================= */
const DOC={
  lat  :{base:96,  opt:18.6, target:20,  unit:'ms',   d:1, note:'端到端，含无线与推理'},
  ints :{base:3.4, opt:0.06, target:0.1, unit:'次/h', d:2, note:'连续100ms无业务包'},
  blind:{base:8.0, opt:1.9,  target:2,   unit:'%',    d:1, note:'单站盲区面积占比 8%—15%'},
  pos  :{base:42,  opt:7.8,  target:10,  unit:'cm',   d:1, note:'非视距场景'},
  tops :{base:40,  opt:500,  target:500, unit:'TOPS', d:0, note:'单机可用等效算力 20—40→≥500'},
  bom  :{base:100, opt:61,   target:60,  unit:'%',    d:0, note:'相对全端侧部署，1.8万→1.1万'},
  recfg:{base:240, opt:10,   target:10,  unit:'min',  d:0, note:'换线 4 小时→10 分钟'},
  ul   :{base:60,  opt:300,  target:300, unit:'Mbps', d:0, note:'单机上行 60—100→≥300'},
  thr  :{base:38,  opt:210,  unit:'Mbps', d:0, note:'遮挡区平均吞吐 5.5 倍'},
  gain :{base:0,   opt:12.4, unit:'dB',   d:1, note:'遮挡区接收功率 −96.2→−83.8dBm'}
};
const KPI={blind:0,ints:0,lat:0,thr:0,succ:0,pos:0,tops:0,bom:0,recfg:0,ul:0,gain:0,online:0,base:null};

/* 优化度：三项措施的达成比例（方案态=1，5G-A 基线态=0） */
function optDeg(){
  const risF  = STATE.risOn? Math.min(STATE.ris/12,1) : 0;
  const predF = STATE.predOn? Math.max(0,Math.min(1,(STATE.predWin-300)/100)) : 0;
  const senseF= STATE.sensing?1:0;
  const sliceF= STATE.slicing==='dynamic'?1:0;
  return {risF,predF,senseF,sliceF,
          q:0.45*risF+0.25*predF+0.15*senseF+0.15*sliceF};
}
const mix=(k,lift)=>DOC[k].base+(DOC[k].opt-DOC[k].base)*Math.max(0,Math.min(1,lift));

function computeKPI(instant){
  const {risF,predF,senseF,sliceF,q}=optDeg();
  const blind =mix('blind',risF);
  const lat   =mix('lat',q);
  const ints  =mix('ints',0.35*risF+0.45*predF+0.20*sliceF);
  const pos   =mix('pos',0.50*risF+0.50*senseF);
  const thr   =mix('thr',risF);
  const ul    =mix('ul',q);
  const bom   =mix('bom',q);
  const recfg =mix('recfg',q);
  const gain  =DOC.gain.opt*risF;
  // 任务成功率：文档三档 —— 纯端 76%、纯云 82%、方案 96.3%
  const succ  = STATE.slicing==='dynamic' ? 76+(96.3-76)*q
              : (STATE.slicing==='end' ? 76 : 82);
  // 单机可用等效算力：端侧 SoC 40TOPS / 纯云无端侧算力 20TOPS / 方案 500TOPS
  const tops  = STATE.slicing==='dynamic' ? 500
              : (STATE.slicing==='end' ? 40 : 20);
  const online=AGVS.filter(a=>!a.lost).length+FORKLIFTS.filter(f=>!f.lost).length;
  const set=(k,v)=>{ KPI[k]=v; };
  if(instant){
    set('blind',blind); set('lat',lat); set('ints',ints); set('pos',pos);
    set('thr',thr); set('ul',ul); set('bom',bom); set('recfg',recfg);
    set('gain',gain); set('succ',succ); set('tops',tops); set('online',online);
    return;
  }
  const k=0.10;
  ['blind','lat','ints','pos','thr','ul','bom','recfg','gain','succ'].forEach(n=>{
    const target=n==='blind'?blind:n==='lat'?lat:n==='ints'?ints:n==='pos'?pos:
                 n==='thr'?thr:n==='ul'?ul:n==='bom'?bom:n==='recfg'?recfg:
                 n==='gain'?gain:succ;
    KPI[n]+=(target-KPI[n])*k;
  });
  KPI.tops=tops; KPI.online=online;
}
const snapshot=()=>Object.assign({},KPI);

/* =========================================================
   THREE.JS 3D 场景
   ========================================================= */
const cv=document.getElementById('cv');
const renderer=new THREE.WebGLRenderer({canvas:cv,antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,3));
renderer.outputEncoding=THREE.sRGBEncoding;
const scene=new THREE.Scene();
scene.background=new THREE.Color(0xF2EEDF);
scene.fog=new THREE.Fog(0xF2EEDF,150,320);
const camera=new THREE.PerspectiveCamera(48,1,0.1,600);
camera.position.set(38,40,62);
const controls=new THREE.OrbitControls(camera,renderer.domElement);
controls.target.set(30,0,19);
controls.enableDamping=true; controls.dampingFactor=0.08;
controls.maxPolarAngle=Math.PI/2-0.05;
controls.minDistance=10; controls.maxDistance=170;

function resize(){
  const r=cv.parentElement.getBoundingClientRect();
  const w=Math.max(r.width,10), h=Math.max(r.height,10);
  renderer.setSize(w,h,false);
  camera.aspect=w/h; camera.updateProjectionMatrix();
}
window.addEventListener('resize',resize);

scene.add(new THREE.HemisphereLight(0xFFF6E0,0xC4B48E,0.95));
const sun=new THREE.DirectionalLight(0xFFF3DC,0.75);
sun.position.set(18,55,30); scene.add(sun);
const fill=new THREE.DirectionalLight(0xE8D8B6,0.28);
fill.position.set(-30,35,-25); scene.add(fill);

const std=(c,rough=0.85,metal=0.12)=>new THREE.MeshStandardMaterial({color:c,roughness:rough,metalness:metal});
const GOLD=0xA8853F, GOLDL=0xD9BE84, RED=0xB03A2E, AMBER=0xD9A441, GREEN=0x4E7A5E;

/* ---- 地面 + 信号热力层 ---- */
const floorM=new THREE.Mesh(new THREE.PlaneGeometry(SCENE.L,SCENE.D),std(0xFAF7EC,0.95,0.02));
floorM.rotation.x=-Math.PI/2; floorM.position.set(SCENE.L/2,0,SCENE.D/2); scene.add(floorM);

const heatSrc=document.createElement('canvas'); heatSrc.width=HW; heatSrc.height=HH;
const heatDst=document.createElement('canvas'); heatDst.width=HW*8; heatDst.height=HH*8;
const heatTex=new THREE.CanvasTexture(heatDst);
function refreshHeatTexture(){
  const c=heatSrc.getContext('2d');
  const img=c.createImageData(HW,HH);
  for(let i=0;i<HW*HH;i++){
    const v=HEAT[i];
    const t=Math.max(0,Math.min(1,(v-THRESH)/14));
    img.data[i*4]  =Math.round(176+(238-176)*t);
    img.data[i*4+1]=Math.round(58+(226-58)*t);
    img.data[i*4+2]=Math.round(46+(196-46)*t);
    img.data[i*4+3]=Math.round(150-110*t);
  }
  c.putImageData(img,0,0);
  const d=heatDst.getContext('2d');
  d.clearRect(0,0,heatDst.width,heatDst.height);
  d.filter='blur(9px)';
  d.drawImage(heatSrc,0,0,heatDst.width,heatDst.height);
  d.filter='none';
  heatTex.needsUpdate=true;
}
const heatPlane=new THREE.Mesh(new THREE.PlaneGeometry(SCENE.L,SCENE.D),
  new THREE.MeshBasicMaterial({map:heatTex,transparent:true,opacity:0.9,depthWrite:false}));
heatPlane.rotation.x=-Math.PI/2; heatPlane.position.set(SCENE.L/2,0.06,SCENE.D/2); scene.add(heatPlane);

/* ---- 网格 / 墙体 / 高度包络 ---- */
(function(){
  const g=new THREE.Group();
  const mat=new THREE.LineBasicMaterial({color:0xC0A469,transparent:true,opacity:0.16});
  const pts=[];
  for(let m=0;m<=SCENE.L;m+=5){ pts.push(m,0,0, m,0,SCENE.D); }
  for(let m=0;m<=SCENE.D;m+=5){ pts.push(0,0,m, SCENE.L,0,m); }
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.Float32BufferAttribute(pts,3));
  g.add(new THREE.LineSegments(geo,mat));
  // 8m 高度包络
  const env=new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(SCENE.L,SCENE.H,SCENE.D)),
    new THREE.LineDashedMaterial({color:0xA8853F,transparent:true,opacity:0.28,dashSize:1.4,gapSize:1}));
  env.computeLineDistances();
  env.position.set(SCENE.L/2,SCENE.H/2,SCENE.D/2);
  g.add(env);
  // 低矮玻璃墙
  const wallM=new THREE.MeshStandardMaterial({color:0xEFE8D2,transparent:true,opacity:0.16,roughness:0.4,metalness:0});
  [[SCENE.L/2,0.7,0,SCENE.L,1.4,0.25],[SCENE.L/2,0.7,SCENE.D,SCENE.L,1.4,0.25],
   [0,0.7,SCENE.D/2,0.25,1.4,SCENE.D],[SCENE.L,0.7,SCENE.D/2,0.25,1.4,SCENE.D]]
  .forEach(([x,y,z,w,h,d])=>{
    const m=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),wallM);
    m.position.set(x,y,z); g.add(m);
  });
  scene.add(g);
})();

/* ---- 标签 sprite 工厂 ---- */
function makeLabel(text,opts){
  opts=opts||{};
  const fs=opts.fs||26, pad=opts.pad||12;
  const chip=opts.chip||null;
  const c=document.createElement('canvas');
  let x=c.getContext('2d');
  x.font='600 '+fs+'px "PingFang SC","Microsoft YaHei",sans-serif';
  const tw=x.measureText(text).width;
  const cw=Math.ceil(tw+pad*2+(chip?fs+10:0)), chh=Math.ceil(fs*1.5);
  c.width=cw; c.height=chh;
  x=c.getContext('2d');
  x.font='600 '+fs+'px "PingFang SC","Microsoft YaHei",sans-serif';
  x.fillStyle=opts.bg||'rgba(255,253,245,0.95)';
  x.strokeStyle=opts.bd||'rgba(192,164,105,0.55)';
  x.lineWidth=2;
  const r=10;
  x.beginPath();
  x.moveTo(r,1); x.lineTo(cw-r,1); x.quadraticCurveTo(cw-1,1,cw-1,r);
  x.lineTo(cw-1,chh-r); x.quadraticCurveTo(cw-1,chh-1,cw-r,chh-1);
  x.lineTo(r,chh-1); x.quadraticCurveTo(1,chh-1,1,chh-r);
  x.lineTo(1,r); x.quadraticCurveTo(1,1,r,1);
  x.fill(); x.stroke();
  let tx=pad;
  if(chip){
    x.fillStyle=chip; x.strokeStyle='rgba(43,38,32,0.35)'; x.lineWidth=1.5;
    x.fillRect(pad, (chh-fs)/2, fs*0.7, fs*0.7);
    x.strokeRect(pad, (chh-fs)/2, fs*0.7, fs*0.7);
    tx=pad+fs*0.7+10;
  }
  x.fillStyle=opts.fg||'#2B2620';
  x.textBaseline='middle';
  x.fillText(text,tx,chh/2+1);
  const tex=new THREE.CanvasTexture(c);
  tex.minFilter=THREE.LinearFilter;
  const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,transparent:true,depthTest:false}));
  const h=opts.h||1.1;
  sp.scale.set(h*cw/chh,h,1);
  sp.renderOrder=10;
  return sp;
}

/* ---- 货架（12 组） ---- */
RACKS.forEach((r,ri)=>{
  const g=new THREE.Group();
  const upM=std(MAT.rack.edge,0.6,0.5);
  [[0,0],[12,0],[0,1.2],[12,1.2],[4,0],[8,0],[4,1.2],[8,1.2]].forEach(([ox,oz])=>{
    const u=new THREE.Mesh(new THREE.BoxGeometry(0.12,4,0.12),upM);
    u.position.set(ox,2,oz); g.add(u);
  });
  const shM=std(MAT.rack.color,0.8,0.3);
  [1.0,2.15,3.3].forEach((y,si)=>{
    const sh=new THREE.Mesh(new THREE.BoxGeometry(12,0.08,1.2),shM);
    sh.position.set(6,y,0.6); g.add(sh);
    for(let k=0;k<5;k++){
      if((ri+si+k)%3===0) continue;
      const cb=new THREE.Mesh(new THREE.BoxGeometry(1.7,0.62,0.86),std(MAT.carton.color,0.92,0.02));
      cb.position.set(1.4+k*2.2+(k%2?0.25:0),y+0.36,0.6);
      g.add(cb);
    }
  });
  g.position.set(r.x,0,r.z);
  scene.add(g);
});

/* ---- 钢结构立柱 ---- */
COLUMNS.forEach(c=>{
  const m=new THREE.Mesh(new THREE.CylinderGeometry(c.r,c.r,SCENE.H,18),std(MAT.steel.color,0.55,0.6));
  m.position.set(c.x,SCENE.H/2,c.z); scene.add(m);
});

/* ---- 堆垛区：纸箱 + 集装箱 ---- */
STACKS.forEach(s=>{
  const g=new THREE.Group();
  if(s.type==='carton'){
    const cw=s.w/4, cd=s.d/3;
    for(let r=0;r<3;r++)for(let col=0;col<4;col++){
      const h=0.9+((r*4+col)%3)*0.4;
      const b=new THREE.Mesh(new THREE.BoxGeometry(cw-0.35,h,cd-0.35),std(MAT.carton.color,0.92,0.02));
      b.position.set(col*cw+cw/2,h/2,r*cd+cd/2);
      g.add(b);
      if((r+col)%2===0){
        const b2=new THREE.Mesh(new THREE.BoxGeometry(cw-0.5,0.55,cd-0.5),std(0xD2BC92,0.92,0.02));
        b2.position.set(col*cw+cw/2,h+0.28,r*cd+cd/2);
        g.add(b2);
      }
    }
  }else{
    const box=new THREE.Mesh(new THREE.BoxGeometry(s.w,2.6,s.d),std(MAT.container.color,0.6,0.55));
    box.position.set(s.w/2,1.3,s.d/2); g.add(box);
    const ribM=std(MAT.container.edge,0.6,0.55);
    for(let k=1;k<14;k++){
      const rib=new THREE.Mesh(new THREE.BoxGeometry(0.1,2.4,s.d+0.06),ribM);
      rib.position.set(k*s.w/14,1.3,s.d/2); g.add(rib);
    }
  }
  g.position.set(s.x,0,s.z);
  scene.add(g);
});

/* ---- 协作机械臂（4 台，实时摆动） ---- */
const armJoints=[];
ARMS.forEach((a,i)=>{
  const g=new THREE.Group();
  const base=new THREE.Mesh(new THREE.CylinderGeometry(0.55,0.65,0.6,20),std(MAT.arm.edge,0.5,0.5));
  base.position.y=0.3; g.add(base);
  const pivot=new THREE.Group(); pivot.position.y=0.6; g.add(pivot);
  const l1=new THREE.Mesh(new THREE.BoxGeometry(1.7,0.24,0.32),std(MAT.arm.color,0.5,0.45));
  l1.position.set(0.85,0,0); pivot.add(l1);
  const elbow=new THREE.Group(); elbow.position.set(1.7,0,0); pivot.add(elbow);
  const l2=new THREE.Mesh(new THREE.BoxGeometry(1.35,0.2,0.26),std(MAT.arm.color,0.5,0.45));
  l2.position.set(0.675,0,0); elbow.add(l2);
  const wrist=new THREE.Mesh(new THREE.SphereGeometry(0.14,14,12),std(GOLD,0.4,0.6));
  wrist.position.set(1.35,0,0); elbow.add(wrist);
  const ring=new THREE.Mesh(new THREE.RingGeometry(2.72,2.9,48),
    new THREE.MeshBasicMaterial({color:MAT.arm.edge,transparent:true,opacity:0.18,side:THREE.DoubleSide}));
  ring.rotation.x=-Math.PI/2; ring.position.y=0.05; g.add(ring);
  g.position.set(a.x,0,a.z);
  scene.add(g);
  armJoints.push({pivot,elbow,i});
});

/* ---- 6G 基站 ---- */
const bsGroups=[];
BS_POOL.forEach((b,i)=>{
  const g=new THREE.Group();
  const pole=new THREE.Mesh(new THREE.CylinderGeometry(0.09,0.14,SCENE.H,10),std(0x8C8375,0.5,0.6));
  pole.position.y=SCENE.H/2; g.add(pole);
  const head=new THREE.Mesh(new THREE.BoxGeometry(0.85,0.5,0.28),std(GOLD,0.45,0.55));
  head.position.y=SCENE.H-0.5; g.add(head);
  const lb=makeLabel('gNB-'+(i+1),{fs:22,h:0.85});
  lb.position.set(0,SCENE.H+0.7,0); g.add(lb);
  const rings=[];
  for(let k=0;k<3;k++){
    const ring=new THREE.Mesh(new THREE.RingGeometry(0.46,0.62,40),
      new THREE.MeshBasicMaterial({color:GOLD,transparent:true,opacity:0.3,side:THREE.DoubleSide,depthWrite:false}));
    ring.rotation.x=-Math.PI/2; ring.position.y=0.1; g.add(ring);
    rings.push(ring);
  }
  g.position.set(b.x,0,b.z);
  scene.add(g);
  bsGroups.push({g,rings});
});

/* ---- RIS 智能表面面板（0.6m×0.6m，16×16 = 256 单元，1-bit 相位） ---- */
const risVis=[];
const panelGeo=new THREE.BoxGeometry(0.6,0.6,0.05);
const panelTex=(function(){
  const c=document.createElement('canvas'); c.width=c.height=256;
  const x=c.getContext('2d');
  x.fillStyle='#F2E6C8'; x.fillRect(0,0,256,256);
  x.strokeStyle='rgba(168,133,63,0.5)'; x.lineWidth=1;
  for(let i=0;i<=16;i++){
    const p=i*16+0.5;
    x.beginPath(); x.moveTo(p,0); x.lineTo(p,256); x.stroke();
    x.beginPath(); x.moveTo(0,p); x.lineTo(256,p); x.stroke();
  }
  const t=new THREE.CanvasTexture(c);
  t.minFilter=THREE.LinearFilter;
  return t;
})();
RIS_POOL.forEach(p=>{
  const g=new THREE.Group();
  const mat=new THREE.MeshStandardMaterial({color:0xF2E6C8,roughness:0.4,metalness:0.35,map:panelTex});
  const panel=new THREE.Mesh(panelGeo,mat);
  const h=p.onCol?6.0:RIS_H;
  panel.position.y=h;
  panel.rotation.x=-0.14;
  panel.userData.ris=p;
  const edge=new THREE.LineSegments(new THREE.EdgesGeometry(panelGeo),
    new THREE.LineBasicMaterial({color:GOLD}));
  edge.position.y=h; edge.rotation.x=-0.14;
  if(!p.onCol){
    const mast=new THREE.Mesh(new THREE.CylinderGeometry(0.05,0.05,h,8),std(0x8C8375,0.55,0.5));
    mast.position.y=h/2; g.add(mast);
  }
  const glow=new THREE.Mesh(new THREE.SphereGeometry(1.3,12,10),
    new THREE.MeshBasicMaterial({color:0xD4B060,transparent:true,opacity:0,depthWrite:false}));
  glow.position.y=h;
  g.add(panel); g.add(edge); g.add(glow);

  // 波束元素
  const inLine=new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),
    new THREE.LineBasicMaterial({color:GOLD,transparent:true,opacity:0.30}));
  const reflLine=new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),
    new THREE.LineBasicMaterial({color:0xC8A860,transparent:true,opacity:0.8,blending:THREE.AdditiveBlending}));
  const dots=[]; const rdots=[];
  for(let k=0;k<2;k++){
    const d=new THREE.Mesh(new THREE.SphereGeometry(0.11,8,8),
      new THREE.MeshBasicMaterial({color:GOLDL,transparent:true,opacity:0.85}));
    d.visible=false; scene.add(d); dots.push(d);
  }
  for(let k=0;k<3;k++){
    const d=new THREE.Mesh(new THREE.SphereGeometry(0.16,8,8),
      new THREE.MeshBasicMaterial({color:0xDBB664,transparent:true,blending:THREE.AdditiveBlending}));
    d.visible=false; scene.add(d); rdots.push(d);
  }
  const sector=new THREE.Group();
  const secMesh=new THREE.Mesh(new THREE.CircleGeometry(1,26,-0.40,0.80),
    new THREE.MeshBasicMaterial({color:0xD4B060,transparent:true,opacity:0.14,side:THREE.DoubleSide,depthWrite:false,blending:THREE.AdditiveBlending}));
  secMesh.rotation.x=-Math.PI/2;
  sector.add(secMesh);
  scene.add(sector);
  const hitRing=new THREE.Mesh(new THREE.RingGeometry(0.85,1.0,36),
    new THREE.MeshBasicMaterial({color:GOLDL,transparent:true,opacity:0,side:THREE.DoubleSide,depthWrite:false}));
  hitRing.rotation.x=-Math.PI/2; hitRing.position.y=0.12;
  scene.add(hitRing);
  const tag=makeLabel('',{fs:24,h:0.95,fg:'#E4D3AC',bg:'rgba(43,38,32,0.88)',bd:'rgba(43,38,32,0)'});
  tag.visible=false;
  scene.add(tag);
  scene.add(inLine); scene.add(reflLine);
  g.position.set(p.x,0,p.z);
  scene.add(g);
  risVis.push({p,g,panel,mat,edge,glow,inLine,reflLine,dots,rdots,sector,secMesh,hitRing,tag,h});
});

/* ---- 材质标注（3DGS 建图成果） ---- */
const matTags=new THREE.Group(); scene.add(matTags);
function addMatTag(key,x,y,z){
  const m=MAT[key];
  const sp=makeLabel(m.name+'  ρ='+m.rho.toFixed(2),{fs:24,h:1.0,chip:'#'+m.color.toString(16).padStart(6,'0')});
  sp.position.set(x,y,z);
  matTags.add(sp);
  return sp;
}
addMatTag('rack',    16.8,3.6,3.6);
addMatTag('steel',   17,  8.8, 5.6);
addMatTag('carton',  50.8,3.4,26.3);
addMatTag('container',55.8,3.6,31.4);
addMatTag('arm',     10,  4.6, 29.5);
const flTag=addMatTag('forklift',0,3.2,0);   // 叉车标签跟随

/* ---- AGV / 叉车 / 无人机 ---- */
function makeAGVMesh(){
  const g=new THREE.Group();
  const body=new THREE.Mesh(new THREE.BoxGeometry(1.7,0.34,0.95),std(GOLDL,0.6,0.3));
  body.position.y=0.26; g.add(body);
  const top=new THREE.Mesh(new THREE.BoxGeometry(0.7,0.16,0.5),std(0x8F6E2E,0.5,0.4));
  top.position.y=0.5; g.add(top);
  const led=new THREE.Mesh(new THREE.SphereGeometry(0.13,10,8),new THREE.MeshBasicMaterial({color:GOLD}));
  led.position.y=0.68; g.add(led);
  const trail=new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(1,0,0)]),
    new THREE.LineDashedMaterial({color:GOLD,transparent:true,opacity:0.55,dashSize:0.5,gapSize:0.4}));
  trail.computeLineDistances();
  scene.add(trail);
  scene.add(g);
  return {g,led,trail};
}
function makeForkliftMesh(){
  const g=new THREE.Group();
  const body=new THREE.Mesh(new THREE.BoxGeometry(1.2,1.0,1.15),std(MAT.forklift.color,0.6,0.3));
  body.position.set(-0.35,0.55,0); g.add(body);
  const cab=new THREE.Mesh(new THREE.BoxGeometry(0.6,0.5,0.9),std(0x8F6E2E,0.55,0.35));
  cab.position.set(-0.5,1.3,0); g.add(cab);
  const mast=new THREE.Mesh(new THREE.BoxGeometry(0.1,1.6,1.0),std(0x8C8375,0.5,0.5));
  mast.position.set(0.55,0.8,0); g.add(mast);
  [-0.35,0.35].forEach(z=>{
    const fk=new THREE.Mesh(new THREE.BoxGeometry(0.7,0.07,0.16),std(0x6F685D,0.5,0.55));
    fk.position.set(0.95,0.18,z); g.add(fk);
  });
  const load=new THREE.Mesh(new THREE.BoxGeometry(0.62,0.62,0.95),std(MAT.carton.color,0.92,0.02));
  load.position.set(0.62,0.55,0); g.add(load);
  const led=new THREE.Mesh(new THREE.SphereGeometry(0.12,10,8),new THREE.MeshBasicMaterial({color:GOLD}));
  led.position.set(-0.5,1.75,0); g.add(led);
  scene.add(g);
  return {g,led};
}
function makeDrone(){
  const g=new THREE.Group();
  const body=new THREE.Mesh(new THREE.BoxGeometry(0.7,0.22,0.7),std(0x8F6E2E,0.5,0.4));
  g.add(body);
  const rotors=[];
  [[0.42,0.42],[0.42,-0.42],[-0.42,0.42],[-0.42,-0.42]].forEach(([ox,oz])=>{
    const arm=new THREE.Mesh(new THREE.BoxGeometry(0.5,0.04,0.06),std(0x6F685D,0.5,0.5));
    arm.position.set(ox*0.7,0.05,oz*0.7); arm.lookAt(0,0.05,0); g.add(arm);
    const r=new THREE.Mesh(new THREE.CylinderGeometry(0.26,0.26,0.02,16),std(GOLDL,0.4,0.3));
    r.position.set(ox,0.1,oz); g.add(r); rotors.push(r);
  });
  const led=new THREE.Mesh(new THREE.SphereGeometry(0.09,8,8),new THREE.MeshBasicMaterial({color:GOLD}));
  led.position.y=-0.2; g.add(led);
  g.position.set(6,5,21.5);
  scene.add(g);
  return {g,rotors,led};
}

let agvMeshes=[], flMeshes=[];
function rebuildAgvMeshes(){
  agvMeshes.forEach(m=>{scene.remove(m.g);scene.remove(m.trail);});
  flMeshes.forEach(m=>scene.remove(m.g));
  agvMeshes=AGVS.map(()=>makeAGVMesh());
  flMeshes=FORKLIFTS.map(()=>makeForkliftMesh());
}
rebuildAgvMeshes();
const drone=makeDrone();

/* =========================================================
   更新逻辑
   ========================================================= */
let T=0;
function rebuildDynMask(){
  D_MASK.fill(-1);
  DYN.length=0;
  FORKLIFTS.forEach((f,i)=>{
    DYN.push({mat:MAT.forklift,id:i});
    fillMask(D_MASK,{id:i,r:{x:f.x-0.9,z:f.z-0.65,w:1.8,d:1.3}});
  });
  const off=FORKLIFTS.length;
  armJoints.forEach(({i})=>{
    const ang=Math.sin(T*0.55+i*1.7)*0.9-Math.PI/2;
    const ex=ARMS[i].x+Math.cos(ang)*4.0, ez=ARMS[i].z+Math.sin(ang)*4.0;
    DYN.push({mat:MAT.arm,id:off+i});
    fillMask(D_MASK,{id:off+i,r:{x:Math.min(ARMS[i].x,ex)-0.5,z:Math.min(ARMS[i].z,ez)-0.5,
      w:Math.abs(ex-ARMS[i].x)+1.0,d:Math.abs(ez-ARMS[i].z)+1.0}});
  });
}

function futurePos(a,ms){
  const dd=STATE.speed*ms/1000;
  if(a.lane==='loop') return loopPoint(a.s+dd);
  let x0=a.x0+a.mdir*dd;
  if(x0>56){x0=56;} if(x0<4){x0=4;}
  return {x:x0,z:21.5};
}

function update(dt){
  T+=dt;
  rebuildDynMask();
  if(heatDirty||performance.now()-heatAt>3000) rebuildHeat();

  AGVS.forEach((a,i)=>{
    if(a.halt>0){ a.halt-=dt; }
    else if(a.lane==='loop'){
      a.s+=dt*STATE.speed;
      const p=loopPoint(a.s); a.x=p.x; a.z=p.z; a.ang=p.ang;
    }else{
      a.x0+=a.mdir*dt*STATE.speed;
      if(a.x0>56){a.x0=56;a.mdir=-1}
      if(a.x0<4){a.x0=4;a.mdir=1}
      a.x=a.x0; a.z=21.5; a.ang=a.mdir>0?0:Math.PI;
    }
    a.rssi=computeRSSI(a.x,a.z);
    a.lost=a.rssi<THRESH;
    // 遮挡预测：未来 300–500ms 位置（管线第③步）
    const fut=STATE.predOn? futurePos(a,STATE.predWin):null;
    a.futLost = fut? computeRSSI(fut.x,fut.z)<THRESH : false;
    if(!a.lost && a.futLost && !a.warned){
      a.warned=true;
      if(STATE.predOn)
        pushLog('遮挡预测 · AGV-'+String(a.id).padStart(2,'0')+' 将在 '+STATE.predWin+'ms 后进入阴影 · 已提前切换波束',false);
    }
    if(!a.lost && !a.futLost) a.warned=false;
    if(a.lost && !a.wasLost){
      if(STATE.predOn && a.warned){
        a.halt=0.12;
        pushLog('AGV-'+String(a.id).padStart(2,'0')+' 平顺通过弱覆盖区（预测系统已预切换，未急停）',false);
      }else{
        a.halt=0.9;
        pushLog('AGV-'+String(a.id).padStart(2,'0')+' 链路中断 · 触发急停（'+a.rssi.toFixed(0)+' dBm）',true);
      }
    }
    a.wasLost=a.lost;
  });

  FORKLIFTS.forEach(f=>{
    if(f.halt>0){ f.halt-=dt; }
    else{
      f.x+=f.dir*1.0*dt;
      if(f.x>55){f.x=55;f.dir=-1;f.ang=Math.PI}
      if(f.x<4){f.x=4;f.dir=1;f.ang=0}
    }
    f.rssi=computeRSSI(f.x,f.z);
    f.lost=f.rssi<THRESH;
    if(f.lost && !f.wasLost){
      f.halt=1.2;
      pushLog('叉车-'+f.id+' 进入遮挡阴影 · 链路降级',true);
    }
    f.wasLost=f.lost;
  });

  // RIS 每板服务对象与增益（管线第④⑤步）
  const movers=[...AGVS,...FORKLIFTS];
  activeRIS().forEach(p=>{
    p.eg=0; p.svc=null;
    if(!STATE.risOn||!p.on) return;
    let tg=null,bv=1e9;
    for(const a of movers){
      const d=Math.hypot(a.x-p.x,a.z-p.z);
      if(d<RIS_R*0.95 && a.rssi<bv){ bv=a.rssi; tg={x:a.x,z:a.z,obj:a,d}; }
    }
    if(!tg && p.tgt) tg={x:p.tgt.x,z:p.tgt.z,obj:null,d:Math.hypot(p.tgt.x-p.x,p.tgt.z-p.z)};
    if(!tg) return;
    p.svc=tg;
    p.eg=RIS_MAXG*(1-Math.min(tg.d,RIS_MAXD)/RIS_MAXD);
  });

  computeKPI();
  renderKPI();
  const now=performance.now();
  if(now-lastRisSync>400){ lastRisSync=now; syncRisCard(); }
}

/* =========================================================
   3D 每帧渲染
   ========================================================= */
const tmpV=new THREE.Vector3();
function setLine(line,x1,y1,z1,x2,y2,z2){
  const pos=line.geometry.attributes.position;
  pos.setXYZ(0,x1,y1,z1); pos.setXYZ(1,x2,y2,z2);
  pos.needsUpdate=true;
  line.computeLineDistances();
}
function statusColor(o){
  if(o.lost) return ((T*6)%1<0.5)?0xFF5544:0x7A2018;
  if(o.halt>0) return AMBER;
  return GOLD;
}
function applyMesh(m,o){
  const st=o.lost?'L':(o.halt>0?'H':'N');
  if(st!==o._st){
    o._st=st;
    m.led.material.color.setHex(statusColor(o));
    m.g.traverse(n=>{ if(n.isMesh&&n.material.color&&n!==m.led&&n.material._status){
      n.material.color.setHex(st==='L'?0xF3DFDA:(st==='H'?0xE8D5A8:MAT.forklift.color));
    }});
  }
}
let predTick=0;
function render3D(){
  // 热力层
  heatPlane.visible=true;

  // 基站环
  bsGroups.forEach((b,i)=>{
    b.g.visible=i<STATE.bs;
    if(b.g.visible) b.rings.forEach((r,k)=>{
      const ph=((T*0.5+k/3+i*0.13)%1);
      r.scale.setScalar(0.7+ph*4.5);
      r.material.opacity=0.30*(1-ph);
    });
  });

  // 机械臂
  armJoints.forEach(({pivot,elbow,i})=>{
    pivot.rotation.y=-(Math.sin(T*0.55+i*1.7)*0.9-Math.PI/2);
    elbow.rotation.y=Math.sin(T*0.85+i)*0.8+0.7;
  });

  // AGV
  AGVS.forEach((a,i)=>{
    const m=agvMeshes[i];
    if(!m) return;
    m.g.position.set(a.x,0,a.z);
    m.g.rotation.y=-a.ang;
    applyMesh(m,a);
    // 预测轨迹（每 200ms 交错更新）
    const due=((i%5)===(predTick%5));
    if(due){
      predTick++;
      if(STATE.predOn){
        const f=futurePos(a,STATE.predWin);
        setLine(m.trail,a.x,0.15,a.z,f.x,0.15,f.z);
        m.trail.material.color.setHex(a.futLost?0xB03A2E:0xC0A469);
        m.trail.material.opacity=a.futLost?0.75:0.4;
        m.trail.visible=true;
      }else m.trail.visible=false;
    }
  });
  // 叉车
  FORKLIFTS.forEach((f,i)=>{
    const m=flMeshes[i];
    if(!m) return;
    m.g.position.set(f.x,0,f.z);
    m.g.rotation.y=-f.ang;
    applyMesh(m,f);
  });
  // 叉车材质标签跟随第一台
  if(flMeshes[0]) flTag.position.set(FORKLIFTS[0].x,3.0,FORKLIFTS[0].z-1.4);
  matTags.visible=STATE.showMat;

  // 无人机沿横穿道巡航
  const dx=6+((T*1.6)%48);
  const dback=Math.floor((T*1.6)/48)%2;
  drone.g.position.set(dback?54-dx+6:dx,5,21.5);
  drone.g.rotation.y=dback?Math.PI:0;
  drone.rotors.forEach(r=>r.rotation.y+=0.6);

  // RIS 面板与波束
  risVis.forEach(rv=>{
    const p=rv.p;
    const active=STATE.risOn&&p.on&&activeRIS().indexOf(p)>=0;
    rv.mat.color.setHex(active?0xF2E6C8:0xB8B2A4);
    rv.edge.material.color.setHex(active?GOLD:0x8C8375);
    rv.glow.material.opacity=active&&p.fix>0?0.22+0.08*Math.sin(T*2+p.id):0;
    const showBeam=STATE.showBeam&&active&&!!p.svc;
    rv.inLine.visible=rv.reflLine.visible=rv.sector.visible=showBeam;
    rv.dots.forEach(d=>d.visible=showBeam);
    rv.rdots.forEach(d=>d.visible=showBeam);
    if(showBeam){
      const px=p.x,py=rv.h,pz=p.z;
      // 最近基站
      let bb=null,bd=1e9;
      activeBS().forEach(b=>{const d=Math.hypot(b.x-px,b.z-pz);if(d<bd){bd=d;bb=b;}});
      if(bb){
        setLine(rv.inLine,bb.x,SCENE.H-0.5,bb.z,px,py,pz);
        rv.dots.forEach((d,k)=>{
          const t=((T*0.40+k*0.5+p.id*0.07)%1);
          d.visible=true;
          d.position.set(bb.x+(px-bb.x)*t,SCENE.H-0.5+(py-(SCENE.H-0.5))*t,bb.z+(pz-bb.z)*t);
        });
      }
      const tg=p.svc;
      setLine(rv.reflLine,px,py,pz,tg.x,0.4,tg.z);
      rv.reflLine.material.opacity=0.45+Math.min(p.eg,18)/40;
      rv.rdots.forEach((d,k)=>{
        const t=((T*0.78+k/3+p.id*0.11)%1);
        d.visible=true;
        d.position.set(px+(tg.x-px)*t,py+(0.4-py)*t,pz+(tg.z-pz)*t);
        const s=1.25*(1-t*0.4); d.scale.setScalar(s);
      });
      // 覆盖扇区
      rv.sector.position.set(px,py,pz);
      rv.sector.rotation.y=Math.atan2(-(tg.z-pz),tg.x-px);
      rv.sector.scale.setScalar(RIS_R*0.95);
      rv.secMesh.material.opacity=0.10+0.05*Math.sin(T*1.7+p.id);
      // 命中脉冲
      const pr=(T*0.9+p.id*0.17)%1;
      rv.hitRing.position.set(tg.x,0.12,tg.z);
      rv.hitRing.scale.setScalar(0.6+pr*2.2);
      rv.hitRing.material.opacity=0.40*(1-pr);
    }else{
      rv.hitRing.material.opacity=0;
    }
  });

  controls.update();
  renderer.render(scene,camera);
}

/* =========================================================
   UI
   ========================================================= */
const SLIDERS=[
  {k:'agvCount', label:'AGV 数量',       min:4, max:24, step:1,  unit:'台', fmt:v=>v},
  {k:'speed',    label:'行驶速度',       min:0.4,max:2.0,step:0.1,unit:'m/s',fmt:v=>v.toFixed(1)},
  {k:'forklifts',label:'无人叉车',       min:0, max:4,  step:1,  unit:'台', fmt:v=>v},
  {k:'shadow',   label:'金属遮挡强度',   min:40,max:130,step:5,  unit:'%',  fmt:v=>v},
  {k:'ris',      label:'RIS 智能反射面', min:0, max:16, step:1,  unit:'块', fmt:v=>v},
  {k:'bs',       label:'6G 基站节点',    min:2, max:6,  step:1,  unit:'个', fmt:v=>v},
  {k:'predWin',  label:'遮挡预测窗口',   min:300,max:500,step:20, unit:'ms', fmt:v=>v}
];
document.getElementById('sliders').innerHTML=SLIDERS.map(s=>`
  <div class="row">
    <div class="rowhead">
      <span class="lab">${s.label}</span>
      <span class="val"><b id="v-${s.k}">${s.fmt(STATE[s.k])}</b> ${s.unit}</span>
    </div>
    <input type="range" id="s-${s.k}" min="${s.min}" max="${s.max}" step="${s.step}" value="${STATE[s.k]}">
  </div>`).join('');

SLIDERS.forEach(s=>{
  const inp=document.getElementById('s-'+s.k);
  inp.addEventListener('input',()=>{
    STATE[s.k]=parseFloat(inp.value);
    document.getElementById('v-'+s.k).textContent=s.fmt(STATE[s.k]);
    heatDirty=true; heatAt=0;
    if(s.k==='agvCount'){ buildAGVs(); rebuildAgvMeshes(); }
    if(s.k==='forklifts'){ buildForklifts(); rebuildAgvMeshes(); }
    if(s.k==='predWin') AGVS.forEach(a=>a.warned=false);
  });
});

function syncUI(){
  SLIDERS.forEach(s=>{
    const i=document.getElementById('s-'+s.k);
    if(i) i.value=STATE[s.k];
    const l=document.getElementById('v-'+s.k);
    if(l) l.textContent=s.fmt(STATE[s.k]);
  });
  document.getElementById('swSensing').classList.toggle('on',STATE.sensing);
  document.getElementById('swPred').classList.toggle('on',STATE.predOn);
  document.getElementById('swMat').classList.toggle('on',STATE.showMat);
  document.getElementById('swBeam').classList.toggle('on',STATE.showBeam);
  const br=document.getElementById('btnRis');
  br.textContent='RIS：'+(STATE.risOn?'开':'关');
  br.classList.toggle('warnOn',!STATE.risOn);
  [...document.querySelectorAll('#segSlice button')].forEach(b=>b.classList.toggle('on',b.dataset.s===STATE.slicing));
  const sv=document.getElementById('succVal');
  if(sv) sv.textContent=KPI.succ.toFixed(1);
}

document.getElementById('matlist').innerHTML=
  ['steel','rack','arm','forklift','carton','container'].map(k=>{
    const m=MAT[k];
    return `<div class="matline">
      <span class="matsw" style="background:#${m.color.toString(16).padStart(6,'0')};border-color:#${m.edge.toString(16).padStart(6,'0')}"></span>
      <span class="matname">${m.name}</span>
      <span class="matrho">ρ=${m.rho.toFixed(2)} · 穿透损耗 ${m.loss}dB</span>
    </div>`;
  }).join('');

const risGridEl=document.getElementById('risgrid');
risGridEl.innerHTML=RIS_POOL.map(p=>`<div class="riscell" data-id="${p.id}" title="RIS-${String(p.id).padStart(2,'0')}"></div>`).join('');
risGridEl.addEventListener('click',ev=>{
  const c=ev.target.closest('.riscell'); if(!c) return;
  const p=RIS_POOL[+c.dataset.id-1];
  togglePanel(p);
});

function togglePanel(p){
  p.on=!p.on;
  heatDirty=true; heatAt=0;
  rebuildHeat(); computeKPI(true); renderKPI(); syncRisCard();
  pushLog(p.on
    ? 'RIS-'+String(p.id).padStart(2,'0')+' 恢复供电 · 波束重新指向盲区'
    : 'RIS-'+String(p.id).padStart(2,'0')+' 断电 · 该区盲区回升至 '+DOC.blind.base.toFixed(1)+'%（文档 8%—15%）', !p.on);
}

function syncRisCard(){
  const cells=risGridEl.children;
  const live=STATE.risOn? activeRIS().filter(p=>p.on) : [];
  for(let i=0;i<cells.length;i++){
    const p=RIS_POOL[i];
    cells[i].classList.toggle('off', !(STATE.risOn&&p.on));
    cells[i].style.opacity = i<STATE.ris? 1 : 0.3;
  }
  const {risF}=optDeg();
  document.getElementById('risStat').textContent=live.length+' / '+STATE.ris+' 在线';
  document.getElementById('risGain').textContent=(DOC.gain.opt*risF).toFixed(1);
  document.getElementById('risBlind').textContent=(DOC.blind.base+(DOC.blind.opt-DOC.blind.base)*risF).toFixed(1);
  const bd=document.getElementById('risBadge');
  if(bd) bd.innerHTML = live.length
    ? '智能表面 <b>'+live.length+'</b> 块在线 · 补盲 <b>'+(DOC.gain.opt*risF).toFixed(1)+'</b> dB · 盲区 <b>'
      +DOC.blind.base.toFixed(1)+'%</b>→<b>'+(DOC.blind.base+(DOC.blind.opt-DOC.blind.base)*risF).toFixed(1)+'%</b> · 点面板可断电对比'
    : '智能表面阵列未通电 · 盲区 <b>'+DOC.blind.base.toFixed(1)+'%</b> · 点「RIS」恢复';
}

/* —— 底部八项指标：与文档《八项指标表》逐项对应 —— */
const KPI_DEF=[
  {k:'lat',  label:'端到端时延 P99', unit:'ms',  d:1, dir:'≤', base:'96',    target:20},
  {k:'ints', label:'遮挡区链路中断', unit:'次/h',d:2, dir:'≤', base:'3.4',   target:0.1},
  {k:'blind',label:'单站盲区占比',   unit:'%',   d:1, dir:'≤', base:'8.0',   target:2},
  {k:'pos',  label:'非视距定位误差', unit:'cm',  d:1, dir:'≤', base:'42',    target:10},
  {k:'tops', label:'单机等效算力',   unit:'TOPS',d:0, dir:'≥', base:'20–40', target:500},
  {k:'bom',  label:'算力通信 BOM',   unit:'%',   d:0, dir:'≤', base:'100',   target:60},
  {k:'recfg',label:'换任务重配置',   unit:'min', d:0, dir:'≤', base:'240',   target:10},
  {k:'ul',   label:'单机上行吞吐',   unit:'Mbps',d:0, dir:'≥', base:'60',    target:300}
];
document.getElementById('kpis').innerHTML=KPI_DEF.map(x=>`
  <div class="kpi">
    <div class="k">${x.label}</div>
    <div class="n"><span id="k-${x.k}">--</span><small>${x.unit}</small></div>
    <div class="c" id="c-${x.k}">&nbsp;</div>
  </div>`).join('');

function renderKPI(){
  KPI_DEF.forEach(x=>{
    const el=document.getElementById('k-'+x.k);
    if(!el) return;
    const v=KPI[x.k];
    el.textContent=v.toFixed(x.d);
    const ok = x.dir==='≤' ? v<=x.target*1.0001 : v>=x.target*0.9999;
    const c=document.getElementById('c-'+x.k);
    c.innerHTML='基线 '+x.base+' · 目标 '+x.dir+' '+x.target
      +' <b class="'+(ok?'ok':'bad')+'">'+(ok?'达标':'未达')+'</b>';
  });
  const sv=document.getElementById('succVal');
  if(sv) sv.textContent=KPI.succ.toFixed(1);
}

const logEl=document.getElementById('log');
function pushLog(text,warn){
  const d=document.createElement('div');
  d.className='lg'+(warn?' warn':'');
  d.textContent=text;
  logEl.appendChild(d);
  while(logEl.children.length>4) logEl.removeChild(logEl.firstChild);
  setTimeout(()=>{ if(d.parentNode) d.parentNode.removeChild(d); },5000);
}

/* ---- 五步管线 overlay 动画（周期 1.2s，对应慢环 1Hz） ---- */
const PSTEPS=['ps0','ps1','ps2','ps3','ps4'].map(id=>document.getElementById(id));
const pbar=document.getElementById('pbar');
const PIPE_PHASES=[0.16,0.34,0.53,0.72,1.01];
function renderPipe(){
  const ph=(T%1.2)/1.2;
  pbar.style.width=(ph*100).toFixed(1)+'%';
  PSTEPS.forEach((el,i)=>{
    el.classList.toggle('on', ph<PIPE_PHASES[i]);
  });
}

/* ---- 事件 ---- */
let playing=true;
document.getElementById('btnPlay').addEventListener('click',e=>{
  playing=!playing; e.target.textContent=playing?'暂停':'继续';
});
document.getElementById('btnReset').addEventListener('click',()=>{
  buildAGVs(); buildForklifts(); rebuildAgvMeshes();
  pushLog('场景已重置 · 20 台 AGV 按初始任务排布',false);
});
const modal=document.getElementById('modal');
document.getElementById('btnSrc').addEventListener('click',()=>modal.classList.add('on'));
document.getElementById('mclose').addEventListener('click',()=>modal.classList.remove('on'));
modal.addEventListener('click',e=>{ if(e.target===modal) modal.classList.remove('on'); });
document.getElementById('btnStage').addEventListener('click',()=>{
  document.body.classList.add('stage-mode'); setTimeout(resize,60);
});
cv.addEventListener('dblclick',()=>{ document.body.classList.remove('stage-mode'); setTimeout(resize,60); });
document.addEventListener('keydown',e=>{
  if(e.key==='Escape'){ document.body.classList.remove('stage-mode'); setTimeout(resize,60); }
});
document.getElementById('swSensing').addEventListener('click',ev=>{
  STATE.sensing=!STATE.sensing;
  ev.currentTarget.classList.toggle('on',STATE.sensing);
  heatDirty=true;
  pushLog('通感一体 '+(STATE.sensing?'已开启 · 通信波形兼做环境感知':'已关闭'),false);
});
document.getElementById('swPred').addEventListener('click',ev=>{
  STATE.predOn=!STATE.predOn;
  ev.currentTarget.classList.toggle('on',STATE.predOn);
  AGVS.forEach(a=>a.warned=false); heatDirty=true;
  pushLog(STATE.predOn
    ? '遮挡预测开启：孪生以 50ms 周期滚动预测未来 '+STATE.predWin+'ms 遮挡状态'
    : '遮挡预测关闭：链路只能在中断发生后被动切换',false);
});
document.getElementById('swMat').addEventListener('click',ev=>{
  STATE.showMat=!STATE.showMat;
  ev.currentTarget.classList.toggle('on',STATE.showMat);
});
document.getElementById('swBeam').addEventListener('click',ev=>{
  STATE.showBeam=!STATE.showBeam;
  ev.currentTarget.classList.toggle('on',STATE.showBeam);
  pushLog(STATE.showBeam?'波束可视化已开启：基站→面板→盲区 完整链路':'波束可视化已关闭',false);
});
document.getElementById('btnRis').addEventListener('click',()=>{
  STATE.risOn=!STATE.risOn;
  heatDirty=true; heatAt=0; rebuildHeat();
  computeKPI(true); renderKPI(); syncUI(); syncRisCard();
  pushLog(STATE.risOn
    ? '智能表面阵列通电：盲区 '+DOC.blind.base.toFixed(1)+'% → '+DOC.blind.opt.toFixed(1)+'%'
    : '智能表面阵列断电：盲区回到 '+DOC.blind.base.toFixed(1)+'%', !STATE.risOn);
});
document.getElementById('segSlice').addEventListener('click',ev=>{
  const b=ev.target.closest('button'); if(!b) return;
  STATE.slicing=b.dataset.s; syncUI();
  const names={end:'纯端侧',cloud:'纯云端',dynamic:'动态三级'};
  pushLog('算力切分切换为「'+names[STATE.slicing]+'」',false);
});
document.querySelectorAll('.presets button').forEach(b=>{
  b.addEventListener('click',()=>{
    document.querySelectorAll('.presets button').forEach(x=>x.classList.remove('active'));
    b.classList.add('active');
    applyPreset(b.dataset.preset);
  });
});
function applyPreset(name){
  Object.assign(STATE,PRESETS[name]);
  RIS_POOL.forEach(p=>p.on=true);
  STATE.risOn=true;
  buildAGVs(); buildForklifts(); rebuildAgvMeshes();
  heatDirty=true; heatAt=0;
  syncUI(); rebuildDynMask(); rebuildHeat(); computeKPI(true);
  syncRisCard();
  if(name==='baseline') KPI.base=snapshot();
  pushLog(name==='baseline'
    ? '已载入「5G-A 基线」：无 RIS、无预测、算力全在云端'
    : '已载入「6G 联合优化」：RIS+通感一体+遮挡预测+动态切分',false);
}

/* ---- 3D 点选 RIS 面板通断 ---- */
const ray=new THREE.Raycaster(), ptr=new THREE.Vector2();
const panelMeshes=risVis.map(rv=>rv.panel);
cv.addEventListener('click',ev=>{
  const r=cv.getBoundingClientRect();
  ptr.x=((ev.clientX-r.left)/r.width)*2-1;
  ptr.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(ptr,camera);
  const hits=ray.intersectObjects(panelMeshes.filter(m=>m.parent.visible),false);
  if(hits.length){
    const p=hits[0].object.userData.ris;
    if(p && STATE.risOn && activeRIS().indexOf(p)>=0) togglePanel(p);
  }
});
cv.addEventListener('mousemove',ev=>{
  const r=cv.getBoundingClientRect();
  ptr.x=((ev.clientX-r.left)/r.width)*2-1;
  ptr.y=-((ev.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(ptr,camera);
  cv.style.cursor=ray.intersectObjects(panelMeshes,false).length?'pointer':'default';
});

/* =========================================================
   主循环
   ========================================================= */
let last=performance.now();
let lastRisSync=0;
let pipeTick=0;
function loop(now){
  const dt=Math.min((now-last)/1000,0.05);
  last=now;
  if(playing){ update(dt); renderPipe(); }
  render3D();
  requestAnimationFrame(loop);
}

resize();
(function initBaseline(){
  const save={...STATE};
  Object.assign(STATE,PRESETS.baseline);
  rebuildDynMask(); heatDirty=true; rebuildHeat();
  computeKPI(true); KPI.base=snapshot();
  Object.assign(STATE,save);
  rebuildDynMask(); heatDirty=true; heatAt=0;
  syncUI();
})();
window.__DBG={scene,AGVS,FORKLIFTS,risVis};
pushLog('3D 仿真已启动 · 拖动旋转视角 · 拖动右侧参数观察实时变化',false);
requestAnimationFrame(loop);
})();