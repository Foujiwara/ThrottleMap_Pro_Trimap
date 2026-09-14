// Runs the real JavaScript out of ui.qml.in, with only Qt and the transport
// mocked. It covers the receive path and the command queue - which is where
// every interface bug in this project has actually been - and deliberately
// not rendering or layout.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const source=fs.readFileSync('ui.qml.in','utf8');
function extract(name) {
 const at=source.indexOf(`function ${name}(`); assert(at>=0,name);
 const start=source.indexOf('{',at);let depth=1,i=start+1,quote=null,comment=null;
 for(;depth&&i<source.length;i++) {
  const ch=source[i],next=source[i+1];
  if(comment==='line'){if(ch==='\n')comment=null;continue;}
  if(comment==='block'){if(ch==='*'&&next==='/'){comment=null;i++;}continue;}
  if(quote){if(ch==='\\'){i++;continue;}if(ch===quote)quote=null;continue;}
  if(ch==='"'||ch==="'"){quote=ch;continue;}
  if(ch==='/'&&next==='/'){comment='line';i++;continue;}
  if(ch==='/'&&next==='*'){comment='block';i++;continue;}
  if(ch==='{')depth++;if(ch==='}')depth--;
 }
 assert.equal(depth,0,name);return source.slice(at,i);
}
const sent=[];
const timeout={interval:20000,restart(){},stop(){}};
const noDebounce={running:false,stop(){}};
const state={console,ArrayBuffer,DataView,Date,Math,Array,isFinite,Error,
 VescIf:{isPortConnected:()=>true},vescCommands:{sendCustomAppData:b=>sent.push(new Uint8Array(b))},
 commandTimeout:timeout,regenDebounce:noDebounce,brakeRegenDebounce:noDebounce,revRegenDebounce:noDebounce,
 // Geometry: the ragged grid. 11 brake rows of 21 columns, 20 throttle rows
 // of 11, sharing one flat buffer.
 mapThrN:31,mapDutyN:21,mapThrZero:10,mapCells:451,mapData:Array(451).fill(0),
 cfgPreset:1,cfgTorqueResp:0.85,cfgSpeedCoupling:1,cfgTransWidth:0.1,cfgTransShape:1,
 cfgHighHold:0.55,cfgEngineBrake:0.15,cfgOverrunRegen:0.12,cfgRegenCurve:1,
 cfgBrakeMap:0,cfgBrakeType:0,cfgDutyFilter:0.3,
 cfgBrakeStr:1,cfgBrakeResp:1,cfgBrakeDep:0,cfgBrakeCurve:0,
 cfgRevStr:1,cfgRevResp:1,cfgRevHold:0,cfgRevCoupling:1,cfgRevWidth:0.1,
 cfgRevShape:1,cfgRevOverrun:0.12,cfgRevCurve:1,
 thrSource:0,thrInvert:0,thrDeadband:0.02,thrFilter:1,thrBrakeMode:0,thrBidirCenter:1.65,
 txQueue:[],pendingPacket:null,mapReceived:true,cfgReceived:true,rowSeen:[],
 lastLiveTime:0,lastAutoReq:0,lastCmdStatus:'',statusText:'',benchValue:0,
 liveThrottle:0,liveDuty:0,liveErpm:0,liveCurRel:0,liveCurA:0,liveBrake:0,
 liveAdcVoltage:0,liveRpmFast:0,pkgEnabled:1,lockState:0,lockReason:0,lockErr:0,
 scriptBooting:false,autoLoadDone:false,autoLoadPending:false,
 mapScratch:[],mapRowsSeen:0,mapRetries:0,mapMaxRetries:5,clearArmed:false,lockTimeout:60,lockTravel:0.25,lockMax:0.15,lockDamp:0.3,lockFree:0,
 cmdSetCell:1,cmdSetMapRow:2,cmdSetConfig:3,cmdSetThr:4,cmdSave:5,cmdLoad:6,cmdReset:7,
 cmdReqMap:8,cmdReqCfg:9,cmdSetTestThr:10,cmdCalibrateBidir:11,cmdSetEnabled:12,
 cmdSetLock:13,cmdLockCmd:14,cmdClear:15,rxLive:128,rxMapRow:129,rxStatus:130,rxCfgEcho:131};
vm.createContext(state);
for(const name of ['validateImport','quantizeCell','mapRowCols','cellIdx','getCell','fxEnc','fxDec',
 'i16Bytes','transmitPacket','sendPacket','pumpTx','failTransfer','sendSetCell','sendSetMapRow',
 'sendAllRows','sendSetConfig','sendSetThrottle','sendLockCfg','sendSave','sendLoad','sendReset','sendClear',
 'requestMap','requestCfg','sendTestThrottle','handleRx','thermalPeak','shapeCurve',
 'regenerateLocalMap','regenerateLocalBrake','regenerateLocalRev']) {
 vm.runInContext(extract(name),state);
}
function ack(code=0){state.handleRx(Uint8Array.from([130,code,state.pendingPacket[0]]).buffer);}
function livePacket({booting=false,lockOn=false,reason=0,enabled=1}={}) {
 const b=new Uint8Array(23);const v=new DataView(b.buffer);
 b[0]=128;v.setUint8(17,enabled);
 v.setUint8(18,(lockOn?1:0)+(reason<<4)+(booting?128:0));
 return b.buffer;
}

// ---- the command queue ---------------------------------------------------
// One acknowledged command at a time, and all 31 rows of the ragged grid.
state.sendAllRows();assert.equal(sent.length,1);assert.equal(state.txQueue.length,30);
for(let i=0;i<31;i++)ack();assert.equal(sent.length,31);assert.equal(state.pendingPacket,null);

// The configuration packet is 44 bytes, with three independent regenerate
// flags so one half of the map never rewrites another.
sent.length=0;state.sendSetConfig();assert.equal(sent[0].length,44);
assert.equal(sent[0][16],1);assert.equal(sent[0][28],0);assert.equal(sent[0][43],0);ack();
state.sendSetConfig(false,true,true);
assert.equal(sent.at(-1)[16],0);assert.equal(sent.at(-1)[28],1);assert.equal(sent.at(-1)[43],1);ack();

// Save uploads every displayed parameter and all rows before the flash
// command, and never regenerates on the way.
sent.length=0;state.sendSave();while(state.pendingPacket!==null)ack(state.pendingPacket[0]===5?1:0);
assert.deepEqual(sent.map(b=>b[0]),[3,4,...Array(31).fill(2),5]);assert.equal(sent[0][16],0);
assert.match(state.lastCmdStatus,/verified/);
sent.length=0;state.sendSave();ack(6);assert.equal(state.txQueue.length,0);assert.equal(sent.length,1);
assert.match(state.lastCmdStatus,/Invalid/);

// The lock packet is 10 bytes: timeout, travel, ceiling, damping, dead travel.
sent.length=0;state.pendingPacket=null;state.txQueue=[];
state.sendLockCfg();assert.equal(sent[0].length,10);assert.equal(sent[0][1],60);ack();

// ---- read-back timeouts --------------------------------------------------
// A read-back is 31 packets and ~150 ms of pacing. Waiting 20 s to notice it
// was dropped is what made a cold start look like a hang; only the EEPROM
// operations need that long.
state.pendingPacket=null;state.txQueue=[];state.rowSeen=[];
state.requestMap();assert.equal(timeout.interval,3000);ack();
state.requestCfg();assert.equal(timeout.interval,3000);
state.handleRx(Uint8Array.from([131,...Array(51).fill(0)]).buffer);ack();
state.sendSave();assert.equal(timeout.interval,20000);
state.failTransfer('reset');

// ---- the receive path ----------------------------------------------------
// Decode must happen before any early return. A "still starting" guard placed
// above the decode latched the whole interface on the first packet it saw:
// the value it tests is set BY the decode it skipped.
state.scriptBooting=false;state.lastAutoReq=0;
state.handleRx(livePacket({booting:true}));
assert.equal(state.scriptBooting,true);
assert.equal(state.statusText,'Connected','telemetry must decode while booting');
state.handleRx(livePacket({booting:false,lockOn:true,reason:3}));
assert.equal(state.scriptBooting,false,'the boot flag must be able to clear');
assert.equal(state.lockState,1);assert.equal(state.lockReason,3);

// The automatic read-back retry is rate limited. Unthrottled it re-queued all
// 31 rows twenty times a second and took the link down with it.
state.mapReceived=false;state.cfgReceived=false;
state.pendingPacket=null;state.txQueue=[];state.lastAutoReq=0;
state.handleRx(livePacket());
const queuedAfterFirst=state.txQueue.length+(state.pendingPacket?1:0);
assert.ok(queuedAfterFirst>0,'a read-back should be queued');
state.pendingPacket=null;state.txQueue=[];
state.handleRx(livePacket());
assert.equal(state.txQueue.length+(state.pendingPacket?1:0),0,'retry must be rate limited');

// A fresh connection pulls the stored map out of EEPROM before reading it
// back, so the graph shows what is saved and not whatever is in RAM.
state.mapReceived=false;state.cfgReceived=false;
state.autoLoadDone=false;state.autoLoadPending=false;
state.pendingPacket=null;state.txQueue=[];state.lastAutoReq=0;
state.handleRx(livePacket());
assert.equal(state.pendingPacket[0],state.cmdLoad,'a connection loads from EEPROM first');
assert.deepEqual(state.txQueue.map(b=>b[0]),[state.cmdReqCfg,state.cmdReqMap]);
// A controller with nothing saved answers that load with an error. It is a
// normal state on a fresh install, not a failed transfer, and the read-backs
// queued behind it must still go out.
ack(5);
assert.equal(state.autoLoadDone,true);
assert.match(state.lastCmdStatus,/Nothing saved/);
assert.equal(state.pendingPacket[0],state.cmdReqCfg,'the read-back survives an empty EEPROM');
state.pendingPacket=null;state.txQueue=[];
state.mapReceived=true;state.cfgReceived=true;

// Nothing is requested at all while the script is still starting.
state.lastAutoReq=0;state.pendingPacket=null;state.txQueue=[];
state.handleRx(livePacket({booting:true}));
assert.equal(state.txQueue.length+(state.pendingPacket?1:0),0,'no requests while booting');
state.scriptBooting=false;state.mapReceived=true;state.cfgReceived=true;

// Neither side may throw on data belonging to another package.
for(const id of [128,129,130,131]) for(let n=0;n<2;n++) {
 const b=new Uint8Array(n);if(n)b[0]=id;assert.doesNotThrow(()=>state.handleRx(b.buffer));
}
// A row index outside the grid is ignored rather than growing the buffer.
const outOfBounds=new Uint8Array(44);outOfBounds[0]=129;outOfBounds[1]=255;
state.handleRx(outOfBounds.buffer);assert.equal(state.mapData.length,451);
// A brake row carries 21 columns, a throttle row 11, into the same buffer.
assert.equal(state.mapRowCols(0),21);assert.equal(state.mapRowCols(30),11);
assert.equal(state.cellIdx(0,0),0);assert.equal(state.cellIdx(30,10),450);

// ---- import validation ---------------------------------------------------
assert.throws(()=>state.validateImport({map:Array(451).fill(NaN)}));
assert.throws(()=>state.validateImport({map:Array(450).fill(0)}));
assert.throws(()=>state.validateImport({map:Array(451).fill(0),cfg:{preset:1}}));
assert.doesNotThrow(()=>state.validateImport({map:Array(451).fill(-0.5)}));

// ---- the map read-back ---------------------------------------------------
function rowPacket(row,value) {
 const b=new Uint8Array(44);const v=new DataView(b.buffer);
 b[0]=129;v.setUint8(1,row);
 for(let d=0;d<state.mapRowCols(row);d++)v.setInt16(2+d*2,value,false);
 return b.buffer;
}
state.pendingPacket=null;state.txQueue=[];state.rowSeen=[];state.mapRetries=0;
state.mapData=Array(451).fill(0);
state.requestMap();
for(let r=0;r<31;r++)state.handleRx(rowPacket(r,250));
// Nothing reaches mapData mid-read: reassigning it per row repaints all 451
// cells while the next packet is already arriving, which is where the top
// rows of the grid were being lost.
assert.deepEqual(state.mapData,Array(451).fill(0),'nothing is published mid-read');
// The count is driven by the rows themselves, not sampled at 20 Hz by the
// telemetry handler - sampling froze it at whatever number it last caught.
assert.equal(state.mapRowsSeen,31);
assert.match(state.lastCmdStatus,/31\/31 rows/);
ack();
assert.equal(state.mapReceived,true);
assert.equal(state.mapData[0],0.25,'the buffer is published on completion');
assert.equal(state.mapData[450],0.25,'including the last row');
assert.match(state.lastCmdStatus,/Map read: 31\/31/);

// A duplicate row must not be counted twice.
state.pendingPacket=null;state.txQueue=[];state.requestMap();
state.handleRx(rowPacket(0,100));state.handleRx(rowPacket(0,100));
assert.equal(state.mapRowsSeen,1,'a repeated row counts once');

// A dropped row is a link hiccup, not a broken map: ask again. Giving up at
// the first miss left the grid drawn as zeros with no way back but a manual
// Load.
state.pendingPacket=null;state.txQueue=[];state.rowSeen=[];state.mapRetries=0;
const before=state.mapData.slice();
state.requestMap();
ack();assert.match(state.lastCmdStatus,/retrying \(1\/5\)/);
assert.equal(state.pendingPacket[0],state.cmdReqMap);
ack();ack();ack();ack();
assert.equal(state.mapReceived,false);
ack();assert.match(state.lastCmdStatus,/Incomplete/);
// A partial map is never published: a hole drawn as zeros is
// indistinguishable from a real cell value.
assert.deepEqual(state.mapData,before,'a failed read leaves the grid alone');

// ---- clear ---------------------------------------------------------------
sent.length=0;state.pendingPacket=null;state.txQueue=[];
state.sendClear();assert.deepEqual(Array.from(sent[0]),[15]);
console.log('QML JavaScript regression tests passed');
