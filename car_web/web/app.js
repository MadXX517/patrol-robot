const api = {
  status: '/api/status',
  events: '/api/events',
  features: '/api/features',
  featureStart: '/api/features/start',
  featureStop: '/api/features/stop',
  reportJob: '/api/report/job',
  uimode: '/api/uimode',
  navStop: '/api/nav/stop',
  mappingStart: '/api/nav/mapping/start',
  exploreStop: '/api/nav/explore/stop',
  mapSave: '/api/nav/map/save',
  mapLoad: '/api/nav/map/load',
  navGoto: '/api/nav/goto',
  navHome: '/api/nav/home',
  navCancel: '/api/nav/cancel',
  navInitialPose: '/api/nav/initialpose',
  navSpeed: '/api/nav/speed',
  navCruise: '/api/nav/cruise',
  mapsList: '/api/nav/maps',
  pointsList: '/api/nav/points',
  pointSave: '/api/nav/point/save',
  pointDelete: '/api/nav/point/delete',
  pointGoto: '/api/nav/point/goto',
  manual: '/api/mode/manual',
  report: '/api/mode/report',
  coreStop: '/api/core/stop',
  reportStop: '/api/report/stop',
  allStop: '/api/all/stop',
  notifyStart: '/api/notify/start',
  notifyStop: '/api/notify/stop',
  drive: '/api/drive',
  chassisReset: '/api/chassis/reset',
  servoReset: '/api/servo/reset',
  cameraLeft: '/api/servo/camera_left',
  cameraRight: '/api/servo/camera_right',
  patrolRelock: '/api/patrol/relock',
  patrolCommand: '/api/patrol/command',
  patrolSpeed: '/api/patrol/speed',
  gestureStart: '/api/gesture/start',
  gestureStop: '/api/gesture/stop',
  voiceTtsStart: '/api/voice/tts/start',
  voiceTtsStop: '/api/voice/tts/stop',
  voiceAsrStart: '/api/voice/asr/start',
  voiceAsrStop: '/api/voice/asr/stop',
  generateReport: '/api/report/generate',
};

const state = {
  videoTopic: '',
  videoUrl: '',
  driveTimer: null,
  activeDrive: null,
  featuresLoaded: false,
  features: [],
  selectedFeatureId: '',
  activeFeature: '',
  uiMode: 'follow',
  pendingUiMode: null,   // 切换进行中目标模式;轮询期间不让后端旧状态覆盖乐观 UI
  navMap: null,
  videoFailed: false,
  manualTopic: null,   // 非空=用户手动选了流,不再被后端自动 topic 覆盖
};

const els = {
  statusText: document.getElementById('statusText'),
  batteryBadge: document.getElementById('batteryBadge'),
  coreBadge: document.getElementById('coreBadge'),
  reportBadge: document.getElementById('reportBadge'),
  notifyBadge: document.getElementById('notifyBadge'),
  videoTopic: document.getElementById('videoTopic'),
  videoHint: document.getElementById('videoHint'),
  videoLink: document.getElementById('videoLink'),
  videoHomeLink: document.getElementById('videoHomeLink'),
  videoStream: document.getElementById('videoStream'),
  streamSelect: document.getElementById('streamSelect'),
  videoError: document.getElementById('videoError'),
  eventsList: document.getElementById('eventsList'),
  eventCount: document.getElementById('eventCount'),
  reportJob: document.getElementById('reportJob'),
  manualBtn: document.getElementById('manualBtn'),
  coreStopBtn: document.getElementById('coreStopBtn'),
  allStopBtn: document.getElementById('allStopBtn'),
  eventsToggleBtn: document.getElementById('eventsToggleBtn'),
  featureToggleBtn: document.getElementById('featureToggleBtn'),
  featureCloseBtn: document.getElementById('featureCloseBtn'),
  featurePanel: document.getElementById('featurePanel'),
  featureListView: document.getElementById('featureListView'),
  featureDetailView: document.getElementById('featureDetailView'),
  featureGrid: document.getElementById('featureGrid'),
  featureBackBtn: document.getElementById('featureBackBtn'),
  featureDetailName: document.getElementById('featureDetailName'),
  featureDetailDesc: document.getElementById('featureDetailDesc'),
  featureDetailStatus: document.getElementById('featureDetailStatus'),
  featureStartBtn: document.getElementById('featureStartBtn'),
  featureStopBtn: document.getElementById('featureStopBtn'),
  cameraLeftBtn: document.getElementById('cameraLeftBtn'),
  cameraRightBtn: document.getElementById('cameraRightBtn'),
  chassisResetBtn: document.getElementById('chassisResetBtn'),
  servoResetBtn: document.getElementById('servoResetBtn'),
  patrolRelockBtn: document.getElementById('patrolRelockBtn'),
  patrolFollowBtn: document.getElementById('patrolFollowBtn'),
  patrolTrackBtn: document.getElementById('patrolTrackBtn'),
  patrolStopBtn: document.getElementById('patrolStopBtn'),
  gestureToggleBtn: document.getElementById('gestureToggleBtn'),
  voiceTtsToggleBtn: document.getElementById('voiceTtsToggleBtn'),
  voiceAsrToggleBtn: document.getElementById('voiceAsrToggleBtn'),
  voiceState: document.getElementById('voiceState'),
  patrolState: document.getElementById('patrolState'),
  uiModeFollowBtn: document.getElementById('uiModeFollowBtn'),
  uiModeNavBtn: document.getElementById('uiModeNavBtn'),
  followGroup: document.getElementById('followGroup'),
  navGroup: document.getElementById('navGroup'),
  navState: document.getElementById('navState'),
  navStopBtn: document.getElementById('navStopBtn'),
  navHomeBtn: document.getElementById('navHomeBtn'),
  navCancelBtn: document.getElementById('navCancelBtn'),
  setPoseBtn: document.getElementById('setPoseBtn'),
  navSpeed: document.getElementById('navSpeed'),
  navFasterBtn: document.getElementById('navFasterBtn'),
  navSlowerBtn: document.getElementById('navSlowerBtn'),
  cruiseModeBtn: document.getElementById('cruiseModeBtn'),
  cruiseClearBtn: document.getElementById('cruiseClearBtn'),
  cruiseStartBtn: document.getElementById('cruiseStartBtn'),
  cruiseCount: document.getElementById('cruiseCount'),
  navCanvas: document.getElementById('navCanvas'),
  mapManualBtn: document.getElementById('mapManualBtn'),
  mapExploreBtn: document.getElementById('mapExploreBtn'),
  exploreStopBtn: document.getElementById('exploreStopBtn'),
  mapNameInput: document.getElementById('mapNameInput'),
  mapSaveBtn: document.getElementById('mapSaveBtn'),
  mapRefreshBtn: document.getElementById('mapRefreshBtn'),
  mapList: document.getElementById('mapList'),
  pointNameInput: document.getElementById('pointNameInput'),
  pointSaveBtn: document.getElementById('pointSaveBtn'),
  pointList: document.getElementById('pointList'),
  followSpeed: document.getElementById('followSpeed'),
  followFasterBtn: document.getElementById('followFasterBtn'),
  followSlowerBtn: document.getElementById('followSlowerBtn'),
  gestureState: document.getElementById('gestureState'),
  notifyStartBtn: document.getElementById('notifyStartBtn'),
  notifyStopBtn: document.getElementById('notifyStopBtn'),
  reportTextBtn: document.getElementById('reportTextBtn'),
  reportVisionBtn: document.getElementById('reportVisionBtn'),
};

async function request(path, options = {}) {
  const response = await fetch(path, {
    method: options.method || 'GET',
    headers: { 'Content-Type': 'application/json' },
    body: options.body ? JSON.stringify(options.body) : undefined,
  });
  const data = await response.json();
  if (!response.ok || data.ok === false) {
    throw new Error(data.error || `HTTP ${response.status}`);
  }
  return data;
}

function setStatus(text, isError = false) {
  els.statusText.textContent = text;
  els.statusText.style.color = isError ? '#c62828' : '#607080';
}

function setBadge(element, label, running) {
  element.textContent = running ? `${label} 运行中` : `${label} 未启动`;
  element.classList.toggle('running', Boolean(running));
}

function buildVideoUrl(videoPath) {
  const path = videoPath || '/stream?topic=/camera/color/image_raw&type=mjpeg';
  return `http://${window.location.hostname}:8080${path}${path.includes('?') ? '&' : '?'}_=${Date.now()}`;
}

function buildVideoHomeUrl() {
  return `http://${window.location.hostname}:8080/`;
}

function setVideo(data, force = false) {
  const topic = data.video_topic || '/camera/color/image_raw';
  const videoPath = data.video_url || `/stream?topic=${topic}&type=mjpeg`;
  const href = buildVideoUrl(videoPath);

  els.videoTopic.textContent = topic;
  els.videoLink.href = href;
  els.videoHomeLink.href = buildVideoHomeUrl();
  els.videoHint.textContent = data.stream_hint || '如果画面不显示，点击视频直链排查。';

  if (!force && !state.videoFailed && topic === state.videoTopic && videoPath === state.videoUrl && els.videoStream.src) {
    return;
  }

  state.videoTopic = topic;
  state.videoUrl = videoPath;
  state.videoFailed = false;
  els.videoError.textContent = '视频加载中';
  els.videoError.classList.remove('hidden');
  els.videoStream.src = href;
}

function updateButtons(processes) {
  const coreRunning = processes.core && processes.core.running;
  const reportRunning = processes.report && processes.report.running;
  const notifyRunning = processes.notify && processes.notify.running;

  els.coreStopBtn.disabled = !coreRunning;
  els.notifyStartBtn.disabled = !reportRunning || notifyRunning;
  els.notifyStopBtn.disabled = !notifyRunning;
  els.reportTextBtn.disabled = !reportRunning;
  els.reportVisionBtn.disabled = !reportRunning;
  els.allStopBtn.disabled = !coreRunning && !reportRunning && !notifyRunning;
  els.chassisResetBtn.disabled = !coreRunning;
  els.cameraLeftBtn.disabled = !coreRunning;
  els.cameraRightBtn.disabled = !coreRunning;
  els.servoResetBtn.disabled = !coreRunning;
  const followRunning = !!(processes.follow && processes.follow.running);
  [els.patrolRelockBtn, els.patrolFollowBtn, els.patrolTrackBtn, els.patrolStopBtn]
    .forEach((b) => { if (b) b.disabled = !followRunning; });
  // 手势控制:独立叠加开关,任意时刻可开(会自动拉起 core/follow)
  const gestureRunning = !!(processes.gesture && processes.gesture.running);
  if (els.gestureToggleBtn) {
    els.gestureToggleBtn.textContent = gestureRunning ? '停止手势控制' : '开启手势控制';
    els.gestureToggleBtn.classList.toggle('primary', !gestureRunning);
    els.gestureToggleBtn.classList.toggle('danger', gestureRunning);
  }
  updateFeatureDetailButtons();
}

function renderVoiceState(data) {
  const ttsOn = !!data.tts_on;
  const asrOn = !!data.asr_on;
  if (els.voiceTtsToggleBtn) {
    els.voiceTtsToggleBtn.textContent = ttsOn ? '停止语音播报' : '开启语音播报';
    els.voiceTtsToggleBtn.classList.toggle('primary', !ttsOn);
    els.voiceTtsToggleBtn.classList.toggle('danger', ttsOn);
  }
  if (els.voiceAsrToggleBtn) {
    els.voiceAsrToggleBtn.textContent = asrOn ? '停止语音触发' : '开启语音触发';
    els.voiceAsrToggleBtn.classList.toggle('primary', !asrOn);
    els.voiceAsrToggleBtn.classList.toggle('danger', asrOn);
  }
  if (els.voiceState) {
    els.voiceState.textContent = `语音:播报${ttsOn ? '开' : '关'} / 触发${asrOn ? '开' : '关'}`;
  }
}

function renderBattery(battery) {
  if (!els.batteryBadge) return;
  const el = els.batteryBadge;
  el.classList.remove('battery-ok', 'battery-low', 'battery-critical', 'battery-stale');
  if (!battery || !battery.available) {
    el.textContent = '电量 --';
    el.classList.add('battery-stale');
    return;
  }
  el.textContent = `电量 ${battery.percent}% (${battery.voltage}V)`;
  if (!battery.fresh) {
    el.classList.add('battery-stale');
  } else if (battery.level === 'critical') {
    el.classList.add('battery-critical');
  } else if (battery.level === 'low') {
    el.classList.add('battery-low');
  } else {
    el.classList.add('battery-ok');
  }
}

function renderPatrolState(raw) {
  if (!els.patrolState) return;
  if (!raw) {
    els.patrolState.textContent = '巡逻:未运行';
    if (els.followSpeed) els.followSpeed.textContent = '跟随速度:-';
    return;
  }
  try {
    const s = JSON.parse(raw);
    const lock = s.locked ? (s.target_present ? '已锁定' : '目标丢失') : '搜索中';
    const front = (s.front_min === null || s.front_min === undefined) ? '-' : (s.front_min + 'm');
    els.patrolState.textContent = `巡逻:${s.mode || '-'} / ${s.running ? '运行' : '停止'} / ${lock} / 前方${front}`;
    if (els.followSpeed) {
      els.followSpeed.textContent = (s.max_lin === undefined || s.max_lin === null)
        ? '跟随速度:-' : `跟随速度:${s.max_lin} m/s`;
    }
  } catch (e) {
    els.patrolState.textContent = '巡逻:' + raw;
  }
}

function renderGestureState(raw) {
  if (!els.gestureState) return;
  if (!raw) {
    els.gestureState.textContent = '手势:未运行';
    return;
  }
  try {
    const s = JSON.parse(raw);
    if (s.state !== undefined) {
      // v2 状态机:sleep/armed/confirm
      const stMap = { armed: '已唤醒', confirm: '已触发', sleep: '休眠(挥手唤醒)' };
      const st = stMap[s.state] || s.state;
      const act = s.action ? ` 动作:${s.action}` : '';
      const last = s.last_cmd ? ` 最近:${s.last_cmd}` : '';
      els.gestureState.textContent = `手势:${st} / ${s.hands || 0}手[${s.statics || '-'}]${act}${last}`;
    } else {
      // v1 兼容
      const g = s.gesture && s.gesture !== 'none' ? s.gesture : '-';
      const fired = s.fired ? ` 触发→${s.fired}` : '';
      const last = s.last_cmd ? ` 最近:${s.last_cmd}` : '';
      els.gestureState.textContent = `手势:${g}${fired}${last}`;
    }
  } catch (e) {
    els.gestureState.textContent = '手势:' + raw;
  }
}

function setPoseMode(on) {
  const nm = state.navMap;
  if (!nm) { return; }
  // 先清掉巡航高亮(不动 clickMode),再设本模式,避免互斥 helper 把 clickMode 覆盖回 goto
  if (on && els.cruiseModeBtn) els.cruiseModeBtn.classList.remove('active');
  nm.clickMode = on ? 'setpose' : 'goto';
  if (els.setPoseBtn) els.setPoseBtn.classList.toggle('active', on);
  setStatus(on ? '设初始位姿:在车的真实位置按住并拖出朝向' : '导航模式:点地图设目标', false);
}

function cruiseMode(on) {
  const nm = state.navMap;
  if (!nm) { return; }
  if (on && els.setPoseBtn) els.setPoseBtn.classList.remove('active');
  nm.clickMode = on ? 'cruise' : 'goto';
  if (els.cruiseModeBtn) els.cruiseModeBtn.classList.toggle('active', on);
  setStatus(on ? '巡航打点:逐个点地图排路线,再"开始巡航"' : '导航模式:点地图设目标', false);
}

function updateCruiseCount() {
  const n = (state.navMap && state.navMap.waypoints) ? state.navMap.waypoints.length : 0;
  if (els.cruiseCount) els.cruiseCount.textContent = String(n);
}

function ensureNavMap() {
  if (state.navMap || !window.NavMap || !els.navCanvas) { return state.navMap; }
  const nm = new NavMap(els.navCanvas, {});
  nm.onStatus = (text, bad) => {
    if (els.navState && state.uiMode === 'nav') {
      els.navState.textContent = `地图:${text}`;
    }
  };
  nm.onClickWorld = (x, y) => {
    if (state.uiMode !== 'nav') { return; }
    if (nm.clickMode === 'cruise') {
      nm.waypoints.push({ x, y });
      nm.render();
      updateCruiseCount();
      setStatus(`巡航点 ${nm.waypoints.length}:(${x.toFixed(2)}, ${y.toFixed(2)})`, false);
      return;
    }
    postAndRefresh(api.navGoto, { x, y, yaw: 0 });
    setStatus(`导航目标:x=${x.toFixed(2)} y=${y.toFixed(2)}`, false);
  };
  nm.onPoseWorld = (x, y, yaw) => {
    if (state.uiMode !== 'nav') { return; }
    const deg = (yaw * 180 / Math.PI).toFixed(0);
    if (nm.clickMode === 'setpose') {
      postAndRefresh(api.navInitialPose, { x, y, yaw });
      setStatus(`设初始位姿:(${x.toFixed(2)}, ${y.toFixed(2)}) ${deg}°`, false);
      setPoseMode(false);  // 设完自动退出,回到导航点击
    } else if (nm.clickMode === 'cruise') {
      nm.waypoints.push({ x, y, yaw });
      nm.render();
      updateCruiseCount();
      setStatus(`巡航点 ${nm.waypoints.length}:(${x.toFixed(2)}, ${y.toFixed(2)}) ${deg}°`, false);
    } else {
      postAndRefresh(api.navGoto, { x, y, yaw });
      setStatus(`导航目标:(${x.toFixed(2)}, ${y.toFixed(2)}) ${deg}°`, false);
    }
  };
  nm.bindClick();
  state.navMap = nm;
  window.addEventListener('resize', () => {
    if (state.uiMode === 'nav' && state.navMap) {
      if (state.navMap.resize()) { state.navMap.render(); }
    }
  });
  return nm;
}

function applyUiMode(mode) {
  state.uiMode = mode === 'nav' ? 'nav' : 'follow';
  const isNav = state.uiMode === 'nav';
  document.body.classList.toggle('nav-ui', isNav);  // 控制 nav 模式专属布局(隐藏事件栏等)
  if (els.followGroup) els.followGroup.classList.toggle('hidden', isNav);
  if (els.navGroup) els.navGroup.classList.toggle('hidden', !isNav);
  if (els.uiModeFollowBtn) els.uiModeFollowBtn.classList.toggle('active', !isNav);
  if (els.uiModeNavBtn) els.uiModeNavBtn.classList.toggle('active', isNav);
  // 画布与视频互斥显示
  if (els.navCanvas) els.navCanvas.classList.toggle('hidden', !isNav);
  if (els.videoStream) els.videoStream.classList.toggle('hidden', isNav);
  // nav 模式隐藏"视频未加载"覆盖层(不属于建图画面)
  if (els.videoError) els.videoError.classList.toggle('hidden', isNav);
  if (isNav) {
    // 停止视频加载:清空 src,否则旧流报错会异步重弹"视频未加载",
    // 且可见的 <img> 会与 canvas 在 flex 容器里并排挤成两半。
    if (els.videoStream) els.videoStream.removeAttribute('src');
    state.videoUrl = '';
    state.videoTopic = '';
    state.videoFailed = false;
    const nm = ensureNavMap();
    if (nm && !nm.connected) {
      nm.connect(`ws://${location.hostname}:9090`);
    }
    // canvas 此刻刚显示,缓冲尺寸需同步到实际像素再渲染
    if (nm) {
      requestAnimationFrame(() => { nm.resize(); nm.render(); });
      nm.render();
    }
  } else if (state.navMap) {
    state.navMap.disconnect();
  }
}

function renderUiMode(data) {
  const mode = data.ui_mode || 'follow';
  // 切换进行中(pendingUiMode):忽略后端尚未更新的旧 ui_mode,避免乐观 UI 被轮询打回造成跳变。
  // 后端报到目标模式后清除挂起标记。
  if (state.pendingUiMode) {
    if (mode === state.pendingUiMode) { state.pendingUiMode = null; }
    else { return; }
  }
  if (mode !== state.uiMode) {
    applyUiMode(mode);
  }
  if (els.navState) {
    const ns = data.nav_state || 'idle';
    const label = {
      idle: '空闲', mapping: '建图中', exploring: '自动探索中',
      localizing: '定位中', navigating: '导航中', cruising: '巡航中',
    }[ns] || ns;
    const mapPart = data.current_map ? ` · 地图:${data.current_map}` : '';
    els.navState.textContent = `导航:${label}${mapPart}${data.nav_feedback ? ' · ' + data.nav_feedback : ''}`;
  }
  if (els.navSpeed) {
    const sc = data.nav_speed_scale;
    els.navSpeed.textContent = (sc === undefined || sc === null)
      ? '导航限速:-' : `导航限速:×${Number(sc).toFixed(1)} (${(0.18 * sc).toFixed(2)} m/s)`;
  }
}

async function setUiMode(mode) {
  // 乐观切 UI,后端编排(停跟随/起导航栈)可能耗时。pendingUiMode 防止这期间
  // 1.5s 轮询拿到后端旧 ui_mode 把 UI 打回(跳变 bug)。
  state.pendingUiMode = mode;
  applyUiMode(mode);
  try {
    await request(api.uimode, { method: 'POST', body: { mode } });
    await refreshStatus();
    if (mode === 'nav') { refreshMapList(); refreshPointList(); }
  } catch (error) {
    state.pendingUiMode = null;
    setStatus(`切换模式失败：${error.message}`, true);
  }
}

async function refreshMapList() {
  if (!els.mapList) { return; }
  try {
    const data = await request(api.mapsList);
    renderMapList(data.maps || [], data.current_map || '');
  } catch (error) {
    els.mapList.innerHTML = `<div class="empty-state">地图列表读取失败</div>`;
  }
}

function renderMapList(maps, current) {
  if (!maps.length) {
    els.mapList.innerHTML = `<div class="empty-state">暂无已存地图</div>`;
    return;
  }
  els.mapList.innerHTML = maps.map((name) => {
    const active = name === current ? ' active' : '';
    return `<button class="map-item${active}" data-map="${escapeHtml(name)}">${escapeHtml(name)}</button>`;
  }).join('');
  els.mapList.querySelectorAll('.map-item').forEach((btn) => {
    btn.addEventListener('click', () => loadMap(btn.dataset.map));
  });
}

async function loadMap(name) {
  if (!confirm(`加载地图「${name}」并进入定位导航?`)) { return; }
  try {
    await request(api.mapLoad, { method: 'POST', body: { name } });
    await refreshStatus();
    refreshMapList();
    refreshPointList();
  } catch (error) {
    setStatus(`加载地图失败：${error.message}`, true);
  }
}

async function refreshPointList() {
  if (!els.pointList) { return; }
  try {
    const data = await request(api.pointsList);
    renderPointList(data.points || []);
  } catch (error) {
    els.pointList.innerHTML = '';
  }
}

function renderPointList(points) {
  if (!points.length) {
    els.pointList.innerHTML = `<li class="empty-state">暂无命名点</li>`;
    return;
  }
  els.pointList.innerHTML = points.map((p) => {
    const n = escapeHtml(p.name);
    return `<li><span class="point-name" data-pt="${n}" title="点此前往 ${n}">${n}</span>`
      + `<button class="point-del" data-pt="${n}" title="删除">×</button></li>`;
  }).join('');
  els.pointList.querySelectorAll('.point-name').forEach((el) => {
    el.addEventListener('click', () => gotoPoint(el.dataset.pt));
  });
  els.pointList.querySelectorAll('.point-del').forEach((el) => {
    el.addEventListener('click', () => deletePoint(el.dataset.pt));
  });
}

async function savePoint() {
  const name = (els.pointNameInput.value || '').trim();
  if (!name) { setStatus('请先输入点名', true); return; }
  try {
    const data = await request(api.pointSave, { method: 'POST', body: { name } });
    els.pointNameInput.value = '';
    renderPointList(data.points || []);
    setStatus(`已存点:${name}`, false);
  } catch (error) {
    setStatus(`存点失败：${error.message}`, true);
  }
}

async function gotoPoint(name) {
  try {
    await request(api.pointGoto, { method: 'POST', body: { name } });
    await refreshStatus();
    setStatus(`前往 ${name}`, false);
  } catch (error) {
    setStatus(`前往失败：${error.message}`, true);
  }
}

async function deletePoint(name) {
  if (!confirm(`删除命名点「${name}」?`)) { return; }
  try {
    const data = await request(api.pointDelete, { method: 'POST', body: { name } });
    renderPointList(data.points || []);
  } catch (error) {
    setStatus(`删除失败：${error.message}`, true);
  }
}

async function saveMap() {
  const name = (els.mapNameInput.value || '').trim();
  try {
    const data = await request(api.mapSave, { method: 'POST', body: { name } });
    setStatus(`地图已保存：${data.name}`, false);
    renderMapList(data.maps || [], data.name);
  } catch (error) {
    setStatus(`保存地图失败：${error.message}`, true);
  }
}

async function refreshStatus() {
  try {
    const data = await request(api.status);
    const processes = data.processes || {};
    setBadge(els.coreBadge, 'core', processes.core && processes.core.running);
    setBadge(els.reportBadge, 'report', processes.report && processes.report.running);
    setBadge(els.notifyBadge, 'notify', processes.notify && processes.notify.running);
    state.activeFeature = data.active_feature || '';
    if (!state.manualTopic && state.uiMode !== 'nav') {
      setVideo(data);
    }
    updateButtons(processes);
    updateFeatureCards();
    renderSelectedFeatureDetail();
    const modeText = data.mode === 'report' ? '识别记录模式' : data.mode === 'manual' ? '普通操作模式' : '空闲';
    setStatus(`${modeText}，事件 ${data.events_count || 0} 条${data.last_error ? '，错误：' + data.last_error : ''}`, Boolean(data.last_error));
    renderReportJob(data.report_job);
    renderPatrolState(data.patrol_state);
    renderGestureState(data.gesture_state);
    renderBattery(data.battery);
    renderVoiceState(data);
    renderUiMode(data);
  } catch (error) {
    setStatus(`后端连接失败：${error.message}`, true);
  }
}

async function refreshEvents() {
  try {
    const data = await request(api.events);
    renderEvents(data.events || []);
  } catch (error) {
    els.eventsList.innerHTML = `<div class="empty-state">事件读取失败：${escapeHtml(error.message)}</div>`;
  }
}

async function loadFeatures() {
  if (state.featuresLoaded) {
    return;
  }
  try {
    const data = await request(api.features);
    state.features = data.features || [];
    state.activeFeature = data.active_feature || state.activeFeature;
    renderFeatures(state.features);
    state.featuresLoaded = true;
  } catch (error) {
    els.featureGrid.innerHTML = `<div class="empty-state">功能列表读取失败：${escapeHtml(error.message)}</div>`;
  }
}

function renderFeatures(features) {
  if (!features.length) {
    els.featureGrid.innerHTML = '<div class="empty-state">暂无功能</div>';
    return;
  }

  els.featureGrid.innerHTML = features.map((feature) => `
    <button class="feature-card${feature.id === state.activeFeature ? ' active' : ''}${feature.available ? '' : ' disabled'}" data-feature-id="${escapeHtml(feature.id)}">
      <strong>${escapeHtml(feature.name)}</strong>
      <span>${escapeHtml(feature.description || '')}</span>
      <em>${feature.id === state.activeFeature ? '运行中' : feature.available ? '可进入' : '预留'}</em>
    </button>
  `).join('');

  els.featureGrid.querySelectorAll('.feature-card').forEach((card) => {
    card.addEventListener('click', () => showFeatureDetail(card.dataset.featureId));
  });
}

function updateFeatureCards() {
  els.featureGrid.querySelectorAll('.feature-card').forEach((card) => {
    const feature = getFeature(card.dataset.featureId);
    const active = feature && feature.id === state.activeFeature;
    card.classList.toggle('active', Boolean(active));
    const status = card.querySelector('em');
    if (feature && status) {
      status.textContent = active ? '运行中' : feature.available ? '可进入' : '预留';
    }
  });
}

function getFeature(id) {
  return state.features.find((feature) => feature.id === id);
}

function showFeatureList() {
  state.selectedFeatureId = '';
  els.featureListView.classList.remove('hidden');
  els.featureDetailView.classList.add('hidden');
}

function showFeatureDetail(id) {
  state.selectedFeatureId = id;
  els.featureListView.classList.add('hidden');
  els.featureDetailView.classList.remove('hidden');
  renderSelectedFeatureDetail();
}

function renderSelectedFeatureDetail() {
  if (!state.selectedFeatureId || els.featureDetailView.classList.contains('hidden')) {
    return;
  }

  const feature = getFeature(state.selectedFeatureId);
  if (!feature) {
    els.featureDetailName.textContent = '功能详情';
    els.featureDetailDesc.textContent = '未找到该功能。';
    els.featureDetailStatus.textContent = '';
    updateFeatureDetailButtons();
    return;
  }

  const active = feature.id === state.activeFeature;
  els.featureDetailName.textContent = feature.name;
  els.featureDetailDesc.textContent = feature.description || '';
  els.featureDetailStatus.textContent = active ? '状态：运行中' : feature.available ? '状态：未启动' : '状态：预留入口，暂未接入启动逻辑';
  updateFeatureDetailButtons();
}

function updateFeatureDetailButtons() {
  const feature = getFeature(state.selectedFeatureId);
  const active = feature && feature.id === state.activeFeature;
  const available = feature && feature.available;
  els.featureStartBtn.disabled = !available || active;
  els.featureStopBtn.disabled = !active;
}

async function startFeature(id) {
  try {
    await request(api.featureStart, { method: 'POST', body: { id } });
    await refreshStatus();
    await refreshEvents();
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function stopFeature() {
  try {
    await request(api.featureStop, { method: 'POST' });
    await refreshStatus();
    await refreshEvents();
  } catch (error) {
    setStatus(error.message, true);
  }
}

function renderEvents(events) {
  els.eventCount.textContent = `${events.length} 条`;
  if (!events.length) {
    els.eventsList.innerHTML = '<div class="empty-state">暂无事件</div>';
    return;
  }

  els.eventsList.innerHTML = events.map((event) => {
    const name = event.event_name || event.class_name || event.event_type || '事件';
    const target = event.class_name ? `目标: ${event.class_name}` : '';
    const risk = event.risk_level ? `风险: ${event.risk_level}` : '';
    const area = event.area ? `区域: ${event.area}` : '';
    const action = event.action ? `建议: ${event.action}` : '';
    const score = typeof event.score === 'number' ? event.score.toFixed(2) : '未记录';
    const time = event.time || '';
    const image = event.image_path || '无截图';
    const bbox = Array.isArray(event.bbox) ? event.bbox.join(', ') : '无';
    const tags = [target, risk, area].filter(Boolean).join(' | ');
    return `
      <div class="event-item">
        <div class="event-main">
          <strong>${escapeHtml(name)}</strong>
          <span>${escapeHtml(score)}</span>
        </div>
        ${tags ? `<div class="event-meta">${escapeHtml(tags)}</div>` : ''}
        <div class="event-meta">${escapeHtml(time)} | bbox: ${escapeHtml(bbox)}</div>
        ${action ? `<div class="event-meta">${escapeHtml(action)}</div>` : ''}
        <div class="event-meta">${escapeHtml(image)}</div>
      </div>
    `;
  }).join('');
}

function renderReportJob(job) {
  if (!job) {
    els.reportJob.textContent = '报告任务：空闲';
    return;
  }
  if (job.running) {
    els.reportJob.textContent = `报告任务：${job.mode} 生成中`;
    return;
  }
  const result = job.returncode === 0 ? '完成' : '失败';
  const output = job.stdout || job.stderr || '';
  els.reportJob.textContent = `报告任务：${result}${output ? '，' + output.trim().slice(-120) : ''}`;
}

async function postAndRefresh(path, body) {
  try {
    await request(path, { method: 'POST', body });
    await refreshStatus();
    await refreshEvents();
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function postAction(path, label) {
  try {
    const data = await request(path, { method: 'POST' });
    setStatus(data.message || `${label}已发送`);
    await refreshStatus();
  } catch (error) {
    setStatus(error.message, true);
  }
}

async function postPatrolCmd(cmd, label) {
  try {
    await request(api.patrolCommand, { method: 'POST', body: { cmd } });
    setStatus(`${label}已发送`);
    await refreshStatus();
  } catch (error) {
    setStatus(error.message, true);
  }
}

function startDrive(linear, angular, lateral = 0) {
  state.activeDrive = { linear, angular, lateral };
  if (!state.driveTimer) {
    sendDrive();
    state.driveTimer = setInterval(sendDrive, 160);
  }
}

function stopDriveTimer() {
  if (state.driveTimer) {
    clearInterval(state.driveTimer);
    state.driveTimer = null;
  }
}

async function sendDrive() {
  if (!state.activeDrive) {
    return;
  }
  try {
    await request(api.drive, { method: 'POST', body: state.activeDrive });
  } catch (error) {
    setStatus(`遥控失败：${error.message}`, true);
    stopDriveTimer();
  }
}

async function stopDrive() {
  stopDriveTimer();
  state.activeDrive = null;
  try {
    await request(api.drive, { method: 'POST', body: { linear: 0, angular: 0, lateral: 0 } });
  } catch (error) {
    setStatus(`停止失败：${error.message}`, true);
  }
}

// 麦轮摇杆:上下=前进/后退(linear.x),左右按模式=转向(angular.z)或横移(linear.y)。
const JOY_MAX_LINEAR = 0.2;
const JOY_MAX_ANGULAR = 1.0;
const JOY_MAX_LATERAL = 0.2;

function initJoystick(wrap) {
  const pad = wrap.querySelector('.joystick-pad');
  const knob = wrap.querySelector('.joystick-knob');
  const speedInput = wrap.querySelector('[data-jspeed]');
  const speedLabel = wrap.querySelector('[data-jspeed-label]');
  const modeBtns = wrap.querySelectorAll('[data-jmode]');
  const ctx = { mode: 'steer', active: false };

  modeBtns.forEach((btn) => btn.addEventListener('click', () => {
    ctx.mode = btn.dataset.jmode;
    modeBtns.forEach((b) => b.classList.toggle('primary', b === btn));
  }));
  if (speedInput && speedLabel) {
    speedInput.addEventListener('input', () => { speedLabel.textContent = `${speedInput.value}%`; });
  }

  const send = (nx, ny) => {
    const speed = (speedInput ? Number(speedInput.value) : 50) / 100;
    const linear = -ny * JOY_MAX_LINEAR * speed;     // 上(屏幕-y)=前进
    const side = -nx * speed;                        // 左(屏幕-x)=左转/左移(ROS +)
    if (ctx.mode === 'strafe') startDrive(linear, 0, side * JOY_MAX_LATERAL);
    else startDrive(linear, side * JOY_MAX_ANGULAR, 0);
  };

  const onMove = (event) => {
    if (!ctx.active) return;
    const rect = pad.getBoundingClientRect();
    const r = rect.width / 2;
    let dx = event.clientX - (rect.left + r);
    let dy = event.clientY - (rect.top + r);
    const dist = Math.hypot(dx, dy);
    if (dist > r) { dx *= r / dist; dy *= r / dist; }
    knob.style.transform = `translate(${dx}px, ${dy}px)`;
    send(dx / r, dy / r);
  };
  const release = () => {
    if (!ctx.active) return;
    ctx.active = false;
    knob.style.transform = 'translate(0, 0)';
    stopDrive();
  };
  pad.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    ctx.active = true;
    pad.setPointerCapture(event.pointerId);
    onMove(event);
  });
  pad.addEventListener('pointermove', onMove);
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
}

function escapeHtml(value) {
  return String(value)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#39;');
}

function bindControls() {
  els.manualBtn.addEventListener('click', () => postAndRefresh(api.manual));
  if (els.uiModeFollowBtn) els.uiModeFollowBtn.addEventListener('click', () => setUiMode('follow'));
  if (els.uiModeNavBtn) els.uiModeNavBtn.addEventListener('click', () => setUiMode('nav'));
  if (els.navStopBtn) els.navStopBtn.addEventListener('click', () => postAndRefresh(api.navStop));
  if (els.navHomeBtn) els.navHomeBtn.addEventListener('click', () => postAndRefresh(api.navHome));
  if (els.setPoseBtn) els.setPoseBtn.addEventListener('click', () => { ensureNavMap(); setPoseMode(state.navMap && state.navMap.clickMode !== 'setpose'); });
  if (els.navFasterBtn) els.navFasterBtn.addEventListener('click', () => postAndRefresh(api.navSpeed, { action: 'up' }));
  if (els.navSlowerBtn) els.navSlowerBtn.addEventListener('click', () => postAndRefresh(api.navSpeed, { action: 'down' }));
  if (els.cruiseModeBtn) els.cruiseModeBtn.addEventListener('click', () => { ensureNavMap(); cruiseMode(state.navMap && state.navMap.clickMode !== 'cruise'); });
  if (els.cruiseClearBtn) els.cruiseClearBtn.addEventListener('click', () => {
    const nm = ensureNavMap();
    if (nm) { nm.waypoints = []; nm.render(); updateCruiseCount(); setStatus('已清空巡航点', false); }
  });
  if (els.cruiseStartBtn) els.cruiseStartBtn.addEventListener('click', async () => {
    const nm = ensureNavMap();
    const pts = (nm && nm.waypoints) ? nm.waypoints.slice() : [];
    if (!pts.length) { setStatus('请先在地图上点几个巡航点', true); return; }
    cruiseMode(false);
    try {
      await request(api.navCruise, { method: 'POST', body: { points: pts } });
      await refreshStatus();
      setStatus(`开始巡航 ${pts.length} 点`, false);
    } catch (error) {
      setStatus(`巡航失败:${error.message}`, true);
    }
  });
  if (els.navCancelBtn) els.navCancelBtn.addEventListener('click', () => postAndRefresh(api.navCancel));
  if (els.mapManualBtn) els.mapManualBtn.addEventListener('click', () => postAndRefresh(api.mappingStart, { explore: false }));
  if (els.mapExploreBtn) els.mapExploreBtn.addEventListener('click', () => postAndRefresh(api.mappingStart, { explore: true }));
  if (els.exploreStopBtn) els.exploreStopBtn.addEventListener('click', () => postAndRefresh(api.exploreStop));
  if (els.mapSaveBtn) els.mapSaveBtn.addEventListener('click', () => saveMap());
  if (els.mapRefreshBtn) els.mapRefreshBtn.addEventListener('click', () => refreshMapList());
  if (els.pointSaveBtn) els.pointSaveBtn.addEventListener('click', () => savePoint());
  if (els.pointNameInput) els.pointNameInput.addEventListener('keydown', (e) => { if (e.key === 'Enter') savePoint(); });
  els.coreStopBtn.addEventListener('click', () => postAndRefresh(api.coreStop));
  els.allStopBtn.addEventListener('click', () => postAndRefresh(api.allStop));
  els.notifyStartBtn.addEventListener('click', () => postAndRefresh(api.notifyStart));
  els.notifyStopBtn.addEventListener('click', () => postAndRefresh(api.notifyStop));
  els.reportTextBtn.addEventListener('click', () => postAndRefresh(api.generateReport, { mode: 'text' }));
  els.reportVisionBtn.addEventListener('click', () => postAndRefresh(api.generateReport, { mode: 'vision', max_images: 3 }));
  els.featureToggleBtn.addEventListener('click', async () => {
    els.featurePanel.classList.toggle('hidden');
    if (!els.featurePanel.classList.contains('hidden')) {
      showFeatureList();
    }
    await loadFeatures();
  });
  els.featureCloseBtn.addEventListener('click', () => els.featurePanel.classList.add('hidden'));
  if (els.eventsToggleBtn) {
    els.eventsToggleBtn.addEventListener('click', () => {
      const collapsed = document.body.classList.toggle('events-collapsed');
      els.eventsToggleBtn.textContent = collapsed ? '展开' : '收起';
    });
  }
  els.featureBackBtn.addEventListener('click', showFeatureList);
  els.featureStartBtn.addEventListener('click', () => startFeature(state.selectedFeatureId));
  els.featureStopBtn.addEventListener('click', stopFeature);
  els.chassisResetBtn.addEventListener('click', () => postAction(api.chassisReset, '底盘复位'));
  if (els.streamSelect) {
    els.streamSelect.addEventListener('change', () => {
      const topic = els.streamSelect.value;
      if (!topic) {
        state.manualTopic = null;          // 回到自动:下次轮询用后端 topic
        return;
      }
      state.manualTopic = topic;           // 锁定手动选择
      setVideo({ video_topic: topic, video_url: `/stream?topic=${topic}&type=mjpeg` }, true);
    });
  }
  els.cameraLeftBtn.addEventListener('click', () => postAction(api.cameraLeft, '摄像头左看'));
  els.cameraRightBtn.addEventListener('click', () => postAction(api.cameraRight, '摄像头右看'));
  els.servoResetBtn.addEventListener('click', () => postAction(api.servoReset, '云台回中'));
  if (els.patrolRelockBtn) {
    els.patrolRelockBtn.addEventListener('click', () => postAction(api.patrolRelock, '重新锁定'));
  }
  if (els.patrolFollowBtn) {
    els.patrolFollowBtn.addEventListener('click', () => postPatrolCmd('follow', '智能跟随'));
  }
  if (els.patrolTrackBtn) {
    els.patrolTrackBtn.addEventListener('click', () => postPatrolCmd('track_only', '云台追踪'));
  }
  if (els.patrolStopBtn) {
    els.patrolStopBtn.addEventListener('click', () => postPatrolCmd('stop', '停止跟随'));
  }
  if (els.followFasterBtn) {
    els.followFasterBtn.addEventListener('click', () => postAndRefresh(api.patrolSpeed, { action: 'up' }));
  }
  if (els.followSlowerBtn) {
    els.followSlowerBtn.addEventListener('click', () => postAndRefresh(api.patrolSpeed, { action: 'down' }));
  }
  if (els.gestureToggleBtn) {
    els.gestureToggleBtn.addEventListener('click', () => {
      const running = els.gestureToggleBtn.textContent.includes('停止');
      postAndRefresh(running ? api.gestureStop : api.gestureStart);
    });
  }
  if (els.voiceTtsToggleBtn) {
    els.voiceTtsToggleBtn.addEventListener('click', () => {
      const on = els.voiceTtsToggleBtn.textContent.includes('停止');
      postAndRefresh(on ? api.voiceTtsStop : api.voiceTtsStart);
    });
  }
  if (els.voiceAsrToggleBtn) {
    els.voiceAsrToggleBtn.addEventListener('click', () => {
      const on = els.voiceAsrToggleBtn.textContent.includes('停止');
      postAndRefresh(on ? api.voiceAsrStop : api.voiceAsrStart);
    });
  }
  document.querySelectorAll('[data-joystick]').forEach(initJoystick);

  els.videoStream.addEventListener('load', () => {
    if (state.uiMode === 'nav') return;
    state.videoFailed = false;
    els.videoError.classList.add('hidden');
  });
  els.videoStream.addEventListener('error', () => {
    if (state.uiMode === 'nav') return;
    state.videoFailed = true;
    els.videoError.textContent = '视频未加载，点击右上角视频直链排查';
    els.videoError.classList.remove('hidden');
  });

  window.addEventListener('blur', stopDrive);
  window.addEventListener('beforeunload', () => {
    navigator.sendBeacon(api.drive, JSON.stringify({ linear: 0, angular: 0, lateral: 0 }));
  });
}

function startPolling() {
  refreshStatus().then(() => setVideo({ video_topic: state.videoTopic || '/camera/color/image_raw', video_url: state.videoUrl }, true));
  refreshEvents();
  setInterval(refreshStatus, 1500);
  setInterval(refreshEvents, 2000);
}

bindControls();
startPolling();
