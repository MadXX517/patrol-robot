const api = {
  status: '/api/status',
  events: '/api/events',
  features: '/api/features',
  featureStart: '/api/features/start',
  featureStop: '/api/features/stop',
  reportJob: '/api/report/job',
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
  cameraUp: '/api/servo/camera_up',
  cameraDown: '/api/servo/camera_down',
  patrolRelock: '/api/patrol/relock',
  generateReport: '/api/report/generate',
  voiceStartChat: '/api/voice/start_chat',
  voiceStartControl: '/api/voice/start_control',
  voiceStop: '/api/voice/stop',
  voiceSafeStop: '/api/voice/safe_stop',
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
  activeFeatures: [],
  videoFailed: false,
  voiceWakeWord: '小星',
  voiceConfigured: false,
  voiceMode: '',
};

const els = {
  statusText: document.getElementById('statusText'),
  batteryBadge: document.getElementById('batteryBadge'),
  coreBadge: document.getElementById('coreBadge'),
  reportBadge: document.getElementById('reportBadge'),
  notifyBadge: document.getElementById('notifyBadge'),
  voiceBadge: document.getElementById('voiceBadge'),
  videoTopic: document.getElementById('videoTopic'),
  videoHint: document.getElementById('videoHint'),
  videoLink: document.getElementById('videoLink'),
  videoHomeLink: document.getElementById('videoHomeLink'),
  videoStream: document.getElementById('videoStream'),
  videoError: document.getElementById('videoError'),
  eventsList: document.getElementById('eventsList'),
  eventCount: document.getElementById('eventCount'),
  reportJob: document.getElementById('reportJob'),
  manualBtn: document.getElementById('manualBtn'),
  coreStopBtn: document.getElementById('coreStopBtn'),
  reportBtn: document.getElementById('reportBtn'),
  reportStopBtn: document.getElementById('reportStopBtn'),
  allStopBtn: document.getElementById('allStopBtn'),
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
  featureActionRow: document.getElementById('featureActionRow'),
  featureStartBtn: document.getElementById('featureStartBtn'),
  featureStopBtn: document.getElementById('featureStopBtn'),
  voiceControls: document.getElementById('voiceControls'),
  voiceChatBtn: document.getElementById('voiceChatBtn'),
  voiceControlBtn: document.getElementById('voiceControlBtn'),
  voiceStopBtn: document.getElementById('voiceStopBtn'),
  voiceSafeStopBtn: document.getElementById('voiceSafeStopBtn'),
  voiceExample: document.getElementById('voiceExample'),
  cameraLeftBtn: document.getElementById('cameraLeftBtn'),
  cameraRightBtn: document.getElementById('cameraRightBtn'),
  cameraUpBtn: document.getElementById('cameraUpBtn'),
  cameraDownBtn: document.getElementById('cameraDownBtn'),
  chassisResetBtn: document.getElementById('chassisResetBtn'),
  servoResetBtn: document.getElementById('servoResetBtn'),
  patrolRelockBtn: document.getElementById('patrolRelockBtn'),
  patrolState: document.getElementById('patrolState'),
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
  const voiceRunning = processes.voice && processes.voice.running;
  const voiceControlRunning = voiceRunning && state.voiceMode === 'control';

  els.coreStopBtn.disabled = !coreRunning;
  els.reportBtn.disabled = reportRunning;
  els.reportStopBtn.disabled = !reportRunning;
  els.notifyStartBtn.disabled = !reportRunning || notifyRunning;
  els.notifyStopBtn.disabled = !notifyRunning;
  els.reportTextBtn.disabled = !reportRunning;
  els.reportVisionBtn.disabled = !reportRunning;
  const followRunning = processes.follow && processes.follow.running;
  els.allStopBtn.disabled = !coreRunning && !reportRunning && !notifyRunning && !voiceRunning && !followRunning;
  els.chassisResetBtn.disabled = !coreRunning;
  const controlBusy = voiceControlRunning || followRunning;
  els.cameraLeftBtn.disabled = !coreRunning || controlBusy;
  els.cameraRightBtn.disabled = !coreRunning || controlBusy;
  els.cameraUpBtn.disabled = !coreRunning || controlBusy;
  els.cameraDownBtn.disabled = !coreRunning || controlBusy;
  els.servoResetBtn.disabled = !coreRunning || controlBusy;
  if (els.patrolRelockBtn) {
    els.patrolRelockBtn.disabled = !followRunning;
  }
  document.querySelectorAll('.drive[data-linear]').forEach((button) => {
    button.disabled = !coreRunning || controlBusy;
  });
  updateFeatureDetailButtons();
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
    return;
  }
  try {
    const s = JSON.parse(raw);
    const lock = s.locked ? (s.target_present ? '已锁定' : '目标丢失') : '搜索中';
    const front = (s.front_min === null || s.front_min === undefined) ? '-' : (s.front_min + 'm');
    els.patrolState.textContent = `巡逻:${s.mode || '-'} / ${s.running ? '运行' : '停止'} / ${lock} / 前方${front}`;
  } catch (e) {
    els.patrolState.textContent = '巡逻:' + raw;
  }
}

async function refreshStatus() {
  try {
    const data = await request(api.status);
    const processes = data.processes || {};
    setBadge(els.coreBadge, 'core', processes.core && processes.core.running);
    setBadge(els.reportBadge, 'report', processes.report && processes.report.running);
    setBadge(els.notifyBadge, 'notify', processes.notify && processes.notify.running);
    setBadge(els.voiceBadge, 'voice', processes.voice && processes.voice.running);
    state.activeFeature = data.active_feature || '';
    state.activeFeatures = data.active_features || (state.activeFeature ? [state.activeFeature] : []);
    state.voiceWakeWord = data.voice_wake_word || '小星';
    state.voiceConfigured = Boolean(data.voice_configured);
    state.voiceMode = data.voice_mode || '';
    setVideo(data);
    updateButtons(processes);
    updateFeatureCards();
    renderSelectedFeatureDetail();
    const modeText = buildModeText(data.mode);
    setStatus(`${modeText}，事件 ${data.events_count || 0} 条${data.last_error ? '，错误：' + data.last_error : ''}`, Boolean(data.last_error));
    renderReportJob(data.report_job);
    renderPatrolState(data.patrol_state);
    renderBattery(data.battery);
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
    state.activeFeatures = data.active_features || (state.activeFeature ? [state.activeFeature] : state.activeFeatures);
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
    <button class="feature-card${isFeatureActive(feature.id) ? ' active' : ''}${feature.available ? '' : ' disabled'}" data-feature-id="${escapeHtml(feature.id)}">
      <strong>${escapeHtml(feature.name)}</strong>
      <span>${escapeHtml(feature.description || '')}</span>
      <em>${isFeatureActive(feature.id) ? '运行中' : feature.available ? '可进入' : '预留'}</em>
    </button>
  `).join('');

  els.featureGrid.querySelectorAll('.feature-card').forEach((card) => {
    card.addEventListener('click', () => showFeatureDetail(card.dataset.featureId));
  });
}

function updateFeatureCards() {
  els.featureGrid.querySelectorAll('.feature-card').forEach((card) => {
    const feature = getFeature(card.dataset.featureId);
    const active = feature && isFeatureActive(feature.id);
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

  const active = isFeatureActive(feature.id);
  els.featureDetailName.textContent = feature.name;
  els.featureDetailDesc.textContent = feature.description || '';
  if (feature.id === 'voice_control') {
    const wake = state.voiceWakeWord || '小星';
    const configured = state.voiceConfigured ? '已配置 API Key' : '未配置 API Key';
    const modeText = state.voiceMode === 'control' ? '语音控制小车' : state.voiceMode === 'chat' ? '仅语音对话' : '未运行';
    els.featureDetailStatus.textContent = active
      ? `状态：${modeText}；唤醒词：${wake}。`
      : `状态：未启动；${configured}；启动后先说唤醒词“${wake}”。`;
    els.voiceExample.textContent = `示例：${wake}，自我介绍一下；${wake}，向前移动十厘米。`;
  } else {
    els.featureDetailStatus.textContent = active ? '状态：运行中' : feature.available ? '状态：未启动' : '状态：预留入口，暂未接入启动逻辑';
  }
  updateFeatureDetailButtons();
}

function updateFeatureDetailButtons() {
  const feature = getFeature(state.selectedFeatureId);
  const active = feature && isFeatureActive(feature.id);
  const available = feature && feature.available;
  const isVoice = feature && feature.id === 'voice_control';
  const voiceRunning = isVoice && active;
  const voiceAvailable = Boolean(available && state.voiceConfigured);
  els.featureActionRow.classList.toggle('hidden', Boolean(isVoice));
  els.voiceControls.classList.toggle('hidden', !isVoice);
  els.featureStartBtn.disabled = !available || active;
  els.featureStopBtn.disabled = !active;
  els.voiceChatBtn.disabled = !voiceAvailable || (voiceRunning && state.voiceMode === 'chat');
  els.voiceControlBtn.disabled = !voiceAvailable || (voiceRunning && state.voiceMode === 'control');
  els.voiceStopBtn.disabled = !voiceRunning;
  els.voiceSafeStopBtn.disabled = !voiceRunning;
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
    await request(api.featureStop, { method: 'POST', body: { id: state.selectedFeatureId } });
    await refreshStatus();
    await refreshEvents();
  } catch (error) {
    setStatus(error.message, true);
  }
}

function isFeatureActive(id) {
  return state.activeFeatures.includes(id);
}

function buildModeText(mode) {
  const parts = [];
  if (isFeatureActive('human_follow')) {
    parts.push('人体跟随');
  } else if (isFeatureActive('gimbal_track')) {
    parts.push('云台追踪');
  } else if (mode === 'report' || isFeatureActive('visual_patrol')) {
    parts.push('识别记录');
  } else if (mode === 'manual') {
    parts.push('普通画面');
  }

  if (isFeatureActive('voice_control')) {
    parts.push(state.voiceMode === 'control' ? '语音控制小车' : '仅语音对话');
  }

  return parts.length ? `${parts.join(' + ')}模式` : '空闲';
}

async function startVoice(path) {
  try {
    await request(path, { method: 'POST' });
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
    const name = event.class_name || event.event_type || '事件';
    const score = typeof event.score === 'number' ? event.score.toFixed(2) : '未记录';
    const time = event.time || '';
    const image = event.image_path || '无截图';
    const bbox = Array.isArray(event.bbox) ? event.bbox.join(', ') : '无';
    return `
      <div class="event-item">
        <div class="event-main">
          <strong>${escapeHtml(name)}</strong>
          <span>${escapeHtml(score)}</span>
        </div>
        <div class="event-meta">${escapeHtml(time)} | bbox: ${escapeHtml(bbox)}</div>
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

function startDrive(linear, angular) {
  stopDriveTimer();
  state.activeDrive = { linear, angular };
  sendDrive();
  state.driveTimer = setInterval(sendDrive, 160);
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
    await request(api.drive, { method: 'POST', body: { linear: 0, angular: 0 } });
  } catch (error) {
    setStatus(`停止失败：${error.message}`, true);
  }
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
  els.coreStopBtn.addEventListener('click', () => postAndRefresh(api.coreStop));
  els.reportBtn.addEventListener('click', () => postAndRefresh(api.report));
  els.reportStopBtn.addEventListener('click', () => postAndRefresh(api.reportStop));
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
  els.featureBackBtn.addEventListener('click', showFeatureList);
  els.featureStartBtn.addEventListener('click', () => startFeature(state.selectedFeatureId));
  els.featureStopBtn.addEventListener('click', stopFeature);
  els.voiceChatBtn.addEventListener('click', () => startVoice(api.voiceStartChat));
  els.voiceControlBtn.addEventListener('click', () => startVoice(api.voiceStartControl));
  els.voiceStopBtn.addEventListener('click', () => startVoice(api.voiceStop));
  els.voiceSafeStopBtn.addEventListener('click', () => startVoice(api.voiceSafeStop));
  els.chassisResetBtn.addEventListener('click', () => postAction(api.chassisReset, '底盘复位'));
  els.cameraLeftBtn.addEventListener('click', () => postAction(api.cameraLeft, '摄像头左看'));
  els.cameraRightBtn.addEventListener('click', () => postAction(api.cameraRight, '摄像头右看'));
  els.cameraUpBtn.addEventListener('click', () => postAction(api.cameraUp, '摄像头上看'));
  els.cameraDownBtn.addEventListener('click', () => postAction(api.cameraDown, '摄像头下看'));
  els.servoResetBtn.addEventListener('click', () => postAction(api.servoReset, '云台回中'));
  if (els.patrolRelockBtn) {
    els.patrolRelockBtn.addEventListener('click', () => postAction(api.patrolRelock, '重新锁定'));
  }
  document.getElementById('stopBtn').addEventListener('click', stopDrive);

  document.querySelectorAll('.drive[data-linear]').forEach((button) => {
    const linear = Number(button.dataset.linear);
    const angular = Number(button.dataset.angular);
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      startDrive(linear, angular);
    });
    button.addEventListener('pointerup', stopDrive);
    button.addEventListener('pointercancel', stopDrive);
    button.addEventListener('pointerleave', stopDrive);
  });

  els.videoStream.addEventListener('load', () => {
    state.videoFailed = false;
    els.videoError.classList.add('hidden');
  });
  els.videoStream.addEventListener('error', () => {
    state.videoFailed = true;
    els.videoError.textContent = '视频未加载，点击右上角视频直链排查';
    els.videoError.classList.remove('hidden');
  });

  window.addEventListener('blur', stopDrive);
  window.addEventListener('beforeunload', () => {
    navigator.sendBeacon(api.drive, JSON.stringify({ linear: 0, angular: 0 }));
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
