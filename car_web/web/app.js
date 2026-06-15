const api = {
  status: '/api/status',
  events: '/api/events',
  reportJob: '/api/report/job',
  manual: '/api/mode/manual',
  report: '/api/mode/report',
  notifyStart: '/api/notify/start',
  notifyStop: '/api/notify/stop',
  drive: '/api/drive',
  estop: '/api/estop',
  generateReport: '/api/report/generate',
};

const state = {
  videoTopic: '/camera/color/image_raw',
  driveTimer: null,
  activeDrive: null,
};

const els = {
  statusText: document.getElementById('statusText'),
  coreBadge: document.getElementById('coreBadge'),
  reportBadge: document.getElementById('reportBadge'),
  notifyBadge: document.getElementById('notifyBadge'),
  videoTopic: document.getElementById('videoTopic'),
  videoStream: document.getElementById('videoStream'),
  eventsList: document.getElementById('eventsList'),
  eventCount: document.getElementById('eventCount'),
  reportJob: document.getElementById('reportJob'),
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

function setVideoTopic(topic) {
  if (!topic || topic === state.videoTopic) {
    return;
  }
  state.videoTopic = topic;
  els.videoTopic.textContent = topic;
  const encoded = encodeURIComponent(topic);
  els.videoStream.src = `http://${window.location.hostname}:8080/stream?topic=${encoded}&_=${Date.now()}`;
}

async function refreshStatus() {
  try {
    const data = await request(api.status);
    const processes = data.processes || {};
    setBadge(els.coreBadge, 'core', processes.core && processes.core.running);
    setBadge(els.reportBadge, 'report', processes.report && processes.report.running);
    setBadge(els.notifyBadge, 'notify', processes.notify && processes.notify.running);
    setVideoTopic(data.video_topic || '/camera/color/image_raw');
    const modeText = data.mode === 'report' ? '识别记录模式' : data.mode === 'manual' ? '普通操作模式' : '空闲';
    setStatus(`${modeText}，事件 ${data.events_count || 0} 条${data.last_error ? '，错误：' + data.last_error : ''}`, Boolean(data.last_error));
    renderReportJob(data.report_job);
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

function startDrive(linear, angular) {
  stopDriveTimer();
  state.activeDrive = { linear, angular };
  sendDrive();
  state.driveTimer = setInterval(sendDrive, 180);
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
  document.getElementById('manualBtn').addEventListener('click', () => postAndRefresh(api.manual));
  document.getElementById('reportBtn').addEventListener('click', () => postAndRefresh(api.report));
  document.getElementById('notifyStartBtn').addEventListener('click', () => postAndRefresh(api.notifyStart));
  document.getElementById('notifyStopBtn').addEventListener('click', () => postAndRefresh(api.notifyStop));
  document.getElementById('reportTextBtn').addEventListener('click', () => postAndRefresh(api.generateReport, { mode: 'text' }));
  document.getElementById('reportVisionBtn').addEventListener('click', () => postAndRefresh(api.generateReport, { mode: 'vision', max_images: 3 }));
  document.getElementById('estopBtn').addEventListener('click', () => postAndRefresh(api.estop));
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

  window.addEventListener('blur', stopDrive);
}

function startPolling() {
  setVideoTopic('/camera/color/image_raw');
  refreshStatus();
  refreshEvents();
  setInterval(refreshStatus, 1500);
  setInterval(refreshEvents, 2000);
}

bindControls();
startPolling();
