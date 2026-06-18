// nav_map.js — 用 roslibjs 订阅 /map /robot_pose /plan,在 canvas 上渲染地图、
// 机器人位姿、规划路径;支持点击地图回调世界坐标(供设导航目标/打点)。
// 依赖全局 ROSLIB(vendor/roslib.min.js)。
(function (global) {
  'use strict';

  function NavMap(canvas, opts) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.opts = opts || {};
    this.ros = null;
    this.connected = false;
    this._url = null;            // 末次 connect 的 ws 地址,断线自动重连用
    this._wantConnected = false; // 期望保持连接;disconnect() 置 false 停止重连
    this._reconnectTimer = null;
    this.grid = null;        // {res, w, h, ox, oy, data}
    this.robot = null;       // {x, y, yaw}
    this.plan = null;        // [{x,y}...]
    this.waypoints = [];     // 前端临时巡航点 [{x,y}]
    this.goal = null;        // 最近设定目标 {x,y}
    this.view = { scale: 1, offX: 0, offY: 0 };  // 世界->屏幕
    this._subs = [];
    this.onClickWorld = null;   // function(x, y){}  快速点击(无拖拽,goto)
    this.onPoseWorld = null;    // function(x, y, yaw){}  按下拖拽设朝向后释放
    this.clickMode = 'goto';    // 'goto' | 'setpose':决定拖拽释放回调哪个
    this._drag = null;          // 拖拽中 {x0,y0,x1,y1} 世界坐标,画临时箭头
    this.onStatus = null;       // function(text, bad){}
  }

  NavMap.prototype.connect = function (url) {
    var self = this;
    if (url) { this._url = url; }
    this._wantConnected = true;
    if (this.ros) { this._teardownRos(); }
    this.ros = new ROSLIB.Ros({ url: this._url });
    this.ros.on('connection', function () {
      self.connected = true;
      self._status('已连接地图服务', false);
      self._subscribe();
    });
    this.ros.on('error', function () {
      self.connected = false;
      self._status('地图服务连接出错', true);
    });
    this.ros.on('close', function () {
      self.connected = false;
      self._status('地图服务已断开', true);
      // rosbridge 重启(如加载地图)或网络抖动会断开;只要仍想连就自动重连,
      // 重连后 connection 回调会重新 _subscribe,canvas 恢复 /map。
      if (self._wantConnected) {
        if (self._reconnectTimer) { clearTimeout(self._reconnectTimer); }
        self._reconnectTimer = setTimeout(function () {
          if (self._wantConnected) { self.connect(); }
        }, 1500);
      }
    });
  };

  NavMap.prototype._teardownRos = function () {
    this._subs.forEach(function (s) { try { s.unsubscribe(); } catch (e) {} });
    this._subs = [];
    if (this.ros) { try { this.ros.close(); } catch (e) {} }
    this.ros = null;
    this.connected = false;
  };

  NavMap.prototype.disconnect = function () {
    this._wantConnected = false;
    if (this._reconnectTimer) { clearTimeout(this._reconnectTimer); this._reconnectTimer = null; }
    this._teardownRos();
  };

  NavMap.prototype._status = function (text, bad) {
    if (this.onStatus) { this.onStatus(text, bad); }
  };

  NavMap.prototype._subscribe = function () {
    var self = this;
    var mapSub = new ROSLIB.Topic({
      ros: this.ros, name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
    });
    mapSub.subscribe(function (msg) { self._onMap(msg); });
    this._subs.push(mapSub);

    var poseSub = new ROSLIB.Topic({
      ros: this.ros, name: '/robot_pose',
      messageType: 'geometry_msgs/PoseStamped',
    });
    poseSub.subscribe(function (msg) { self._onPose(msg); });
    this._subs.push(poseSub);

    var planSub = new ROSLIB.Topic({
      ros: this.ros, name: '/plan',
      messageType: 'nav_msgs/Path',
    });
    planSub.subscribe(function (msg) { self._onPlan(msg); });
    this._subs.push(planSub);
  };

  NavMap.prototype._onMap = function (msg) {
    var info = msg.info;
    this.grid = {
      res: info.resolution,
      w: info.width,
      h: info.height,
      ox: info.origin.position.x,
      oy: info.origin.position.y,
      data: msg.data,
    };
    this._fit();
    this.render();
  };

  NavMap.prototype._onPose = function (msg) {
    var q = msg.pose.orientation;
    var yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z));
    this.robot = { x: msg.pose.position.x, y: msg.pose.position.y, yaw: yaw };
    this.render();
  };

  NavMap.prototype._onPlan = function (msg) {
    this.plan = (msg.poses || []).map(function (p) {
      return { x: p.pose.position.x, y: p.pose.position.y };
    });
    this.render();
  };

  // 把 canvas 绘图缓冲尺寸同步到实际显示像素,避免 4:3 缓冲被拉伸进 16:9 框。
  // 切到建图模式、窗口缩放时调用。返回是否发生了尺寸变化。
  NavMap.prototype.resize = function () {
    var rect = this.canvas.getBoundingClientRect();
    var w = Math.max(1, Math.round(rect.width));
    var h = Math.max(1, Math.round(rect.height));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
      this._fit();
      return true;
    }
    return false;
  };

  // 让地图适配 canvas:只按"已探索栅格"的包围盒来缩放居中,
  // 而不是整张含大片未知边距的栅格,这样地图像相机画面一样铺满、居中。
  NavMap.prototype._fit = function () {
    var g = this.grid;
    if (!g) { return; }
    var cw = this.canvas.width, ch = this.canvas.height;
    // 扫描已知栅格(空闲/占用)的行列包围盒
    var minc = g.w, maxc = -1, minr = g.h, maxr = -1;
    for (var row = 0; row < g.h; row++) {
      for (var col = 0; col < g.w; col++) {
        if (g.data[row * g.w + col] >= 0) {
          if (col < minc) minc = col;
          if (col > maxc) maxc = col;
          if (row < minr) minr = row;
          if (row > maxr) maxr = row;
        }
      }
    }
    var bx, by, bw, bh; // 世界坐标包围盒(米)
    if (maxc < 0) {
      // 还没有已知栅格,退回整张图
      bx = g.ox; by = g.oy; bw = g.w * g.res; bh = g.h * g.res;
    } else {
      bx = g.ox + minc * g.res;
      by = g.oy + minr * g.res;
      bw = (maxc - minc + 1) * g.res;
      bh = (maxr - minr + 1) * g.res;
    }
    var scale = Math.min(cw / bw, ch / bh) * 0.92;
    this.view.scale = scale;
    // 包围盒中心对齐画布中心。屏幕 y 向下,世界 y 向上。
    var cxWorld = bx + bw / 2, cyWorld = by + bh / 2;
    this.view.offX = cw / 2 - cxWorld * scale;
    this.view.offY = ch / 2 + cyWorld * scale;
  };

  NavMap.prototype.worldToScreen = function (x, y) {
    return {
      sx: x * this.view.scale + this.view.offX,
      sy: this.view.offY - y * this.view.scale,
    };
  };

  NavMap.prototype.screenToWorld = function (sx, sy) {
    return {
      x: (sx - this.view.offX) / this.view.scale,
      y: (this.view.offY - sy) / this.view.scale,
    };
  };

  // 绑定交互:按下记起点,拖拽实时画朝向箭头,释放按拖拽方向算 yaw。
  // 几乎没拖动(<8px)视为快速点击:goto 模式回调 onClickWorld(yaw=0);
  // setpose 模式则必须拖拽给朝向,释放回调 onPoseWorld(x,y,yaw)。
  NavMap.prototype._evWorld = function (ev) {
    var rect = this.canvas.getBoundingClientRect();
    var sx = (ev.clientX - rect.left) * (this.canvas.width / rect.width);
    var sy = (ev.clientY - rect.top) * (this.canvas.height / rect.height);
    return this.screenToWorld(sx, sy);
  };

  NavMap.prototype.bindClick = function () {
    var self = this;
    var downPx = null;
    this.canvas.addEventListener('mousedown', function (ev) {
      if (!self.grid) { return; }
      downPx = { x: ev.clientX, y: ev.clientY };
      var w = self._evWorld(ev);
      self._drag = { x0: w.x, y0: w.y, x1: w.x, y1: w.y };
    });
    this.canvas.addEventListener('mousemove', function (ev) {
      if (!self._drag) { return; }
      var w = self._evWorld(ev);
      self._drag.x1 = w.x; self._drag.y1 = w.y;
      self.render();
    });
    window.addEventListener('mouseup', function (ev) {
      if (!self._drag) { return; }
      var d = self._drag; self._drag = null;
      var moved = downPx ? Math.hypot(ev.clientX - downPx.x, ev.clientY - downPx.y) : 0;
      var yaw = Math.atan2(d.y1 - d.y0, d.x1 - d.x0);
      self.render();
      if (moved < 8) {
        // 快速点击:goto 用(yaw=0);setpose 模式提示需拖拽
        if (self.clickMode === 'setpose') {
          self._status('设初始位姿:按住并拖出朝向再松开', true);
          return;
        }
        if (self.onClickWorld) { self.onClickWorld(d.x0, d.y0); }
      } else {
        if (self.clickMode === 'setpose') {
          if (self.onPoseWorld) { self.onPoseWorld(d.x0, d.y0, yaw); }
        } else if (self.onClickWorld) {
          // goto 也支持拖拽给朝向:经 onPoseWorld 若挂了优先,否则退回 onClickWorld
          if (self.onPoseWorld) { self.onPoseWorld(d.x0, d.y0, yaw); }
          else { self.onClickWorld(d.x0, d.y0); }
        }
      }
    });
  };

  NavMap.prototype.render = function () {
    var ctx = this.ctx, g = this.grid;
    var cw = this.canvas.width, ch = this.canvas.height;
    ctx.fillStyle = '#1a1d23';
    ctx.fillRect(0, 0, cw, ch);
    if (!g) {
      ctx.fillStyle = '#888';
      ctx.font = '16px sans-serif';
      ctx.fillText('等待 /map ... (开始建图或加载地图)', 20, 30);
      return;
    }
    this._drawGrid();
    this._drawPlan();
    this._drawWaypoints();
    this._drawGoal();
    this._drawDrag();
    this._drawRobot();
  };

  // 拖拽设位姿/目标时的临时朝向箭头(青色)
  NavMap.prototype._drawDrag = function () {
    if (!this._drag) { return; }
    var d = this._drag;
    var a = this.worldToScreen(d.x0, d.y0);
    var b = this.worldToScreen(d.x1, d.y1);
    var ctx = this.ctx;
    ctx.strokeStyle = '#1abc9c'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(a.sx, a.sy, 7, 0, 2 * Math.PI); ctx.stroke();
    if (Math.hypot(b.sx - a.sx, b.sy - a.sy) > 6) {
      ctx.beginPath(); ctx.moveTo(a.sx, a.sy); ctx.lineTo(b.sx, b.sy); ctx.stroke();
      var ang = Math.atan2(b.sy - a.sy, b.sx - a.sx);
      ctx.beginPath(); ctx.moveTo(b.sx, b.sy);
      ctx.lineTo(b.sx - 10 * Math.cos(ang - 0.4), b.sy - 10 * Math.sin(ang - 0.4));
      ctx.moveTo(b.sx, b.sy);
      ctx.lineTo(b.sx - 10 * Math.cos(ang + 0.4), b.sy - 10 * Math.sin(ang + 0.4));
      ctx.stroke();
    }
  };

  // 把 OccupancyGrid 画成像素块。-1 未知=深灰,0 空闲=白,100 占用=黑。
  NavMap.prototype._drawGrid = function () {
    var ctx = this.ctx, g = this.grid;
    var cell = g.res * this.view.scale;
    var sz = Math.max(1, Math.ceil(cell));
    for (var row = 0; row < g.h; row++) {
      for (var col = 0; col < g.w; col++) {
        var v = g.data[row * g.w + col];
        var color;
        if (v < 0) { color = '#3a3f47'; }
        else if (v >= 65) { color = '#0c0d10'; }
        else if (v <= 20) { color = '#e8e8e8'; }
        else { continue; }
        // 栅格 (col,row) 中心的世界坐标(cell 中心)
        var wx = g.ox + (col + 0.5) * g.res;
        var wy = g.oy + (row + 0.5) * g.res;
        var s = this.worldToScreen(wx, wy);
        ctx.fillStyle = color;
        ctx.fillRect(s.sx - sz / 2, s.sy - sz / 2, sz, sz);
      }
    }
  };

  NavMap.prototype._drawPlan = function () {
    if (!this.plan || this.plan.length < 2) { return; }
    var ctx = this.ctx, self = this;
    ctx.strokeStyle = '#36c'; ctx.lineWidth = 2; ctx.beginPath();
    this.plan.forEach(function (p, i) {
      var s = self.worldToScreen(p.x, p.y);
      if (i === 0) { ctx.moveTo(s.sx, s.sy); } else { ctx.lineTo(s.sx, s.sy); }
    });
    ctx.stroke();
  };

  NavMap.prototype._drawWaypoints = function () {
    var ctx = this.ctx, self = this;
    this.waypoints.forEach(function (p, i) {
      var s = self.worldToScreen(p.x, p.y);
      ctx.fillStyle = '#f5a623';
      ctx.beginPath(); ctx.arc(s.sx, s.sy, 6, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = '#000'; ctx.font = '11px sans-serif';
      ctx.fillText(String(i + 1), s.sx - 3, s.sy + 4);
    });
  };

  NavMap.prototype._drawGoal = function () {
    if (!this.goal) { return; }
    var ctx = this.ctx, s = this.worldToScreen(this.goal.x, this.goal.y);
    ctx.strokeStyle = '#2ecc71'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(s.sx, s.sy, 8, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(s.sx - 11, s.sy); ctx.lineTo(s.sx + 11, s.sy);
    ctx.moveTo(s.sx, s.sy - 11); ctx.lineTo(s.sx, s.sy + 11);
    ctx.stroke();
  };

  NavMap.prototype._drawRobot = function () {
    if (!this.robot) { return; }
    var ctx = this.ctx, s = this.worldToScreen(this.robot.x, this.robot.y);
    var r = 9;
    ctx.fillStyle = '#e74c3c';
    ctx.beginPath(); ctx.arc(s.sx, s.sy, r, 0, 2 * Math.PI); ctx.fill();
    // 朝向箭头(世界 y 向上,屏幕 y 向下 -> 屏幕角取负)
    var a = -this.robot.yaw;
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(s.sx, s.sy);
    ctx.lineTo(s.sx + Math.cos(a) * (r + 8), s.sy + Math.sin(a) * (r + 8));
    ctx.stroke();
  };

  global.NavMap = NavMap;
})(window);
