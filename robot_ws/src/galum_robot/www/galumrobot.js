// galumrobot.js
(function(){
  // ---------- CONFIG ----------
  const DEFAULTS = {
    host: 'papa.local',
    ws_port: '9090',
    cam_port: '8080',
    topicA: '/cameras/cam_front/image_raw',
    topicB: '/cameras/cam_back/image_raw'
  };

  const LSKEY = k => 'galum:'+k;

  function readQuery() {
    const q = new URLSearchParams(location.search);
    const out = {};
    for (const k of ['host','ws_port','cam_port','topic','topicA','topicB']) {
      if (q.has(k) && q.get(k)) out[k] = q.get(k);
    }
    // รองรับแบบเก่า ?topic= ใช้เป็น topicA
    if (out.topic && !out.topicA) out.topicA = out.topic;
    return out;
  }

  function loadConfig() {
    const fromQ = readQuery();
    const fromLS = {};
    for (const k of ['host','ws_port','cam_port','topicA','topicB']) {
      const v = localStorage.getItem(LSKEY(k));
      if (v && !(k in fromQ)) fromLS[k] = v;
    }
    const cfg = {...DEFAULTS, ...fromLS, ...fromQ};

    // ----- แก้จุดปัญหา: อย่าบังคับ papa.local เวลาเปิดจาก localhost -----
    // เลือก host อัตโนมัติเมื่อไม่ได้ส่งผ่าน ?host= และไม่มีใน localStorage:
    // - ถ้า location.hostname ไม่ใช่ localhost/127.0.0.1 → ใช้อันนั้น
    // - ไม่งั้นใช้ 127.0.0.1 (ออฟไลน์ก็วิ่ง loopback ได้)
    if (!('host' in fromQ) && !('host' in fromLS)) {
      const lh = (location.hostname || '').toLowerCase();
      cfg.host = (lh && lh !== 'localhost' && lh !== '127.0.0.1') ? lh : '127.0.0.1';
    }
    // ---------------------------------------------------------------------

    // เซฟกลับ
    for (const k of Object.keys(cfg)) localStorage.setItem(LSKEY(k), cfg[k]);

    return cfg;
  }

  const CFG = loadConfig();

  // เติมค่าใน quick panel
  const $ = s => document.querySelector(s);
  $('#cfg_host').value = CFG.host;
  $('#cfg_ws').value   = CFG.ws_port;
  $('#cfg_cam').value  = CFG.cam_port;

  $('#cfg_apply').addEventListener('click', ()=>{
    const host = $('#cfg_host').value.trim();
    const ws   = $('#cfg_ws').value.trim();
    const cam  = $('#cfg_cam').value.trim();
    ['host','ws_port','cam_port'].forEach(k=>localStorage.removeItem(LSKEY(k)));
    localStorage.setItem(LSKEY('host'), host);
    localStorage.setItem(LSKEY('ws_port'), ws);
    localStorage.setItem(LSKEY('cam_port'), cam);
    const q = new URLSearchParams({...readQuery(), host, ws_port: ws, cam_port: cam});
    location.search = '?'+q.toString();
  });

  $('#cfg_clear').addEventListener('click', ()=>{
    ['host','ws_port','cam_port','topicA','topicB'].forEach(k=>localStorage.removeItem(LSKEY(k)));
    alert('Cleared localStorage. Reload the page.');
  });

  // ---------- ROS ----------
  const rosURL = `ws://${CFG.host}:${CFG.ws_port}`;
  const camBase = `http://${CFG.host}:${CFG.cam_port}`;

  const badge = document.createElement('div');
  badge.style.cssText = 'position:fixed;right:8px;bottom:8px;background:#000a;color:#fff;padding:6px 10px;border-radius:8px;font:12px monospace;z-index:9999';
  badge.textContent = `ROS: ${rosURL} | CAM: ${camBase}`;
  document.body.appendChild(badge);

  let ros = null;

  function connectROS() {
    if (!window.ROSLIB) return console.error('ROSLIB not loaded');

    ros = new ROSLIB.Ros({ url: rosURL });
    ros.on('connection', ()=> console.log('Connected to ROSBridge ✅', rosURL));
    ros.on('close',      ()=> console.warn('ROSBridge closed'));
    ros.on('error', e => console.error('ROSBridge error', e));
  }

  function mjpegURL(topic, compressed) {
    let url = `${camBase}/stream?topic=${topic}`;   // ❗ ไม่ encode '/' อีกต่อไป
    if (compressed) url += `&default_transport=compressed`;
    return url;
  }

  function setCam(imgId, statusId, topic, compressed) {
    const img = document.getElementById(imgId);
    const st  = document.getElementById(statusId);
    if (!img) return;

    const url = mjpegURL(topic, compressed);
    img.src = url;
    if (st) st.textContent = `stream: ${url}`;
    img.onerror = ()=> st && (st.textContent = 'กล้องโหลดไม่ได้ ❌ ตรวจ host/ports/topic');
  }

  // ---------- ดึงรายชื่อ topics จาก rosapi ----------
  function listImageTopics(cb) {
    // ใช้ rosapi service: /rosapi/topics
    const srv = new ROSLIB.Service({
      ros,
      name: '/rosapi/topics',
      serviceType: 'rosapi/GetTopics'
    });
    srv.callService(new ROSLIB.ServiceRequest({}), (res)=>{
      const all = res.topics || [];
      // คัดเฉพาะหัวข้อภาพ
      const imgs = all.filter(t => /image(_raw|\/compressed)?$/.test(t));
      cb(imgs.sort());
    }, (err)=> {
      console.error('rosapi GetTopics error', err);
      cb([]);
    });
  }

  function populateSelect(selId, topics, currentTopic) {
    const sel = document.getElementById(selId);
    if (!sel) return;
    sel.innerHTML = '';
    const add = (val, text) => {
      const opt = document.createElement('option');
      opt.value = val; opt.textContent = text || val;
      sel.appendChild(opt);
    };
    if (!topics.length) add(currentTopic || '', currentTopic || '(no topics)');
    topics.forEach(t => add(t));
    if (currentTopic && topics.includes(currentTopic)) sel.value = currentTopic;
  }

  // ---------- Motor & System stats subscribers ----------
  function setupSubs() {
    const leftfrontRPM  = $('#left_front_rpm');
    const rightfrontRPM = $('#right_front_rpm');
    const leftbackRPM   = $('#left_back_rpm');
    const rightbackRPM  = $('#right_back_rpm');

    const motorTopic = new ROSLIB.Topic({
      ros,
      name: '/galum/debug/cmd_move/rpm',
      messageType: 'geometry_msgs/Twist'
    });
    motorTopic.subscribe((msg)=>{
      leftfrontRPM.textContent  = Number(msg?.linear?.x  ?? 0).toFixed(2);
      leftbackRPM.textContent   = Number(msg?.linear?.y  ?? 0).toFixed(2);
      rightfrontRPM.textContent = Number(msg?.angular?.x ?? 0).toFixed(2);
      rightbackRPM.textContent  = Number(msg?.angular?.y ?? 0).toFixed(2);
    });

    const cpuTotalEl = $('#cpuTotal');
    const cpuTempEl  = $('#cpuTemp');
    const cpuCoresEl = $('#cpuCores');
    const cpuFreqsEl = $('#cpuFreqs');
    const memUsedEl  = $('#memUsed');
    const memAvailEl = $('#memAvail');
    const memTotalEl = $('#memTotal');
    const l1El = $('#l1'), l5El = $('#l5'), l15El = $('#l15');
    const armClockEl = $('#armClock');
    const voltsEl    = $('#volts');
    const thrNowEl   = $('#throttledNow');
    const thrEverEl  = $('#throttledEver');

    const sysTopic = new ROSLIB.Topic({
      ros,
      name: '/system/stats',
      messageType: 'std_msgs/String'
    });
    sysTopic.subscribe((msg)=>{
      try {
        const s = JSON.parse(msg.data || '{}');
        cpuTotalEl.textContent = `${Number(s?.cpu?.total_percent ?? 0).toFixed(1)} %`;
        cpuTempEl.textContent  = `${Number(s?.cpu?.temp_c ?? 0).toFixed(1)} °C`;

        const perCore = Array.isArray(s?.cpu?.per_core_percent) ? s.cpu.per_core_percent : [];
        cpuCoresEl.textContent = perCore.length ? perCore.map(v => `${Number(v||0).toFixed(0)}%`).join(', ') : '-';

        const freqs = Array.isArray(s?.pi5?.cpu_freq_mhz) ? s.pi5.cpu_freq_mhz : [];
        cpuFreqsEl.textContent = freqs.length ? freqs.map(f => `${Number(f||0).toFixed(0)} MHz`).join(', ') : '-';

        const bytesHuman = (n)=>{
          const u=['B','KB','MB','GB','TB']; let i=0; n=Number(n)||0;
          while(n>=1024 && i<u.length-1){ n/=1024; i++; }
          return n.toFixed(1)+' '+u[i];
        };

        const memP = Number(s?.memory?.percent ?? 0);
        memUsedEl.textContent  = `${bytesHuman(s?.memory?.used)} (${memP.toFixed(1)}%)`;
        memAvailEl.textContent = bytesHuman(s?.memory?.available);
        memTotalEl.textContent = bytesHuman(s?.memory?.total);

        l1El.textContent  = Number(s?.loadavg?.['1min']  ?? 0).toFixed(2);
        l5El.textContent  = Number(s?.loadavg?.['5min']  ?? 0).toFixed(2);
        l15El.textContent = Number(s?.loadavg?.['15min'] ?? 0).toFixed(2);

        const armClk = s?.pi5?.arm_clock_mhz;
        armClockEl.textContent = (armClk==null) ? '-' : `${Number(armClk).toFixed(0)} MHz`;

        const volts = s?.pi5?.volts;
        voltsEl.textContent = (volts==null) ? '-' : `${Number(volts).toFixed(3)} V`;

        const thr = s?.pi5?.throttled;
        if (thr) {
          thrNowEl.textContent  = `UV:${thr.under_voltage_now||0} Cap:${thr.freq_capped_now||0} Thr:${thr.throttled_now||0}`;
          thrEverEl.textContent = `UV:${thr.under_voltage_ever||0} Cap:${thr.freq_capped_ever||0} Thr:${thr.throttled_ever||0}`;
        } else {
          thrNowEl.textContent = '-';
          thrEverEl.textContent = '-';
        }
      } catch(e){ console.error('stats parse', e); }
    });
  }

  // ---------- เริ่มทำงาน ----------
  connectROS();

  // เติม select และ set stream ให้สองกล้อง
  function initCams(imageTopics) {
    // A
    populateSelect('camASelect', imageTopics, CFG.topicA);
    const aSel = $('#camASelect');
    const aCmp = $('#camACompressed');
    // เริ่มต้นสตรีม A
    setCam('camA', 'camAStatus', aSel.value || CFG.topicA, aCmp.checked);
    // เปลี่ยนค่าตอนเลือก
    aSel.addEventListener('change', ()=>{
      localStorage.setItem(LSKEY('topicA'), aSel.value);
      setCam('camA', 'camAStatus', aSel.value, aCmp.checked);
    });
    aCmp.addEventListener('change', ()=>{
      setCam('camA', 'camAStatus', aSel.value, aCmp.checked);
    });

    // B
    populateSelect('camBSelect', imageTopics, CFG.topicB);
    const bSel = $('#camBSelect');
    const bCmp = $('#camBCompressed');
    setCam('camB', 'camBStatus', bSel.value || CFG.topicB, bCmp.checked);
    bSel.addEventListener('change', ()=>{
      localStorage.setItem(LSKEY('topicB'), bSel.value);
      setCam('camB', 'camBStatus', bSel.value, bCmp.checked);
    });
    bCmp.addEventListener('change', ()=>{
      setCam('camB', 'camBStatus', bSel.value, bCmp.checked);
    });
  }

  // รอให้เชื่อม ROS แล้วค่อยเรียก rosapi
  setTimeout(()=>{
      if (!ros) return;
      setupSubs();
      listImageTopics((topics)=>{
        // ถ้าไม่มี rosapi ให้ใช้ค่าเดิม
        if (!topics.length) topics = [CFG.topicA, CFG.topicB].filter(Boolean);
        initCams(topics);
      });
    }, 600);

  function updateOverlayB(){
    const card    = document.querySelector('.camera-cardB');
    const wrap    = card?.querySelector('.camera-frame');  // ใช้กรอบรูปเป็นอ้างอิง
    const overlay = card?.querySelector('.overlay');
    if (!wrap || !overlay) return;

    // เปิดกรอบ debug (ถ้ารกตา ลบบรรทัดนี้ได้)
    overlay.classList.add('debug-outline');

    const pxPerCm = parseFloat(document.getElementById('pxPerCmB')?.value) || 10;
    const offsetCm  = parseFloat(document.getElementById('centerOffsetCmB')?.value) || 0;

    // ขนาดเฟรมปัจจุบัน
    const W  = wrap.clientWidth;
    const cx = W / 2;
    const cX  = cx + (offsetCm * pxPerCm);

    // ระยะคงที่ 15 cm
    const d15 = 15 * pxPerCm;

    // setter ลัด
    const setLeft = (el, x) => { if (el) el.style.left = `${x}px`; };

    // อ้างอิงเส้น
    const center = overlay.querySelector('.center-line');
    const left15 = overlay.querySelector('.side15-line.left');
    const right15= overlay.querySelector('.side15-line.right');

    // จัดตำแหน่ง
    + setLeft(center,  cX);
    + setLeft(left15,  cX - d15);
    + setLeft(right15, cX + d15);

    // โชว์/ซ่อน overlay ตาม checkbox
    const show = document.getElementById('toggleOverlayB')?.checked !== false;
    card.classList.toggle('overlay-hidden', !show);
  }
  window.addEventListener('resize', updateOverlayB);
  document.getElementById('pxPerCmB')?.addEventListener('input', updateOverlayB);
  document.getElementById('centerOffsetCmB')?.addEventListener('input', updateOverlayB);
  document.getElementById('toggleOverlayB')?.addEventListener('change', updateOverlayB);

  // เรียกหลังรูปโหลด เพื่อให้ได้ขนาดจริง
  document.getElementById('camB')?.addEventListener('load', updateOverlayB);

  // กันพลาด: เรียกซ้ำหลังตั้งค่ากล้องเสร็จ
  setTimeout(updateOverlayB, 1200);
})();

