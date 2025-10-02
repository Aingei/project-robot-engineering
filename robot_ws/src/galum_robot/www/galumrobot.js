
document.addEventListener('DOMContentLoaded', () => {
  const statusEl = document.getElementById('status');
  const webcam   = document.getElementById('webcam');

  // -------- Host/port/topic auto config --------
  const qs   = new URLSearchParams(location.search);
  const HOST = qs.get('host') || location.hostname || 'papa.local';

  // web_video_server port
  const CAM_PORT = Number(qs.get('cam_port') || 8080);
  // rosbridge port
  const WS_PORT  = Number(qs.get('ws_port')  || 9090);

  // topic (แก้ให้ตรงกับที่คุณ publish จริง ๆ)
  const TOPIC = qs.get('topic') || '/camera/image_raw';
  const EXTRA = qs.get('compressed') === '1' ? '&default_transport=compressed' : '';

  // Set camera src (ทดสอบ URL นี้โดยวางในแถบเบราว์เซอร์ได้)
  const camURL = `http://${HOST}:${CAM_PORT}/stream?topic=${TOPIC}${EXTRA}`;
  if (webcam) {
    webcam.src = camURL;
    webcam.onload  = () => { if (statusEl) statusEl.textContent = 'กล้องพร้อมใช้งาน ✅'; };
    webcam.onerror = () => {
      if (statusEl) statusEl.textContent = 'กล้องโหลดไม่ได้ ❌ (เช็ค host/port/topic)';
      console.error('Camera failed:', camURL);
    };
  }

  // --- Elements ---
  const leftfrontRPM  = document.getElementById('left_front_rpm');
  const rightfrontRPM = document.getElementById('right_front_rpm');
  const leftbackRPM   = document.getElementById('left_back_rpm');
  const rightbackRPM  = document.getElementById('right_back_rpm');

  const cpuTotalEl = document.getElementById('cpuTotal');
  const cpuTempEl  = document.getElementById('cpuTemp');
  const cpuCoresEl = document.getElementById('cpuCores');
  const cpuFreqsEl = document.getElementById('cpuFreqs');
  const memUsedEl  = document.getElementById('memUsed');
  const memAvailEl = document.getElementById('memAvail');
  const memTotalEl = document.getElementById('memTotal');
  const l1El  = document.getElementById('l1');
  const l5El  = document.getElementById('l5');
  const l15El = document.getElementById('l15');
  const armClockEl = document.getElementById('armClock');
  const voltsEl    = document.getElementById('volts');
  const thrNowEl   = document.getElementById('throttledNow');
  const thrEverEl  = document.getElementById('throttledEver');

  // ---------- ROSBridge ----------
  if (window.ROSLIB) {
    const ros = new ROSLIB.Ros({ url: `ws://${HOST}:${WS_PORT}` });
    ros.on('connection', () => console.log('Connected to ROSBridge ✅'));
    ros.on('error',      (e) => console.error('ROSBridge error', e));
    ros.on('close',      ()  => console.log('ROSBridge closed'));

    // Motor Topic
    const motorTopic = new ROSLIB.Topic({
      ros,
      name: '/galum/debug/cmd_move/rpm',
      messageType: 'geometry_msgs/Twist'
    });
    motorTopic.subscribe((msg) => {
      leftfrontRPM.textContent  = Number(msg?.linear?.x  ?? 0).toFixed(2);
      leftbackRPM.textContent   = Number(msg?.linear?.y  ?? 0).toFixed(2);
      rightfrontRPM.textContent = Number(msg?.angular?.x ?? 0).toFixed(2);
      rightbackRPM.textContent  = Number(msg?.angular?.y ?? 0).toFixed(2);
    });

    // System Stats Topic (std_msgs/String JSON)
    const sysTopic = new ROSLIB.Topic({
      ros,
      name: '/system/stats',
      messageType: 'std_msgs/String'
    });
    sysTopic.subscribe((msg) => {
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
      } catch (e) {
        console.error('System stats parse error:', e);
      }
    });
  } else {
    console.error('ROSLIB not loaded');
  }
});
