const $ = id => document.getElementById(id);
const PHASE = { off: '未接続', disconnected: '切断', observing: '観測中', following: '追従中', stopped: '停止', fault: '異常停止' };
const clone = value => JSON.parse(JSON.stringify(value));
const num = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : '—';

export function createTwinUI({ clientId, getSimState, getJointNames, sendGoals, onStop }) {
  let state = null, draft = null, dirty = false, busy = false, revision = -1, draftBaseRevision = -1;
  let lease = null, heartbeatTimer = null, heartbeatBusy = false, mode = 'design';
  let libraryKey = '', stepNodes = [], localError = '', connectionError = '';
  let stopEpoch = 0;
  let ownsPlayback = false;
  const header = document.querySelector('header');
  header.insertAdjacentHTML('afterbegin', `
    <div class="seg twin-mode" role="group" aria-label="作業モード">
      <button id="twinDesign" class="on" aria-pressed="true">動きを作る</button>
      <button id="twinTrial" aria-pressed="false">実機で試す</button>
    </div>`);
  header.insertAdjacentHTML('beforeend', `
    <button id="twinStop" class="twin-stop" title="実機追従と仮想再生を停止。脱力はしません">停止 <kbd>Esc</kbd></button>
    <div class="twin-status" aria-label="実機の接続状態">
      <span>接続先 <strong id="twinConnection">仮想のみ (off)</strong></span>
      <span id="twinPhase" role="status">未接続</span><span id="twinAge" class="mono">受信 —</span><span id="twinStopConfirmed"></span>
      <span id="twinOwner"></span><span id="twinReason" class="twin-reason"></span>
      <span id="twinError" class="twin-error" role="alert"></span>
    </div>`);
  $('cmd').insertAdjacentHTML('afterbegin', `
    <section class="grp twin-panel" id="twinHardware" aria-label="実機確認" hidden>
      <h2>実機の状態を観測</h2>
      <p class="note">接続して角度を読みます。追従を開始するまで実機は動かしません。</p>
      <div class="twin-actions"><button id="twinConnect">接続して観測</button><button id="twinDisconnect">切断</button></div>
      <div class="twin-actions"><button id="twinImportActual">実機姿勢を仮想へ</button></div>
      <p class="note">仮想の姿勢だけを合わせます。実機は動きません。</p>
      <p class="note" id="twinAlignment">目標と実機の角度差 —</p>
      <dl class="twin-errors" id="twinJointErrors" aria-label="各軸の目標と実機の角度差"></dl>
      <p class="note" id="twinCalibration"></p>
      <p class="note" id="twinRealVerification" hidden>この連携機能は実機での接続・追従・停止が未検証です。</p>
      <label class="twin-check"><input id="twinClear" type="checkbox">周囲に人や物がない</label>
      <label class="twin-check"><input id="twinBase" type="checkbox">台座を固定した</label>
      <label class="twin-check"><input id="twinEstop" type="checkbox">停止手段に手が届く</label>
      <div class="twin-actions"><button id="twinFollow" class="primary">実機を追従させる</button></div>
      <p class="note" id="twinFollowHint">姿勢差を3°以内に合わせてから開始します。</p>
      <p class="note mono" id="twinCounts">受信 0 / 送信 0</p>
    </section>
    <section class="grp twin-panel" id="twinProgram" aria-label="名前付き動作">
      <h2>名前付き動作</h2>
      <p class="note">目標姿勢を追加し、移動時間と待機時間を調整して再生します。</p>
      <label>動作名<input id="twinProgramName" type="text" maxlength="80" autocomplete="off" placeholder="例：手を振る"></label>
      <div class="twin-draft" id="twinDraftState" aria-live="polite"></div>
      <div class="twin-actions"><button id="twinCapture">現在の目標姿勢を追加</button></div>
      <ol id="twinSteps" class="twin-steps"></ol>
      <p class="note" id="twinEmpty">目標姿勢を作って追加してください。</p>
      <div class="twin-actions"><button id="twinApply">編集を反映</button><button id="twinSave">保存</button></div>
      <div class="twin-actions"><button id="twinReloadCurrent" title="未反映の編集を破棄して、現在のサーバの動作を読み直します">再読込（編集を破棄）</button></div>
      <div class="twin-library"><select id="twinLibrary" aria-label="保存した動作"><option value="">保存した動作</option></select><button id="twinLoad">読込</button></div>
      <div class="twin-actions"><button id="twinPlay" class="primary">仮想再生</button><button id="twinPlaybackStop">停止</button></div>
      <p id="twinProgress" class="twin-progress" aria-label="再生位置">停止中</p>
      <p class="note" id="twinPlaybackNote">追従中の再生は実機にも送信されます。</p>
      <div class="twin-actions"><button id="twinExport">JSON 書出</button><button id="twinImport">JSON 読込</button></div>
      <input id="twinImportFile" type="file" accept=".json,application/json" aria-label="動作JSONファイル">
    </section>`);
  $('servo').insertAdjacentHTML('afterbegin', '<p class="twin-sim-note">以下は仮想サーボです。熱・PID・電流のモデル設定は実機に書き込みません。</p>');
  $('arm').insertAdjacentHTML('beforeend', '<div class="twin-legend" aria-label="姿勢の凡例"><span class="goal">指令</span><span>仮想</span><span class="measured">実測・青い線</span></div>');
  getJointNames().forEach((name, index) => {
    const cell = document.createElement('div'), title = document.createElement('dt'), value = document.createElement('dd');
    title.textContent = `軸${index + 1}`; cell.title = name; value.id = `twinJointError${index}`;
    value.textContent = '—'; cell.append(title, value); $('twinJointErrors').append(cell);
  });

  function reportError(error) {
    localError = String(error?.message || error || '');
    if (localError === 'Failed to fetch') localError = '操作を送信できませんでした。接続を確認してください。';
    $('twinError').textContent = localError || connectionError;
  }
  function reportConnectionError() {
    connectionError = 'サーバとの通信が途切れました。再接続を待っています。';
    $('twinError').textContent = localError || connectionError;
  }
  function ownsFollow() { return !!state?.hardware?.following && state.hardware.owner === clientId && !!lease; }
  function canEditGoals() { return !!state && (!state.hardware?.following || ownsFollow()); }
  function physicsLocked() { return !!state?.hardware?.following; }
  function editable() { return canEditGoals() && !busy && !state?.playback?.playing; }

  async function request(op, body = {}) {
    const response = await fetch('/api/twin/cmd', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ op, ...body, client_id: clientId }),
    });
    const data = await response.json();
    if (!response.ok || data.ok === false || data.error) throw new Error(data.error || '操作を送信できませんでした');
    const snapshot = data.twin || data;
    if (snapshot.program && snapshot.hardware) render(snapshot);
    return data;
  }
  function action(id, fn) {
    $(id).addEventListener('click', async () => {
      if (busy) return;
      busy = true; reportError(''); updateControls();
      const epoch = stopEpoch;
      try { await fn(epoch); } catch (error) { reportError(error); }
      finally { busy = false; updateControls(); }
    });
  }
  function changeMode(next) {
    mode = next;
    $('twinHardware').hidden = mode !== 'trial';
    for (const [id, active] of [['twinDesign', mode === 'design'], ['twinTrial', mode === 'trial']]) {
      $(id).classList.toggle('on', active); $(id).setAttribute('aria-pressed', String(active));
    }
  }
  $('twinDesign').onclick = () => changeMode('design');
  $('twinTrial').onclick = () => changeMode('trial');
  function markDirty() { if (!dirty) draftBaseRevision = revision; dirty = true; updateControls(); }
  $('twinProgramName').oninput = () => { if (draft) { draft.name = $('twinProgramName').value; markDirty(); } };
  for (const id of ['twinClear', 'twinBase', 'twinEstop']) $(id).onchange = updateControls;
  $('twinLibrary').onchange = updateControls;

  function validateProgram(program) {
    if (!program || program.version !== 1 || typeof program.name !== 'string' || !program.name.trim() || program.name.length > 80 || !Array.isArray(program.steps)) throw new Error('動作名と version: 1 の動作データが必要です');
    if (program.steps.length > 200) throw new Error('姿勢は200個以内にしてください');
    const names = getJointNames();
    program.steps.forEach((step, index) => {
      if (typeof step.label !== 'string' || !step.label.trim() || step.label.length > 80 || !Array.isArray(step.angles) || step.angles.length !== names.length || !step.angles.every(Number.isFinite)) throw new Error(`姿勢${index + 1}の名前と6軸の角度を確認してください`);
      if (!Number.isFinite(step.move_s) || step.move_s <= 0 || !Number.isFinite(step.hold_s) || step.hold_s < 0) throw new Error(`姿勢${index + 1}の移動時間は0秒より長く、待機時間は0秒以上にしてください`);
    });
    return program;
  }
  function buildSteps() {
    stepNodes = [];
    $('twinSteps').replaceChildren();
    $('twinEmpty').hidden = !!draft?.steps.length;
    (draft?.steps || []).forEach((step, index) => {
      const li = document.createElement('li'); li.className = 'twin-step';
      li.innerHTML = `<div class="twin-step-title"><span>${index + 1}</span><label>姿勢名<input type="text" maxlength="80"></label></div>
        <div class="twin-step-times"><label>移動 [秒]<input type="number" min="0.01" step="0.1" inputmode="decimal"></label><label>待機 [秒]<input type="number" min="0" step="0.1" inputmode="decimal"></label></div>
        <div class="twin-step-angles"></div><div class="twin-actions"><button type="button" data-do="preview">姿勢を確認</button><button type="button" data-do="up">↑</button><button type="button" data-do="down">↓</button><button type="button" data-do="remove">削除</button></div>`;
      const fields = li.querySelectorAll('input');
      for (const [input, key] of [[fields[0], 'label'], [fields[1], 'move_s'], [fields[2], 'hold_s']]) {
        input.value = step[key];
        input.setAttribute('aria-label', `姿勢${index + 1} ${key === 'label' ? '名前' : key === 'move_s' ? '移動時間 秒' : '待機時間 秒'}`);
        input.oninput = () => { step[key] = key === 'label' ? input.value : input.value.trim() ? Number(input.value) : NaN; markDirty(); };
      }
      li.querySelector('.twin-step-angles').textContent = step.angles.map((angle, j) => `${j + 1}: ${num(angle)}°`).join(' / ');
      const buttons = li.querySelectorAll('button');
      buttons.forEach(button => {
        const op = button.dataset.do;
        button.setAttribute('aria-label', `姿勢${index + 1} ${op === 'up' ? 'を上へ移動' : op === 'down' ? 'を下へ移動' : button.textContent}`);
        button.onclick = async () => {
          if (!editable()) return;
          try {
            reportError('');
            if (op === 'preview') { await sendGoals(Object.fromEntries(getJointNames().map((name, i) => [name, step.angles[i]]))); return; }
            if (op === 'remove') draft.steps.splice(index, 1);
            else {
              const next = index + (op === 'up' ? -1 : 1);
              if (next < 0 || next >= draft.steps.length) return;
              [draft.steps[index], draft.steps[next]] = [draft.steps[next], draft.steps[index]];
            }
            markDirty(); buildSteps(); updateControls();
          } catch (error) { reportError(error); }
        };
      });
      $('twinSteps').append(li); stepNodes.push(li);
    });
  }
  async function commitDraft() {
    if (!draft) throw new Error('動作データを取得できていません');
    validateProgram(draft);
    if (!dirty) return;
    const program = clone(draft);
    const data = await request('program', { program, expected_revision: draftBaseRevision });
    const snapshot = data.twin || data;
    dirty = false;
    draftBaseRevision = snapshot.revision;
    if (snapshot.program) { draft = clone(snapshot.program); $('twinProgramName').value = draft.name; buildSteps(); }
    updateControls();
  }
  action('twinApply', commitDraft);
  action('twinReloadCurrent', async () => {
    const response = await fetch('/api/twin/state');
    const snapshot = await response.json();
    if (!response.ok || !snapshot.program || !snapshot.hardware) throw new Error(snapshot.error || '現在の動作を読み込めませんでした');
    dirty = false; draft = null; render(snapshot);
  });
  action('twinCapture', async () => {
    await commitDraft();
    await request('capture', { label: `姿勢${(draft?.steps.length || 0) + 1}`, move_s: 2, hold_s: 0.5 });
  });
  action('twinSave', async () => { await commitDraft(); await request('save'); });
  action('twinLoad', async () => {
    if (!$('twinLibrary').value) throw new Error('読み込む動作を選んでください');
    const data = await request('load', { name: $('twinLibrary').value });
    const snapshot = data.twin || data;
    dirty = false;
    draftBaseRevision = snapshot.revision;
    if (snapshot.program) { draft = clone(snapshot.program); $('twinProgramName').value = draft.name; buildSteps(); }
  });
  action('twinPlay', async epoch => {
    await commitDraft();
    if (epoch !== stopEpoch) return;
    await request('play');
    ownsPlayback = true;
    if (epoch !== stopEpoch) await stop();
  });
  action('twinConnect', async () => { await request('hardware_connect'); });
  action('twinDisconnect', async () => { clearHeartbeat(); await request('hardware_disconnect'); });
  action('twinImportActual', async () => { await request('import_actual'); });
  action('twinFollow', async epoch => {
    const ready = ['twinClear', 'twinBase', 'twinEstop'].every(id => $(id).checked);
    if (!ready) throw new Error('実機の準備を確認してください');
    const data = await request('follow', { ready: true });
    lease = typeof data.lease === 'string' ? data.lease : null;
    if (!lease) { await stop(); throw new Error('追従の接続を維持できません。停止しました'); }
    if (document.hidden || epoch !== stopEpoch) { await stop(); return; }
    heartbeatTimer = setInterval(heartbeat, 250); updateControls();
  });
  function clearHeartbeat() { if (heartbeatTimer) clearInterval(heartbeatTimer); heartbeatTimer = null; lease = null; }
  async function heartbeat() {
    if (heartbeatBusy || !lease || document.hidden) return;
    heartbeatBusy = true;
    try { await request('heartbeat', { lease }); }
    catch (error) { clearHeartbeat(); reportError(error); try { await request('stop'); } catch (_) { /* lease expiry also stops the server */ } }
    finally { heartbeatBusy = false; }
  }
  async function stop() {
    stopEpoch++;
    ownsPlayback = false;
    clearHeartbeat(); onStop();
    try { await request('stop'); } catch (error) { reportError(error); }
    updateControls();
  }
  $('twinStop').onclick = stop; $('twinPlaybackStop').onclick = stop;
  addEventListener('keydown', event => { if (event.code === 'Escape') { event.preventDefault(); stop(); } });
  function leave() {
    stopEpoch++;
    const shouldStop = !!lease || ownsPlayback;
    ownsPlayback = false;
    clearHeartbeat(); onStop();
    if (!shouldStop) return;
    const body = JSON.stringify({ op: 'stop', client_id: clientId });
    const sent = navigator.sendBeacon?.('/api/twin/cmd', new Blob([body], { type: 'application/json' }));
    if (!sent) fetch('/api/twin/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body, keepalive: true }).catch(() => {});
  }
  addEventListener('pagehide', leave);
  document.addEventListener('visibilitychange', () => { if (document.hidden) leave(); });
  action('twinExport', async () => {
    validateProgram(draft);
    const url = URL.createObjectURL(new Blob([JSON.stringify(draft, null, 2) + '\n'], { type: 'application/json' }));
    const link = document.createElement('a'); link.href = url; link.download = 'so101-motion.json'; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  $('twinImport').onclick = () => $('twinImportFile').click();
  $('twinImportFile').onchange = async event => {
    const file = event.target.files[0]; if (!file) return;
    try {
      if (!editable()) throw new Error('再生または追従を停止してから読み込んでください');
      if (file.size > 1024 * 1024) throw new Error('JSONは1 MB以下にしてください');
      const program = validateProgram(JSON.parse(await file.text()));
      draft = clone(program); $('twinProgramName').value = draft.name; markDirty(); buildSteps(); reportError(''); updateControls();
    } catch (error) { reportError(error); }
    finally { event.target.value = ''; }
  };
  function updateControls() {
    const hardware = state?.hardware || {}, following = !!hardware.following, connected = !!hardware.connected;
    const canEdit = editable(), fresh = Number.isFinite(hardware.age_s) && hardware.age_s <= 0.5;
    const aligned = Number.isFinite(hardware.max_error_deg) && hardware.max_error_deg <= 3;
    const calibrated = hardware.calibration?.valid === true;
    const simulation = getSimState(), normalSpeed = simulation?.rtf === 1 && !simulation?.paused;
    const ready = ['twinClear', 'twinBase', 'twinEstop'].every(id => $(id).checked);
    $('twinProgramName').disabled = !canEdit;
    for (const id of ['twinCapture', 'twinApply', 'twinSave', 'twinLoad', 'twinImport', 'twinLibrary', 'twinReloadCurrent']) $(id).disabled = !canEdit || !draft;
    $('twinApply').disabled ||= !dirty;
    $('twinLoad').disabled ||= !$('twinLibrary').value;
    $('twinExport').disabled = busy || !draft;
    $('twinPlay').disabled = busy || !draft?.steps.length || !canEditGoals() || !!state?.playback?.playing;
    $('twinPlay').textContent = following ? '実機と仮想を再生' : '仮想再生';
    $('twinConnect').disabled = busy || connected || following || hardware.mode === 'off';
    $('twinDisconnect').disabled = busy || !connected || (following && !ownsFollow());
    $('twinImportActual').disabled = busy || !connected || following || !fresh;
    $('twinFollow').disabled = busy || !connected || following || !fresh || !aligned || !ready || !calibrated || !normalSpeed || !!state?.playback?.playing || !['observing', 'stopped'].includes(hardware.phase);
    $('twinFollowHint').textContent = following ? (ownsFollow() ? 'このタブから追従中。速度は1×です。画面を離れると停止します。' : '別のタブから追従中です。このタブからは停止できます。') : !connected ? 'まず接続して角度を観測してください。' : !fresh ? '実機の新しい角度データを待っています。' : !calibrated ? '校正の確認が必要です。表示された理由を確認してください。' : !aligned ? '「実機姿勢を仮想へ」で角度差を3°以内に合わせてください。' : !normalSpeed ? '仮想の再生速度を1×にして、一時停止を解除してください。' : !ready ? '周囲・台座・停止手段を確認してください。' : '追従を開始できます。目標姿勢の変更が実機にも送信されます。';
    $('twinDraftState').textContent = dirty ? '未反映の編集があります' : state && state.revision !== state.saved_revision ? '反映済み・未保存' : state ? '保存済み' : '動作を取得中…';
    if (hardware.phase === 'fault') $('twinFollowHint').textContent = '異常の原因を確認し、切断してから接続し直してください。追従は自動では再開しません。';
    stepNodes.forEach((node, index) => {
      node.querySelectorAll('input,button').forEach(input => { input.disabled = !canEdit; });
      node.querySelector('[data-do=up]').disabled ||= index === 0;
      node.querySelector('[data-do=down]').disabled ||= index === stepNodes.length - 1;
    });
    document.querySelectorAll('#rtfSeg button,#pauseBtn,#resetBtn,#floorBtn,#torqueBtn,#payload,#regs input,#regsReset,.model-input input').forEach(input => { input.disabled = following; });
    document.querySelectorAll('#poses button,[data-step]').forEach(input => { input.disabled = !canEditGoals() || !!state?.playback?.playing; });
  }
  function render(next) {
    if (!next?.hardware || !next.program) return;
    connectionError = '';
    $('twinError').textContent = localError;
    if (Number.isFinite(next.revision) && next.revision < revision) return;
    state = next;
    const hardware = state.hardware, playback = state.playback || {};
    $('twinRealVerification').hidden = hardware.mode !== 'real';
    if (!hardware.following && lease) clearHeartbeat();
    if (!dirty && (!draft || revision !== state.revision)) {
      draft = clone(state.program); draftBaseRevision = state.revision; $('twinProgramName').value = draft.name; buildSteps();
    }
    revision = state.revision;
    const libraries = JSON.stringify(state.library || []);
    if (libraries !== libraryKey) {
      libraryKey = libraries; const selected = $('twinLibrary').value;
      $('twinLibrary').replaceChildren(new Option('保存した動作', ''));
      for (const item of state.library || []) $('twinLibrary').add(new Option(item.name, item.name));
      $('twinLibrary').value = selected;
    }
    $('twinConnection').textContent = ({ off: '仮想のみ (off)', mock: '模擬実機 (mock)', real: '実機 (real)' })[hardware.mode] || hardware.mode || '仮想のみ (off)';
    $('twinPhase').textContent = (PHASE[hardware.phase] || hardware.phase || '未接続') +
      (['stopped', 'fault'].includes(hardware.phase) ? (hardware.stop_confirmed ? '・停止確認済み' : '・停止未確認') : '');
    $('twinStopConfirmed').textContent = ['stopped', 'fault'].includes(hardware.phase) ? (hardware.stop_confirmed ? '停止確認済み' : '停止未確認') : '';
    $('twinAge').textContent = `受信 ${num(hardware.age_s, 2)} s前`;
    $('twinAge').classList.toggle('stale', hardware.connected && (!Number.isFinite(hardware.age_s) || hardware.age_s > 0.5));
    $('twinOwner').textContent = hardware.following ? (hardware.owner === clientId && lease ? '操作中：このタブ' : '操作中：別のタブ') : '';
    const reasons = { 'operator stop': '停止ボタンで解除しました', 'follow heartbeat expired': '画面との通信期限が切れました', disconnected: '接続を終了しました', 'link closed': 'サーバを終了しました' };
    $('twinReason').textContent = reasons[hardware.reason] || hardware.reason || '';
    $('twinAlignment').textContent = `目標と実機の最大角度差 ${num(hardware.max_error_deg)}° / 開始条件 3°以内`;
    const simulation = getSimState();
    getJointNames().forEach((name, index) => {
      const measured = hardware.joints?.[name]?.q, virtual = simulation?.joints?.[index]?.goal;
      $(`twinJointError${index}`).textContent = Number.isFinite(measured) && Number.isFinite(virtual) ? `${num(virtual - measured)}°` : '—';
    });
    const calibrationReason = hardware.calibration?.reason;
    const calibrationLabel = { 'not checked': '未確認', mock: '模擬データ', verified: '確認済み' }[calibrationReason] || calibrationReason;
    $('twinCalibration').textContent = typeof hardware.calibration === 'string' ? `校正：${hardware.calibration}` : hardware.calibration ? `校正：${calibrationLabel || hardware.calibration.status || (hardware.calibration.valid ? '確認済み' : '未確認')}` : '校正：未確認';
    $('twinCounts').textContent = `受信 ${hardware.sample_seq || 0} / 目標 ${hardware.targets_received || 0} / 送信 ${hardware.commands_sent || 0}`;
    $('twinProgress').textContent = playback.playing ? `姿勢 ${(playback.index ?? 0) + 1} / ${state.program.steps.length} · ${num(playback.elapsed)} / ${num(playback.duration)} s` : `停止中${playback.reason ? ' · ' + playback.reason : ''}`;
    if (!playback.playing) ownsPlayback = false;
    stepNodes.forEach((node, index) => node.classList.toggle('playing', !!playback.playing && index === playback.index));
    if (hardware.following) $('twinClear').disabled = $('twinBase').disabled = $('twinEstop').disabled = true;
    else for (const id of ['twinClear', 'twinBase', 'twinEstop']) $(id).disabled = false;
    updateControls();
  }
  updateControls();
  return { render, reportError, reportConnectionError, canEditGoals, physicsLocked, stop };
}
