<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Search, RefreshCw, Pin, Trash2, HardDrive, ArrowUp, ArrowDown, Download } from 'lucide-svelte';

  type Episode = {
    episode_id: string;
    state: string;
    task_description: string;
    profile: string;
    robot_id: string | null;
    started_at: string;
    stopped_at: string | null;
    duration_s: number | null;
    outcome: string | null;
    pinned: boolean;
    size_bytes: number;
    marker_count: number;
    tags: string[];
    source: string;
  };

  type Disk = {
    bytes_free: number; bytes_total: number; bytes_used: number;
    percent_free: number; percent_used: number;
    episode_count: number; total_episode_bytes: number;
    policy?: { warn_pct_free?: number; crit_pct_free?: number; note?: string };
    active: { rate_bytes_per_s: number; elapsed_s: number; hours_left_at_current_rate: number | null } | null;
  };

  // Recorder API base — same host as the iframe's parent dashboard, port 8083.
  // (CORS allowed in api_server.py.)
  const API = `http://${location.hostname}:8083/api/v1`;

  // Profile scope — when the dashboard embeds us with ?profile=xxx, hide
  // episodes from other profiles. No combo box; the dashboard already knows
  // which profile is active.
  const activeProfile = new URLSearchParams(location.search).get('profile') || '';

  let episodes = $state<Episode[]>([]);
  let disk = $state<Disk | null>(null);
  let loading = $state(true);
  let error = $state<string | null>(null);
  let query = $state('');
  let pollTimer: number | null = null;

  // Sort state — clickable column headers cycle asc/desc.
  type SortKey = 'started_at' | 'task_description' | 'duration_s' | 'size_bytes' | 'marker_count';
  let sortKey = $state<SortKey>('started_at');
  let sortDir = $state<'asc' | 'desc'>('desc');
  function toggleSort(k: SortKey) {
    if (sortKey === k) sortDir = sortDir === 'asc' ? 'desc' : 'asc';
    else { sortKey = k; sortDir = k === 'task_description' ? 'asc' : 'desc'; }
  }

  // Play modal state
  type MarkerAttr = { key: string; value: string; unit?: string; note?: string };
  type MarkerItem = {
    marker_id: string;
    kind: string;
    task_description: string;
    started_at: string;
    stopped_at: string | null;
    outcome: string | null;
    tags?: string[];
    attributes?: MarkerAttr[];
    rev?: number;
  };
  type EpisodeDetail = {
    episode_id: string;
    task_description: string;
    profile: string;
    duration_s: number | null;
    started_at: string;
    cameras: Array<{ name: string; topic: string; segments?: Array<{ file: string }> }>;
    markers: MarkerItem[];
  };
  let playEpisode = $state<EpisodeDetail | null>(null);
  let selectedCamera = $state<string>('');

  // Multi-camera shared timeline state
  let videoEls = $state<Record<string, HTMLVideoElement>>({});
  let sharedTime = $state(0);
  let maxDuration = $state(0);
  let playState = $state<'paused' | 'playing'>('paused');
  let playbackRate = $state(1);
  let _isSyncing = false;  // re-entrancy guard for cross-video sync

  function fmtTimeSec(s: number) {
    if (!isFinite(s) || s < 0) s = 0;
    const m = Math.floor(s / 60);
    const sec = Math.floor(s % 60);
    return `${m}:${String(sec).padStart(2,'0')}`;
  }

  // Convert an ISO timestamp into seconds elapsed from the episode start.
  function markerOffset(iso: string | null): number {
    if (!iso || !playEpisode?.started_at) return 0;
    return (new Date(iso).getTime() - new Date(playEpisode.started_at).getTime()) / 1000;
  }
  function markerDuration(m: MarkerItem): number | null {
    if (!m.stopped_at) return null;
    return (new Date(m.stopped_at).getTime() - new Date(m.started_at).getTime()) / 1000;
  }
  function kindColor(kind: string) {
    if (kind === 'subtask') return '#22dd88';   // green
    if (kind === 'event')   return '#ffaa33';   // amber
    if (kind === 'note')    return '#00d9ff';   // cyan
    return '#888';
  }
  function jumpToMarker(m: MarkerItem) {
    const t = markerOffset(m.started_at);
    seekAll(t);
  }

  // -------- Marker drag editor (Phase 4 minimum) --------
  // Drag a subtask marker's left edge → adjust start. Right edge → adjust end.
  // Middle → translate the whole region. PATCH on pointerup.
  let markerBandEl: HTMLDivElement | null = $state(null);
  type DragState = {
    marker: MarkerItem;
    mode: 'move' | 'resize-l' | 'resize-r';
    startClientX: number;
    origStartMs: number;
    origStopMs: number;
  };
  let dragging: DragState | null = null;
  const EDGE_PX = 8;            // hit zone for resize handles
  const MIN_DUR_MS = 100;       // refuse to collapse a marker to zero

  function _isoFromMs(ms: number): string {
    const d = new Date(ms);
    // Match Python "%Y-%m-%dT%H:%M:%S.%fZ" — toISOString gives milliseconds,
    // Python's %f accepts 1-6 digits so this parses cleanly.
    return d.toISOString();
  }

  function onMarkerPointerDown(e: PointerEvent, m: MarkerItem) {
    // Only subtask markers with a stop time are draggable. Event/note pins
    // are single-time points — they need a different gesture (skip for now).
    if (m.kind !== 'subtask' || !m.stopped_at) {
      jumpToMarker(m);
      return;
    }
    const target = e.currentTarget as HTMLElement;
    const rect = target.getBoundingClientRect();
    const localX = e.clientX - rect.left;
    let mode: 'move' | 'resize-l' | 'resize-r' = 'move';
    if (localX < EDGE_PX) mode = 'resize-l';
    else if (rect.width - localX < EDGE_PX) mode = 'resize-r';
    dragging = {
      marker: m,
      mode,
      startClientX: e.clientX,
      origStartMs: new Date(m.started_at).getTime(),
      origStopMs: new Date(m.stopped_at).getTime(),
    };
    try { target.setPointerCapture(e.pointerId); } catch {}
    e.stopPropagation();
    e.preventDefault();
  }

  function onMarkerPointerMove(e: PointerEvent) {
    if (!dragging || !markerBandEl || !playEpisode || maxDuration <= 0) return;
    const band = markerBandEl.getBoundingClientRect();
    const deltaMs = ((e.clientX - dragging.startClientX) / band.width) * maxDuration * 1000;
    const epStartMs = new Date(playEpisode.started_at).getTime();
    const epEndMs = epStartMs + maxDuration * 1000;
    let s = dragging.origStartMs;
    let p = dragging.origStopMs;
    if (dragging.mode === 'move') {
      const width = p - s;
      s = Math.max(epStartMs, Math.min(epEndMs - width, s + deltaMs));
      p = s + width;
    } else if (dragging.mode === 'resize-l') {
      s = Math.max(epStartMs, Math.min(p - MIN_DUR_MS, s + deltaMs));
    } else {
      p = Math.min(epEndMs, Math.max(s + MIN_DUR_MS, p + deltaMs));
    }
    // Live preview — update the marker in place so the bar follows the cursor.
    const idx = playEpisode.markers.findIndex(x => x.marker_id === dragging!.marker.marker_id);
    if (idx >= 0) {
      playEpisode.markers[idx] = {
        ...playEpisode.markers[idx],
        started_at: _isoFromMs(s),
        stopped_at: _isoFromMs(p),
      };
      // Trigger Svelte reactivity.
      playEpisode = { ...playEpisode, markers: [...playEpisode.markers] };
    }
  }

  async function onMarkerPointerUp(e: PointerEvent) {
    if (!dragging) return;
    const m = dragging.marker;
    dragging = null;
    // PATCH only if value actually changed. (Click-through still seeks.)
    if (!playEpisode) return;
    const cur = playEpisode.markers.find(x => x.marker_id === m.marker_id);
    if (!cur) return;
    try {
      const res = await fetch(`${API}/markers/${m.marker_id}`, {
        method: 'PATCH',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({started_at: cur.started_at, stopped_at: cur.stopped_at}),
      });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        alert('マーカー更新失敗: ' + (err.error || res.status));
      }
    } catch (err: any) {
      alert('マーカー更新失敗: ' + (err.message || err));
    }
  }

  // -------- Marker attribute editor --------
  // Use case: an asparagus subtask gets weight_g / grade / length_cm
  // attached *after* the recording when the operator measures them. Each
  // attribute also supports a free-text note ("少し曲がってた" etc.).
  let attrEditor = $state<{marker: MarkerItem; rows: MarkerAttr[]} | null>(null);
  function openAttrEditor(m: MarkerItem, e: MouseEvent) {
    e.stopPropagation();
    attrEditor = {
      marker: m,
      rows: (m.attributes && m.attributes.length > 0)
        ? m.attributes.map(a => ({...a}))
        : [{key: '', value: '', unit: '', note: ''}],
    };
  }
  function addAttrRow() {
    if (!attrEditor) return;
    attrEditor = { ...attrEditor, rows: [...attrEditor.rows, {key: '', value: '', unit: '', note: ''}] };
  }
  function removeAttrRow(i: number) {
    if (!attrEditor) return;
    attrEditor = { ...attrEditor, rows: attrEditor.rows.filter((_, idx) => idx !== i) };
  }
  async function saveAttrs() {
    if (!attrEditor || !playEpisode) return;
    const clean = attrEditor.rows
      .filter(r => r.key.trim() !== '' || r.value.trim() !== '')
      .map(r => ({
        key: r.key.trim(),
        value: r.value.trim(),
        ...(r.unit && r.unit.trim() ? {unit: r.unit.trim()} : {}),
        ...(r.note && r.note.trim() ? {note: r.note.trim()} : {}),
      }));
    try {
      const r = await fetch(`${API}/markers/${attrEditor.marker.marker_id}`, {
        method: 'PATCH',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({attributes: clean}),
      });
      if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${r.status}`);
      }
      const idx = playEpisode.markers.findIndex(x => x.marker_id === attrEditor!.marker.marker_id);
      if (idx >= 0) {
        playEpisode.markers[idx] = { ...playEpisode.markers[idx], attributes: clean };
        playEpisode = { ...playEpisode, markers: [...playEpisode.markers] };
      }
      attrEditor = null;
    } catch (e: any) {
      alert('属性保存失敗: ' + (e.message || e));
    }
  }

  // Click on chart background → add an event marker at that time.
  // Click on a series line → just seek (handled by chart pointermove +
  // shift-click is reserved for "add note", future).
  async function addEventAtTime(seconds: number) {
    if (!playEpisode) return;
    const label = prompt('イベント名:', '気になる動き');
    if (!label) return;
    const epStartMs = new Date(playEpisode.started_at).getTime();
    const tIso = _isoFromMs(epStartMs + seconds * 1000);
    try {
      const r = await fetch(`${API}/episodes/${playEpisode.episode_id}/markers/start`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({task_description: label, kind: 'event', started_at: tIso}),
      });
      if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${r.status}`);
      }
      const m = await r.json();
      playEpisode = { ...playEpisode, markers: [...playEpisode.markers, m] };
    } catch (e: any) {
      alert('イベント追加失敗: ' + (e.message || e));
    }
  }

  function markerCursor(e: PointerEvent, m: MarkerItem): string {
    if (m.kind !== 'subtask' || !m.stopped_at) return 'pointer';
    const target = e.currentTarget as HTMLElement;
    const rect = target.getBoundingClientRect();
    const localX = e.clientX - rect.left;
    if (localX < EDGE_PX || rect.width - localX < EDGE_PX) return 'ew-resize';
    return 'grab';
  }
  function onMarkerHoverMove(e: PointerEvent, m: MarkerItem) {
    const target = e.currentTarget as HTMLElement;
    target.style.cursor = dragging ? 'grabbing' : markerCursor(e, m);
  }

  function onVideoMetadata(camName: string) {
    const v = videoEls[camName];
    if (!v) return;
    if (v.duration > maxDuration) maxDuration = v.duration;
  }

  function syncPlayFrom(camName: string) {
    if (_isSyncing) return;
    _isSyncing = true;
    playState = 'playing';
    for (const [n, v] of Object.entries(videoEls)) {
      if (n !== camName && v && v.paused) { try { v.play().catch(() => {}); } catch {} }
    }
    _isSyncing = false;
  }
  function syncPauseFrom(camName: string) {
    if (_isSyncing) return;
    _isSyncing = true;
    playState = 'paused';
    for (const [n, v] of Object.entries(videoEls)) {
      if (n !== camName && v && !v.paused) { try { v.pause(); } catch {} }
    }
    _isSyncing = false;
  }
  function syncSeekFrom(camName: string) {
    if (_isSyncing) return;
    _isSyncing = true;
    const src = videoEls[camName];
    if (src) {
      sharedTime = src.currentTime;
      for (const [n, v] of Object.entries(videoEls)) {
        if (n !== camName && v && Math.abs(v.currentTime - src.currentTime) > 0.1) {
          try { v.currentTime = src.currentTime; } catch {}
        }
      }
    }
    _isSyncing = false;
  }
  function onTimeUpdate(camName: string) {
    // Use one camera (the first key) as the timeline driver.
    const firstKey = Object.keys(videoEls)[0];
    if (camName === firstKey) {
      const v = videoEls[firstKey];
      if (v) sharedTime = v.currentTime;
    }
  }
  function togglePlayAll() {
    if (playState === 'playing') {
      for (const v of Object.values(videoEls)) { if (v && !v.paused) { try { v.pause(); } catch {} } }
      playState = 'paused';
    } else {
      for (const v of Object.values(videoEls)) { if (v && v.paused) { try { v.play().catch(() => {}); } catch {} } }
      playState = 'playing';
    }
  }
  function seekAll(t: number) {
    sharedTime = t;
    _isSyncing = true;
    for (const v of Object.values(videoEls)) { if (v) { try { v.currentTime = t; } catch {} } }
    _isSyncing = false;
  }
  function applyPlaybackRate() {
    for (const v of Object.values(videoEls)) { if (v) v.playbackRate = playbackRate; }
  }

  // -------- Joint chart (Phase 3a) --------
  type JointSeries = { topic: string; names: string[]; t_ms: number[]; q: number[][] };
  let jointData = $state<JointSeries[]>([]);
  let jointSelectedTopic = $state<string>('');
  let jointLoading = $state(false);

  async function fetchJoints(epId: string) {
    jointLoading = true;
    jointData = [];
    jointSelectedTopic = '';
    try {
      const r = await fetch(`${API}/episodes/${epId}/joints`);
      const d = await r.json();
      jointData = d.joints || [];
      jointSelectedTopic = jointData[0]?.topic || '';
    } catch (e) {
      // Non-fatal — chart just stays empty.
      console.warn('joints fetch failed', e);
    } finally {
      jointLoading = false;
    }
  }

  async function openPlay(ep: Episode) {
    try {
      const res = await fetch(`${API}/episodes/${ep.episode_id}`);
      const data = await res.json();
      // Reset multi-cam state
      videoEls = {};
      sharedTime = 0;
      maxDuration = ep.duration_s || 0;
      playState = 'paused';
      playbackRate = 1;
      playEpisode = data;
      // Default to the first camera that has a segment (legacy single-cam).
      const firstWithSeg = (data.cameras || []).find((c: any) => c.segments?.length > 0);
      selectedCamera = firstWithSeg?.name || (data.cameras?.[0]?.name ?? '');
      // Joint chart loads in parallel — fire and forget.
      fetchJoints(ep.episode_id);
    } catch (e) {
      alert('再生情報の取得に失敗: ' + e);
    }
  }
  function closePlay() {
    // Pause all videos before closing to avoid background audio/playback.
    for (const v of Object.values(videoEls)) { if (v && !v.paused) { try { v.pause(); } catch {} } }
    playEpisode = null;
    selectedCamera = '';
    videoEls = {};
    sharedTime = 0;
    maxDuration = 0;
    playState = 'paused';
  }

  function videoUrl(epId: string, camera: string, segment: string = '0000.mp4') {
    return `${API}/episodes/${epId}/files/videos/${camera}/${segment}`;
  }

  const fmtBytes = (b: number | null | undefined) => {
    if (b == null) return '—';
    if (b >= 1e9) return (b/1e9).toFixed(2) + ' GB';
    if (b >= 1e6) return (b/1e6).toFixed(1) + ' MB';
    if (b >= 1e3) return (b/1e3).toFixed(1) + ' KB';
    return b + ' B';
  };

  const fmtDuration = (s: number | null) => {
    if (s == null) return '—';
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    const sec = Math.floor(s % 60);
    return h > 0 ? `${h}:${String(m).padStart(2,'0')}:${String(sec).padStart(2,'0')}` : `${m}:${String(sec).padStart(2,'0')}`;
  };

  const fmtTime = (iso: string | null) => {
    if (!iso) return '—';
    const d = new Date(iso);
    return d.toLocaleString('ja-JP', { hour12: false });
  };

  const outcomeBadge = (o: string | null) => {
    if (o === 'success') return { label: '成功', cls: 'bg-emerald-500/15 text-emerald-300 border border-emerald-500/30' };
    if (o === 'abort')   return { label: '失敗', cls: 'bg-red-500/15 text-red-300 border border-red-500/30' };
    if (o === 'discard') return { label: '破棄', cls: 'bg-zinc-500/15 text-zinc-400 border border-zinc-500/30' };
    return { label: '—', cls: 'bg-zinc-500/15 text-zinc-400 border border-zinc-500/30' };
  };

  async function load() {
    try {
      const [epRes, dkRes] = await Promise.all([
        fetch(`${API}/episodes?limit=200`),
        fetch(`${API}/disk/status`),
      ]);
      const epJson = await epRes.json();
      const dkJson = await dkRes.json();
      episodes = epJson.episodes || [];
      disk = dkJson;
      error = null;
    } catch (e: any) {
      error = e.message || String(e);
    } finally {
      loading = false;
    }
  }

  const filteredEpisodes = $derived(
    episodes
      .filter(e => {
        if (activeProfile && e.profile !== activeProfile) return false;
        if (query.trim()) {
          const q = query.toLowerCase();
          if (!e.task_description.toLowerCase().includes(q) &&
              !e.tags.some(t => t.toLowerCase().includes(q))) return false;
        }
        return true;
      })
      .slice()
      .sort((a, b) => {
        const dir = sortDir === 'asc' ? 1 : -1;
        const av = (a as any)[sortKey];
        const bv = (b as any)[sortKey];
        if (av == null && bv == null) return 0;
        if (av == null) return 1;
        if (bv == null) return -1;
        if (typeof av === 'string') return av.localeCompare(bv) * dir;
        return (av - bv) * dir;
      })
  );

  async function deleteEpisode(ep: Episode, e: MouseEvent) {
    e.stopPropagation();
    const label = ep.task_description || ep.episode_id.slice(-8);
    if (!confirm(`削除しますか？\n\n  タスク: ${label}\n  サイズ: ${fmtBytes(ep.size_bytes)}\n\nこの操作は取り消せません。`)) return;
    try {
      const res = await fetch(`${API}/episodes/${ep.episode_id}${ep.pinned ? '?force=true' : ''}`, { method: 'DELETE' });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${res.status}`);
      }
      episodes = episodes.filter(x => x.episode_id !== ep.episode_id);
    } catch (e: any) {
      alert('削除失敗: ' + (e.message || e));
    }
  }

  onMount(() => {
    load();
    pollTimer = window.setInterval(load, 5000);
  });
  onDestroy(() => { if (pollTimer) clearInterval(pollTimer); });
</script>

<main class="min-h-screen p-6 max-w-[1400px] mx-auto">
  <!-- Header -->
  <header class="flex items-center justify-between mb-6">
    <div>
      <h1 class="text-2xl font-semibold text-white tracking-tight">録画タスク一覧</h1>
      <p class="text-xs text-(--color-text-mute) mt-0.5">
        fv_episode_recorder · {API}
        {#if activeProfile} · <span class="text-(--color-accent)">profile: {activeProfile}</span>{/if}
      </p>
    </div>
    <button
      onclick={load}
      class="flex items-center gap-2 px-3 py-1.5 text-sm rounded-lg bg-(--color-accent-soft) border border-(--color-accent-glow) text-(--color-accent) hover:bg-[color-mix(in_oklch,_var(--color-accent-soft),_white_5%)] transition">
      <RefreshCw class="size-4 {loading ? 'animate-spin' : ''}" />
      更新
    </button>
  </header>

  <!-- Disk summary card -->
  {#if disk}
    {@const barColor = disk.percent_free < 10 ? 'bg-red-500' : disk.percent_free < 20 ? 'bg-amber-500' : 'bg-emerald-500'}
    {@const textColor = disk.percent_free < 10 ? 'text-red-400' : disk.percent_free < 20 ? 'text-amber-400' : 'text-emerald-400'}
    <section class="card p-4 mb-5">
      <div class="flex items-center justify-between mb-2">
        <div class="flex items-center gap-2">
          <HardDrive class="size-4 text-(--color-text-dim)" />
          <span class="text-sm font-medium">ディスク</span>
          <span class="text-xs text-(--color-text-mute) font-mono">{fmtBytes(disk.bytes_free)} 空き / {fmtBytes(disk.bytes_total)}</span>
        </div>
        <div class="flex items-center gap-4">
          <span class="text-xs text-(--color-text-mute)">保存済 <b class="text-(--color-text)">{disk.episode_count}</b> 本 ({fmtBytes(disk.total_episode_bytes)})</span>
          <span class="text-sm {textColor} font-semibold">{disk.percent_free.toFixed(1)}% 空き</span>
        </div>
      </div>
      <div class="h-2 rounded-full bg-zinc-900 overflow-hidden">
        <div class="h-full {barColor} transition-all duration-500" style="width: {disk.percent_used.toFixed(1)}%"></div>
      </div>
      {#if disk.active}
        <div class="mt-3 pt-3 border-t border-(--color-border) flex items-center gap-6 text-xs">
          <span class="flex items-center gap-1.5">
            <span class="size-2 rounded-full bg-red-500" style="animation: rec-pulse 1.4s ease-in-out infinite"></span>
            <span class="text-red-400 font-semibold">録画中</span>
          </span>
          <span class="text-(--color-text-mute)">速度: <span class="text-(--color-text) font-mono">{fmtBytes(disk.active.rate_bytes_per_s)}/s</span></span>
          <span class="text-(--color-text-mute)">経過: <span class="text-(--color-text) font-mono">{fmtDuration(disk.active.elapsed_s)}</span></span>
          {#if disk.active.hours_left_at_current_rate != null}
            <span class="text-(--color-text-mute)">この速度であと: <span class="text-(--color-text) font-mono">{disk.active.hours_left_at_current_rate.toFixed(1)} 時間</span></span>
          {/if}
        </div>
      {/if}

      <!-- Auto-maintenance (retention) explanation. Phase 1.5 = alert only,
           no auto-delete yet. Be explicit so the operator knows what to expect
           (otherwise they assume "保存先" silently rotates and lose data). -->
      {@const warnPct = disk.policy?.warn_pct_free ?? 20}
      {@const critPct = disk.policy?.crit_pct_free ?? 10}
      {@const stage = disk.percent_free < critPct ? 'crit' : disk.percent_free < warnPct ? 'warn' : 'ok'}
      <details class="mt-3 pt-3 border-t border-(--color-border) text-xs" open={stage !== 'ok'}>
        <summary class="cursor-pointer text-(--color-text-dim) hover:text-(--color-text) select-none flex items-center gap-2">
          <span>自動メンテ</span>
          {#if stage === 'ok'}
            <span class="text-emerald-400">● 正常</span>
          {:else if stage === 'warn'}
            <span class="text-amber-400">● 警告 (空き {disk.percent_free.toFixed(1)}% &lt; {warnPct}%)</span>
          {:else}
            <span class="text-red-400">● 危険 (空き {disk.percent_free.toFixed(1)}% &lt; {critPct}%)</span>
          {/if}
        </summary>
        <div class="mt-2 space-y-1.5 text-(--color-text-mute) pl-1">
          <div>
            <span class="text-(--color-text-dim)">しきい値:</span>
            空き <b class="text-amber-400">{warnPct}%</b> で警告 /
            <b class="text-red-400">{critPct}%</b> で危険 (header 右上のチップが色変化)
          </div>
          <div>
            <span class="text-(--color-text-dim)">通知:</span>
            <code class="px-1 rounded bg-(--color-bg-3)/50">/vlabor/events</code> に
            <code class="px-1 rounded bg-(--color-bg-3)/50">DISK_LOW</code> /
            <code class="px-1 rounded bg-(--color-bg-3)/50">DISK_CRITICAL</code> をパブリッシュ
            → dashboard の event log + 外部 MCP 購読者にも自動波及
          </div>
          <div>
            <span class="text-(--color-text-dim)">現状の自動削除:</span>
            <span class="text-amber-300">未実装 (Phase 2)</span>
            — 古いタスクは手動で削除してください (各行 hover で 🗑 ボタン)
          </div>
          <div class="text-(--color-text-mute)">
            <span class="text-(--color-text-dim)">予定 (Phase 2):</span>
            profile yaml <code>episode_recorder.retention</code> から
            <code>max_age_days</code> / <code>max_episodes</code> /
            <code>free_min_pct</code> を読み、60s grace 経過後に unpinned を削除。
            pinned (📌) は対象外。
          </div>
        </div>
      </details>
    </section>
  {/if}

  <!-- Search bar -->
  <div class="mb-3 relative">
    <Search class="size-4 absolute left-3 top-1/2 -translate-y-1/2 text-(--color-text-mute)" />
    <input
      type="text"
      bind:value={query}
      placeholder="タスク名 / タグで検索…"
      class="w-full pl-9 pr-4 py-2 rounded-lg bg-(--color-bg-2) border border-(--color-border) text-sm placeholder:text-(--color-text-mute) focus:border-(--color-accent-glow) focus:outline-none transition"
    />
  </div>

  <!-- Episodes table -->
  {#if error}
    <div class="card p-4 text-red-400 text-sm">通信失敗: {error}</div>
  {:else if loading && episodes.length === 0}
    <div class="card p-8 text-center text-(--color-text-mute)">読み込み中…</div>
  {:else if filteredEpisodes.length === 0}
    <div class="card p-8 text-center text-(--color-text-mute)">
      {query ? '一致するタスクなし' : 'まだ録画タスクなし'}
    </div>
  {:else}
    <div class="card overflow-hidden">
      <table class="w-full text-sm">
        <thead class="text-xs text-(--color-text-dim) bg-(--color-bg-3)/30">
          <tr>
            <th class="text-left px-4 py-2.5 font-medium">状態</th>
            <th class="text-left px-4 py-2.5 font-medium">
              <button class="hover:text-white transition" onclick={() => toggleSort('task_description')}>
                タスク
                {#if sortKey === 'task_description'}
                  {#if sortDir === 'asc'}<ArrowUp class="inline size-3" />{:else}<ArrowDown class="inline size-3" />{/if}
                {/if}
              </button>
            </th>
            <th class="text-left px-4 py-2.5 font-medium">
              <button class="hover:text-white transition" onclick={() => toggleSort('started_at')}>
                開始
                {#if sortKey === 'started_at'}
                  {#if sortDir === 'asc'}<ArrowUp class="inline size-3" />{:else}<ArrowDown class="inline size-3" />{/if}
                {/if}
              </button>
            </th>
            <th class="text-right px-4 py-2.5 font-medium">
              <button class="hover:text-white transition" onclick={() => toggleSort('duration_s')}>
                長さ
                {#if sortKey === 'duration_s'}
                  {#if sortDir === 'asc'}<ArrowUp class="inline size-3" />{:else}<ArrowDown class="inline size-3" />{/if}
                {/if}
              </button>
            </th>
            <th class="text-right px-4 py-2.5 font-medium">
              <button class="hover:text-white transition" onclick={() => toggleSort('size_bytes')}>
                サイズ
                {#if sortKey === 'size_bytes'}
                  {#if sortDir === 'asc'}<ArrowUp class="inline size-3" />{:else}<ArrowDown class="inline size-3" />{/if}
                {/if}
              </button>
            </th>
            <th class="text-right px-4 py-2.5 font-medium">
              <button class="hover:text-white transition" onclick={() => toggleSort('marker_count')}>
                marker
                {#if sortKey === 'marker_count'}
                  {#if sortDir === 'asc'}<ArrowUp class="inline size-3" />{:else}<ArrowDown class="inline size-3" />{/if}
                {/if}
              </button>
            </th>
            <th class="w-10"></th>
          </tr>
        </thead>
        <tbody>
          {#each filteredEpisodes as ep (ep.episode_id)}
            {@const b = outcomeBadge(ep.outcome)}
            <tr class="group border-t border-(--color-border) hover:bg-(--color-bg-3)/40 transition-colors cursor-pointer"
                onclick={() => openPlay(ep)}
                title="クリックで再生">
              <td class="px-4 py-2.5">
                <span class="inline-block px-2 py-0.5 rounded text-[11px] {b.cls}">{b.label}</span>
                {#if ep.state === 'recording'}
                  <span class="ml-1 inline-block px-2 py-0.5 rounded text-[11px] bg-red-500/20 text-red-400 border border-red-500/40" style="animation: rec-pulse 1.4s ease-in-out infinite">REC</span>
                {/if}
                {#if ep.pinned}
                  <Pin class="inline size-3.5 ml-1 text-amber-400" />
                {/if}
              </td>
              <td class="px-4 py-2.5">
                <div class="text-(--color-text) font-medium">{ep.task_description}</div>
                {#if ep.tags.length}
                  <div class="flex gap-1 mt-1 flex-wrap">
                    {#each ep.tags as t}
                      <span class="text-[10px] px-1.5 py-0.5 rounded bg-(--color-accent-soft) text-(--color-accent)">{t}</span>
                    {/each}
                  </div>
                {/if}
              </td>
              <td class="px-4 py-2.5 text-(--color-text-dim) text-xs font-mono">{fmtTime(ep.started_at)}</td>
              <td class="px-4 py-2.5 text-right font-mono text-xs">{fmtDuration(ep.duration_s)}</td>
              <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{fmtBytes(ep.size_bytes)}</td>
              <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{ep.marker_count}</td>
              <td class="px-2 py-2.5 text-right whitespace-nowrap">
                <a
                  href={`${API}/episodes/${ep.episode_id}/download.tar`}
                  onclick={(e) => e.stopPropagation()}
                  class="opacity-0 group-hover:opacity-100 inline-block text-(--color-text-mute) hover:text-(--color-accent) transition p-1 rounded hover:bg-(--color-accent-soft) mr-1"
                  title="ダウンロード (tar)">
                  <Download class="size-4" />
                </a>
                <button
                  onclick={(e) => deleteEpisode(ep, e)}
                  class="opacity-0 group-hover:opacity-100 text-(--color-text-mute) hover:text-red-400 transition p-1 rounded hover:bg-red-500/10"
                  title={ep.pinned ? '削除 (pinned: force)' : '削除'}>
                  <Trash2 class="size-4" />
                </button>
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}

  <footer class="mt-6 text-center text-[11px] text-(--color-text-mute)">
    {filteredEpisodes.length} / {episodes.length} 件 · 5 秒毎自動更新 · fv_episode_ui v0.1.0
  </footer>
</main>

<!-- Playback modal — multi-camera grid + shared timeline (Final Cut style) -->
{#if playEpisode}
  <div
    class="fixed inset-0 z-50 flex items-center justify-center bg-black/80 backdrop-blur-sm p-4"
    onclick={(e) => { if (e.target === e.currentTarget) closePlay(); }}
    onkeydown={(e) => { if (e.key === 'Escape') closePlay(); }}
    role="dialog"
    tabindex="-1">
    <div class="card w-full max-w-6xl p-5 flex flex-col" style="max-height: 92vh;">
      <header class="flex items-start justify-between mb-3 shrink-0">
        <div>
          <h2 class="text-lg font-semibold text-white">{playEpisode.task_description}</h2>
          <p class="text-xs text-(--color-text-mute) mt-1">
            {playEpisode.profile} · {fmtDuration(playEpisode.duration_s)} · {playEpisode.cameras.length} カメラ
          </p>
        </div>
        <button onclick={closePlay} class="text-(--color-text-mute) hover:text-white text-2xl leading-none">×</button>
      </header>

      {#if playEpisode.cameras.length === 0}
        <div class="text-center py-12 text-(--color-text-mute)">カメラ録画なし</div>
      {:else}
        <!-- Multi-camera grid -->
        <div
          class="grid gap-2 mb-3 overflow-auto min-h-0"
          style="grid-template-columns: repeat({Math.min(playEpisode.cameras.length, 2)}, minmax(0, 1fr));">
          {#each playEpisode.cameras as cam (cam.name)}
            {@const segments = cam.segments || [{ file: '0000.mp4' }]}
            <div class="flex flex-col">
              <div class="text-[11px] text-(--color-accent) mb-1 font-medium">{cam.name}</div>
              <video
                preload="metadata"
                bind:this={videoEls[cam.name]}
                onloadedmetadata={() => onVideoMetadata(cam.name)}
                onplay={() => syncPlayFrom(cam.name)}
                onpause={() => syncPauseFrom(cam.name)}
                onseeked={() => syncSeekFrom(cam.name)}
                ontimeupdate={() => onTimeUpdate(cam.name)}
                class="w-full rounded-md bg-black aspect-video"
                src={videoUrl(playEpisode.episode_id, cam.name, segments[0].file)}>
              </video>
            </div>
          {/each}
        </div>

        <!-- Shared timeline -->
        <div class="shrink-0 border-t border-(--color-border) pt-3">
          <div class="flex items-center gap-3">
            <button
              onclick={togglePlayAll}
              class="size-9 rounded-full bg-(--color-accent) text-(--color-bg) flex items-center justify-center hover:scale-105 transition shrink-0"
              title={playState === 'playing' ? '一時停止' : '再生'}>
              {#if playState === 'playing'}■{:else}▶{/if}
            </button>
            <span class="text-xs font-mono text-(--color-text-dim) shrink-0 tabular-nums w-14 text-right">
              {fmtTimeSec(sharedTime)}
            </span>
            <div class="flex-1 relative">
              <input
                type="range"
                min="0"
                max={maxDuration || 1}
                step="0.01"
                value={sharedTime}
                oninput={(e) => seekAll(parseFloat(e.currentTarget.value))}
                class="w-full accent-(--color-accent)"
              />
              <!-- Marker band: colored segments + event dots overlaid on the timeline.
                   Subtask markers are draggable: left/right ~8px = resize handles,
                   middle = move. Single-click on event/note still seeks. -->
              {#if playEpisode.markers && playEpisode.markers.length > 0 && maxDuration > 0}
                <div bind:this={markerBandEl}
                     class="relative h-3 mt-1 bg-(--color-bg-3)/40 rounded-sm overflow-visible">
                  {#each playEpisode.markers as m (m.marker_id)}
                    {@const start = markerOffset(m.started_at)}
                    {@const dur = markerDuration(m)}
                    {@const leftPct = Math.max(0, (start / maxDuration) * 100)}
                    {#if m.kind === 'subtask' && dur != null}
                      {@const widthPct = (dur / maxDuration) * 100}
                      <div
                        role="button"
                        tabindex="0"
                        onpointerdown={(e) => onMarkerPointerDown(e, m)}
                        onpointermove={(e) => { if (dragging) onMarkerPointerMove(e); else onMarkerHoverMove(e, m); }}
                        onpointerup={(e) => onMarkerPointerUp(e)}
                        onkeydown={(e) => { if (e.key === 'Enter') jumpToMarker(m); }}
                        class="absolute top-0 h-full select-none touch-none rounded-[2px] hover:brightness-125 transition-[filter]"
                        style="left:{leftPct}%; width:{Math.max(widthPct, 0.4)}%; background:{kindColor(m.kind)}; opacity:{m.outcome === 'abort' ? 0.45 : 0.85}; box-shadow:inset 2px 0 0 rgba(255,255,255,0.35), inset -2px 0 0 rgba(255,255,255,0.35);"
                        title="{m.task_description} ({fmtTimeSec(start)} – {fmtTimeSec(start + dur)}) — ドラッグで移動、端でリサイズ">
                      </div>
                    {:else}
                      <button
                        type="button"
                        onclick={() => jumpToMarker(m)}
                        class="absolute top-0 h-full w-[3px] rounded-sm hover:scale-y-150 transition"
                        style="left:{leftPct}%; background:{kindColor(m.kind)};"
                        title="{m.kind}: {m.task_description} ({fmtTimeSec(start)})">
                      </button>
                    {/if}
                  {/each}
                </div>
              {/if}
            </div>
            <span class="text-xs font-mono text-(--color-text-dim) shrink-0 tabular-nums w-14">
              {fmtTimeSec(maxDuration)}
            </span>
            <select bind:value={playbackRate} onchange={applyPlaybackRate} class="text-xs bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-1 shrink-0">
              <option value={0.25}>0.25×</option>
              <option value={0.5}>0.5×</option>
              <option value={1}>1×</option>
              <option value={2}>2×</option>
              <option value={4}>4×</option>
            </select>
          </div>
        </div>

        <!-- Marker list (Phase 2 minimum: read + click-to-seek; drag editing is Phase 4) -->
        {#if playEpisode.markers && playEpisode.markers.length > 0}
          <div class="shrink-0 mt-3 max-h-32 overflow-auto border-t border-(--color-border) pt-2">
            <div class="text-[11px] text-(--color-text-mute) mb-1.5 flex items-center gap-3">
              <span>マーカー ({playEpisode.markers.length})</span>
              <span class="flex items-center gap-1">
                <span class="inline-block size-2 rounded-sm" style="background:{kindColor('subtask')}"></span>
                <span>subtask</span>
              </span>
              <span class="flex items-center gap-1">
                <span class="inline-block size-2 rounded-sm" style="background:{kindColor('event')}"></span>
                <span>event</span>
              </span>
              <span class="flex items-center gap-1">
                <span class="inline-block size-2 rounded-sm" style="background:{kindColor('note')}"></span>
                <span>note</span>
              </span>
            </div>
            <table class="w-full text-xs">
              <tbody>
                {#each playEpisode.markers as m (m.marker_id)}
                  {@const start = markerOffset(m.started_at)}
                  {@const dur = markerDuration(m)}
                  <tr class="group border-t border-(--color-border)/50 hover:bg-(--color-bg-3)/40 cursor-pointer"
                      onclick={() => jumpToMarker(m)}>
                    <td class="px-2 py-1.5">
                      <span class="inline-block size-2 rounded-sm" style="background:{kindColor(m.kind)}" title={m.kind}></span>
                    </td>
                    <td class="px-2 py-1.5 text-(--color-text)">
                      {m.task_description}
                      {#if m.attributes && m.attributes.length > 0}
                        <span class="ml-2 inline-flex flex-wrap gap-1">
                          {#each m.attributes as a}
                            <span class="text-[10px] px-1.5 py-0.5 rounded bg-(--color-accent-soft) text-(--color-accent)"
                                  title={a.note || ''}>
                              {a.key}: {a.value}{a.unit ? ' ' + a.unit : ''}
                            </span>
                          {/each}
                        </span>
                      {/if}
                    </td>
                    <td class="px-2 py-1.5 text-right font-mono text-(--color-text-dim) w-16">{fmtTimeSec(start)}</td>
                    <td class="px-2 py-1.5 text-right font-mono text-(--color-text-mute) w-16">
                      {dur != null ? fmtTimeSec(dur) : '·'}
                    </td>
                    <td class="px-2 py-1.5 text-right w-12">
                      {#if m.outcome === 'success'}<span class="text-emerald-400">✓</span>
                      {:else if m.outcome === 'abort'}<span class="text-red-400">✗</span>
                      {/if}
                    </td>
                    <td class="px-2 py-1.5 text-right w-10">
                      <button onclick={(e) => openAttrEditor(m, e)}
                              class="opacity-30 group-hover:opacity-100 text-(--color-text-mute) hover:text-(--color-accent) transition text-[10px] px-1.5 py-0.5 rounded border border-(--color-border)"
                              title="属性 (重量・グレード等) を編集">
                        + 属性
                      </button>
                    </td>
                  </tr>
                {/each}
              </tbody>
            </table>
          </div>
        {/if}
      {/if}

      <!-- Joint angle chart (Phase 3a) — synced playhead, click bg to add event -->
      {#if jointLoading}
        <div class="shrink-0 mt-3 pt-3 border-t border-(--color-border) text-xs text-(--color-text-mute)">
          ジョイントデータ読み込み中…
        </div>
      {:else if jointData.length > 0}
        {@const series = jointData.find(s => s.topic === jointSelectedTopic) || jointData[0]}
        {#if series && series.t_ms.length > 0}
          {@const totalMs = (maxDuration || 1) * 1000}
          {@const numJoints = series.names.length || (series.q[0]?.length ?? 0)}
          {@const W = 1100}
          {@const H = 140}
          {@const PAD_L = 40}
          {@const PAD_R = 8}
          {@const PAD_T = 8}
          {@const PAD_B = 22}
          {@const innerW = W - PAD_L - PAD_R}
          {@const innerH = H - PAD_T - PAD_B}
          <!-- Compute global min/max across selected joints (degrees). -->
          {@const allVals = series.q.flat()}
          {@const yMinRad = Math.min(...allVals)}
          {@const yMaxRad = Math.max(...allVals)}
          {@const yMin = (yMinRad * 180 / Math.PI)}
          {@const yMax = (yMaxRad * 180 / Math.PI)}
          {@const ySpan = (yMax - yMin) || 1}
          {@const xOf = (t_ms: number) => PAD_L + (t_ms / totalMs) * innerW}
          {@const yOf = (val_rad: number) => PAD_T + innerH - ((val_rad * 180 / Math.PI - yMin) / ySpan) * innerH}
          {@const palette = ['#22dd88','#00d9ff','#ffaa33','#ff6688','#bb88ff','#88ccff','#ffcc44','#66e0c0']}
          <div class="shrink-0 mt-3 pt-3 border-t border-(--color-border)">
            <div class="flex items-center gap-3 text-[11px] text-(--color-text-mute) mb-1.5">
              <span class="text-(--color-text-dim) font-medium">ジョイント角 (deg)</span>
              {#if jointData.length > 1}
                <select bind:value={jointSelectedTopic} class="text-xs bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-0.5">
                  {#each jointData as s (s.topic)}
                    <option value={s.topic}>{s.topic}</option>
                  {/each}
                </select>
              {:else}
                <span class="font-mono">{series.topic}</span>
              {/if}
              <a href={`${API}/episodes/${playEpisode.episode_id}/joints.csv?topic=${encodeURIComponent(series.topic)}`}
                 class="text-(--color-accent) hover:underline text-[10px] flex items-center gap-0.5"
                 title="このトピックを CSV ダウンロード (pandas / Excel 解析用)">
                <Download class="size-3" /> CSV
              </a>
              <a href={`${API}/episodes/${playEpisode.episode_id}/download.tar`}
                 class="text-(--color-accent) hover:underline text-[10px] flex items-center gap-0.5"
                 title="bag + mp4 + meta を tar ダウンロード (オフライン解析用)">
                <Download class="size-3" /> エピソード全体
              </a>
              <span class="ml-auto text-[10px]">クリックで event マーカー追加</span>
            </div>
            <svg viewBox="0 0 {W} {H}" preserveAspectRatio="none" class="w-full" style="height:{H}px; touch-action:none; cursor:crosshair;"
                 onclick={(e) => {
                   const svg = e.currentTarget;
                   const rect = svg.getBoundingClientRect();
                   const xPx = (e.clientX - rect.left) / rect.width * W;
                   if (xPx < PAD_L || xPx > W - PAD_R) return;
                   const t = ((xPx - PAD_L) / innerW) * totalMs / 1000;
                   addEventAtTime(t);
                 }}>
              <!-- Background + grid -->
              <rect x={PAD_L} y={PAD_T} width={innerW} height={innerH} fill="rgba(255,255,255,0.02)" stroke="rgba(255,255,255,0.06)" />
              {#each [0, 0.25, 0.5, 0.75, 1] as gy}
                <line x1={PAD_L} y1={PAD_T + innerH * gy} x2={PAD_L + innerW} y2={PAD_T + innerH * gy} stroke="rgba(255,255,255,0.04)" />
              {/each}
              <!-- Y axis labels (max/mid/min in degrees) -->
              <text x={PAD_L - 4} y={PAD_T + 4} font-size="9" fill="#888" text-anchor="end">{yMax.toFixed(0)}</text>
              <text x={PAD_L - 4} y={PAD_T + innerH / 2 + 3} font-size="9" fill="#666" text-anchor="end">{((yMin + yMax)/2).toFixed(0)}</text>
              <text x={PAD_L - 4} y={PAD_T + innerH - 1} font-size="9" fill="#888" text-anchor="end">{yMin.toFixed(0)}</text>
              <!-- X axis labels -->
              <text x={PAD_L} y={H - 4} font-size="9" fill="#888" text-anchor="start">0s</text>
              <text x={PAD_L + innerW} y={H - 4} font-size="9" fill="#888" text-anchor="end">{fmtTimeSec(maxDuration)}</text>

              <!-- Per-joint polyline. Skip joints whose name starts with "_" if any.
                   Index drives both name and color so legend lines up. -->
              {#each Array.from({length: numJoints}) as _, j (j)}
                {@const pathStr = series.q.map((row, i) => {
                  const x = xOf(series.t_ms[i]);
                  const y = yOf(row[j]);
                  return (i === 0 ? 'M' : 'L') + x.toFixed(1) + ',' + y.toFixed(1);
                }).join(' ')}
                <path d={pathStr} fill="none" stroke={palette[j % palette.length]} stroke-width="1.2" opacity="0.9" />
              {/each}

              <!-- Playhead -->
              <line x1={xOf(sharedTime * 1000)} y1={PAD_T} x2={xOf(sharedTime * 1000)} y2={PAD_T + innerH} stroke="#fff" stroke-width="1" opacity="0.55" />

              <!-- Existing markers as dashed verticals so the user sees what already exists -->
              {#each playEpisode.markers as m (m.marker_id)}
                {@const ms = markerOffset(m.started_at) * 1000}
                {#if ms >= 0 && ms <= totalMs}
                  <line x1={xOf(ms)} y1={PAD_T} x2={xOf(ms)} y2={PAD_T + innerH} stroke={kindColor(m.kind)} stroke-dasharray="2,2" opacity="0.5" stroke-width="1" />
                {/if}
              {/each}
            </svg>

            <!-- Joint legend -->
            <div class="flex flex-wrap gap-x-3 gap-y-1 mt-1 text-[10px]">
              {#each Array.from({length: numJoints}) as _, j (j)}
                <span class="flex items-center gap-1">
                  <span class="inline-block w-3 h-0.5" style="background:{palette[j % palette.length]}"></span>
                  <span class="text-(--color-text-mute)">{series.names[j] || `j${j}`}</span>
                </span>
              {/each}
            </div>
          </div>
        {/if}
      {/if}

      <div class="mt-3 text-[10px] text-(--color-text-mute) font-mono shrink-0">
        ID: {playEpisode.episode_id}
      </div>
    </div>
  </div>
{/if}

<!-- Marker attribute editor — post-hoc metadata (asparagus weight/grade etc.) -->
{#if attrEditor}
  <div class="fixed inset-0 z-[60] flex items-center justify-center bg-black/85 backdrop-blur-sm p-4"
       onclick={(e) => { if (e.target === e.currentTarget) attrEditor = null; }}
       onkeydown={(e) => { if (e.key === 'Escape') attrEditor = null; }}
       role="dialog"
       tabindex="-1">
    <div class="card w-full max-w-2xl p-5">
      <header class="flex items-start justify-between mb-3">
        <div>
          <h2 class="text-base font-semibold text-white">マーカー属性</h2>
          <p class="text-xs text-(--color-text-mute) mt-1">
            {attrEditor.marker.task_description} ·
            <span class="text-(--color-accent)">{attrEditor.marker.kind}</span>
            <span class="ml-2">例: 重量 / グレード / 計測値 + メモ</span>
          </p>
        </div>
        <button onclick={() => (attrEditor = null)} class="text-(--color-text-mute) hover:text-white text-2xl leading-none">×</button>
      </header>
      <div class="space-y-1.5">
        <div class="grid grid-cols-[1fr_1fr_72px_2fr_36px] gap-1.5 text-[10px] text-(--color-text-mute) px-1">
          <span>キー</span><span>値</span><span>単位</span><span>メモ</span><span></span>
        </div>
        {#each attrEditor.rows as row, i (i)}
          <div class="grid grid-cols-[1fr_1fr_72px_2fr_36px] gap-1.5">
            <input type="text" bind:value={row.key}   placeholder="weight" class="bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-1 text-sm" />
            <input type="text" bind:value={row.value} placeholder="65"     class="bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-1 text-sm" />
            <input type="text" bind:value={row.unit}  placeholder="g"      class="bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-1 text-sm" />
            <input type="text" bind:value={row.note}  placeholder="少し曲がってた" class="bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-1 text-sm" />
            <button onclick={() => removeAttrRow(i)} class="text-(--color-text-mute) hover:text-red-400" title="削除">×</button>
          </div>
        {/each}
        <button onclick={addAttrRow} class="text-xs text-(--color-accent) hover:underline mt-1">+ 行を追加</button>
      </div>
      <!-- Suggestion chips for common asparagus workflow -->
      <div class="mt-3 flex flex-wrap gap-1.5 text-[10px]">
        <span class="text-(--color-text-mute)">よく使う:</span>
        {#each [['weight','65','g'],['grade','A',''],['length','25','cm'],['diameter','12','mm'],['defect','false','']] as p}
          <button class="px-2 py-0.5 rounded border border-(--color-border) hover:border-(--color-accent) hover:text-(--color-accent) text-(--color-text-mute)"
                  onclick={() => { attrEditor!.rows = [...attrEditor!.rows, {key:p[0], value:p[1], unit:p[2], note:''}]; }}>
            + {p[0]}{p[2] ? ' ('+p[2]+')' : ''}
          </button>
        {/each}
      </div>
      <div class="mt-5 flex justify-end gap-2">
        <button onclick={() => (attrEditor = null)} class="px-3 py-1.5 text-sm rounded bg-(--color-bg-3) text-(--color-text-dim) hover:text-white">キャンセル</button>
        <button onclick={saveAttrs} class="px-4 py-1.5 text-sm rounded bg-(--color-accent) text-(--color-bg) font-semibold hover:brightness-110">保存</button>
      </div>
    </div>
  </div>
{/if}
