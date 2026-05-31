<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Search, RefreshCw, Pin, Trash2, HardDrive, ArrowUp, ArrowDown, Download, Settings } from 'lucide-svelte';

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
    env?: 'real' | 'sim';
    controller_label?: string | null;
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
    cameras: Array<{ name: string; topic: string; kind?: string; frame_count?: number; segments?: Array<{ file: string }> }>;
    markers: MarkerItem[];
    trim_start_s?: number | null;
    trim_end_s?: number | null;
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
  async function deleteMarker(m: MarkerItem, e: MouseEvent) {
    e.stopPropagation();
    if (!playEpisode) return;
    const label = m.task_description || m.marker_id.slice(-8);
    if (!confirm(`マーカーを削除しますか？\n\n  ${m.kind}: ${label}\n\nこの操作は取り消せません。`)) return;
    try {
      const r = await fetch(`${API}/markers/${m.marker_id}`, {method: 'DELETE'});
      if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${r.status}`);
      }
      playEpisode = {
        ...playEpisode,
        markers: playEpisode.markers.filter(x => x.marker_id !== m.marker_id),
      };
    } catch (err: any) {
      alert('マーカー削除失敗: ' + (err.message || err));
    }
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

  // -------- Clip trim (non-destructive) --------
  // Saves trim_start_s / trim_end_s into meta.json via PATCH /episodes/{id}.
  // The underlying bag + mp4 stay intact; playback + future exports honor
  // the trim and the operator can widen it later.
  async function patchTrim(start: number | null, end: number | null) {
    if (!playEpisode) return;
    try {
      const r = await fetch(`${API}/episodes/${playEpisode.episode_id}`, {
        method: 'PATCH',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({trim_start_s: start, trim_end_s: end}),
      });
      if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${r.status}`);
      }
      playEpisode = { ...playEpisode, trim_start_s: start, trim_end_s: end };
    } catch (e: any) {
      alert('クリップ範囲の保存失敗: ' + (e.message || e));
    }
  }
  function setTrimStart() {
    if (!playEpisode) return;
    const end = playEpisode.trim_end_s ?? null;
    const start = Math.min(sharedTime, end != null ? end - 0.1 : maxDuration);
    patchTrim(start, end);
  }
  function setTrimEnd() {
    if (!playEpisode) return;
    const start = playEpisode.trim_start_s ?? null;
    const end = Math.max(sharedTime, start != null ? start + 0.1 : 0);
    patchTrim(start, end);
  }
  function clearTrim() {
    patchTrim(null, null);
  }

  // -------- Add marker on finalized episode (Phase 4 review workflow) --------
  // Opens a small modal so the operator can pick kind / label / start / end.
  // Defaults: start = current playhead, kind = event, end = start + 3s (subtask).
  let addMarkerForm = $state<{
    kind: 'subtask' | 'event' | 'note';
    label: string;
    start_s: number;
    end_s: number;
  } | null>(null);

  function openAddMarker(defaultKind: 'subtask' | 'event' | 'note' = 'event') {
    if (!playEpisode) return;
    const s = sharedTime || 0;
    addMarkerForm = {
      kind: defaultKind,
      label: '',
      start_s: Math.max(0, Math.min(maxDuration, s)),
      end_s: Math.max(0, Math.min(maxDuration, s + 3)),
    };
  }

  async function submitAddMarker() {
    if (!playEpisode || !addMarkerForm) return;
    const f = addMarkerForm;
    if (!f.label.trim()) {
      alert('ラベルを入力してください');
      return;
    }
    if (f.kind === 'subtask' && f.end_s <= f.start_s) {
      alert('subtask の終了時刻は開始より後にしてください');
      return;
    }
    const epStartMs = new Date(playEpisode.started_at).getTime();
    const startIso = _isoFromMs(epStartMs + f.start_s * 1000);
    const stopIso = f.kind === 'subtask' ? _isoFromMs(epStartMs + f.end_s * 1000) : undefined;
    try {
      const r = await fetch(`${API}/episodes/${playEpisode.episode_id}/markers/start`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          task_description: f.label.trim(),
          kind: f.kind,
          started_at: startIso,
          ...(stopIso ? {stopped_at: stopIso} : {}),
        }),
      });
      if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${r.status}`);
      }
      const m = await r.json();
      playEpisode = { ...playEpisode, markers: [...playEpisode.markers, m] };
      addMarkerForm = null;
    } catch (e: any) {
      alert('マーカー追加失敗: ' + (e.message || e));
    }
  }

  // Click on joint chart background → quick-add an event at that time (legacy
  // fast path; opens the proper modal so the operator can change kind / label).
  function addEventAtTime(seconds: number) {
    openAddMarker('event');
    if (addMarkerForm) {
      addMarkerForm = { ...addMarkerForm, start_s: seconds, end_s: seconds + 3 };
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
    if (!isFinite(s)) return '—';
    if (s < 0) return '⚠︎ <0';   // clock skew / manual meta edit — surface, don't silently render "-1:00:-5"
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

  // Cursor pagination state. First load = fresh top-of-list, replaces
  // `episodes`. loadMore = appends the next page if next_cursor is set.
  let nextCursor = $state<string | null>(null);
  let totalCount = $state<number>(0);
  let loadingMore = $state(false);
  const PAGE_SIZE = 50;

  async function load() {
    try {
      const params = new URLSearchParams({ limit: String(PAGE_SIZE) });
      if (activeProfile) params.set('profile', activeProfile);
      const [epRes, dkRes] = await Promise.all([
        fetch(`${API}/episodes?${params}`),
        fetch(`${API}/disk/status`),
      ]);
      const epJson = await epRes.json();
      const dkJson = await dkRes.json();
      episodes = epJson.episodes || [];
      nextCursor = epJson.next_cursor || null;
      totalCount = epJson.total ?? episodes.length;
      disk = dkJson;
      error = null;
    } catch (e: any) {
      error = e.message || String(e);
    } finally {
      loading = false;
    }
  }

  async function loadMore() {
    if (loadingMore || !nextCursor) return;
    loadingMore = true;
    try {
      const params = new URLSearchParams({ limit: String(PAGE_SIZE), cursor: nextCursor });
      if (activeProfile) params.set('profile', activeProfile);
      const r = await fetch(`${API}/episodes?${params}`);
      const j = await r.json();
      const newOnes: Episode[] = j.episodes || [];
      // Dedup by episode_id in case the top-of-list poll and loadMore overlap.
      const seen = new Set(episodes.map(e => e.episode_id));
      episodes = [...episodes, ...newOnes.filter(e => !seen.has(e.episode_id))];
      nextCursor = j.next_cursor || null;
      totalCount = j.total ?? totalCount;
    } catch (e: any) {
      // Non-fatal — keep what we have, user can scroll-retry.
      console.warn('loadMore failed', e);
    } finally {
      loadingMore = false;
    }
  }

  // IntersectionObserver-driven infinite scroll. Sentinel <div> at the
  // table foot is observed; when it enters the viewport we kick loadMore.
  let scrollSentinel: HTMLDivElement | null = $state(null);
  let scrollObserver: IntersectionObserver | null = null;
  $effect(() => {
    if (scrollObserver) { scrollObserver.disconnect(); scrollObserver = null; }
    if (!scrollSentinel) return;
    scrollObserver = new IntersectionObserver((entries) => {
      if (entries.some(e => e.isIntersecting) && nextCursor && !loadingMore) {
        loadMore();
      }
    }, { rootMargin: '300px 0px' });
    scrollObserver.observe(scrollSentinel);
    return () => { if (scrollObserver) { scrollObserver.disconnect(); scrollObserver = null; } };
  });

  // Disk-card derived thresholds. Pulled out of the markup because
  // Svelte 5 only allows {@const} as an immediate child of control-flow
  // blocks like {#if} / {#each}, not inside a regular <section>.
  const diskWarnPct = $derived(disk?.policy?.warn_pct_free ?? 20);
  const diskCritPct = $derived(disk?.policy?.crit_pct_free ?? 10);
  const diskStage = $derived(
    !disk ? 'ok'
      : disk.percent_free < diskCritPct ? 'crit'
      : disk.percent_free < diskWarnPct ? 'warn'
      : 'ok'
  );

  // ---- Batch grouping (LeRobot-style runs collapse into one folder row) ----
  type BatchStats = {
    rawTask: string;
    count: number; success: number; abort: number; pending: number;
    totalBytes: number; totalMarkers: number;
    durMin: number | null; durMax: number | null; durMedian: number | null;
    controllers: string[]; envs: string[];
    earliest: string; latest: string;
  };
  type Row =
    | { type: 'single'; ep: Episode }
    | { type: 'batchHeader'; batchId: string; eps: Episode[]; stats: BatchStats }
    | { type: 'batchChild'; ep: Episode; batchId: string };

  let expandedBatches = $state<Set<string>>(new Set());
  let groupBatches = $state(true);  // toggle to disable grouping

  // ---- Settings (⚙ popover) ----
  let settingsOpen = $state(false);
  let pollIntervalS = $state(5);   // auto-refresh seconds; 0 = off
  let confirmDelete = $state(true);
  // Persist across sessions so the operator's preferences survive reload.
  try {
    const saved = JSON.parse(localStorage.getItem('fv_episode_ui.settings') || '{}');
    if (typeof saved.pollIntervalS === 'number') pollIntervalS = saved.pollIntervalS;
    if (typeof saved.confirmDelete === 'boolean') confirmDelete = saved.confirmDelete;
    if (typeof saved.groupBatches === 'boolean') groupBatches = saved.groupBatches;
  } catch {}
  $effect(() => {
    try {
      localStorage.setItem('fv_episode_ui.settings', JSON.stringify({
        pollIntervalS, confirmDelete, groupBatches,
      }));
    } catch {}
    // Re-arm the poll timer when interval changes.
    if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
    if (pollIntervalS > 0) {
      pollTimer = window.setInterval(load, pollIntervalS * 1000);
    }
  });

  function batchTagOf(ep: Episode): string | null {
    const t = ep.tags?.find(tag => tag.startsWith('batch:'));
    return t ? t.slice(6) : null;
  }
  function toggleBatch(id: string) {
    const next = new Set(expandedBatches);
    if (next.has(id)) next.delete(id); else next.add(id);
    expandedBatches = next;
  }
  function computeBatchStats(eps: Episode[]): BatchStats {
    const rawTask = (eps[0]?.task_description || '').replace(/\s+#\d+\/\d+$/, '');
    const success = eps.filter(e => e.outcome === 'success').length;
    const abort   = eps.filter(e => e.outcome === 'abort').length;
    const pending = eps.filter(e => e.outcome == null).length;
    const totalBytes   = eps.reduce((s, e) => s + (e.size_bytes   || 0), 0);
    const totalMarkers = eps.reduce((s, e) => s + (e.marker_count || 0), 0);
    const durs = eps.map(e => e.duration_s).filter((d): d is number => d != null).slice().sort((a, b) => a - b);
    const durMin    = durs.length ? durs[0] : null;
    const durMax    = durs.length ? durs[durs.length - 1] : null;
    const durMedian = durs.length ? durs[Math.floor(durs.length / 2)] : null;
    const controllers = Array.from(new Set(eps.map(e => e.controller_label).filter((x): x is string => !!x)));
    const envs = Array.from(new Set(eps.map(e => e.env).filter((x): x is string => !!x)));
    const dates = eps.map(e => e.started_at).filter(Boolean).slice().sort();
    return { rawTask, count: eps.length, success, abort, pending,
             totalBytes, totalMarkers, durMin, durMax, durMedian,
             controllers, envs, earliest: dates[0] || '', latest: dates[dates.length - 1] || '' };
  }

  const filteredEpisodes = $derived(
    episodes
      .filter(e => {
        // Active recording lives in the header chip + disk card "録画中"
        // section; surfacing it here too just adds noise (playback isn't
        // possible until it finishes anyway).
        if (e.state === 'recording') return false;
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

  // Fold same-batch episodes into one collapsible "folder" row. Singletons
  // (no batch tag) render unchanged. Preserves the outer sort order — a
  // batch occupies the position of its first (highest-sorted) episode.
  const groupedRows = $derived.by((): Row[] => {
    if (!groupBatches) return filteredEpisodes.map(ep => ({ type: 'single', ep } as Row));
    const byBatch = new Map<string, Episode[]>();
    for (const ep of filteredEpisodes) {
      const bid = batchTagOf(ep);
      if (!bid) continue;
      const arr = byBatch.get(bid) ?? [];
      arr.push(ep);
      byBatch.set(bid, arr);
    }
    const seen = new Set<string>();
    const rows: Row[] = [];
    for (const ep of filteredEpisodes) {
      const bid = batchTagOf(ep);
      if (!bid) { rows.push({ type: 'single', ep }); continue; }
      if (!seen.has(bid)) {
        const eps = byBatch.get(bid)!;
        rows.push({ type: 'batchHeader', batchId: bid, eps, stats: computeBatchStats(eps) });
        seen.add(bid);
      }
      if (expandedBatches.has(bid)) {
        rows.push({ type: 'batchChild', ep, batchId: bid });
      }
    }
    return rows;
  });

  let bulkDeleteProgress = $state<{ total: number; done: number; failed: number } | null>(null);
  // ---- Retention (auto-maintenance) — operator policy + preview/run ----
  type RetentionPolicy = {
    enabled: boolean;
    max_age_days: number | null;
    max_episodes: number | null;
    free_min_pct: number | null;
    grace_period_s: number;
    interval_s: number;
  };
  let retentionPolicy = $state<RetentionPolicy | null>(null);
  let retentionPreview = $state<any>(null);
  let retentionBusy = $state(false);

  async function loadRetentionPolicy() {
    try {
      const r = await fetch(`${API}/retention/policy`);
      if (r.ok) retentionPolicy = await r.json();
    } catch {}
  }
  async function saveRetentionPolicy() {
    if (!retentionPolicy) return;
    retentionBusy = true;
    try {
      const r = await fetch(`${API}/retention/policy`, {
        method: 'PUT',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(retentionPolicy),
      });
      if (r.ok) retentionPolicy = await r.json();
    } finally { retentionBusy = false; }
  }
  async function previewRetention() {
    if (!retentionPolicy) return;
    retentionBusy = true;
    try {
      const r = await fetch(`${API}/retention/run?dry_run=true`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(retentionPolicy),
      });
      if (r.ok) retentionPreview = await r.json();
    } finally { retentionBusy = false; }
  }
  async function runRetentionNow() {
    if (!retentionPolicy) return;
    if (!confirm('保存済みポリシーで今すぐ削除を実行します。grace_period_s 内の候補は次回 tick で実削除されます。')) return;
    retentionBusy = true;
    try {
      const r = await fetch(`${API}/retention/run?dry_run=false`, {method: 'POST'});
      if (r.ok) {
        retentionPreview = await r.json();
        await load();
      }
    } finally { retentionBusy = false; }
  }

  async function deleteAllEpisodes() {
    const targets = episodes.filter(e => !e.pinned && e.state !== 'recording');
    if (targets.length === 0) {
      alert('削除対象なし (録画中・pinned を除く)');
      return;
    }
    const totalBytes = targets.reduce((s, e) => s + (e.size_bytes || 0), 0);
    // Two-step confirm so a stray click doesn't nuke everything.
    if (!confirm(`【全削除】 ${targets.length} 件のエピソード (${fmtBytes(totalBytes)}) を削除します。\n\npinned エピソードは保護されます。録画中エピソードも対象外。\nこの操作は取り消せません。`)) return;
    if (!confirm(`本当にすべて削除しますか？\n(${targets.length} 件 / ${fmtBytes(totalBytes)})`)) return;
    bulkDeleteProgress = { total: targets.length, done: 0, failed: 0 };
    for (const ep of targets) {
      try {
        const r = await fetch(`${API}/episodes/${ep.episode_id}`, { method: 'DELETE' });
        if (r.ok) bulkDeleteProgress.done++; else bulkDeleteProgress.failed++;
      } catch { bulkDeleteProgress.failed++; }
      bulkDeleteProgress = { ...bulkDeleteProgress };
    }
    const removed = bulkDeleteProgress.done;
    const failed = bulkDeleteProgress.failed;
    bulkDeleteProgress = null;
    await load();
    alert(`完了: ${removed} 件削除 / ${failed} 件失敗`);
  }

  async function togglePinBatch(batchId: string, eps: Episode[], e: MouseEvent) {
    e.stopPropagation();
    // Decide direction: if any unpinned in the batch → pin all, else unpin all.
    const anyUnpinned = eps.some(x => !x.pinned);
    const next = anyUnpinned;
    const label = (eps[0]?.task_description || '').replace(/\s+#\d+\/\d+$/, '');
    if (!confirm(`バッチ ${eps.length} 本を ${next ? '📌 pin' : 'unpin'} します。\n\n  ${label}`)) return;
    let done = 0, failed = 0;
    for (const ep of eps) {
      if (ep.pinned === next) { done++; continue; }
      try {
        const r = await fetch(`${API}/episodes/${ep.episode_id}`, {
          method: 'PATCH',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({pinned: next}),
        });
        if (r.ok) done++; else failed++;
      } catch { failed++; }
    }
    // Optimistic local update.
    episodes = episodes.map(x => eps.some(e2 => e2.episode_id === x.episode_id) ? { ...x, pinned: next } : x);
    if (failed > 0) alert(`pin 切替: ${done} 件成功 / ${failed} 件失敗`);
  }

  async function deleteBatch(batchId: string, eps: Episode[], e: MouseEvent) {
    e.stopPropagation();
    const totalBytes = eps.reduce((s, x) => s + (x.size_bytes || 0), 0);
    const label = (eps[0]?.task_description || '').replace(/\s+#\d+\/\d+$/, '');
    if (confirmDelete && !confirm(`バッチ全体を削除しますか？\n\n  タスク: ${label}\n  本数: ${eps.length}\n  合計: ${fmtBytes(totalBytes)}\n\nこの操作は取り消せません。`)) return;
    let removed = 0, failed = 0;
    for (const ep of eps) {
      try {
        const res = await fetch(`${API}/episodes/${ep.episode_id}${ep.pinned ? '?force=true' : ''}`, { method: 'DELETE' });
        if (res.ok) removed++; else failed++;
      } catch { failed++; }
    }
    episodes = episodes.filter(x => !eps.some(e => e.episode_id === x.episode_id) || failed > 0 && !eps.find(e => e.episode_id === x.episode_id));
    // Simpler: drop ones we attempted, then refetch.
    episodes = episodes.filter(x => !eps.some(e => e.episode_id === x.episode_id));
    if (failed > 0) alert(`削除: ${removed} 件成功 / ${failed} 件失敗`);
  }

  async function togglePin(ep: Episode, e: MouseEvent) {
    e.stopPropagation();
    const next = !ep.pinned;
    try {
      const r = await fetch(`${API}/episodes/${ep.episode_id}`, {
        method: 'PATCH',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({pinned: next}),
      });
      if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || `HTTP ${r.status}`);
      }
      // Optimistic local update so the icon flips without waiting for poll.
      const idx = episodes.findIndex(x => x.episode_id === ep.episode_id);
      if (idx >= 0) {
        episodes[idx] = { ...episodes[idx], pinned: next };
        episodes = [...episodes];
      }
    } catch (err: any) {
      alert('pin 切替失敗: ' + (err.message || err));
    }
  }

  async function deleteEpisode(ep: Episode, e: MouseEvent) {
    e.stopPropagation();
    const label = ep.task_description || ep.episode_id.slice(-8);
    if (confirmDelete && !confirm(`削除しますか？\n\n  タスク: ${label}\n  サイズ: ${fmtBytes(ep.size_bytes)}\n\nこの操作は取り消せません。`)) return;
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
    // $effect (above) arms the poll timer from pollIntervalS — no double-arming here.
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
    <div class="flex items-center gap-2 relative">
      <!-- Live indicator (spinning while polling) replaces the manual button -->
      <div class="flex items-center gap-1.5 text-[11px] text-(--color-text-mute)" title="自動更新中">
        <RefreshCw class="size-3 {loading ? 'animate-spin text-(--color-accent)' : ''}" />
        {#if pollIntervalS > 0}<span>{pollIntervalS}s 毎</span>{:else}<span>off</span>{/if}
      </div>
      <button
        onclick={() => { settingsOpen = !settingsOpen; if (settingsOpen) loadRetentionPolicy(); }}
        class="flex items-center gap-2 px-3 py-1.5 text-sm rounded-lg bg-(--color-bg-2) border border-(--color-border) text-(--color-text-dim) hover:text-(--color-accent) hover:border-(--color-accent-glow) transition"
        title="設定">
        <Settings class="size-4 {settingsOpen ? 'rotate-45' : ''} transition-transform" />
        設定
      </button>
      {#if settingsOpen}
        <div class="absolute right-0 top-full mt-2 w-72 z-50 card p-4 shadow-2xl"
             onclick={(e) => e.stopPropagation()}
             role="dialog">
          <div class="flex items-center justify-between mb-3">
            <h3 class="text-sm font-semibold text-white">設定</h3>
            <button onclick={() => (settingsOpen = false)} class="text-(--color-text-mute) hover:text-white text-xl leading-none">×</button>
          </div>
          <div class="space-y-3 text-xs">
            <label class="flex flex-col gap-1">
              <span class="text-(--color-text-dim)">自動更新</span>
              <select bind:value={pollIntervalS} class="bg-(--color-bg-2) border border-(--color-border) rounded px-2 py-1.5 text-(--color-text)">
                <option value={0}>off (手動のみ)</option>
                <option value={2}>2 秒</option>
                <option value={5}>5 秒 (推奨)</option>
                <option value={10}>10 秒</option>
                <option value={30}>30 秒</option>
                <option value={60}>1 分</option>
              </select>
            </label>
            <label class="flex items-center gap-2 cursor-pointer">
              <input type="checkbox" bind:checked={groupBatches} class="accent-(--color-accent)" />
              <span>🔁 バッチを折り畳む</span>
            </label>
            <label class="flex items-center gap-2 cursor-pointer">
              <input type="checkbox" bind:checked={confirmDelete} class="accent-(--color-accent)" />
              <span>削除前に確認ダイアログ</span>
            </label>
            <div class="border-t border-(--color-border) pt-3 space-y-2">
              <button onclick={() => { load(); }} class="w-full px-3 py-1.5 rounded bg-(--color-accent-soft) text-(--color-accent) border border-(--color-accent-glow) hover:brightness-125 transition">
                ↻ 今すぐ更新
              </button>
              <button onclick={() => { expandedBatches = new Set(); }}
                      class="w-full px-3 py-1.5 rounded bg-(--color-bg-2) border border-(--color-border) text-(--color-text-dim) hover:text-white transition">
                バッチを全部畳む
              </button>
              <button onclick={() => { localStorage.removeItem('fv_episode_ui.settings'); pollIntervalS = 5; confirmDelete = true; groupBatches = true; }}
                      class="w-full px-3 py-1.5 rounded text-(--color-text-mute) hover:text-amber-400 transition text-[11px]">
                設定を初期化
              </button>
            </div>

            <!-- Retention / 自動メンテ -->
            {#if retentionPolicy}
              <div class="border-t border-(--color-border) pt-3 mt-1 space-y-2">
                <div class="text-(--color-text-dim) text-[11px] flex items-center justify-between">
                  <span>🧹 自動メンテ (古い録画削除)</span>
                  <label class="flex items-center gap-1 cursor-pointer">
                    <input type="checkbox" bind:checked={retentionPolicy.enabled} class="accent-(--color-accent)" />
                    <span class="text-[10px]">{retentionPolicy.enabled ? 'ON' : 'OFF'}</span>
                  </label>
                </div>
                <div class="grid grid-cols-3 gap-1.5 text-[10px]" class:opacity-50={!retentionPolicy.enabled}>
                  <label class="flex flex-col gap-0.5">
                    <span class="text-(--color-text-mute)">日数</span>
                    <input type="number" min="0" step="1"
                           value={retentionPolicy.max_age_days ?? ''}
                           oninput={(e) => { if (retentionPolicy) retentionPolicy.max_age_days = e.currentTarget.value ? parseInt(e.currentTarget.value) : null; }}
                           placeholder="—" disabled={!retentionPolicy.enabled}
                           class="w-full bg-(--color-bg-2) border border-(--color-border) rounded px-1.5 py-1 font-mono text-(--color-text)" />
                  </label>
                  <label class="flex flex-col gap-0.5">
                    <span class="text-(--color-text-mute)">本数</span>
                    <input type="number" min="0" step="10"
                           value={retentionPolicy.max_episodes ?? ''}
                           oninput={(e) => { if (retentionPolicy) retentionPolicy.max_episodes = e.currentTarget.value ? parseInt(e.currentTarget.value) : null; }}
                           placeholder="—" disabled={!retentionPolicy.enabled}
                           class="w-full bg-(--color-bg-2) border border-(--color-border) rounded px-1.5 py-1 font-mono text-(--color-text)" />
                  </label>
                  <label class="flex flex-col gap-0.5">
                    <span class="text-(--color-text-mute)">空き %</span>
                    <input type="number" min="0" max="100" step="1"
                           value={retentionPolicy.free_min_pct ?? ''}
                           oninput={(e) => { if (retentionPolicy) retentionPolicy.free_min_pct = e.currentTarget.value ? parseFloat(e.currentTarget.value) : null; }}
                           placeholder="—" disabled={!retentionPolicy.enabled}
                           class="w-full bg-(--color-bg-2) border border-(--color-border) rounded px-1.5 py-1 font-mono text-(--color-text)" />
                  </label>
                </div>
                <div class="flex gap-1.5">
                  <button onclick={saveRetentionPolicy} disabled={retentionBusy}
                          class="flex-1 px-2 py-1 rounded bg-(--color-accent-soft) text-(--color-accent) border border-(--color-accent-glow) hover:brightness-125 transition text-[10px]">
                    保存
                  </button>
                  <button onclick={previewRetention} disabled={retentionBusy || !retentionPolicy.enabled}
                          class="flex-1 px-2 py-1 rounded bg-(--color-bg-2) border border-(--color-border) text-(--color-text-dim) hover:text-white transition text-[10px]">
                    プレビュー
                  </button>
                  <button onclick={runRetentionNow} disabled={retentionBusy || !retentionPolicy.enabled}
                          class="flex-1 px-2 py-1 rounded bg-amber-500/15 border border-amber-500/40 text-amber-300 hover:bg-amber-500/25 transition text-[10px]">
                    今すぐ実行
                  </button>
                </div>
                {#if retentionPreview}
                  {@const cands = retentionPreview.candidates || []}
                  {@const deleted = retentionPreview.deleted || []}
                  <div class="text-[10px] text-(--color-text-mute) bg-(--color-bg-3)/30 rounded p-2 space-y-0.5">
                    <div>候補: <b class="text-(--color-text)">{cands.length}</b> 件 / 今回削除: <b class="text-amber-400">{deleted.length}</b> 件</div>
                    {#if cands.length > 0}
                      <div class="text-[9px] opacity-70 max-h-24 overflow-auto font-mono">
                        {#each cands.slice(0, 10) as c}
                          <div>· {c.task} ({c.reasons.join(',')}) {(c.size_bytes/1e6).toFixed(1)}MB</div>
                        {/each}
                        {#if cands.length > 10}<div>… 他 {cands.length - 10} 件</div>{/if}
                      </div>
                    {/if}
                  </div>
                {/if}
                <div class="text-[9px] text-(--color-text-mute)">
                  3 ルール union: 日数超 / 本数超 / 空き不足。pinned は保護。
                  grace {retentionPolicy.grace_period_s}s → 候補→削除。
                </div>
              </div>
            {/if}

            <div class="border-t border-red-500/30 pt-3 mt-1">
              <div class="text-[10px] text-red-400/70 mb-1.5">⚠ 危険ゾーン</div>
              <button onclick={() => { settingsOpen = false; deleteAllEpisodes(); }}
                      class="w-full px-3 py-1.5 rounded bg-red-500/15 border border-red-500/40 text-red-300 hover:bg-red-500/25 transition text-xs"
                      disabled={bulkDeleteProgress !== null}>
                🗑 全ての録画を削除 (pinned 除く)
              </button>
              {#if bulkDeleteProgress}
                <div class="text-[10px] text-(--color-text-mute) mt-1 font-mono">
                  削除中… {bulkDeleteProgress.done + bulkDeleteProgress.failed}/{bulkDeleteProgress.total}
                  {#if bulkDeleteProgress.failed > 0}<span class="text-red-400">(失敗 {bulkDeleteProgress.failed})</span>{/if}
                </div>
              {/if}
            </div>
            <div class="text-[10px] text-(--color-text-mute) pt-2 border-t border-(--color-border)">
              fv_episode_ui v0.1.0 · API: <code class="text-(--color-accent)">{API}</code>
            </div>
          </div>
        </div>
      {/if}
    </div>
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
           (otherwise they assume "保存先" silently rotates and lose data).
           Thresholds come from $derived diskWarnPct / diskCritPct / diskStage
           in the script (Svelte 5 forbids {@const} as a direct child of a
           non-control-flow element like <section>). -->
      <details class="mt-3 pt-3 border-t border-(--color-border) text-xs" open={diskStage !== 'ok'}>
        <summary class="cursor-pointer text-(--color-text-dim) hover:text-(--color-text) select-none flex items-center gap-2">
          <span>自動メンテ</span>
          {#if diskStage === 'ok'}
            <span class="text-emerald-400">● 正常</span>
          {:else if diskStage === 'warn'}
            <span class="text-amber-400">● 警告 (空き {disk.percent_free.toFixed(1)}% &lt; {diskWarnPct}%)</span>
          {:else}
            <span class="text-red-400">● 危険 (空き {disk.percent_free.toFixed(1)}% &lt; {diskCritPct}%)</span>
          {/if}
        </summary>
        <div class="mt-2 space-y-1.5 text-(--color-text-mute) pl-1">
          <div>
            <span class="text-(--color-text-dim)">しきい値:</span>
            空き <b class="text-amber-400">{diskWarnPct}%</b> で警告 /
            <b class="text-red-400">{diskCritPct}%</b> で危険 (header 右上のチップが色変化)
          </div>
          <div>
            <span class="text-(--color-text-dim)">通知:</span>
            <code class="px-1 rounded bg-(--color-bg-3)/50">/vlabor/events</code> に
            <code class="px-1 rounded bg-(--color-bg-3)/50">DISK_LOW</code> /
            <code class="px-1 rounded bg-(--color-bg-3)/50">DISK_CRITICAL</code> をパブリッシュ
            → dashboard の event log + 外部 MCP 購読者にも自動波及
          </div>
          <div>
            <span class="text-(--color-text-dim)">自動削除:</span>
            <code class="px-1 rounded bg-(--color-bg-3)/50">max_age_days</code> /
            <code class="px-1 rounded bg-(--color-bg-3)/50">max_episodes</code> /
            <code class="px-1 rounded bg-(--color-bg-3)/50">free_min_pct</code>
            の 3 ルール union (oldest-first)、grace 60s。pinned (📌) は保護。
          </div>
          <div class="text-(--color-text-mute)">
            <span class="text-(--color-text-dim)">設定:</span>
            ⚙ 設定 → 🧹 自動メンテ で ON/OFF + しきい値編集 + プレビュー / 今すぐ実行。
            profile yaml デフォルトより operator 設定を優先 (
            <code class="px-1 rounded bg-(--color-bg-3)/50">.retention_policy.json</code>
            に永続化)。
          </div>
        </div>
      </details>
    </section>
  {/if}

  <!-- Search bar (grouping toggle moved to ⚙ 設定) -->
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
            <th class="text-left px-4 py-2.5 font-medium">ソース</th>
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
          {#each groupedRows as row, ri (row.type + ':' + (row.type === 'batchHeader' ? row.batchId : row.ep.episode_id))}
            {#if row.type === 'batchHeader'}
              {@const s = row.stats}
              {@const open = expandedBatches.has(row.batchId)}
              {@const successRate = s.count > 0 ? (s.success / s.count) * 100 : 0}
              {@const rateColor = successRate >= 90 ? 'text-emerald-400' : successRate >= 50 ? 'text-amber-400' : 'text-red-400'}
              {@const allPinned = row.eps.every(x => x.pinned)}
              {@const anyPinned = row.eps.some(x => x.pinned)}
              <tr class="group border-t border-(--color-border) hover:bg-(--color-accent-soft)/30 cursor-pointer transition-colors"
                  style="background: rgba(0,217,255,0.04);"
                  onclick={() => toggleBatch(row.batchId)}
                  title={open ? 'クリックで畳む' : `クリックで ${s.count} 件を展開`}>
                <td class="px-4 py-2.5 whitespace-nowrap">
                  <span class="text-(--color-accent) inline-block w-4 text-center">{open ? '▼' : '▶'}</span>
                  <span class="inline-block px-2 py-0.5 rounded text-[11px] bg-cyan-500/15 text-cyan-300 border border-cyan-500/30">🔁 ×{s.count}</span>
                  <span class="ml-1 text-[11px] {rateColor} font-mono">{s.success}/{s.count}{#if s.abort > 0} <span class="text-red-400">✗{s.abort}</span>{/if}</span>
                </td>
                <td class="px-4 py-2.5">
                  <div class="text-(--color-text) font-medium">{s.rawTask || '(no task)'}</div>
                  <div class="text-[10px] text-(--color-text-mute) mt-0.5 font-mono">batch:{row.batchId.slice(0, 8)}…</div>
                </td>
                <td class="px-4 py-2.5 text-xs whitespace-nowrap">
                  {#each s.envs as env}
                    {#if env === 'sim'}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-violet-500/15 text-violet-300 border border-violet-500/30 mr-1">sim</span>
                    {:else}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-zinc-500/15 text-zinc-400 border border-zinc-500/30 mr-1">実機</span>
                    {/if}
                  {/each}
                  {#each s.controllers as ctl}
                    {#if ctl.startsWith('VLA')}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-cyan-500/15 text-cyan-300 border border-cyan-500/30 mr-1">🤖 {ctl}</span>
                    {:else if ctl === 'teleop'}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-amber-500/15 text-amber-300 border border-amber-500/30 mr-1">🎮 teleop</span>
                    {:else}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-zinc-500/15 text-zinc-400 border border-zinc-500/30 mr-1">{ctl}</span>
                    {/if}
                  {/each}
                </td>
                <td class="px-4 py-2.5 text-(--color-text-dim) text-xs font-mono">{fmtTime(s.earliest)}</td>
                <td class="px-4 py-2.5 text-right font-mono text-xs">
                  {#if s.durMedian != null}
                    <span class="text-(--color-text)">~{fmtDuration(s.durMedian)}</span>
                    <div class="text-[10px] text-(--color-text-mute)">min {fmtDuration(s.durMin)} · max {fmtDuration(s.durMax)}</div>
                  {:else}
                    —
                  {/if}
                </td>
                <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{fmtBytes(s.totalBytes)}</td>
                <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{s.totalMarkers}</td>
                <td class="px-2 py-2.5 text-right whitespace-nowrap">
                  <button
                    onclick={(e) => togglePinBatch(row.batchId, row.eps, e)}
                    class="{anyPinned ? 'opacity-100 text-amber-400' : 'opacity-0 group-hover:opacity-100 text-(--color-text-mute)'} hover:text-amber-400 transition p-1 rounded hover:bg-amber-500/10 mr-1"
                    title={allPinned ? `バッチ全体 ${s.count} 件を unpin` : anyPinned ? `バッチを完全 pin (現状 ${row.eps.filter(x => x.pinned).length}/${s.count} pinned)` : `バッチ全体 ${s.count} 件を 📌 pin (retention 保護)`}>
                    <Pin class="size-4" />
                  </button>
                  <button
                    onclick={(e) => deleteBatch(row.batchId, row.eps, e)}
                    class="opacity-0 group-hover:opacity-100 text-(--color-text-mute) hover:text-red-400 transition p-1 rounded hover:bg-red-500/10"
                    title={`バッチ全体 (${s.count} 件) を削除`}>
                    <Trash2 class="size-4" />
                  </button>
                </td>
              </tr>
            {:else}
              {@const ep = row.ep}
              {@const b = outcomeBadge(ep.outcome)}
              {@const isChild = row.type === 'batchChild'}
              <tr class="group border-t border-(--color-border) hover:bg-(--color-bg-3)/40 transition-colors cursor-pointer"
                  onclick={() => openPlay(ep)}
                  title="クリックで再生">
                <td class="px-4 py-2.5">
                  {#if isChild}<span class="inline-block w-4 text-(--color-text-mute) text-center">└</span>{/if}
                  <span class="inline-block px-2 py-0.5 rounded text-[11px] {b.cls}">{b.label}</span>
                </td>
                <td class="px-4 py-2.5" style={isChild ? 'padding-left: 32px;' : ''}>
                  <div class="text-(--color-text) {isChild ? 'text-xs text-(--color-text-dim)' : 'font-medium'}">{ep.task_description}</div>
                  {#if ep.tags.length && !isChild}
                    <div class="flex gap-1 mt-1 flex-wrap">
                      {#each ep.tags as t}
                        <span class="text-[10px] px-1.5 py-0.5 rounded bg-(--color-accent-soft) text-(--color-accent)">{t}</span>
                      {/each}
                    </div>
                  {/if}
                </td>
                <td class="px-4 py-2.5 text-xs whitespace-nowrap">
                  {#if !isChild}
                    {#if ep.env === 'sim'}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-violet-500/15 text-violet-300 border border-violet-500/30 mr-1">sim</span>
                    {:else}
                      <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-zinc-500/15 text-zinc-400 border border-zinc-500/30 mr-1">実機</span>
                    {/if}
                    {#if ep.controller_label}
                      {#if ep.controller_label.startsWith('VLA')}
                        <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-cyan-500/15 text-cyan-300 border border-cyan-500/30">🤖 {ep.controller_label}</span>
                      {:else if ep.controller_label === 'teleop'}
                        <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-amber-500/15 text-amber-300 border border-amber-500/30">🎮 teleop</span>
                      {:else}
                        <span class="inline-block px-1.5 py-0.5 rounded text-[10px] bg-zinc-500/15 text-zinc-400 border border-zinc-500/30">{ep.controller_label}</span>
                      {/if}
                    {/if}
                  {/if}
                </td>
                <td class="px-4 py-2.5 text-(--color-text-dim) text-xs font-mono">{fmtTime(ep.started_at)}</td>
                <td class="px-4 py-2.5 text-right font-mono text-xs">{fmtDuration(ep.duration_s)}</td>
                <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{fmtBytes(ep.size_bytes)}</td>
                <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{ep.marker_count}</td>
                <td class="px-2 py-2.5 text-right whitespace-nowrap">
                  <button
                    onclick={(e) => togglePin(ep, e)}
                    class="{ep.pinned ? 'opacity-100 text-amber-400' : 'opacity-0 group-hover:opacity-100 text-(--color-text-mute)'} hover:text-amber-400 transition p-1 rounded hover:bg-amber-500/10 mr-1"
                    title={ep.pinned ? '📌 pinned — クリックで解除 (retention 保護解除)' : 'pin (retention から保護)'}>
                    <Pin class="size-4" />
                  </button>
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
            {/if}
          {/each}
        </tbody>
      </table>
      <!-- Infinite-scroll sentinel — IntersectionObserver fires loadMore
           when this enters the viewport (rootMargin 300px so it pre-loads). -->
      {#if nextCursor}
        <div bind:this={scrollSentinel} class="px-4 py-3 text-center text-[11px] text-(--color-text-mute)">
          {#if loadingMore}
            読み込み中…
          {:else}
            ↓ scroll でさらに読み込み
          {/if}
        </div>
      {:else if episodes.length >= PAGE_SIZE}
        <div class="px-4 py-3 text-center text-[10px] text-(--color-text-mute)">— 末尾 —</div>
      {/if}
    </div>
  {/if}

  <footer class="mt-6 text-center text-[11px] text-(--color-text-mute)">
    {filteredEpisodes.length} 表示 / {episodes.length} 読込 / 全{totalCount} 件 · {pollIntervalS > 0 ? `${pollIntervalS}s 毎自動更新` : '手動更新'} · fv_episode_ui v0.1.0
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
        <!-- Camera thumbnail strip — horizontal, scrollable, like the dashboard
             Cameras tab. Color cams render their mp4 inline; depth cams show
             a placeholder card with a download link (16-bit PNG sequence is
             not browser-playable). Every video shares the timeline via the
             existing sync handlers. -->
        <div class="flex gap-2 mb-3 overflow-x-auto shrink-0 pb-1">
          {#each playEpisode.cameras as cam (cam.name)}
            {@const segments = cam.segments || [{ file: '0000.mp4' }]}
            {@const isDepth = cam.kind === 'depth_png_seq'}
            {@const w = (cam as any).width || 640}
            {@const h = (cam as any).height || 480}
            <div class="shrink-0 flex flex-col rounded-md overflow-hidden border border-(--color-border) bg-(--color-bg-3)"
                 style="width: 280px;">
              <div class="px-2 py-1 text-[11px] font-medium text-(--color-accent) bg-(--color-accent-soft) flex items-center justify-between">
                <span>{cam.name}</span>
                <span class="text-[9px] opacity-70 font-mono">
                  {w}×{h}{#if isDepth} · depth · {cam.frame_count || 0}f{/if}
                </span>
              </div>
              {#if isDepth}
                {@const fps = (cam as any).fps_actual || 30}
                {@const total = cam.frame_count || 0}
                {@const fIdx = total > 0 ? Math.max(0, Math.min(total - 1, Math.floor(sharedTime * fps))) : 0}
                {@const fStr = String(fIdx).padStart(6, '0')}
                <div class="bg-black relative group/depth" style="aspect-ratio: {w} / {h};">
                  {#if total > 0}
                    <img src={`${API}/episodes/${playEpisode.episode_id}/depth_preview/${cam.name}/${fStr}.jpg?cmap=turbo&max=4000`}
                         alt="depth"
                         class="w-full h-full object-contain"
                         loading="lazy" />
                    <div class="absolute top-1 left-1 text-[9px] px-1.5 py-0.5 rounded bg-black/70 text-white font-mono">
                      f{fIdx}/{total - 1}
                    </div>
                    <div class="absolute bottom-1 right-1 text-[9px] px-1.5 py-0.5 rounded bg-black/70 text-(--color-accent)">
                      turbo · 0–4m
                    </div>
                  {:else}
                    <div class="w-full h-full flex flex-col items-center justify-center text-[11px] text-(--color-text-mute) gap-1">
                      <div class="text-2xl opacity-40">📐</div>
                      <div>0 frames</div>
                    </div>
                  {/if}
                </div>
              {:else}
                <video
                  preload="metadata"
                  bind:this={videoEls[cam.name]}
                  onloadedmetadata={() => onVideoMetadata(cam.name)}
                  onplay={() => syncPlayFrom(cam.name)}
                  onpause={() => syncPauseFrom(cam.name)}
                  onseeked={() => syncSeekFrom(cam.name)}
                  ontimeupdate={() => onTimeUpdate(cam.name)}
                  class="w-full bg-black cursor-pointer object-contain"
                  style="aspect-ratio: {w} / {h};"
                  muted
                  ondblclick={(e) => { try { (e.currentTarget as HTMLVideoElement).requestFullscreen(); } catch {} }}
                  src={videoUrl(playEpisode.episode_id, cam.name, segments[0].file)}>
                </video>
              {/if}
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
              <!-- Trim overlay (dim the regions outside [trim_start, trim_end]).
                   Sits above the range slider but visually under the marker band. -->
              {#if (playEpisode.trim_start_s != null || playEpisode.trim_end_s != null) && maxDuration > 0}
                {@const ts = playEpisode.trim_start_s ?? 0}
                {@const te = playEpisode.trim_end_s ?? maxDuration}
                {@const lPct = (ts / maxDuration) * 100}
                {@const rPct = 100 - (te / maxDuration) * 100}
                <div class="pointer-events-none absolute inset-x-0 top-0 h-5 overflow-hidden rounded-sm">
                  {#if lPct > 0}
                    <div class="absolute top-0 left-0 h-full bg-black/55" style="width:{lPct}%"></div>
                  {/if}
                  {#if rPct > 0}
                    <div class="absolute top-0 right-0 h-full bg-black/55" style="width:{rPct}%"></div>
                  {/if}
                  <!-- Bright tick marks at trim boundaries -->
                  <div class="absolute top-0 h-full w-[2px] bg-(--color-accent)" style="left:calc({lPct}% - 1px)"></div>
                  <div class="absolute top-0 h-full w-[2px] bg-(--color-accent)" style="right:calc({rPct}% - 1px)"></div>
                </div>
              {/if}

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

          <!-- Clip trim controls (non-destructive). Sets meta.trim_start_s /
               trim_end_s; underlying bag+mp4 untouched, future exports can
               honor the range. -->
          <div class="flex items-center gap-2 mt-2 text-[11px]">
            <span class="text-(--color-text-mute)">クリップ</span>
            <button onclick={setTrimStart}
                    class="px-2 py-0.5 rounded border border-(--color-border) text-(--color-text-dim) hover:border-(--color-accent) hover:text-(--color-accent) transition"
                    title="現在のプレイヘッド位置を開始点に">
              ✂ ここから ({fmtTimeSec(sharedTime)})
            </button>
            <button onclick={setTrimEnd}
                    class="px-2 py-0.5 rounded border border-(--color-border) text-(--color-text-dim) hover:border-(--color-accent) hover:text-(--color-accent) transition"
                    title="現在のプレイヘッド位置を終了点に">
              ここまで ({fmtTimeSec(sharedTime)}) ✂
            </button>
            {#if playEpisode.trim_start_s != null || playEpisode.trim_end_s != null}
              <span class="text-(--color-accent) font-mono">
                範囲: {fmtTimeSec(playEpisode.trim_start_s ?? 0)} – {fmtTimeSec(playEpisode.trim_end_s ?? maxDuration)}
                ({(((playEpisode.trim_end_s ?? maxDuration) - (playEpisode.trim_start_s ?? 0))).toFixed(1)}s)
              </span>
              <button onclick={clearTrim}
                      class="ml-auto px-2 py-0.5 rounded text-(--color-text-mute) hover:text-red-400 transition"
                      title="クリップ範囲をクリア">
                クリア
              </button>
            {:else}
              <span class="text-(--color-text-mute) ml-auto">未設定 — bag/mp4 は変更されません</span>
            {/if}
          </div>
        </div>

        <!-- Marker list — read + click-to-seek + per-row "+ 属性" editor.
             "+ マーカー追加" button always visible (even with zero markers) so
             post-hoc marker creation is discoverable. -->
        <div class="shrink-0 mt-3 max-h-40 overflow-auto border-t border-(--color-border) pt-2">
          <div class="text-[11px] text-(--color-text-mute) mb-1.5 flex items-center gap-3 flex-wrap">
            <span>マーカー ({playEpisode.markers?.length || 0})</span>
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
            <button onclick={() => openAddMarker('event')}
                    class="ml-auto px-2 py-0.5 rounded border border-(--color-accent-glow) bg-(--color-accent-soft) text-(--color-accent) hover:brightness-125 transition text-[11px]">
              + マーカー追加 <span class="opacity-60">({fmtTimeSec(sharedTime)})</span>
            </button>
          </div>
        {#if playEpisode.markers && playEpisode.markers.length > 0}
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
                    <td class="px-2 py-1.5 text-right w-20 whitespace-nowrap">
                      <button onclick={(e) => openAttrEditor(m, e)}
                              class="opacity-30 group-hover:opacity-100 text-(--color-text-mute) hover:text-(--color-accent) transition text-[10px] px-1.5 py-0.5 rounded border border-(--color-border) mr-1"
                              title="属性 (重量・グレード等) を編集">
                        + 属性
                      </button>
                      <button onclick={(e) => deleteMarker(m, e)}
                              class="opacity-30 group-hover:opacity-100 text-(--color-text-mute) hover:text-red-400 transition p-1 rounded hover:bg-red-500/10 align-middle"
                              title="マーカーを削除">
                        <Trash2 class="size-3.5 inline" />
                      </button>
                    </td>
                  </tr>
                {/each}
            </tbody>
          </table>
        {/if}
        </div>

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
      {/if}<!-- closes {#if playEpisode.cameras.length === 0} ... {:else} ... at line 819 -->

      <div class="mt-3 text-[10px] text-(--color-text-mute) font-mono shrink-0">
        ID: {playEpisode.episode_id}
      </div>
    </div>
  </div>
{/if}

<!-- Add marker (post-hoc) modal — pick kind/label/start/end, defaults to playhead -->
{#if addMarkerForm}
  <div class="fixed inset-0 z-[60] flex items-center justify-center bg-black/85 backdrop-blur-sm p-4"
       onclick={(e) => { if (e.target === e.currentTarget) addMarkerForm = null; }}
       onkeydown={(e) => { if (e.key === 'Escape') addMarkerForm = null; }}
       role="dialog"
       tabindex="-1">
    <div class="card w-full max-w-md p-5">
      <header class="flex items-start justify-between mb-3">
        <h2 class="text-base font-semibold text-white">マーカー追加</h2>
        <button onclick={() => (addMarkerForm = null)} class="text-(--color-text-mute) hover:text-white text-2xl leading-none">×</button>
      </header>

      <div class="space-y-3">
        <!-- Kind selector -->
        <div>
          <div class="text-[11px] text-(--color-text-mute) mb-1">種類</div>
          <div class="flex gap-1.5">
            {#each ['subtask','event','note'] as k}
              <button
                onclick={() => (addMarkerForm = {...addMarkerForm!, kind: k as any})}
                class="flex-1 px-2 py-1.5 rounded text-xs border transition"
                style="border-color: {addMarkerForm.kind === k ? kindColor(k) : 'rgba(255,255,255,0.1)'}; background: {addMarkerForm.kind === k ? kindColor(k) + '22' : 'transparent'}; color: {addMarkerForm.kind === k ? kindColor(k) : 'rgba(255,255,255,0.6)'}">
                {k === 'subtask' ? '区間 (subtask)' : k === 'event' ? 'イベント' : 'メモ (note)'}
              </button>
            {/each}
          </div>
        </div>

        <!-- Label -->
        <div>
          <div class="text-[11px] text-(--color-text-mute) mb-1">ラベル</div>
          <input type="text"
                 bind:value={addMarkerForm.label}
                 placeholder={addMarkerForm.kind === 'subtask' ? '採取 / アスパラをカット' : addMarkerForm.kind === 'event' ? '気になる動き / 衝突' : '気づきメモ'}
                 class="w-full bg-(--color-bg-2) border border-(--color-border) rounded px-3 py-2 text-sm focus:border-(--color-accent-glow) focus:outline-none"
                 autofocus
                 onkeydown={(e) => { if (e.key === 'Enter') submitAddMarker(); }} />
        </div>

        <!-- Time(s) -->
        <div class="grid {addMarkerForm.kind === 'subtask' ? 'grid-cols-2' : 'grid-cols-1'} gap-2">
          <div>
            <div class="text-[11px] text-(--color-text-mute) mb-1">開始 (秒)</div>
            <input type="number" step="0.1" min="0" max={maxDuration}
                   bind:value={addMarkerForm.start_s}
                   class="w-full bg-(--color-bg-2) border border-(--color-border) rounded px-3 py-2 text-sm font-mono focus:border-(--color-accent-glow) focus:outline-none" />
            <div class="text-[10px] text-(--color-text-mute) mt-1">= {fmtTimeSec(addMarkerForm.start_s)}</div>
          </div>
          {#if addMarkerForm.kind === 'subtask'}
            <div>
              <div class="text-[11px] text-(--color-text-mute) mb-1">終了 (秒)</div>
              <input type="number" step="0.1" min="0" max={maxDuration}
                     bind:value={addMarkerForm.end_s}
                     class="w-full bg-(--color-bg-2) border border-(--color-border) rounded px-3 py-2 text-sm font-mono focus:border-(--color-accent-glow) focus:outline-none" />
              <div class="text-[10px] text-(--color-text-mute) mt-1">= {fmtTimeSec(addMarkerForm.end_s)} (長さ {(addMarkerForm.end_s - addMarkerForm.start_s).toFixed(1)}s)</div>
            </div>
          {/if}
        </div>

        <!-- Quick time helpers -->
        <div class="flex flex-wrap gap-1.5 text-[10px]">
          <span class="text-(--color-text-mute)">クイック設定:</span>
          <button class="px-2 py-0.5 rounded border border-(--color-border) hover:border-(--color-accent) text-(--color-text-mute) hover:text-(--color-accent)"
                  onclick={() => { addMarkerForm = {...addMarkerForm!, start_s: sharedTime, end_s: sharedTime + (addMarkerForm!.end_s - addMarkerForm!.start_s)}; }}>
            開始を今 ({fmtTimeSec(sharedTime)}) に
          </button>
          {#if addMarkerForm.kind === 'subtask'}
            <button class="px-2 py-0.5 rounded border border-(--color-border) hover:border-(--color-accent) text-(--color-text-mute) hover:text-(--color-accent)"
                    onclick={() => { addMarkerForm = {...addMarkerForm!, end_s: sharedTime}; }}>
              終了を今に
            </button>
          {/if}
        </div>
      </div>

      <div class="mt-5 flex justify-end gap-2">
        <button onclick={() => (addMarkerForm = null)} class="px-3 py-1.5 text-sm rounded bg-(--color-bg-3) text-(--color-text-dim) hover:text-white">キャンセル</button>
        <button onclick={submitAddMarker} class="px-4 py-1.5 text-sm rounded font-semibold hover:brightness-110"
                style="background:{kindColor(addMarkerForm.kind)}; color:#0a0a0f;">
          追加
        </button>
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
