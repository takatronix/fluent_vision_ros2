<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Search, RefreshCw, Pin, Trash2, PlayCircle, HardDrive } from 'lucide-svelte';

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
    active: { rate_bytes_per_s: number; elapsed_s: number; hours_left_at_current_rate: number | null } | null;
  };

  // Recorder API base — same host as the iframe's parent dashboard, port 8083.
  // (CORS allowed in api_server.py.)
  const API = `http://${location.hostname}:8083/api/v1`;

  let episodes = $state<Episode[]>([]);
  let disk = $state<Disk | null>(null);
  let loading = $state(true);
  let error = $state<string | null>(null);
  let query = $state('');
  let selectedProfile = $state<string>('');  // '' = all profiles
  let pollTimer: number | null = null;

  // Play modal state
  type EpisodeDetail = {
    episode_id: string;
    task_description: string;
    profile: string;
    duration_s: number | null;
    cameras: Array<{ name: string; topic: string; segments?: Array<{ file: string }> }>;
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

  const uniqueProfiles = $derived(
    Array.from(new Set(episodes.map(e => e.profile))).sort()
  );

  const filteredEpisodes = $derived(
    episodes.filter(e => {
      if (selectedProfile && e.profile !== selectedProfile) return false;
      if (query.trim()) {
        const q = query.toLowerCase();
        if (!e.task_description.toLowerCase().includes(q) &&
            !e.tags.some(t => t.toLowerCase().includes(q))) return false;
      }
      return true;
    })
  );

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
      <p class="text-xs text-(--color-text-mute) mt-0.5">fv_episode_recorder · {API}</p>
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
    </section>
  {/if}

  <!-- Search bar + profile filter -->
  <div class="mb-3 flex gap-2">
    <div class="relative flex-1">
      <Search class="size-4 absolute left-3 top-1/2 -translate-y-1/2 text-(--color-text-mute)" />
      <input
        type="text"
        bind:value={query}
        placeholder="タスク名 / タグで検索…"
        class="w-full pl-9 pr-4 py-2 rounded-lg bg-(--color-bg-2) border border-(--color-border) text-sm placeholder:text-(--color-text-mute) focus:border-(--color-accent-glow) focus:outline-none transition"
      />
    </div>
    <select
      bind:value={selectedProfile}
      class="px-3 py-2 rounded-lg bg-(--color-bg-2) border border-(--color-border) text-sm focus:border-(--color-accent-glow) focus:outline-none transition min-w-[200px]"
    >
      <option value="">すべての profile</option>
      {#each uniqueProfiles as p (p)}
        <option value={p}>{p}</option>
      {/each}
    </select>
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
            <th class="text-left px-4 py-2.5 font-medium">タスク</th>
            <th class="text-left px-4 py-2.5 font-medium">開始</th>
            <th class="text-right px-4 py-2.5 font-medium">長さ</th>
            <th class="text-right px-4 py-2.5 font-medium">marker</th>
            <th class="w-16"></th>
          </tr>
        </thead>
        <tbody>
          {#each filteredEpisodes as ep (ep.episode_id)}
            {@const b = outcomeBadge(ep.outcome)}
            <tr class="border-t border-(--color-border) hover:bg-(--color-bg-3)/40 transition-colors">
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
              <td class="px-4 py-2.5 text-right font-mono text-xs text-(--color-text-dim)">{ep.marker_count}</td>
              <td class="px-4 py-2.5">
                <button
                  onclick={() => openPlay(ep)}
                  class="text-(--color-accent) hover:text-white hover:scale-110 transition"
                  title="再生">
                  <PlayCircle class="size-5" />
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
            <input
              type="range"
              min="0"
              max={maxDuration || 1}
              step="0.01"
              value={sharedTime}
              oninput={(e) => seekAll(parseFloat(e.currentTarget.value))}
              class="flex-1 accent-(--color-accent)"
            />
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
      {/if}

      <div class="mt-3 text-[10px] text-(--color-text-mute) font-mono shrink-0">
        ID: {playEpisode.episode_id}
      </div>
    </div>
  </div>
{/if}
