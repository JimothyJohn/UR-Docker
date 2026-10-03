// Perceptronic — Application Node presenter: a plain custom element (no
// framework, no build). PolyScope X sets `applicationNode`, `applicationAPI`,
// `robotSettings` and `robotContext` on it (SDK 6.5.65 JavaScript template).
//
// Two tabs, so nothing scrolls: Camera (the feed and the moves) and Pick areas (the areas
// the 3D Pick node looks at, on a map of the arm's reach).
//
// The page talks to the RealSense cockpit (`perceptronics gui --cors <this origin>`)
// over its HTTP API: the colour feed is `GET /api/color.png` long-polled by
// sequence number (the toggle in the picture's corner switches it to `/api/depth.png`,
// the depth as a heatmap), hover reads `GET /api/point`, a click runs
// `POST /api/segment` → `POST /api/robot/locate` (hand-eye → base frame,
// approach pose, reach check), and the two Move buttons either hand the target
// to PolyScope (inverse kinematics + UR's own auto-move screen, operator in the
// loop) or ask the cockpit to move over Primary (urctl's safety envelope).
(() => {
  const TAG = "advin-perceptronic";
  const DEFAULT_COCKPIT_PORT = 7621;
  // The pick PC's factory address (pickscript.js DEFAULT_COCKPIT_HOST; a test holds them equal).
  const DEFAULT_COCKPIT_HOST = "192.168.3.20";
  const POLL_TIMEOUT_MS = 1500;
  // One feed request, connect + long poll + picture: an address nothing answers on (the pick
  // PC's default before it is plugged in) must fail in seconds, not after TCP gives up.
  const FEED_TIMEOUT_MS = POLL_TIMEOUT_MS + 3500;
  const HOVER_MS = 150;
  const PROBE_TIMEOUT_MS = 2500;
  const LOOPBACK = new Set(["localhost", "127.0.0.1", "[::1]", "::1"]);

  const CSS = `
    .rsp { font: 14px/1.4 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color: #1f2a37; padding: 12px 16px; max-width: 1100px; }
    .rsp h2 { margin: 0 0 8px; font-size: 18px; display: flex; align-items: center; gap: 10px; }
    .rsp .dot { width: 10px; height: 10px; border-radius: 50%; background: #c0c8d2; display: inline-block; }
    .rsp .dot.live { background: #1d9a5a; } .rsp .dot.dead { background: #d64545; }
    .rsp .row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin: 6px 0; }
    .rsp input[type=text] { flex: 1 1 320px; min-width: 240px; padding: 8px 10px; border: 1px solid #b9c3cf; border-radius: 6px; font: inherit; }
    .rsp button { padding: 9px 14px; border: 1px solid #8e9bab; border-radius: 6px; background: #fff; font: inherit; cursor: pointer; }
    .rsp button:disabled { opacity: .45; cursor: default; }
    .rsp button.primary { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .rsp button.danger { background: #d64545; color: #fff; border-color: #d64545; }
    .rsp .stage { position: relative; display: inline-block; max-width: 100%; margin-top: 8px; background: #0f1620; border-radius: 8px; overflow: hidden; }
    .rsp .stage img { display: block; max-width: 100%; max-height: max(180px, calc(100vh - 470px)); width: auto; height: auto; min-width: 320px; min-height: 180px; cursor: crosshair; }
    .rsp .stage .view { position: absolute; right: 10px; top: 10px; display: inline-flex; padding: 3px; border-radius: 20px; background: rgba(13,19,26,.85); }
    .rsp .stage .view button { padding: 6px 14px; border: 0; border-radius: 16px; background: none; color: #fff; font: inherit; font-weight: 600; }
    .rsp .stage .view button.on { background: #1f5fbf; }
    .rsp .stage .nocam { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; background: #1a2029; }
    .rsp .stage .nocam > div { max-width: 90%; padding: 12px 18px; border: 3px solid #d64545; border-radius: 14px; background: #0d131a; color: #c9d3de; font-size: 13px; white-space: pre-wrap; }
    .rsp .stage .nocam b { display: block; color: #fff; font-size: 20px; text-align: center; margin-bottom: 6px; }
    .rsp .tabs { display: inline-flex; margin-left: auto; }
    .rsp .tabs button { border-radius: 0; font-size: 14px; font-weight: 600; } .rsp .tabs button:first-child { border-radius: 6px 0 0 6px; } .rsp .tabs button:last-child { border-radius: 0 6px 6px 0; }
    .rsp .tabs button.on { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .rsp .two { display: flex; gap: 16px; align-items: flex-start; }
    .rsp .two > .card { flex: 1 1 0; min-width: 0; }
    .rsp .two > .reach-card { flex: 0 0 330px; }
    .rsp .hidden { display: none !important; }
    .rsp .mark { position: absolute; width: 22px; height: 22px; margin: -11px 0 0 -11px; border: 2px solid #ffd23f; border-radius: 50%; box-shadow: 0 0 0 2px rgba(0,0,0,.5); pointer-events: none; display: none; }
    .rsp .mark::after { content: ""; position: absolute; left: 9px; top: 9px; width: 4px; height: 4px; background: #ffd23f; border-radius: 50%; }
    .rsp .hover { position: absolute; left: 8px; bottom: 8px; padding: 3px 8px; background: rgba(15,22,32,.8); color: #e8eef6; border-radius: 4px; font-size: 12px; pointer-events: none; }
    .rsp .status { margin-top: 8px; padding: 8px 10px; border-radius: 6px; background: #eef2f7; white-space: pre-wrap; }
    .rsp .status.warn { background: #fff4d6; } .rsp .status.err { background: #fde2e2; } .rsp .status.ok { background: #e3f5ea; }
    .rsp .target { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12.5px; }
    .rsp small { color: #5b6b7d; }
    .rsp .card { border: 1px solid #d5dce5; border-radius: 8px; padding: 10px 12px; margin: 10px 0 0; }
    .rsp .card h3 { margin: 0 0 6px; font-size: 14px; }
    .rsp .card h3 small { font-weight: normal; }
    .rsp .area { display: grid; grid-template-columns: 1fr auto auto auto auto; gap: 6px; align-items: center; padding: 6px 0; border-bottom: 1px solid #eef2f7; }
    .rsp .area input { padding: 5px 8px; border: 1px solid #b9c3cf; border-radius: 6px; font: inherit; min-width: 0; }
    .rsp .area .touch { padding: 5px 9px; font-size: 12.5px; }
    .rsp .area .touch.done { background: #e3f5ea; border-color: #1d9a5a; }
    .rsp .area .plane { grid-column: 1 / -1; font-size: 12px; color: #5b6b7d; }
    .rsp .area .plane.warn { color: #9a6b00; }
    .rsp .reach-card input { padding: 5px 8px; border: 1px solid #b9c3cf; border-radius: 6px; font: inherit; width: 64px; text-align: right; }
    .rsp .reach-map { margin: 4px 0 8px; } .rsp .reach-map svg { display: block; max-width: 100%; height: auto; }
  `;

  const ARCHIVE_PATH = "/advin/perceptronic/perceptronic-frontend/";
  const selfUrl = typeof document !== "undefined" && document.currentScript && document.currentScript.src;
  // pickscript.js (the Pick node's settings + pose math) also serves this node's pick areas and reach:
  // loaded once, by <script>, the first time a node needs it (never at load — tests run this file under node).
  let libPromise = null;
  const loadLib = () => libPromise || (libPromise = new Promise((resolve, reject) => {
    if (window.PerceptronicPick) { resolve(window.PerceptronicPick); return; }
    const libUrl = new URL("pickscript.js", selfUrl || `${location.origin}${ARCHIVE_PATH}`).href;
    const existing = document.querySelector("script[data-perceptronic-lib]");
    const tag = existing || document.createElement("script");
    if (!existing) {
      tag.src = libUrl;
      tag.dataset.perceptronicLib = "1";
      document.head.appendChild(tag);
    }
    tag.addEventListener("load", () => (window.PerceptronicPick ? resolve(window.PerceptronicPick) : reject(new Error("pickscript.js loaded but defined nothing"))));
    tag.addEventListener("error", () => reject(new Error(`could not load ${libUrl}`)));
  }));

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const fmt = (v, d = 3) => (typeof v === "number" ? v.toFixed(d) : "?");
  const fmtVec = (v, d = 3) => (Array.isArray(v) ? v.map((x) => fmt(x, d)).join(", ") : "?");

  // First value of an rxjs-style Observable (robotPositionService.getJointPositions()).
  function firstValue(observable) {
    return new Promise((resolve, reject) => {
      if (!observable || typeof observable.subscribe !== "function") {
        reject(new Error("no joint-position stream from PolyScope"));
        return;
      }
      let done = false;
      const sub = observable.subscribe({
        next: (v) => {
          if (done) return;
          done = true;
          try { sub && sub.unsubscribe && sub.unsubscribe(); } catch (e) { /* already closed */ }
          resolve(v);
        },
        error: reject,
        complete: () => { if (!done) reject(new Error("joint-position stream ended without a value")); },
      });
    });
  }

  // -- pose math (UR pose = [x, y, z, rx, ry, rz], rotation vector; 4x4 row-major) ----------
  function poseToMat(p) {
    const th = Math.hypot(p[3], p[4], p[5]);
    const [x, y, z] = th < 1e-12 ? [0, 0, 1] : [p[3] / th, p[4] / th, p[5] / th];
    const c = Math.cos(th), s = Math.sin(th), v = 1 - c;
    return [
      [c + x * x * v, x * y * v - z * s, x * z * v + y * s, p[0]],
      [y * x * v + z * s, c + y * y * v, y * z * v - x * s, p[1]],
      [z * x * v - y * s, z * y * v + x * s, c + z * z * v, p[2]],
      [0, 0, 0, 1],
    ];
  }
  function matToPose(m) {
    const cos = Math.min(1, Math.max(-1, (m[0][0] + m[1][1] + m[2][2] - 1) / 2));
    const th = Math.acos(cos);
    let r = [0, 0, 0];
    if (th > 1e-9 && Math.PI - th > 1e-6) {
      const k = th / (2 * Math.sin(th));
      r = [(m[2][1] - m[1][2]) * k, (m[0][2] - m[2][0]) * k, (m[1][0] - m[0][1]) * k];
    } else if (th > 1e-9) {
      // ~180°: the axis from the diagonal, signed by the largest component
      const ax = [0, 1, 2].map((i) => Math.sqrt(Math.max(0, (m[i][i] + 1) / 2)));
      const i = ax.indexOf(Math.max(...ax));
      for (let j = 0; j < 3; j++) if (j !== i) ax[j] = Math.sign(m[i][j] + m[j][i] || 1) * ax[j];
      r = ax.map((a) => a * th);
    }
    return [m[0][3], m[1][3], m[2][3], ...r];
  }
  function matMul(a, b) {
    return a.map((row) => [0, 1, 2, 3].map((j) => row.reduce((acc, v, k) => acc + v * b[k][j], 0)));
  }
  function matInv(m) {
    const R = [0, 1, 2].map((i) => [0, 1, 2].map((j) => m[j][i]));
    const t = [0, 1, 2].map((i) => -(R[i][0] * m[0][3] + R[i][1] * m[1][3] + R[i][2] * m[2][3]));
    return [[...R[0], t[0]], [...R[1], t[1]], [...R[2], t[2]], [0, 0, 0, 1]];
  }
  // Flange pose at the given joint angles from PolyScope's own DH table
  // (robotPositionService.getKinematicInfo(): [{DHTheta, DHa, DHd, DHAlpha}] × 6).
  function flangeMat(dh, q) {
    let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    dh.forEach((j, i) => {
      const th = q[i] + (j.DHTheta || 0), ct = Math.cos(th), st = Math.sin(th);
      const ca = Math.cos(j.DHAlpha), sa = Math.sin(j.DHAlpha);
      T = matMul(T, [[ct, -st * ca, st * sa, j.DHa * ct], [st, ct * ca, -ct * sa, j.DHa * st], [0, sa, ca, j.DHd], [0, 0, 0, 1]]);
    });
    return T;
  }
  const withTimeout = (p, ms, why) =>
    Promise.race([p, new Promise((_, reject) => setTimeout(() => reject(new Error(why)), ms))]);

  class Perceptronic extends HTMLElement {
    // PolyScope's IK solves for *PolyScope's* active TCP, which is not the TCP the cockpit's
    // controller reported (a different robot while testing in the sim; a stale training
    // offset on a real one). So the cockpit's flange target is re-expressed in PolyScope's
    // TCP: offset = FK_flange(0)⁻¹ · TCP(0), both from PolyScope at the zero joint vector.
    static polyScopeTarget(flangeTarget, dh, tcpAtZeroPose) {
      const offset = matMul(matInv(flangeMat(dh, [0, 0, 0, 0, 0, 0])), poseToMat(tcpAtZeroPose));
      return matToPose(matMul(poseToMat(flangeTarget), offset));
    }

    constructor() {
      super();
      this._node = null;
      this._api = null;
      this._robotSettings = null;
      this._robotContext = null;
      this._built = false;
      this._seq = 0;
      this._polling = false;
      this._stopped = false;
      this._blobUrl = null;
      this._located = null;
      this._hoverTimer = null;
      this._hoverPending = null;
      this._lib = null;
      this._modelAsked = false;
      this._tab = "camera";
      this._depth = false; // the picture's toggle: the depth as a heatmap
      this._logged = "";
      this._colourWidth = 0;
    }

    // -- properties PolyScope X sets -----------------------------------------------
    get applicationNode() { return this._node; }
    set applicationNode(value) { this._node = value; this.render(); }
    get applicationAPI() { return this._api; }
    set applicationAPI(value) { this._api = value; }
    get robotSettings() { return this._robotSettings; }
    set robotSettings(value) { this._robotSettings = value; }
    get robotContext() { return this._robotContext; }
    set robotContext(value) { this._robotContext = value; }

    connectedCallback() { this._stopped = false; this.render(); }
    disconnectedCallback() {
      this._stopped = true;
      if (this._blobUrl) { URL.revokeObjectURL(this._blobUrl); this._blobUrl = null; }
    }

    // -- cockpit --------------------------------------------------------------------
    cockpitUrl() {
      return Perceptronic.cockpitBase(this._node && this._node.cockpitUrl, location);
    }

    // The saved field as an absolute base URL. Empty → the pick PC's factory address
    // (http://192.168.3.20:7621). Shorthand is completed against this page's
    // host — ":7621" / "7621" → http://<page host>:7621, "host[:port]" → http://host[:port],
    // a bare host gets :7621. Anything without a scheme would otherwise be fetched *relative
    // to PolyScope's own page*, and PolyScope's 404 read as "the cockpit is outdated".
    static cockpitBase(saved, loc) {
      const raw = (saved ? String(saved) : "").trim().replace(/\/+$/, "");
      const pageHost = `${loc.protocol}//${loc.hostname}`;
      if (!raw) return `http://${DEFAULT_COCKPIT_HOST}:${DEFAULT_COCKPIT_PORT}`;
      const port = /^:?(\d{1,5})$/.exec(raw);
      if (port) return `${pageHost}:${port[1]}`;
      if (/^[a-z][a-z0-9+.-]*:\/\//i.test(raw)) return raw;
      const withScheme = `http://${raw}`;
      try {
        const u = new URL(withScheme);
        if (!u.port && u.pathname === "/") return `${withScheme}:${DEFAULT_COCKPIT_PORT}`;
      } catch (e) { /* diagnose() names the bad value */ }
      return withScheme;
    }

    // "Failed to fetch" is all the browser says, whether the cockpit refused this page's
    // origin (CORS) or nothing answered. A no-cors request tells them apart: it resolves
    // (opaque) when the server is up, whatever its CORS list, and rejects when it isn't.
    async diagnose() {
      const base = this.cockpitUrl();
      const origin = location.origin;
      let target;
      try { target = new URL(base); } catch (e) {
        return `"${base}" is not a URL — enter it as http://<host>:7621`;
      }
      if (location.protocol === "https:" && target.protocol === "http:") {
        return `this page is https and the cockpit is http — the browser blocks that (mixed content).`;
      }
      const ctl = typeof AbortController === "function" ? new AbortController() : null;
      const timer = ctl ? setTimeout(() => ctl.abort(), PROBE_TIMEOUT_MS) : null;
      try {
        await fetch(`${base}/api/info`, { mode: "no-cors", cache: "no-store", signal: ctl ? ctl.signal : undefined });
        return (
          `the cockpit at ${base} is running but refuses this page (origin ${origin}).\n` +
          `Restart it with  --cors ${origin}  (or PERCEPTRONICS_CORS=${origin}); ` +
          `a cockpit started before --cors existed needs a restart on the new code.`
        );
      } catch (e) {
        const timedOut = e && e.name === "AbortError";
        const hints = [];
        if (LOOPBACK.has(target.hostname) && !LOOPBACK.has(location.hostname)) {
          hints.push(`"${target.hostname}" is the machine running this browser, not necessarily the cockpit's — use the cockpit host's IP`);
        } else if (!LOOPBACK.has(target.hostname)) {
          hints.push("a cockpit listens on loopback only unless started with --bind 0.0.0.0");
        }
        hints.push(timedOut ? `no answer in ${PROBE_TIMEOUT_MS / 1000} s — firewall or wrong subnet?` : "wrong host or port, or the cockpit is not running");
        return `nothing answers at ${base} from this browser.\n` + hints.map((h) => `· ${h}`).join("\n");
      } finally {
        if (timer) clearTimeout(timer);
      }
    }

    async api(method, path, body) {
      const init = { method, headers: {} };
      if (body !== undefined) {
        init.headers["Content-Type"] = "application/json";
        init.body = JSON.stringify(body);
      }
      const r = await fetch(this.cockpitUrl() + path, init);
      let out;
      try { out = await r.json(); } catch (e) { out = { ok: false, error: `HTTP ${r.status}` }; }
      if (out && out.ok === undefined) out.ok = r.ok;
      return out;
    }

    // -- rendering --------------------------------------------------------------------
    $(name) { return this.querySelector(`[data-rsp="${name}"]`); }

    render() {
      if (!this._node) return;
      if (!this._built) {
        this.innerHTML = `
          <style>${CSS}</style>
          <div class="rsp">
            <h2><span class="dot" data-rsp="dot"></span> Perceptronic <small data-rsp="fps"></small>
              <span class="tabs" data-rsp="tabs"><button data-tab="camera" class="on">Camera</button><button data-tab="areas">Pick areas</button></span>
            </h2>
            <div data-rsp="tab-camera">
            <div class="row">
              <label for="rsp-url">Cockpit</label>
              <input id="rsp-url" type="text" data-rsp="url" placeholder="the pick PC's address (192.168.3.20 out of the box)" />
              <button data-rsp="save">Save</button>
              <a data-rsp="open" href="#" target="_blank" rel="noopener">Open cockpit</a>
            </div>
            <div class="stage" data-rsp="stage">
              <img data-rsp="img" alt="wrist camera" draggable="false" />
              <div class="mark" data-rsp="mark"></div>
              <div class="hover" data-rsp="hover">hover for depth · click a point</div>
              <div class="nocam" data-rsp="nocam"><div><b>NO CAMERA CONNECTED</b><span data-rsp="nocam-text">connecting…</span></div></div>
              <div class="view hidden" data-rsp="view"><button data-view="picture" class="on">Picture</button><button data-view="depth">Depth</button></div>
            </div>
            <div class="status" data-rsp="status">connecting…</div>
            <div class="target" data-rsp="target"></div>
            <div class="row">
              <button class="primary" data-rsp="move-ps" disabled title="Inverse kinematics + PolyScope's auto-move screen (hold to move)">Move (PolyScope)</button>
              <button data-rsp="move-ck" disabled title="The cockpit moves the arm over Primary through urctl's safety envelope">Move (cockpit)</button>
              <button data-rsp="bringup" title="power on + brake release + unlock protective stop, via the cockpit">Bring up</button>
              <button class="danger" data-rsp="stop">STOP</button>
              <button data-rsp="clear">Clear</button>
            </div>
            <small>Click = segment at the pixel → point in the base frame through the hand-eye → approach pose above it. Reach is checked before a move is offered.</small>
            </div>
            <div class="two hidden" data-rsp="tab-areas">
              <div class="card" data-rsp="areas-card">
                <h3>Pick areas <small>for the 3D Pick node — touch the table with the fingertips at a corner, along one edge, and on the far side</small></h3>
                <div data-rsp="areas"></div>
                <div class="row"><button data-rsp="area-add">New area</button> <small data-rsp="areas-note"></small></div>
              </div>
              <div class="card reach-card" data-rsp="reach-card">
                <h3>The arm's reach <small data-rsp="reach-model"></small></h3>
                <div class="reach-map" data-rsp="reach-map"></div>
                <div class="row">
                  <label>Tool length <input type="text" inputmode="numeric" data-rsp="tipMm" /> mm</label>
                </div>
                <small data-rsp="reach-text"></small>
              </div>
            </div>
          </div>`;
        this.wire();
        this._built = true;
        this.startPolling();
        this.startAreas();
      }
      this.$("url").value = this._node.cockpitUrl || DEFAULT_COCKPIT_HOST;
      this.$("open").href = this.cockpitUrl() + "/";
    }

    wire() {
      const img = this.$("img");
      img.addEventListener("click", (ev) => this.onClick(ev));
      img.addEventListener("mousemove", (ev) => this.onHover(ev));
      img.addEventListener("mouseleave", () => { this.$("hover").textContent = "hover for depth · click a point"; });
      this.$("save").addEventListener("click", () => this.saveCockpitUrl());
      this.$("url").addEventListener("keydown", (ev) => { if (ev.key === "Enter") this.saveCockpitUrl(); });
      this.$("move-ps").addEventListener("click", () => this.movePolyScope());
      this.$("move-ck").addEventListener("click", () => this.moveCockpit());
      this.$("bringup").addEventListener("click", () => this.bringUp());
      this.$("stop").addEventListener("click", () => this.stop());
      this.$("clear").addEventListener("click", () => this.clearTarget());
      this.$("tabs").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
        this._tab = b.dataset.tab;
        this.$("tab-camera").classList.toggle("hidden", this._tab !== "camera");
        this.$("tab-areas").classList.toggle("hidden", this._tab !== "areas");
        this.$("tabs").querySelectorAll("button").forEach((t) => t.classList.toggle("on", t.dataset.tab === this._tab));
      }));
      // the Picture / Depth toggle, inside the picture's frame
      this.$("view").querySelectorAll("button").forEach((b) => b.addEventListener("click", (ev) => {
        ev.stopPropagation();
        this._depth = b.dataset.view === "depth";
        this._seq = 0;
        this.$("view").querySelectorAll("button").forEach((t) => t.classList.toggle("on", (t.dataset.view === "depth") === this._depth));
      }));
    }

    setStatus(text, kind) {
      const el = this.$("status");
      if (!el) return;
      el.textContent = text;
      el.className = "status" + (kind ? ` ${kind}` : "");
    }

    async persist() {
      try {
        if (this._api && this._api.applicationNodeService) await this._api.applicationNodeService.updateNode(this._node);
      } catch (err) {
        this.setStatus(`could not save the node: ${err && err.message ? err.message : err}`, "err");
      }
    }

    // -- pick areas (what the 3D Pick program node reads from this node), on the arm's reach ---------
    async startAreas() {
      try {
        this._lib = await loadLib();
      } catch (err) {
        this.$("areas-note").textContent = err.message;
        return;
      }
      this.$("area-add").addEventListener("click", () => this.areaAdd());
      this.$("tipMm").addEventListener("change", (ev) => {
        const v = Math.round(parseFloat(String(ev.target.value).replace(",", ".")));
        this._node.tipMm = Number.isFinite(v) ? Math.max(0, Math.min(500, v)) : this._node.tipMm;
        this.persist();
        this.syncAreas();
      });
      this.syncAreas();
      await this.askRobotModel();
    }

    async askRobotModel() {
      if (this._modelAsked || !this._api || !this._api.robotInfoService) return;
      this._modelAsked = true;
      try {
        const model = String((await this._api.robotInfoService.getRobotType()) || "").trim();
        if (model && model !== this._node.robotModel) {
          this._node.robotModel = model;
          await this.persist();
        }
      } catch (e) {
        this._modelAsked = false;
      }
      this.syncAreas();
    }

    syncAreas() {
      const P = this._lib;
      if (!P || !this._built) return;
      const node = this._node;
      const areas = Array.isArray(node.areas) ? node.areas : (node.areas = []);
      const tip = Number.isFinite(node.tipMm) ? node.tipMm : P.DEFAULT_TIP_MM;
      const box = this.$("areas");
      box.innerHTML = "";
      areas.forEach((a, i) => {
        const row = document.createElement("div");
        row.className = "area";
        const pl = P.plane(a.p0, a.p1, a.p2);
        const missing = ["p0", "p1", "p2"].filter((k) => !a[k]).length;
        row.innerHTML = `
          <input type="text" value="${(a.name || `Area ${i + 1}`).replace(/"/g, "&quot;")}" maxlength="24" title="name" />
          <button class="touch ${a.p0 ? "done" : ""}" data-k="p0" title="touch the table at a corner of the area">Corner</button>
          <button class="touch ${a.p1 ? "done" : ""}" data-k="p1" title="touch along one edge from that corner">Edge</button>
          <button class="touch ${a.p2 ? "done" : ""}" data-k="p2" title="touch the far side">Far side</button>
          <button class="touch" data-k="del" title="remove this area">✕</button>
          <div class="plane"></div>`;
        const plane = row.querySelector(".plane");
        if (pl) {
          const tilt = P.tiltDeg(pl);
          plane.textContent = `${Math.round(Math.abs(pl[6]) * 1000)} × ${Math.round(Math.abs(pl[7]) * 1000)} mm, origin [${pl.slice(0, 3).map((v) => v.toFixed(3)).join(", ")}] m` +
            (tilt > 2 ? ` — tilted ${tilt.toFixed(1)}° from the base plane: re-touch (the table is flat)` : ` · level (${tilt.toFixed(1)}°)`);
          plane.className = tilt > 2 ? "plane warn" : "plane";
        } else if (!missing) {
          plane.textContent = "the three touches are in a line or too close — touch a corner, along one edge, and the far side";
          plane.className = "plane warn";
        } else {
          plane.textContent = `touch ${missing} more point${missing === 1 ? "" : "s"} with the fingertips (${tip} mm past the flange)`;
        }
        row.querySelector("input").addEventListener("change", (ev) => {
          const name = String(ev.target.value).replace(/[^A-Za-z0-9 ._-]/g, "").trim().slice(0, 24);
          if (name) { a.name = name; this.persist(); }
          this.syncAreas();
        });
        row.querySelectorAll(".touch").forEach((b) => b.addEventListener("click", () => (b.dataset.k === "del" ? this.areaRemove(i) : this.touch(i, b.dataset.k))));
        box.appendChild(row);
      });
      this.$("area-add").disabled = areas.length >= P.MAX_AREAS;
      this.$("areas-note").textContent = areas.length ? "" : "no pick area yet: the Pick node finds the table live";
      this.$("tipMm").value = String(Number.isFinite(node.tipMm) ? node.tipMm : P.DEFAULT_TIP_MM);
      const model = node.robotModel || "";
      const mr = P.modelReach(model);
      const baseR = mr ? mr[0] : 0.064;
      // the cell from above: the base, how far this arm reaches, every taught area (the PolyScope 5 node's map)
      const drawn = areas.map((a, i) => ({ a, pl: P.plane(a.p0, a.p1, a.p2), i })).filter((x) => x.pl)
        .map((x) => ({ name: x.a.name || `Area ${x.i + 1}`, corners: P.areaCorners(x.pl) }));
      this.$("reach-map").innerHTML = P.svgReachMap(model, baseR, baseR + P.KEEP_OUT_M, mr ? mr[1] : 0, drawn, -1, 300, 300);
      this.$("reach-model").textContent = mr ? `${model}: ${(mr[1] * 1000).toFixed(0)} mm` : model ? `robot ${model}` : "robot model unknown";
      const past = mr ? drawn.find((a) => P.farthest(a.corners) > mr[1]) : null;
      this.$("reach-text").textContent = past
        ? `${past.name} goes ${Math.round((P.farthest(past.corners) - mr[1]) * 1000)} mm past the arm's reach: parts out there are seen but not picked`
        : "which parts can be picked is the arm's kinematics' answer, part by part — there are no reach margins to set";
    }

    areaAdd() {
      const P = this._lib;
      const areas = this._node.areas || (this._node.areas = []);
      if (areas.length >= P.MAX_AREAS) return;
      areas.push({ name: `Area ${areas.length + 1}`, p0: null, p1: null, p2: null });
      this.persist();
      this.syncAreas();
    }

    areaRemove(i) {
      this._node.areas.splice(i, 1);
      this.persist();
      this.syncAreas();
      this.setStatus("area removed — check the picture points of any Pick node that used it", "warn");
    }

    // A touch: PolyScope's joint positions → its DH table → the flange → the fingertips (tipMm along +Z).
    async touch(i, key) {
      const P = this._lib;
      const rps = this._api && this._api.robotPositionService;
      if (!rps || typeof rps.getJointPositions !== "function" || typeof rps.getKinematicInfo !== "function") {
        this.setStatus("PolyScope's position services are not on this API", "warn");
        return;
      }
      try {
        const q = await withTimeout(firstValue(rps.getJointPositions()), 5000, "no joint positions from PolyScope in 5 s");
        const dh = await withTimeout(rps.getKinematicInfo(), 5000, "no kinematic info from PolyScope in 5 s");
        const qa = Array.isArray(q) ? q.map(Number) : ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"].map((k) => Number(q[k]));
        const flange = matToPose(flangeMat(dh, qa));
        const tip = P.fingertip(flange, Number.isFinite(this._node.tipMm) ? this._node.tipMm : P.DEFAULT_TIP_MM);
        this._node.areas[i][key] = tip.map((v) => Math.round(v * 1e5) / 1e5);
        await this.persist();
        this.syncAreas();
        this.setStatus(`${this._node.areas[i].name}: ${key === "p0" ? "corner" : key === "p1" ? "edge" : "far side"} at [${fmtVec(tip)}] m (fingertips)`, "ok");
      } catch (err) {
        this.setStatus(`touch: ${err && err.message ? err.message : err}`, "err");
      }
    }

    setLive(live, fps) {
      const dot = this.$("dot");
      if (dot) dot.className = "dot " + (live ? "live" : "dead");
      const card = this.$("nocam");
      if (card) card.classList.toggle("hidden", !!live);
      const view = this.$("view");
      if (view) view.classList.toggle("hidden", !live);
      const f = this.$("fps");
      if (f) f.textContent = live && fps ? `${Number(fps).toFixed(1)} fps` : "";
    }

    async saveCockpitUrl() {
      const value = this.$("url").value.trim();
      this._node.cockpitUrl = value;
      this._seq = 0;
      // a request still waiting on the old address is dropped: the feed asks the new one now
      this._urlChanged = true;
      if (this._pollCtl) this._pollCtl.abort();
      this.$("open").href = this.cockpitUrl() + "/";
      try {
        if (this._api && this._api.applicationNodeService) {
          await this._api.applicationNodeService.updateNode(this._node);
        }
        this.setStatus(`cockpit: ${this.cockpitUrl()}`);
      } catch (err) {
        this.setStatus(`could not save the node: ${err && err.message ? err.message : err}`, "err");
      }
    }

    // -- the feed -----------------------------------------------------------------------
    async startPolling() {
      if (this._polling) return;
      this._polling = true;
      const img = this.$("img");
      let announced = false;
      while (!this._stopped && this.isConnected) {
        try {
          const depth = this._depth;
          this._urlChanged = false;
          const ctl = typeof AbortController === "function" ? new AbortController() : null;
          this._pollCtl = ctl;
          const timer = ctl ? setTimeout(() => ctl.abort(), FEED_TIMEOUT_MS) : 0;
          let r, blob;
          try {
            r = await fetch(`${this.cockpitUrl()}/api/${depth ? "depth" : "color"}.png?after=${this._seq}&timeout_ms=${POLL_TIMEOUT_MS}`,
              ctl ? { signal: ctl.signal } : undefined);
            if (r.ok) blob = await r.blob();
          } finally {
            clearTimeout(timer);
            this._pollCtl = null;
          }
          if (r.status === 503) {
            this.setLive(false);
            this.noCamera("nopicture", "HTTP 503: the cockpit has no frame", "warn");
            await sleep(500);
            continue;
          }
          if (r.status === 404 && depth) {
            // a camera computer older than 0.7.0 has no heatmap: the picture instead
            this._depth = false;
            this.$("view").querySelectorAll("button").forEach((t) => t.classList.toggle("on", t.dataset.view === "picture"));
            console.warn(`Perceptronic: ${this.cockpitUrl()} answered 404 on /api/depth.png: it predates the depth view; update it`);
            continue;
          }
          if (!r.ok) {
            const isPolyScope = new URL(this.cockpitUrl()).origin === location.origin;
            const e = new Error(isPolyScope
              ? `${this.cockpitUrl()} is PolyScope itself, not the cockpit — the cockpit listens on port ${DEFAULT_COCKPIT_PORT}.`
              : `the cockpit at ${this.cockpitUrl()} answered HTTP ${r.status} on /api/color.png` +
                (r.status === 404 ? " — it predates the URCap routes; update it and restart." : "."));
            e.http = r.status;
            throw e;
          }
          const seq = Number(r.headers.get("X-Seq") || 0);
          const url = URL.createObjectURL(blob);
          const previous = this._blobUrl;
          img.onload = () => {
            if (previous) URL.revokeObjectURL(previous);
            if (!depth) this._colourWidth = img.naturalWidth;
          };
          img.src = url;
          this._blobUrl = url;
          if (seq) this._seq = seq;
          this.setLive(true, r.headers.get("X-Fps"));
          if (!announced) { this.setStatus(`live from ${this.cockpitUrl()}`, "ok"); announced = true; }
        } catch (err) {
          announced = false;
          if (this._urlChanged) continue; // aborted by Save: straight on to the new address
          this.setLive(false);
          const why = err && err.message ? err.message : String(err);
          // Only a network-level failure is ambiguous; an HTTP status already says what happened.
          let detail = err && err.http ? why : null;
          if (!detail) { try { detail = await this.diagnose(); } catch (e) { detail = null; } }
          const up = detail && detail.includes("is running but");
          // the screen says what to check; the long story (the URL, the command) goes to the console
          const long = (detail || `no cockpit at ${this.cockpitUrl()} (${why}).`) +
            (up ? "" : `\nStart one where the camera is:  perceptronics --cell <cell> gui --cors ${location.origin}` +
              `\nthen set its URL above.`);
          const kind = err && err.http ? "outdated" : up ? "cors"
            : detail && detail.includes("is not a URL") ? "badurl" : detail && detail.includes("no answer in") ? "silent" : "refused";
          this.noCamera(kind, long, "err");
          await sleep(1500);
        }
      }
      this._polling = false;
    }

    /** No picture: what to check, where the picture would be and on the status line; `detail`
     * (the exception, the URL, the command) only in the console. */
    noCamera(kind, detail, level) {
      const P = this._lib || window.PerceptronicPick;
      const text = P ? P.advise(kind, this.cockpitUrl(), detail).text : detail;
      const t = this.$("nocam-text");
      if (t) t.textContent = text;
      this.setStatus(text, level);
      if (detail !== this._logged) {
        this._logged = detail;
        console.warn(`Perceptronic: ${detail}`);
      }
    }

    // -- pixels -----------------------------------------------------------------------
    pixelOf(ev) {
      const img = this.$("img");
      const rect = img.getBoundingClientRect();
      if (!img.naturalWidth || !rect.width) return null;
      // in the colour picture's pixels, whichever view is shown (the depth view is half its size)
      const cw = this._colourWidth || img.naturalWidth;
      const ch = Math.round(cw * (img.naturalHeight / img.naturalWidth));
      const x = Math.round((ev.clientX - rect.left) * (cw / rect.width));
      const y = Math.round((ev.clientY - rect.top) * (ch / rect.height));
      return {
        x: Math.max(0, Math.min(cw - 1, x)),
        y: Math.max(0, Math.min(ch - 1, y)),
        fx: (ev.clientX - rect.left) / rect.width,
        fy: (ev.clientY - rect.top) / rect.height,
      };
    }

    onHover(ev) {
      const px = this.pixelOf(ev);
      if (!px) return;
      this._hoverPending = px;
      if (this._hoverTimer) return;
      this._hoverTimer = setTimeout(async () => {
        this._hoverTimer = null;
        const p = this._hoverPending;
        try {
          const res = await this.api("GET", `/api/point?x=${p.x}&y=${p.y}`);
          const el = this.$("hover");
          if (!el) return;
          el.textContent = res.ok
            ? `(${p.x}, ${p.y}) → ${fmtVec(res.point_m)} m in the camera · ${fmt(res.point_m[2], 2)} m away`
            : `(${p.x}, ${p.y}) → no depth`;
        } catch (e) { /* the feed loop reports connectivity */ }
      }, HOVER_MS);
    }

    async onClick(ev) {
      const px = this.pixelOf(ev);
      if (!px) return;
      const mark = this.$("mark");
      mark.style.left = `${px.fx * 100}%`;
      mark.style.top = `${px.fy * 100}%`;
      mark.style.display = "block";
      this._located = null;
      this.$("move-ps").disabled = true;
      this.$("move-ck").disabled = true;
      this.setStatus(`segmenting at (${px.x}, ${px.y})…`);
      try {
        const seg = await this.api("POST", "/api/segment", { x: px.x, y: px.y });
        if (!seg.ok) throw new Error(seg.error || "segment failed");
        const point = seg.features && seg.features.point_m;
        if (!point) throw new Error("the segment has no depth point — click on the object, not the void");
        const loc = await this.api("POST", "/api/robot/locate", { point_m: point });
        if (!loc.ok) throw new Error(loc.error || "locate failed (is the cockpit's robot link up?)");
        this._located = loc;
        const reach = loc.reachable === false ? "OUT OF REACH" : loc.reachable === true ? "reachable" : "reach unknown";
        this.$("target").textContent =
          `object  base ${fmtVec(loc.point_base_m)} m  (${fmt(loc.point_distance_m, 2)} m from the base)\n` +
          (loc.reference === "fingertip"
            ? `fingertips ${fmtVec(loc.approach_pose)}  ${fmt(loc.standoff_m, 3)} m above the object (tool ${fmt(loc.tip_m, 3)} m)\n`
            : loc.reference === "flange" && loc.flange_target_pose
              ? `flange   ${fmtVec(loc.flange_target_pose)}  standoff ${fmt(loc.standoff_m, 2)} m above the object\n`
              : `approach ${fmtVec(loc.approach_pose)}  standoff ${fmt(loc.standoff_m, 2)} m\n`) +
          `${reach}` +
          (loc.reach_check === "controller_ik"
            ? "  (the controller's inverse kinematics)"
            : loc.max_reach_m ? `  (${fmt(loc.max_reach_m, 2)} m datasheet radius${loc.model ? `, ${loc.model}` : ""} — no IK answer)` : "");
        const offer = loc.reachable !== false;
        this.$("move-ps").disabled = !offer;
        this.$("move-ck").disabled = !offer;
        this.setStatus(offer ? "target located — choose how to move" : "target located but out of reach — pick a nearer point", offer ? "ok" : "warn");
      } catch (err) {
        this.setStatus(`${err && err.message ? err.message : err}`, "err");
      }
    }

    clearTarget() {
      this._located = null;
      this.$("mark").style.display = "none";
      this.$("target").textContent = "";
      this.$("move-ps").disabled = true;
      this.$("move-ck").disabled = true;
      this.api("POST", "/api/clear").catch(() => {});
      this.setStatus("cleared");
    }

    // -- moving ------------------------------------------------------------------------
    async movePolyScope() {
      const loc = this._located;
      if (!loc) return;
      const api = this._api;
      if (!api || !api.robotPositionService || !api.robotMoveService) {
        this.setStatus("PolyScope's move services are not on this API — use Move (cockpit)", "warn");
        return;
      }
      const rps = api.robotPositionService;
      try {
        this.setStatus("asking PolyScope for the joint solution…");
        const qNear = await firstValue(rps.getJointPositions());
        let pose = loc.approach_pose, tcpNote = "";
        if (Array.isArray(loc.flange_target_pose) && typeof rps.getKinematicInfo === "function"
            && typeof rps.convertJointPositionsToTcpPose === "function") {
          const dh = await rps.getKinematicInfo();
          const zero = { base: 0, shoulder: 0, elbow: 0, wrist1: 0, wrist2: 0, wrist3: 0 };
          const t0 = await rps.convertJointPositionsToTcpPose(zero);
          pose = Perceptronic.polyScopeTarget(loc.flange_target_pose, dh, [...t0.position, ...t0.orientation]);
        } else if (Array.isArray(loc.flange_target_pose)) {
          // convertJointPositionsToTcpPose is 10.10+; before that the flange target stands for the TCP
          pose = loc.flange_target_pose;
          tcpNote = " (this PolyScope can't report its TCP: the target assumes the TCP is at the flange)";
        }
        // PolyScope's IK does not reject an unsolvable pose, it never answers (10.13 sim).
        const joints = await withTimeout(
          rps.getInverseKinematics(
            { position: [pose[0], pose[1], pose[2]], orientation: [pose[3], pose[4], pose[5]] },
            qNear,
          ),
          8000,
          `PolyScope found no joint solution for [${fmtVec(pose)}] in 8 s — unreachable for its IK`,
        );
        // autoMove resolves as soon as PolyScope's screen opens (10.13 sim), not when the arm arrives
        await api.robotMoveService.autoMove(joints);
        this.setStatus(`PolyScope's move screen is open — hold Move To Position to go there${tcpNote}`, "ok");
      } catch (err) {
        this.setStatus(`PolyScope move: ${err && err.message ? err.message : err}`, "err");
      }
    }

    async moveCockpit() {
      const loc = this._located;
      if (!loc) return;
      this.$("move-ck").disabled = true;
      this.setStatus("moving through the cockpit (Primary, safety-enveloped)…");
      try {
        const res = await this.api("POST", "/api/robot/move", { pose: loc.approach_pose, velocity: 0.1 });
        if (res.ok) {
          this.setStatus(`landed at ${fmtVec(res.landed)}`, "ok");
        } else {
          const why = res.error || (res.safety && res.safety.violations && res.safety.violations.join("; ")) ||
            (res.protective_stop ? "protective stop — Bring up, then retry" : "the move did not confirm");
          this.setStatus(`move refused: ${why}`, "err");
        }
      } catch (err) {
        this.setStatus(`move: ${err && err.message ? err.message : err}`, "err");
      } finally {
        this.$("move-ck").disabled = !this._located;
      }
    }

    async bringUp() {
      this.setStatus("bringing the robot up…");
      try {
        const res = await this.api("POST", "/api/robot/bring_up");
        this.setStatus(res.ok ? `robot ${res.robot_mode || "up"} / ${res.safety_mode || ""}` : `bring-up failed: ${res.error || "?"}`, res.ok ? "ok" : "err");
      } catch (err) {
        this.setStatus(`bring-up: ${err && err.message ? err.message : err}`, "err");
      }
    }

    async stop() {
      try {
        const res = await this.api("POST", "/api/robot/stop");
        this.setStatus(res.ok ? "stopped" : `stop failed: ${res.error || "?"}`, res.ok ? "warn" : "err");
      } catch (err) {
        this.setStatus(`stop: ${err && err.message ? err.message : err}`, "err");
      }
    }
  }

  if (!window.customElements.get(TAG)) {
    window.customElements.define(TAG, Perceptronic);
  }
})();
