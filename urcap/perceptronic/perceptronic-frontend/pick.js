// Perceptronic Pick — Program Node presenters, plain custom elements like main.js.
//
// PolyScope X draws a program node's presenter *inside its tree row* (an
// `ur-inline-presenter` in a 48 px row, 10.13), so the Pick node's row is one line — the
// part, the picture points, the order, the verdict — with a button that opens the real
// screen as a PolyScope dialog (`presenterAPI.dialogService.openCustomDialog(tag,
// inputData, options)`: PolyScope creates the element, sets `inputData`, `presenterApi`
// (a WebComponentDialogAPI) and `afterOpen` on it, and closes it from its own footer).
// The dialog is the PolyScope 5 node's screen in the browser: the camera computer's live
// picture with the parts the program would find outlined and numbered in the pick order
// (`GET /api/pick/scene?opts=<the program's own request options>`), the picture points
// (taught from where the arm is — PolyScope's joint positions — Go = UR's auto-move
// screen), the pick-order tiles, Check approach (the approach over part #1 through
// PolyScope's inverse kinematics + auto-move), and Options (the part's size, approach,
// gripper, motion). It saves through the row's `programNodeService.updateNode` as it goes.
// The cockpit's address, the pick areas and the reach come from the Perceptronic
// application node. The settings, the request options and the script are pickscript.js —
// shared with the behavior worker, so what the screen shows is what the program sends.
(() => {
  const PICK_TAG = "nickarmenta-perceptronic-pick";
  const DIALOG_TAG = "nickarmenta-perceptronic-pick-dialog";
  const AFTER_TAG = "nickarmenta-perceptronic-after";
  const APP_TAG = "nickarmenta-perceptronic";
  const ARCHIVE_PATH = "/nickarmenta/perceptronic/perceptronic-frontend/";
  const POLL_TIMEOUT_MS = 1500;
  const SCENE_EVERY_MS = 700;
  const APP_EVERY_MS = 4000;
  const JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"];

  const selfUrl = typeof document !== "undefined" && document.currentScript && document.currentScript.src;

  // pickscript.js is the worker's too; the page loads it once, by <script>, before the elements render
  // (never at load — tests run this file under node).
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

  const CSS = `
    .pk { font: 14px/1.4 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color: #1f2a37; padding: 4px 8px; }
    .pk h2 { margin: 0 0 8px; font-size: 18px; display: flex; align-items: center; gap: 10px; }
    .pk .dot { width: 10px; height: 10px; border-radius: 50%; background: #c0c8d2; display: inline-block; }
    .pk .dot.live { background: #1d9a5a; } .pk .dot.dead { background: #d64545; }
    .pk .cols { display: flex; gap: 16px; align-items: flex-start; flex-wrap: wrap; }
    .pk .stage { position: relative; display: inline-block; background: #0f1620; border-radius: 8px; overflow: hidden; flex: 0 0 auto; }
    .pk .stage img { display: block; width: 640px; max-width: 100%; height: auto; }
    .pk .stage canvas { position: absolute; left: 0; top: 0; pointer-events: none; }
    .pk .stage .caption { position: absolute; left: 8px; padding: 3px 8px; background: rgba(15,22,32,.8); color: #e8eef6; border-radius: 4px; font-size: 12px; pointer-events: none; }
    .pk .stage .caption.top { top: 8px; } .pk .stage .caption.bottom { bottom: 8px; }
    .pk .side { flex: 1 1 320px; min-width: 300px; }
    .pk .card { border: 1px solid #d5dce5; border-radius: 8px; padding: 10px 12px; margin: 0 0 10px; }
    .pk .card h3 { margin: 0 0 6px; font-size: 14px; display: flex; align-items: center; justify-content: space-between; }
    .pk .card h3 small { color: #5b6b7d; font-weight: normal; }
    .pk .row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin: 6px 0; }
    .pk button { padding: 8px 12px; border: 1px solid #8e9bab; border-radius: 6px; background: #fff; font: inherit; cursor: pointer; }
    .pk button:disabled { opacity: .45; cursor: default; }
    .pk button.primary { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .pk button.on { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .pk button.small { padding: 4px 8px; font-size: 12.5px; }
    .pk .points { list-style: none; margin: 0; padding: 0; }
    .pk .points li { display: flex; align-items: center; gap: 6px; padding: 5px 6px; border-radius: 6px; cursor: pointer; }
    .pk .points li.sel { background: #eef2f7; }
    .pk .points li .n { width: 22px; height: 22px; border-radius: 50%; background: #1f5fbf; color: #fff; display: inline-flex; align-items: center; justify-content: center; font-size: 12px; flex: 0 0 auto; }
    .pk .points li .area { flex: 1; color: #5b6b7d; font-size: 12.5px; }
    .pk .tiles { display: grid; grid-template-columns: repeat(4, minmax(0, 84px)); gap: 6px; }
    .pk .tiles button { padding: 0; border: 0; background: none; line-height: 0; }
    .pk .tiles button svg { width: 100%; height: auto; }
    .pk .withart { display: grid; grid-template-columns: 1fr auto; gap: 8px 12px; align-items: center; }
    .pk .withart .art { grid-row: 1 / span 6; }
    .pk .withart .art svg { display: block; }
    .pk .status { margin-top: 8px; padding: 8px 10px; border-radius: 6px; background: #eef2f7; white-space: pre-wrap; }
    .pk .status.warn { background: #fff4d6; } .pk .status.err { background: #fde2e2; } .pk .status.ok { background: #e3f5ea; }
    .pk .num { display: grid; grid-template-columns: 1fr auto; gap: 4px 10px; align-items: center; padding: 4px 0; border-bottom: 1px solid #eef2f7; }
    .pk .num .lbl { font-size: 13px; } .pk .num .lbl small { display: block; color: #5b6b7d; font-size: 11.5px; }
    .pk .stepper { display: inline-flex; align-items: center; gap: 4px; }
    .pk .stepper input { width: 64px; padding: 5px 6px; border: 1px solid #b9c3cf; border-radius: 6px; font: inherit; text-align: right; }
    .pk .stepper button { padding: 4px 9px; }
    .pk .seg button { border-radius: 0; } .pk .seg button:first-child { border-radius: 6px 0 0 6px; } .pk .seg button:last-child { border-radius: 0 6px 6px 0; }
    .pk .switch { display: flex; align-items: center; gap: 8px; margin: 6px 0; }
    .pk small { color: #5b6b7d; }
    .pk .hidden { display: none !important; }
    /* the tree row: one line, 48 px */
    .pkrow { display: flex; align-items: center; gap: 10px; height: 40px; overflow: hidden; white-space: nowrap; font: 14px/1.2 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color: #1f2a37; padding: 0 4px; }
    .pkrow .txt { overflow: hidden; text-overflow: ellipsis; }
    .pkrow .verdict { overflow: hidden; text-overflow: ellipsis; color: #5b6b7d; font-size: 12.5px; flex: 1 1 auto; }
    .pkrow .verdict.warn { color: #9a6b00; } .pkrow .verdict.ok { color: #1d9a5a; }
    .pkrow button { padding: 6px 12px; border: 1px solid #1f5fbf; border-radius: 6px; background: #1f5fbf; color: #fff; font: inherit; cursor: pointer; flex: 0 0 auto; }
    .pkrow .stepper { display: inline-flex; align-items: center; gap: 4px; }
    .pkrow .stepper input { width: 48px; padding: 4px 6px; border: 1px solid #b9c3cf; border-radius: 6px; font: inherit; text-align: right; }
    .pkrow .stepper button { padding: 3px 9px; background: #fff; color: #1f2a37; border-color: #8e9bab; }
  `;

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const fmt = (v, d = 3) => (typeof v === "number" ? v.toFixed(d) : "?");
  const fmtVec = (v, d = 3) => (Array.isArray(v) ? v.map((x) => fmt(x, d)).join(", ") : "?");
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const withTimeout = (p, ms, why) => Promise.race([p, new Promise((_, reject) => setTimeout(() => reject(new Error(why)), ms))]);

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
          try { sub && sub.unsubscribe && sub.unsubscribe(); } catch (e) { /* closed */ }
          resolve(v);
        },
        error: reject,
        complete: () => { if (!done) reject(new Error("joint-position stream ended without a value")); },
      });
    });
  }
  const jointsToArray = (j) => (Array.isArray(j) ? j.map(Number) : JOINT_NAMES.map((k) => Number(j[k])));
  const arrayToJoints = (q) => Object.fromEntries(JOINT_NAMES.map((k, i) => [k, q[i]]));
  const pageHost = () => `${location.protocol}//${location.hostname}`;

  async function fetchAppNode(api) {
    if (!api || !api.applicationService) return null;
    try {
      return (await api.applicationService.getApplicationNode(APP_TAG)) || null;
    } catch (e) {
      return null;
    }
  }

  // -- the Pick node's row -------------------------------------------------------------------------

  class PerceptronicPickNode extends HTMLElement {
    constructor() {
      super();
      this._node = null;
      this._api = null;
      this._robotSettings = null;
      this._programTree = null;
      this._applicationContext = null;
      this._app = null;
      this._P = null;
      this._built = false;
      this._stopped = false;
      this._variablesEnsured = false;
      this._variablesPromise = null;
      this._declared = null;
      this._dialogOpen = false;
    }

    get contributedNode() { return this._node; }
    set contributedNode(value) { this._node = value; this.render(); }
    get presenterAPI() { return this._api; }
    set presenterAPI(value) { this._api = value; this.render(); }
    get robotSettings() { return this._robotSettings; }
    set robotSettings(value) { this._robotSettings = value; }
    get programTree() { return this._programTree; }
    set programTree(value) { this._programTree = value; }
    get applicationContext() { return this._applicationContext; }
    set applicationContext(value) { this._applicationContext = value; }

    connectedCallback() { this._stopped = false; this.render(); }
    disconnectedCallback() { this._stopped = true; }

    params() {
      if (!this._node.parameters) this._node.parameters = {};
      return this._node.parameters;
    }
    $(name) { return this.querySelector(`[data-pk="${name}"]`); }

    async save() {
      if (!this._api || !this._api.programNodeService) return;
      try {
        await this._api.programNodeService.updateNode(this._node);
      } catch (err) {
        this.setVerdict(`could not save the node: ${err && err.message ? err.message : err}`, "warn");
      }
    }

    render() {
      if (!this._node || !this.isConnected) return;
      if (!this._P) {
        loadLib().then((P) => { this._P = P; this.render(); }, (err) => { this.innerHTML = `<div class="pkrow"><span class="verdict warn">${esc(err.message)}</span></div>`; });
        return;
      }
      if (!this._built) {
        this.innerHTML = `
          <style>${CSS}</style>
          <div class="pkrow">
            <span class="txt" data-pk="txt"></span>
            <button data-pk="open" title="the picture, the picture points, the pick order, Check approach and Options">Teach &amp; options…</button>
            <span class="verdict" data-pk="verdict"></span>
          </div>`;
        this.$("open").addEventListener("click", (ev) => { ev.stopPropagation(); this.openDialog(); });
        this._built = true;
        this.startApp();
      }
      setTimeout(() => this.ensureVariables(), 1500); // after PolyScope's insertion has settled
      this.sync();
    }

    setVerdict(text, kind) {
      const el = this.$("verdict");
      if (!el) return;
      el.textContent = text;
      el.className = "verdict" + (kind ? ` ${kind}` : "");
    }

    sync() {
      if (!this._built) return;
      const P = this._P;
      const st = P.settings(this.params(), this._app, pageHost());
      const v = st.values;
      const l = Math.max(v.partLengthMm, v.partWidthMm), w = Math.min(v.partLengthMm, v.partWidthMm);
      const np = st.points.length;
      this.$("txt").textContent = `${P.num(l)}×${P.num(w)}×${P.num(v.partHeightMm)} mm · ${np} picture${np === 1 ? "" : "s"} · ${P.orderText(st.orderFirst, st.orderRows)}`;
      const why = P.problem(st);
      if (why) this.setVerdict(why, "warn");
      else this.setVerdict(`ready · camera computer ${st.host}:${st.port} · ${st.gripper === "children" ? "your gripper nodes" : st.gripper === "digital" ? `digital output ${Math.round(v.gripperDo)}` : "Robotiq Hand-E"}`, "ok");
    }

    async startApp() {
      while (!this._stopped && this.isConnected) {
        const app = await fetchAppNode(this._api);
        if (JSON.stringify(app) !== JSON.stringify(this._app)) { this._app = app; this.sync(); }
        if (!this._variablesEnsured) this.ensureVariables();
        await sleep(APP_EVERY_MS);
      }
    }

    // The result variables, declared once through PolyScope so the operator's own nodes can read
    // them. One declaration in flight at a time, awaited before the dialog opens, and verified
    // after the save: while an insertion settles PolyScope re-sets contributedNode from its own
    // copy and a declaration saved in between is lost (10.10, whose older service answers
    // slower) — the next call re-attaches the variables already generated instead of new ones.
    ensureVariables() {
      if (this._variablesEnsured) return Promise.resolve();
      if (!this._variablesPromise) {
        this._variablesPromise = this.declareVariables().finally(() => { this._variablesPromise = null; });
      }
      return this._variablesPromise;
    }

    async declareVariables() {
      if (this._variablesEnsured || !this._api) return;
      const p = this.params();
      if (p.foundVariable && p.locVariable) { this._variablesEnsured = true; return; }
      // variableService.createVariable from 10.12; before that symbolService.generateVariable (a
      // URVariable, same `name`) — 10.10 / 10.11 have only the latter (the release matrix, 2026-09-30)
      const vs = this._api.variableService, ss = this._api.symbolService;
      const declare = vs && typeof vs.createVariable === "function"
        ? (name, type) => vs.createVariable(name, type)
        : ss && typeof ss.generateVariable === "function" ? (name, type) => ss.generateVariable(name, type) : null;
      if (!declare) return;
      try {
        const d = this._declared || (this._declared = {});
        if (!d.found) d.found = p.foundVariable || (await declare(this._P.FOUND_VARIABLE, "boolean"));
        if (!d.loc) d.loc = p.locVariable || (await declare(this._P.LOC_VARIABLE, "integer"));
        const now = this.params(); // the node the row holds *now*, not the one it held before the await
        now.foundVariable = d.found;
        now.locVariable = d.loc;
        await this.save();
        await sleep(400);
        const check = this.params();
        if (check.foundVariable && check.locVariable) {
          this._variablesEnsured = true;
          this.sync();
        }
      } catch (err) {
        this.setVerdict(`could not declare the result variables: ${err && err.message ? err.message : err}`, "warn");
      }
    }

    async openDialog() {
      const api = this._api;
      if (!api || !api.dialogService || typeof api.dialogService.openCustomDialog !== "function") {
        this.setVerdict("PolyScope's dialog service is not on this API", "warn");
        return;
      }
      if (this._dialogOpen) return;
      this._dialogOpen = true;
      try {
        await this.ensureVariables();
        // the dialog edits this very node and saves through this API as it goes
        await api.dialogService.openCustomDialog(DIALOG_TAG, { node: this._node, api, app: this._app, row: this }, {
          title: "Perceptronic Pick",
          dialogSize: "XL",
          confirmText: "Done",
          raiseForKeyboard: false,
        });
      } catch (err) {
        this.setVerdict(`dialog: ${err && err.message ? err.message : err}`, "warn");
      } finally {
        this._dialogOpen = false;
        this._app = await fetchAppNode(api);
        this.sync();
      }
    }
  }

  // -- the Pick node's screen (a PolyScope dialog) ---------------------------------------------------

  class PerceptronicPickDialog extends HTMLElement {
    // A flange target re-expressed in PolyScope's active TCP (the same trick as main.js's
    // Move (PolyScope)): offset = FK_flange(0)⁻¹ · TCP(0), both from PolyScope at zero joints.
    static polyScopeTarget(P, flangeTarget, dh, tcpAtZeroPose) {
      const offset = P.matMul(P.matInv(P.flangeMat(dh, [0, 0, 0, 0, 0, 0])), P.poseToMat(tcpAtZeroPose));
      return P.matToPose(P.matMul(P.poseToMat(flangeTarget), offset));
    }

    constructor() {
      super();
      this._input = null;
      this._node = null;
      this._api = null; // the row's ProgramPresenterAPI (saves the node, reads the application node)
      this._dialogApi = null; // PolyScope's WebComponentDialogAPI for this dialog
      this._app = null;
      this._P = null;
      this._built = false;
      this._stopped = false;
      this._seq = 0;
      this._blobUrl = null;
      this._scene = null;
      this._sceneBusy = false;
      this._options = false;
    }

    // -- what PolyScope sets on a custom dialog's element ------------------------------------
    get inputData() { return this._input; }
    set inputData(value) {
      this._input = value || {};
      this._node = this._input.node || null;
      this._api = this._input.api || null;
      this._app = this._input.app || null;
      this._row = this._input.row || null; // the tree row that opened this dialog
      this.render();
    }
    get presenterApi() { return this._dialogApi; }
    set presenterApi(value) { this._dialogApi = value; }
    get afterOpen() { return this._afterOpen; }
    set afterOpen(value) { this._afterOpen = value; }

    connectedCallback() { this._stopped = false; this.render(); }
    disconnectedCallback() {
      this._stopped = true;
      if (this._blobUrl) { URL.revokeObjectURL(this._blobUrl); this._blobUrl = null; }
    }

    // PolyScope's services: the row's API first, the dialog's own as the fallback
    service(name) {
      return (this._api && this._api[name]) || (this._dialogApi && this._dialogApi[name]) || null;
    }
    params() {
      if (!this._node.parameters) this._node.parameters = {};
      return this._node.parameters;
    }
    settings() { return this._P.settings(this.params(), this._app, pageHost()); }
    cockpitUrl() { return this._P.cockpitBase(this._app && this._app.cockpitUrl, pageHost()); }

    async api(method, path, body, timeoutMs) {
      const init = { method, headers: {} };
      if (body !== undefined) {
        init.headers["Content-Type"] = "application/json";
        init.body = JSON.stringify(body);
      }
      const ctl = typeof AbortController === "function" ? new AbortController() : null;
      const timer = ctl && timeoutMs ? setTimeout(() => ctl.abort(), timeoutMs) : null;
      if (ctl) init.signal = ctl.signal;
      try {
        const r = await fetch(this.cockpitUrl() + path, init);
        let out;
        try { out = await r.json(); } catch (e) { out = { ok: false, error: `HTTP ${r.status}` }; }
        if (out && out.ok === undefined) out.ok = r.ok;
        return out;
      } finally {
        if (timer) clearTimeout(timer);
      }
    }

    async save() {
      const pns = this.service("programNodeService");
      if (!pns) return;
      // the row may have declared the result variables on its own copy of the node since this
      // dialog was handed one: never save without them
      const mine = this.params(), rows = this._row && this._row._node && this._row._node.parameters;
      if (rows) {
        if (!mine.foundVariable && rows.foundVariable) mine.foundVariable = rows.foundVariable;
        if (!mine.locVariable && rows.locVariable) mine.locVariable = rows.locVariable;
      }
      try {
        await pns.updateNode(this._node);
        this.dispatchEvent(new CustomEvent("outputDataChange", { detail: this._node, bubbles: true }));
      } catch (err) {
        this.setStatus(`could not save the node: ${err && err.message ? err.message : err}`, "err");
      }
    }

    // -- rendering -------------------------------------------------------------------------------
    $(name) { return this.querySelector(`[data-pk="${name}"]`); }

    render() {
      if (!this._node || !this.isConnected) return;
      if (!this._P) {
        loadLib().then((P) => { this._P = P; this.render(); }, (err) => { this.innerHTML = `<div class="pk"><div class="status err">${esc(err.message)}</div></div>`; });
        return;
      }
      if (!this._built) {
        this.build();
        this._built = true;
        this.startFeed();
        this.startScene();
        this.startApp();
      }
      this.sync();
    }

    build() {
      const P = this._P;
      this.innerHTML = `
        <style>${CSS}</style>
        <div class="pk">
          <h2><span class="dot" data-pk="dot"></span> <span data-pk="title">Picture</span> <small data-pk="fps"></small>
            <span style="flex:1"></span>
            <button class="small" data-pk="toggle">Options</button>
          </h2>
          <div data-pk="main">
            <div class="cols">
              <div class="stage" data-pk="stage">
                <img data-pk="img" alt="wrist camera" draggable="false" />
                <canvas data-pk="overlay"></canvas>
                <div class="caption top" data-pk="cap-top">waiting for the camera computer…</div>
                <div class="caption bottom" data-pk="cap-bottom"></div>
              </div>
              <div class="side">
                <div class="card">
                  <h3>Picture points <small data-pk="points-count"></small></h3>
                  <ul class="points" data-pk="points"></ul>
                  <div class="row">
                    <button class="primary" data-pk="add">+ Add picture point here</button>
                  </div>
                  <small>Move the arm where the camera sees the parts, then Add. Tap a point's area to choose the pick area it looks at.</small>
                </div>
                <div class="card">
                  <h3>Pick order <small data-pk="order-text"></small></h3>
                  <div class="tiles" data-pk="tiles"></div>
                </div>
                <div class="row">
                  <button data-pk="check" title="PolyScope's auto-move screen over part #1: hold Move to see the fingers arrive open, Approach mm over it">Check approach</button>
                </div>
              </div>
            </div>
          </div>
          <div data-pk="options" class="hidden">
            <div class="cols">
              <div class="side">
                <div class="card"><h3>Part <small>as it lies</small></h3>
                  <div class="withart"><div data-pk="card-part"></div><div class="art" data-pk="part-art"></div></div></div>
                <div class="card"><h3>Approach</h3>
                  <div class="withart"><div data-pk="card-approach"></div><div class="art" data-pk="approach-art"></div></div></div>
              </div>
              <div class="side">
                <div class="card" data-pk="card-gripper">
                  <h3>Gripper</h3>
                  <div class="row seg" data-pk="grippers">
                    <button data-gripper="robotiq">Robotiq Hand-E</button>
                    <button data-gripper="digital">Digital output</button>
                    <button data-gripper="children">My own nodes</button>
                  </div>
                  <div data-pk="gripper-fields"></div>
                  <small data-pk="gripper-note"></small>
                </div>
                <div class="card" data-pk="card-motion">
                  <h3>Motion</h3>
                  <div data-pk="motion-fields"></div>
                  <label class="switch"><input type="checkbox" data-pk="popup" /> Popup when nothing is picked</label>
                  <small>The routine after the pick goes inside this node; an <b>After picture N</b> node (toolbox) runs its part only for a pick from that picture point. <code>${P.FOUND_VARIABLE}</code> and <code>${P.LOC_VARIABLE}</code> are there for your own logic.</small>
                  <div class="row"><button class="small" data-pk="reset">Reset to defaults</button></div>
                </div>
              </div>
            </div>
          </div>
          <div class="status" data-pk="status"></div>
        </div>`;
      const cards = { part: this.$("card-part"), approach: this.$("card-approach"), motion: this.$("motion-fields") };
      P.NUMBERS.filter((n) => n.section !== "gripper").forEach((n) => cards[n.section].appendChild(this.stepper(n)));
      const gf = this.$("gripper-fields");
      P.NUMBERS.filter((n) => n.section === "gripper").forEach((n) => gf.appendChild(this.stepper(n)));
      const tiles = this.$("tiles");
      P.ORDER_TILES.forEach(([first, rows]) => {
        const b = document.createElement("button");
        b.dataset.first = first;
        b.dataset.rows = rows;
        b.title = P.orderText(first, rows);
        b.setAttribute("aria-label", P.orderText(first, rows));
        b.addEventListener("click", () => { this.params().orderFirst = first; this.params().orderRows = rows; this.save(); this.sync(); });
        tiles.appendChild(b);
      });
      this.$("toggle").addEventListener("click", () => { this._options = !this._options; this.sync(); });
      this.$("add").addEventListener("click", () => this.addPoint());
      this.$("check").addEventListener("click", () => this.checkApproach());
      this.$("reset").addEventListener("click", () => {
        this.params().values = P.defaults();
        this.params().popupOnFail = true;
        this.save();
        this.sync();
      });
      this.$("popup").addEventListener("change", (ev) => { this.params().popupOnFail = !!ev.target.checked; this.save(); this.sync(); });
      this.$("grippers").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
        this.params().gripper = b.dataset.gripper;
        this.save();
        this.sync();
      }));
    }

    stepper(n) {
      const row = document.createElement("div");
      row.className = "num";
      row.dataset.key = n.key;
      row.innerHTML = `
        <div class="lbl">${esc(n.label)}${n.unit ? ` <small style="display:inline">(${esc(n.unit)})</small>` : ""}<small>${esc(n.help)}</small></div>
        <div class="stepper"><button data-step="-1">−</button><input type="text" inputmode="decimal" /><button data-step="1">+</button></div>`;
      const input = row.querySelector("input");
      const set = (v) => {
        const values = this.params().values || (this.params().values = this._P.defaults());
        values[n.key] = this._P.clamp(n.key, v);
        this.save();
        this.sync();
      };
      row.querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
        const cur = (this.params().values || {})[n.key];
        set((Number.isFinite(cur) ? cur : n.def) + Number(b.dataset.step) * n.step);
      }));
      input.addEventListener("change", () => set(parseFloat(input.value.replace(",", "."))));
      return row;
    }

    setStatus(text, kind) {
      const el = this.$("status");
      if (!el) return;
      el.textContent = text;
      el.className = "status" + (kind ? ` ${kind}` : "");
    }

    setLive(live, fps) {
      const dot = this.$("dot");
      if (dot) dot.className = "dot " + (live ? "live" : "dead");
      const f = this.$("fps");
      if (f) f.textContent = live && fps ? `${Number(fps).toFixed(1)} fps` : "";
    }

    /** Everything that reflects the node's state. Cheap; called after every change. */
    sync() {
      if (!this._built) return;
      const P = this._P;
      const p = this.params();
      const st = this.settings();
      this.$("main").classList.toggle("hidden", this._options);
      this.$("options").classList.toggle("hidden", !this._options);
      this.$("toggle").textContent = this._options ? "Picture" : "Options";
      this.$("title").textContent = this._options ? "Options" : "Picture";
      // picture points
      const sel = Math.max(0, Math.min((p.points || []).length - 1, p.selectedPoint || 0));
      const areas = P.areasOf(this._app);
      const ul = this.$("points");
      ul.innerHTML = "";
      (p.points || []).forEach((pt, i) => {
        const li = document.createElement("li");
        li.className = i === sel ? "sel" : "";
        const area = pt.area >= 0 && pt.area < areas.length ? areas[pt.area] : null;
        const areaText = area ? (area.plane ? area.name : `${area.name} (not taught)`) : "live table";
        li.innerHTML = `<span class="n">${i + 1}</span><span class="area" title="tap to choose the pick area">${esc(areaText)}</span>
          <button class="small" data-act="go" title="PolyScope's auto-move screen to this picture point">Go</button>
          <button class="small" data-act="here" title="retake this point from where the arm is">Here</button>
          <button class="small" data-act="del" title="remove">✕</button>`;
        li.addEventListener("click", () => { p.selectedPoint = i; this.save(); this.sync(); });
        li.querySelector(".area").addEventListener("click", (ev) => { ev.stopPropagation(); this.cycleArea(i); });
        li.querySelector('[data-act="go"]').addEventListener("click", (ev) => { ev.stopPropagation(); this.goPoint(i); });
        li.querySelector('[data-act="here"]').addEventListener("click", (ev) => { ev.stopPropagation(); this.addPoint(i); });
        li.querySelector('[data-act="del"]').addEventListener("click", (ev) => { ev.stopPropagation(); this.removePoint(i); });
        ul.appendChild(li);
      });
      this.$("points-count").textContent = `${(p.points || []).length} of ${P.MAX_POINTS}`;
      this.$("add").disabled = (p.points || []).length >= P.MAX_POINTS;
      // order
      this.$("tiles").querySelectorAll("button").forEach((b) => {
        const on = b.dataset.first === st.orderFirst && b.dataset.rows === st.orderRows;
        if (b.dataset.on !== String(on)) { b.dataset.on = String(on); b.innerHTML = P.svgOrderTile(b.dataset.first, b.dataset.rows, on, 62, 50); }
      });
      // the drawings that explain the numbers as they change
      const v = st.values;
      this.$("part-art").innerHTML = P.svgPart(v.partLengthMm, v.partWidthMm, v.partHeightMm, 140, 140);
      this.$("approach-art").innerHTML = P.svgApproach(v.approachMm, v.gripBelowTopMm, v.liftMm, v.partHeightMm, Math.min(v.partLengthMm, v.partWidthMm), v.strokeMm, 150, 170);
      this.$("order-text").textContent = P.orderText(st.orderFirst, st.orderRows);
      // options
      this.querySelectorAll(".num").forEach((row) => {
        const n = P.BY_KEY[row.dataset.key];
        const v = st.values[n.key];
        row.querySelector("input").value = n.step >= 1 ? P.num(v) : String(Math.round(v * 100) / 100);
      });
      const fields = P.GRIPPER_FIELDS[st.gripper] || [];
      this.$("gripper-fields").querySelectorAll(".num").forEach((row) => row.classList.toggle("hidden", !fields.includes(row.dataset.key)));
      this.$("grippers").querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.gripper === st.gripper));
      this.$("gripper-note").textContent = st.gripper === "children"
        ? "put your gripper's Close nodes inside this node: they run at the grip, with your TCP"
        : st.gripper === "digital" ? "the output goes True to close, False to open" : "the Hand-E through its URCap's socket on the controller";
      this.$("popup").checked = st.popupOnFail;
      // the verdict
      const why = P.problem(st);
      this.$("check").disabled = !!why;
      if (why) this.setStatus(why, this._app ? "warn" : "");
      else this.setStatus(`ready: ${P.partText(st)} · ${P.orderText(st.orderFirst, st.orderRows)} · camera computer ${st.host}:${st.port}`, "ok");
      this.drawScene();
    }

    // -- the application node (cockpit, areas, reach) --------------------------------------------
    async startApp() {
      while (!this._stopped && this.isConnected) {
        const app = await fetchAppNode(this._api || this._dialogApi);
        if (JSON.stringify(app) !== JSON.stringify(this._app)) { this._app = app; this.sync(); }
        await sleep(APP_EVERY_MS);
      }
    }

    // -- picture points ----------------------------------------------------------------------
    async currentJoints() {
      const rps = this.service("robotPositionService");
      if (!rps || typeof rps.getJointPositions !== "function") throw new Error("PolyScope's joint positions are not on this API");
      return jointsToArray(await withTimeout(firstValue(rps.getJointPositions()), 5000, "no joint positions from PolyScope in 5 s"));
    }
    async addPoint(index) {
      const p = this.params();
      const points = p.points || (p.points = []);
      try {
        const q = await this.currentJoints();
        if (index === undefined) {
          if (points.length >= this._P.MAX_POINTS) return;
          const areas = this._P.areasOf(this._app);
          const prev = points.length ? points[points.length - 1].area : areas.findIndex((a) => a.plane);
          points.push({ q, area: Number.isInteger(prev) ? prev : -1 });
          p.selectedPoint = points.length - 1;
        } else {
          points[index].q = q;
          p.selectedPoint = index;
        }
        await this.save();
        this.sync();
        this.setStatus(`picture point ${(index === undefined ? points.length - 1 : index) + 1} taught at joints [${fmtVec(q, 3)}]`, "ok");
      } catch (err) {
        this.setStatus(`picture point: ${err && err.message ? err.message : err}`, "err");
      }
    }
    removePoint(i) {
      const p = this.params();
      p.points.splice(i, 1);
      p.selectedPoint = Math.max(0, Math.min(i, p.points.length - 1));
      this.save();
      this.sync();
    }
    cycleArea(i) {
      const p = this.params();
      const areas = this._P.areasOf(this._app);
      const taught = areas.map((a, k) => (a.plane ? k : -1)).filter((k) => k >= 0);
      const cur = p.points[i].area;
      const ring = [-1, ...taught];
      const at = ring.indexOf(cur);
      p.points[i].area = ring[(at + 1) % ring.length];
      this.save();
      this.sync();
    }
    async goPoint(i) {
      const q = this.params().points[i].q;
      const rms = this.service("robotMoveService");
      if (!rms || typeof rms.autoMove !== "function") { this.setStatus("PolyScope's move service is not on this API", "warn"); return; }
      try {
        this.params().selectedPoint = i;
        this.sync();
        await rms.autoMove(arrayToJoints(q));
        this.setStatus(`PolyScope's move screen is open — hold Move To Position to go to picture point ${i + 1}`, "ok");
      } catch (err) {
        this.setStatus(`Go: ${err && err.message ? err.message : err}`, "err");
      }
    }

    // -- the feed and the scene ------------------------------------------------------------------
    async startFeed() {
      const img = this.$("img");
      while (!this._stopped && this.isConnected) {
        if (!this._app) { await sleep(500); continue; }
        try {
          const r = await fetch(`${this.cockpitUrl()}/api/color.png?after=${this._seq}&timeout_ms=${POLL_TIMEOUT_MS}`);
          if (r.status === 503) { this.setLive(false); await sleep(500); continue; }
          if (!r.ok) throw new Error(`HTTP ${r.status} on /api/color.png`);
          const seq = Number(r.headers.get("X-Seq") || 0);
          const blob = await r.blob();
          const url = URL.createObjectURL(blob);
          const previous = this._blobUrl;
          img.onload = () => { if (previous) URL.revokeObjectURL(previous); this.drawScene(); };
          img.src = url;
          this._blobUrl = url;
          if (seq) this._seq = seq;
          this.setLive(true, r.headers.get("X-Fps"));
        } catch (err) {
          this.setLive(false);
          this.$("cap-top").textContent = `no camera computer at ${this.cockpitUrl()} (${err && err.message ? err.message : err}) — set it in Application → Perceptronic`;
          await sleep(1500);
        }
      }
    }

    async startScene() {
      while (!this._stopped && this.isConnected) {
        await this.refreshScene();
        await sleep(SCENE_EVERY_MS);
      }
    }
    async refreshScene() {
      if (this._sceneBusy || !this._app || this._options) return;
      this._sceneBusy = true;
      try {
        const p = this.params();
        const st = this.settings();
        const sel = (p.points || []).length ? Math.max(0, Math.min(p.points.length - 1, p.selectedPoint || 0)) : -1;
        const res = await this.api("GET", `/api/pick/scene?opts=${encodeURIComponent(this._P.tokens(st, sel))}`, undefined, 3000);
        if (res && Number.isInteger(res.pick_port) && res.pick_port > 0 && res.pick_port !== p.pickPort) {
          p.pickPort = res.pick_port;
          await this.save();
        }
        this._scene = res;
      } catch (e) {
        this._scene = null;
      } finally {
        this._sceneBusy = false;
      }
      this.drawScene();
    }

    drawScene() {
      const img = this.$("img"), cv = this.$("overlay");
      if (!img || !cv) return;
      const w = img.clientWidth, h = img.clientHeight;
      if (!w || !h) return;
      cv.width = w; cv.height = h;
      cv.style.width = `${w}px`; cv.style.height = `${h}px`;
      const ctx = cv.getContext("2d");
      ctx.clearRect(0, 0, w, h);
      const sc = this._scene;
      const top = this.$("cap-top"), bottom = this.$("cap-bottom");
      if (!sc) { bottom.textContent = ""; return; }
      if (!sc.ok) { top.textContent = sc.error || sc.reason || "no scene"; bottom.textContent = ""; return; }
      const sx = w / (sc.width || img.naturalWidth || w), sy = h / (sc.height || img.naturalHeight || h);
      const poly = (corners, stroke, fill) => {
        if (!Array.isArray(corners) || corners.length !== 4) return;
        ctx.beginPath();
        corners.forEach(([u, v], i) => (i ? ctx.lineTo(u * sx, v * sy) : ctx.moveTo(u * sx, v * sy)));
        ctx.closePath();
        ctx.fillStyle = fill; ctx.fill();
        ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.stroke();
      };
      const badge = (px, text, bg) => {
        if (!Array.isArray(px)) return;
        const x = px[0] * sx, y = px[1] * sy;
        ctx.beginPath(); ctx.arc(x, y, 11, 0, Math.PI * 2); ctx.fillStyle = bg; ctx.fill();
        ctx.fillStyle = "#fff"; ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(text, x, y);
      };
      (sc.rejected || []).forEach((r) => {
        poly(r.corners_px, "rgba(160,170,180,.9)", "rgba(160,170,180,.18)");
        if (Array.isArray(r.pixel) && r.why) {
          ctx.fillStyle = "rgba(15,22,32,.75)"; ctx.font = "11px system-ui, sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "top";
          ctx.fillText(r.why, r.pixel[0] * sx + 8, r.pixel[1] * sy + 8);
        }
      });
      (sc.parts || []).forEach((p) => {
        poly(p.corners_px, "#ffd23f", "rgba(255,210,63,.15)");
        badge(p.pixel, String(p.order || "·"), p.order === 1 ? "#1d9a5a" : "#1f5fbf");
      });
      const n = (sc.parts || []).length;
      top.textContent = n ? `${n} part${n === 1 ? "" : "s"} · #1 is picked first` : (sc.rejected || []).length ? sc.reason || "nothing to pick" : "no part in view";
      const surf = sc.surface || {};
      bottom.textContent = (surf.source === "taught" ? `pick area taught · table ${surf.offset_mm >= 0 ? "+" : ""}${Math.round(surf.offset_mm || 0)} mm` : "table found live")
        + (sc.base_frame === false ? " · no robot pose: reach not checked" : "");
    }

    // -- Check approach ------------------------------------------------------------------------------
    async checkApproach() {
      const P = this._P;
      const rps = this.service("robotPositionService"), rms = this.service("robotMoveService");
      const st = this.settings();
      const p = this.params();
      if (!rps || !rms) { this.setStatus("PolyScope's move services are not on this API", "warn"); return; }
      const sel = (p.points || []).length ? Math.max(0, Math.min(p.points.length - 1, p.selectedPoint || 0)) : -1;
      const approach = st.values.approachMm;
      this.setStatus("asking the camera computer for part #1…");
      try {
        const res = await this.api("GET", `/api/pick/scene?opts=${encodeURIComponent(P.tokens(st, sel))}&approach_mm=${P.num(approach)}`, undefined, 15000);
        const part = res && res.ok && Array.isArray(res.parts) && res.parts.length ? res.parts[0] : null;
        if (!part) throw new Error(`no part to check: ${(res && (res.reason || res.error)) || "?"}`);
        if (!Array.isArray(part.grasp_pose) || part.grasp_pose.length !== 6) throw new Error("the camera computer has no robot pose (is its robot link up?)");
        const hover = P.poseTrans(part.grasp_pose, [0, 0, -approach / 1000, 0, 0, 0]);
        const qNear = await firstValue(rps.getJointPositions());
        // PolyScope's IK solves for its active TCP: re-express the flange target in it — where
        // PolyScope can say what its TCP is (convertJointPositionsToTcpPose, 10.10+); before
        // that the flange stands for the TCP
        let pose = hover, tcpNote = "";
        if (typeof rps.convertJointPositionsToTcpPose === "function" && typeof rps.getKinematicInfo === "function") {
          const dh = await rps.getKinematicInfo();
          const t0 = await rps.convertJointPositionsToTcpPose(arrayToJoints([0, 0, 0, 0, 0, 0]));
          pose = PerceptronicPickDialog.polyScopeTarget(P, hover, dh, [...t0.position, ...t0.orientation]);
        } else {
          tcpNote = " (this PolyScope can't report its TCP: the target assumes the TCP is at the flange)";
        }
        const joints = await withTimeout(
          rps.getInverseKinematics({ position: [pose[0], pose[1], pose[2]], orientation: [pose[3], pose[4], pose[5]] }, qNear),
          8000,
          `PolyScope found no joint solution for the approach [${fmtVec(hover)}] in 8 s`,
        );
        await rms.autoMove(joints);
        this.setStatus(`PolyScope's move screen is open — hold Move To Position: fingertips ${P.num(approach)} mm over part #1's top, fingers open${tcpNote}`, "ok");
      } catch (err) {
        this.setStatus(`Check approach: ${err && err.message ? err.message : err}`, "err");
      }
    }
  }

  // -- the "After picture N" node's row ------------------------------------------------------------

  class PerceptronicAfterNode extends HTMLElement {
    constructor() {
      super();
      this._node = null;
      this._api = null;
      this._built = false;
    }
    get contributedNode() { return this._node; }
    set contributedNode(value) { this._node = value; this.render(); }
    get presenterAPI() { return this._api; }
    set presenterAPI(value) { this._api = value; }
    get robotSettings() { return this._robotSettings; }
    set robotSettings(value) { this._robotSettings = value; }
    get programTree() { return this._programTree; }
    set programTree(value) { this._programTree = value; }
    get applicationContext() { return this._applicationContext; }
    set applicationContext(value) { this._applicationContext = value; }
    connectedCallback() { this.render(); }

    render() {
      if (!this._node || !this.isConnected) return;
      if (!this._built) {
        this.innerHTML = `
          <style>${CSS}</style>
          <div class="pkrow">
            <span class="txt">the routine for a pick from picture point</span>
            <span class="stepper"><button data-step="-1">−</button><input type="text" inputmode="numeric" data-pk="point" /><button data-step="1">+</button></span>
            <span class="verdict">of the enclosing Perceptronic Pick node</span>
          </div>`;
        const set = (v) => {
          const n = Math.max(1, Math.min(12, Math.round(Number(v) || 1)));
          this._node.parameters = { ...(this._node.parameters || {}), point: n };
          if (this._api && this._api.programNodeService) this._api.programNodeService.updateNode(this._node).catch(() => {});
          this.render();
        };
        this.querySelectorAll("button").forEach((b) => b.addEventListener("click", (ev) => { ev.stopPropagation(); set(((this._node.parameters || {}).point || 1) + Number(b.dataset.step)); }));
        const input = this.querySelector('[data-pk="point"]');
        input.addEventListener("click", (ev) => ev.stopPropagation());
        input.addEventListener("change", (ev) => set(ev.target.value));
        this._built = true;
      }
      this.querySelector('[data-pk="point"]').value = String((this._node.parameters || {}).point || 1);
    }
  }

  if (!window.customElements.get(PICK_TAG)) window.customElements.define(PICK_TAG, PerceptronicPickNode);
  if (!window.customElements.get(DIALOG_TAG)) window.customElements.define(DIALOG_TAG, PerceptronicPickDialog);
  if (!window.customElements.get(AFTER_TAG)) window.customElements.define(AFTER_TAG, PerceptronicAfterNode);
})();
