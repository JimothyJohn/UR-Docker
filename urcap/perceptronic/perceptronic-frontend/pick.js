// 3D Pick — the Program Node's presenter, plain custom elements like main.js.
//
// PolyScope X draws a program node's presenter *inside its tree row* (an
// `ur-inline-presenter` in a 48 px row, 10.13), so the node's row is one line — the part,
// the picture points, the order, the verdict — with a button that opens the real screen as
// a PolyScope dialog (`presenterAPI.dialogService.openCustomDialog(tag, inputData,
// options)`: PolyScope creates the element, sets `inputData`, `presenterApi` (a
// WebComponentDialogAPI) and `afterOpen` on it, and closes it from its own footer).
// The dialog is the PolyScope 5 node's screen in the browser (0.5.0 = its 0.7.0), and like
// it nothing in it scrolls: the camera computer's live picture — or, with the toggle in its
// top right corner, the depth as a heatmap — carrying only the candidates that are nearly
// the part and are not going to be picked, each outlined with why
// (`GET /api/pick/scene?opts=<the program's own request options>`); the picture points as a
// fixed grid of numbered buttons (taught from where the arm is — PolyScope's joint
// positions — Go = UR's auto-move screen); the pick-order tiles; Check approach (the
// approach over the first part through PolyScope's inverse kinematics + auto-move); and
// Options: two tabs, Part and Approach. It saves through the row's
// `programNodeService.updateNode` as it goes. The cockpit's address and the pick areas come
// from the Perceptronic application node. The settings, the request options and the script
// are pickscript.js — shared with the behavior worker, so what the screen shows is what the
// program sends.
(() => {
  const PICK_TAG = "advin-perceptronic-pick";
  const DIALOG_TAG = "advin-perceptronic-pick-dialog";
  const APP_TAG = "advin-perceptronic";
  const ARCHIVE_PATH = "/advin/perceptronic/perceptronic-frontend/";
  const POLL_TIMEOUT_MS = 1500;
  const FEED_TIMEOUT_MS = POLL_TIMEOUT_MS + 3500; // main.js: a silent address fails in seconds
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
    .pk { font: 14px/1.4 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color: #1f2a37; padding: 4px 8px; overflow: hidden; }
    .pk h2 { margin: 0 0 8px; font-size: 18px; display: flex; align-items: center; gap: 10px; }
    .pk .dot { width: 10px; height: 10px; border-radius: 50%; background: #c0c8d2; display: inline-block; }
    .pk .dot.live { background: #1d9a5a; } .pk .dot.dead { background: #d64545; }
    .pk .cols { display: flex; gap: 16px; align-items: flex-start; }
    .pk .stage { position: relative; background: #0f1620; border-radius: 8px; overflow: hidden; flex: 1 1 640px; min-width: 0; max-width: 760px; aspect-ratio: 848 / 480; }
    .pk .stage img { display: block; width: 100%; height: 100%; object-fit: contain; }
    .pk .stage canvas { position: absolute; left: 0; top: 0; pointer-events: none; }
    .pk .stage .view { position: absolute; right: 10px; top: 10px; display: inline-flex; padding: 3px; border-radius: 20px; background: rgba(13,19,26,.85); }
    .pk .stage .view button { padding: 6px 14px; border: 0; border-radius: 16px; background: none; color: #fff; font: inherit; font-weight: 600; cursor: pointer; }
    .pk .stage .view button.on { background: #1f5fbf; }
    .pk .stage .nocam { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; background: #1a2029; }
    .pk .stage .nocam > div { max-width: 86%; padding: 14px 20px; border: 3px solid #d64545; border-radius: 14px; background: #0d131a; color: #c9d3de; font-size: 13px; white-space: pre-wrap; }
    .pk .stage .nocam b { display: block; color: #fff; font-size: 22px; text-align: center; margin-bottom: 6px; }
    .pk .side { flex: 0 0 320px; width: 320px; }
    .pk .card { border: 1px solid #d5dce5; border-radius: 8px; padding: 10px 12px; margin: 0 0 10px; }
    .pk .card h3 { margin: 0 0 6px; font-size: 14px; display: flex; align-items: center; justify-content: space-between; }
    .pk .card h3 small { color: #5b6b7d; font-weight: normal; }
    .pk .row { display: flex; gap: 8px; align-items: center; margin: 6px 0; }
    .pk button { padding: 8px 12px; border: 1px solid #8e9bab; border-radius: 6px; background: #fff; font: inherit; cursor: pointer; }
    .pk button:disabled { opacity: .45; cursor: default; }
    .pk button.primary { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .pk button.on { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .pk button.small { padding: 6px 10px; font-size: 12.5px; }
    .pk .chips { display: grid; grid-template-columns: repeat(6, 1fr); grid-auto-rows: 40px; gap: 5px; height: 85px; }
    .pk .chips button { padding: 0; font-weight: 700; border-color: #d5dce5; border-radius: 10px; }
    .pk .chips button.sel { background: #1f5fbf; color: #fff; border-color: #1f5fbf; }
    .pk .chips button.add { background: #e6efff; color: #1f5fbf; border-color: #1f5fbf; font-size: 18px; }
    .pk .point { display: flex; align-items: center; gap: 6px; height: 40px; margin-top: 5px; }
    .pk .point .what { flex: 1; min-width: 0; cursor: pointer; }
    .pk .point .what b { display: block; font-size: 13px; }
    .pk .point .what span { color: #5b6b7d; font-size: 12px; }
    .pk .tiles { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; }
    .pk .tiles button { padding: 0; border: 0; background: none; line-height: 0; }
    .pk .tiles button svg { width: 100%; height: auto; }
    .pk .tab { display: flex; gap: 24px; align-items: flex-start; }
    .pk .tab .fields { flex: 0 0 460px; }
    .pk .tab .art svg { display: block; }
    .pk .status { margin-top: 8px; padding: 8px 10px; border-radius: 6px; background: #eef2f7; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .pk .status.warn { background: #fff4d6; } .pk .status.err { background: #fde2e2; } .pk .status.ok { background: #e3f5ea; }
    .pk .num { display: grid; grid-template-columns: 1fr auto; gap: 4px 10px; align-items: center; padding: 4px 0; border-bottom: 1px solid #eef2f7; }
    .pk .num .lbl { font-size: 13px; } .pk .num .lbl small { display: block; color: #5b6b7d; font-size: 11.5px; }
    .pk .stepper { display: inline-flex; align-items: center; gap: 4px; }
    .pk .stepper input { width: 64px; padding: 5px 6px; border: 1px solid #b9c3cf; border-radius: 6px; font: inherit; text-align: right; }
    .pk .stepper button { padding: 4px 11px; }
    .pk .seg { display: inline-flex; }
    .pk .seg button { border-radius: 0; } .pk .seg button:first-child { border-radius: 6px 0 0 6px; } .pk .seg button:last-child { border-radius: 0 6px 6px 0; }
    .pk .check { display: flex; align-items: center; gap: 10px; margin: 10px 0 2px; cursor: pointer; }
    .pk .check input { width: 24px; height: 24px; }
    .pk .check small { display: block; }
    .pk small { color: #5b6b7d; }
    .pk .hidden { display: none !important; }
    /* the tree row: one line, 48 px */
    .pkrow { display: flex; align-items: center; gap: 10px; height: 40px; overflow: hidden; white-space: nowrap; font: 14px/1.2 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color: #1f2a37; padding: 0 4px; }
    .pkrow .txt { overflow: hidden; text-overflow: ellipsis; }
    .pkrow .verdict { overflow: hidden; text-overflow: ellipsis; color: #5b6b7d; font-size: 12.5px; flex: 1 1 auto; }
    .pkrow .verdict.warn { color: #9a6b00; } .pkrow .verdict.ok { color: #1d9a5a; }
    .pkrow button { padding: 6px 12px; border: 1px solid #1f5fbf; border-radius: 6px; background: #1f5fbf; color: #fff; font: inherit; cursor: pointer; flex: 0 0 auto; }
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
      const np = st.points.length;
      this.$("txt").textContent = `${P.partWords(st)} · ${np} picture${np === 1 ? "" : "s"} · ${P.orderText(st.orderFirst, st.orderRows)}`;
      const why = P.problem(st);
      if (why) this.setVerdict(why, "warn");
      else this.setVerdict(`ready · camera computer ${st.host}:${st.port}`, "ok");
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
          title: "3D Pick",
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
      this._tab = "part";
      this._depth = false; // the picture's toggle: the depth as a heatmap instead of the camera's picture
      this._nocam = null; // what to check while there is no picture (pickscript.js advise())
      this._logged = "";
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
          <div data-pk="main">
            <div class="cols">
              <div class="stage" data-pk="stage">
                <img data-pk="img" alt="wrist camera" draggable="false" />
                <canvas data-pk="overlay"></canvas>
                <div class="nocam" data-pk="nocam"><div><b>NO CAMERA CONNECTED</b><span data-pk="nocam-text">waiting for the camera computer…</span></div></div>
                <div class="view" data-pk="view">
                  <button data-view="picture" class="on">Picture</button><button data-view="depth">Depth</button>
                </div>
              </div>
              <div class="side">
                <h2><span class="dot" data-pk="dot"></span> 3D Pick <small>v${P.VERSION}</small></h2>
                <div class="card">
                  <h3>Picture points <small data-pk="points-count"></small></h3>
                  <div class="chips" data-pk="chips"></div>
                  <div class="point" data-pk="point"></div>
                </div>
                <div class="card">
                  <h3>Pick order <small data-pk="order-text"></small></h3>
                  <div class="tiles" data-pk="tiles"></div>
                </div>
                <div class="row">
                  <button data-pk="toggle">Options</button>
                  <button data-pk="check" title="PolyScope's auto-move screen over the first part: hold Move to see the fingers arrive open, Approach mm over it">Check approach</button>
                </div>
              </div>
            </div>
          </div>
          <div data-pk="options" class="hidden">
            <div class="row">
              <button data-pk="back">‹ Back to the picture</button>
              <span style="flex:1"></span>
              <span class="seg" data-pk="tabs"><button data-tab="part">Part</button><button data-tab="approach">Approach</button></span>
              <span style="flex:1"></span>
              <button class="small" data-pk="reset">Reset to defaults</button>
            </div>
            <div class="card" data-pk="tab-part">
              <h3>The part, as it lies on the table</h3>
              <div class="tab">
                <div class="fields">
                  <span class="seg" data-pk="shapes"><button data-shape="box">Box</button><button data-shape="cyl">Cylinder</button></span>
                  <div data-pk="card-part"></div>
                </div>
                <div class="art" data-pk="part-art"></div>
              </div>
            </div>
            <div class="card hidden" data-pk="tab-approach">
              <h3>Approach and grip</h3>
              <div class="tab">
                <div class="fields">
                  <div data-pk="card-approach"></div>
                  <label class="check"><input type="checkbox" data-pk="gripCheck" /><span>Grip check<small>skip a part with less than the finger room on either side</small></span></label>
                  <label class="check" data-pk="gripLongRow"><input type="checkbox" data-pk="gripLongSide" /><span>Grip across the long side<small>off: the fingers close across the short side</small></span></label>
                  <label class="check"><input type="checkbox" data-pk="closeLook" /><span>Closer look<small>a second, nearer measurement before the approach</small></span></label>
                </div>
                <div class="art" data-pk="approach-art"></div>
              </div>
            </div>
          </div>
          <div class="status" data-pk="status"></div>
        </div>`;
      const cards = { part: this.$("card-part"), approach: this.$("card-approach") };
      P.NUMBERS.forEach((n) => cards[n.section].appendChild(this.stepper(n)));
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
      this.$("toggle").addEventListener("click", () => { this._options = true; this.sync(); });
      this.$("back").addEventListener("click", () => { this._options = false; this.sync(); });
      this.$("tabs").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => { this._tab = b.dataset.tab; this.sync(); }));
      this.$("check").addEventListener("click", () => this.checkApproach());
      this.$("reset").addEventListener("click", () => {
        const p = this.params();
        p.values = P.defaults();
        p.shape = "box";
        p.gripCheck = true;
        p.gripLongSide = false;
        p.closeLook = true;
        this.save();
        this.sync();
      });
      this.$("shapes").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
        this.params().shape = b.dataset.shape;
        this.save();
        this.sync();
      }));
      ["gripCheck", "gripLongSide", "closeLook"].forEach((key) => this.$(key).addEventListener("change", (ev) => {
        this.params()[key] = !!ev.target.checked;
        this.save();
        this.sync();
      }));
      // the Picture / Depth toggle, inside the picture's frame
      this.$("view").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
        this._depth = b.dataset.view === "depth";
        this._seq = 0;
        this.sync();
      }));
    }

    stepper(n) {
      const row = document.createElement("div");
      row.className = "num";
      row.dataset.key = n.key;
      row.innerHTML = `
        <div class="lbl"><span data-lbl>${esc(n.label)}</span>${n.unit ? ` <small style="display:inline">(${esc(n.unit)})</small>` : ""}<small data-help>${esc(n.help)}</small></div>
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

    /** An action's own message: it stays for a few seconds before the scene's count takes the line back. */
    tell(text, kind) {
      this._hold = Date.now() + 5000;
      this.setStatus(text, kind);
    }

    setLive(live) {
      const dot = this.$("dot");
      if (dot) dot.className = "dot " + (live ? "live" : "dead");
      const card = this.$("nocam");
      if (card) card.classList.toggle("hidden", !!live);
      const view = this.$("view");
      if (view) view.classList.toggle("hidden", !live);
    }

    /** No picture: say what to check where the picture would be; the long story goes to the console. */
    noCamera(advice) {
      this._nocam = advice;
      this.setLive(false);
      const t = this.$("nocam-text");
      if (t) t.textContent = advice.text;
      if (advice.detail !== this._logged) {
        this._logged = advice.detail;
        console.warn(`Perceptronic: ${advice.detail}`);
      }
      this.setStatus(advice.summary, "err");
    }

    /** Everything that reflects the node's state. Cheap; called after every change. */
    sync() {
      if (!this._built) return;
      const P = this._P;
      const p = this.params();
      const st = this.settings();
      this.$("main").classList.toggle("hidden", this._options);
      this.$("options").classList.toggle("hidden", !this._options);
      this.$("tab-part").classList.toggle("hidden", this._tab !== "part");
      this.$("tab-approach").classList.toggle("hidden", this._tab !== "approach");
      this.$("tabs").querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.tab === this._tab));
      this.$("view").querySelectorAll("button").forEach((b) => b.classList.toggle("on", (b.dataset.view === "depth") === this._depth));
      // picture points: a fixed grid of numbered buttons, the selected one's actions under it
      const points = p.points || [];
      const sel = Math.max(0, Math.min(points.length - 1, p.selectedPoint || 0));
      const areas = P.areasOf(this._app);
      const chips = this.$("chips");
      chips.innerHTML = "";
      points.forEach((pt, i) => {
        const b = document.createElement("button");
        b.textContent = String(i + 1);
        b.className = i === sel ? "sel" : "";
        b.title = `picture point ${i + 1}`;
        b.addEventListener("click", () => { p.selectedPoint = i; this.save(); this.sync(); });
        chips.appendChild(b);
      });
      if (points.length < P.MAX_POINTS) {
        const add = document.createElement("button");
        add.textContent = "+";
        add.className = "add";
        add.dataset.pk = "add";
        add.title = "add a picture point where the arm is now";
        add.addEventListener("click", () => this.addPoint());
        chips.appendChild(add);
      }
      const row = this.$("point");
      if (!points.length) {
        row.innerHTML = "<small>tap + with the arm where the camera sees the parts</small>";
      } else {
        const pt = points[sel];
        const area = pt.area >= 0 && pt.area < areas.length ? areas[pt.area] : null;
        const areaText = area ? (area.plane ? area.name : `${area.name} (not taught)`) : "live table";
        row.innerHTML = `<span class="what" title="tap to choose the pick area this picture looks at"><b>Picture ${sel + 1}</b><span>${esc(areaText)} ›</span></span>
          <button class="small" data-act="go" title="PolyScope's auto-move screen to this picture point">Go</button>
          <button class="small" data-act="here" title="retake this point from where the arm is">Here</button>
          <button class="small" data-act="del" title="remove">✕</button>`;
        row.querySelector(".what").addEventListener("click", () => this.cycleArea(sel));
        row.querySelector('[data-act="go"]').addEventListener("click", () => this.goPoint(sel));
        row.querySelector('[data-act="here"]').addEventListener("click", () => this.addPoint(sel));
        row.querySelector('[data-act="del"]').addEventListener("click", () => this.removePoint(sel));
      }
      this.$("points-count").textContent = `${points.length} of ${P.MAX_POINTS}`;
      // order
      this.$("tiles").querySelectorAll("button").forEach((b) => {
        const on = b.dataset.first === st.orderFirst && b.dataset.rows === st.orderRows;
        if (b.dataset.on !== String(on)) { b.dataset.on = String(on); b.innerHTML = P.svgOrderTile(b.dataset.first, b.dataset.rows, on, 62, 50); }
      });
      this.$("order-text").textContent = P.orderText(st.orderFirst, st.orderRows);
      // the two option tabs
      const v = st.values, round = P.round(st);
      this.$("shapes").querySelectorAll("button").forEach((b) => b.classList.toggle("on", (b.dataset.shape === "cyl") === round));
      this.querySelectorAll(".num").forEach((r) => {
        const n = P.BY_KEY[r.dataset.key];
        const val = st.values[n.key];
        r.querySelector("input").value = n.step >= 1 ? P.num(val) : String(Math.round(val * 100) / 100);
        if (n.key === "partLengthMm") {
          r.querySelector("[data-lbl]").textContent = round ? "Diameter" : "Length";
          r.querySelector("[data-help]").textContent = round ? "across the top, standing on its end" : n.help;
        }
        if (n.key === "partWidthMm") r.classList.toggle("hidden", round);
        if (n.key === "fingerRoomMm") r.classList.toggle("hidden", !st.gripCheck);
      });
      this.$("part-art").innerHTML = P.svgPart(P.longSide(st), P.shortSide(st), v.partHeightMm, round, 300, 240);
      const longWay = st.gripLongSide && !round;
      this.$("approach-art").innerHTML = P.svgApproach(v.approachMm, v.gripBelowTopMm, v.partHeightMm,
        longWay ? P.longSide(st) : P.shortSide(st), st.gripCheck ? v.fingerRoomMm : 0, 300, 240);
      this.$("gripCheck").checked = st.gripCheck;
      this.$("gripLongSide").checked = st.gripLongSide;
      this.$("gripLongRow").classList.toggle("hidden", round); // a cylinder has no side to choose
      this.$("closeLook").checked = st.closeLook;
      // the verdict
      const why = P.problem(st);
      this.$("check").disabled = !!why;
      if (why) this.setStatus(why, this._app ? "warn" : "");
      else if (this._nocam && !this._options) this.setStatus(this._nocam.summary, "err");
      else if (this._options || !this._scene || !this._scene.ok) this.setStatus(`ready: ${P.partWords(st)} · ${P.orderText(st.orderFirst, st.orderRows)} · camera computer ${st.host}:${st.port}`, "ok");
      else this.setStatus(P.sceneSummary(this._scene), (this._scene.parts || []).length ? "ok" : "warn");
      this.drawScene();
    }

    // -- the application node (cockpit, areas) ---------------------------------------------------
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
        this.tell(`picture point ${(index === undefined ? points.length - 1 : index) + 1} taught at joints [${fmtVec(q, 3)}]`, "ok");
      } catch (err) {
        this.tell(`picture point: ${err && err.message ? err.message : err}`, "err");
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
      if (!rms || typeof rms.autoMove !== "function") { this.tell("PolyScope's move service is not on this API", "warn"); return; }
      try {
        this.params().selectedPoint = i;
        this.sync();
        await rms.autoMove(arrayToJoints(q));
        this.tell(`PolyScope's move screen is open — hold Move To Position to go to picture point ${i + 1}`, "ok");
      } catch (err) {
        this.tell(`Go: ${err && err.message ? err.message : err}`, "err");
      }
    }

    // -- the feed and the scene ------------------------------------------------------------------
    async startFeed() {
      const img = this.$("img");
      const P = this._P;
      while (!this._stopped && this.isConnected) {
        if (!this._app) { await sleep(500); continue; }
        const base = this.cockpitUrl();
        const depth = this._depth;
        try {
          const ctl = typeof AbortController === "function" ? new AbortController() : null;
          const timer = ctl ? setTimeout(() => ctl.abort(), FEED_TIMEOUT_MS) : 0;
          let r, blob;
          try {
            r = await fetch(`${base}/api/${depth ? "depth" : "color"}.png?after=${this._seq}&timeout_ms=${POLL_TIMEOUT_MS}`,
              ctl ? { signal: ctl.signal } : undefined);
            if (r.ok) blob = await r.blob();
          } finally {
            clearTimeout(timer);
          }
          if (r.status === 503) { this.noCamera(P.advise("nopicture", base, "HTTP 503: the cockpit has no frame")); await sleep(500); continue; }
          if (r.status === 404 && depth) {
            // a camera computer older than 0.7.0 has no heatmap: the picture instead
            this._depth = false;
            console.warn(`Perceptronic: ${base} answered 404 on /api/depth.png: it predates the depth view; update it`);
            this.sync();
            continue;
          }
          if (!r.ok) { this.noCamera(P.advise("outdated", base, `HTTP ${r.status} on /api/color.png`)); await sleep(1500); continue; }
          const seq = Number(r.headers.get("X-Seq") || 0);
          const url = URL.createObjectURL(blob);
          const previous = this._blobUrl;
          img.onload = () => { if (previous) URL.revokeObjectURL(previous); this.drawScene(); };
          img.src = url;
          this._blobUrl = url;
          if (seq) this._seq = seq;
          const was = this._nocam;
          this._nocam = null;
          this.setLive(true);
          if (was) this.sync();
        } catch (err) {
          const why = err && err.message ? err.message : String(err);
          let kind = "silent";
          try { new URL(base); } catch (e) { kind = "badurl"; }
          if (kind === "silent") kind = await this.probe(base);
          this.noCamera(P.advise(kind, base, `${why} (page origin ${location.origin}; a cockpit must be started with --cors ${location.origin} --bind 0.0.0.0)`));
          await sleep(1500);
        }
      }
    }

    // "Failed to fetch" is all the browser says, whether the cockpit refused this page's origin
    // (CORS) or nothing answered. A no-cors request tells them apart: it resolves (opaque) when
    // the server is up, whatever its CORS list, rejects at once when nothing listens, and hangs
    // when the host is not there at all.
    async probe(base) {
      const ctl = typeof AbortController === "function" ? new AbortController() : null;
      const timer = ctl ? setTimeout(() => ctl.abort(), 2500) : null;
      try {
        await fetch(`${base}/api/info`, { mode: "no-cors", cache: "no-store", signal: ctl ? ctl.signal : undefined });
        return "cors";
      } catch (e) {
        return e && e.name === "AbortError" ? "silent" : "refused";
      } finally {
        if (timer) clearTimeout(timer);
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
        const told = this._P.sceneSummary(this._scene);
        this._scene = res;
        if (res && res.ok && this._P.sceneSummary(res) !== told && Date.now() > (this._hold || 0)) this.sync();
      } catch (e) {
        this._scene = null;
      } finally {
        this._sceneBusy = false;
      }
      this.drawScene();
    }

    /** What is drawn on the picture (0.6.0, Nick 2026-10-01): every part that will be picked
     * in green with its number in the pick order, and every candidate that is nearly the part
     * and will not be in yellow with why. Nothing else. */
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
      if (!sc || !sc.ok || this._nocam) return;
      // the scene's pixels are the colour picture's (the depth view is half its size)
      const sx = w / (sc.width || img.naturalWidth || w), sy = h / (sc.height || img.naturalHeight || h);
      (sc.parts || []).forEach((p) => {
        const corners = p.corners_px;
        if (!Array.isArray(corners) || corners.length !== 4) return;
        ctx.beginPath();
        corners.forEach(([u, v], i) => (i ? ctx.lineTo(u * sx, v * sy) : ctx.moveTo(u * sx, v * sy)));
        ctx.closePath();
        ctx.fillStyle = "rgba(61,220,132,.25)"; ctx.fill();
        ctx.strokeStyle = "#3ddc84"; ctx.lineWidth = 2.4; ctx.stroke();
        if (Array.isArray(p.pixel)) {
          const x = p.pixel[0] * sx, y = p.pixel[1] * sy;
          ctx.beginPath(); ctx.arc(x, y, 13, 0, Math.PI * 2);
          ctx.fillStyle = "#12824a"; ctx.fill();
          ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
          ctx.fillStyle = "#fff"; ctx.font = "bold 13px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
          ctx.fillText(String(p.order || "·"), x, y);
        }
      });
      this._P.nearMisses(sc).forEach((r) => {
        const corners = r.corners_px;
        if (!Array.isArray(corners) || corners.length !== 4) return;
        const path = () => {
          ctx.beginPath();
          corners.forEach(([u, v], i) => (i ? ctx.lineTo(u * sx, v * sy) : ctx.moveTo(u * sx, v * sy)));
          ctx.closePath();
        };
        ctx.setLineDash([]);
        path(); ctx.strokeStyle = "rgba(0,0,0,.45)"; ctx.lineWidth = 4; ctx.stroke();
        ctx.setLineDash([7, 5]);
        path(); ctx.strokeStyle = "#ffc53d"; ctx.lineWidth = 2; ctx.stroke();
        ctx.setLineDash([]);
        if (Array.isArray(r.pixel) && r.why) {
          ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
          const tw = ctx.measureText(r.why).width + 14, x = r.pixel[0] * sx, y = r.pixel[1] * sy + 28;
          ctx.fillStyle = "rgba(20,26,34,.85)";
          ctx.fillRect(x - tw / 2, y - 10, tw, 20);
          ctx.fillStyle = "#ffc53d";
          ctx.fillText(r.why, x, y);
        }
      });
    }

    // -- Check approach ------------------------------------------------------------------------------
    async checkApproach() {
      const P = this._P;
      const rps = this.service("robotPositionService"), rms = this.service("robotMoveService");
      const st = this.settings();
      const p = this.params();
      if (!rps || !rms) { this.tell("PolyScope's move services are not on this API", "warn"); return; }
      const sel = (p.points || []).length ? Math.max(0, Math.min(p.points.length - 1, p.selectedPoint || 0)) : -1;
      const approach = st.values.approachMm;
      this.tell("asking the camera computer for the first part…");
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
        this.tell(`PolyScope's move screen is open — hold Move To Position: fingertips ${P.num(approach)} mm over the first part's top, fingers open${tcpNote}`, "ok");
      } catch (err) {
        this.tell(`Check approach: ${err && err.message ? err.message : err}`, "err");
      }
    }
  }

  if (!window.customElements.get(PICK_TAG)) window.customElements.define(PICK_TAG, PerceptronicPickNode);
  if (!window.customElements.get(DIALOG_TAG)) window.customElements.define(DIALOG_TAG, PerceptronicPickDialog);
})();
