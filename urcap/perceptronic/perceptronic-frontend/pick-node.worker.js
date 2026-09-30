// Perceptronic Pick — Program Node behavior (the web worker PolyScope X loads for the
// program node declared in contribution.json). Same hand-written threads.js protocol as
// perceptronic-node.worker.js (`registerProgramBehavior(b)` is `expose(b)`); the shape
// of every answer is what PolyScope's own serializers read back (web-app main.js,
// 10.13): a code generator answers `{type: "$$ScriptBuilder", script, currentIndent}`
// — PolyScope rebuilds `new ScriptBuilder(script, currentIndent)`, and `append`s it: the
// lines at the builder's indent, the children after them `currentIndent` levels deeper,
// so the after-children builder carries the negative indent that closes those blocks.
// The application context arrives as `{type: "$$ApplicationContext", contributions:
// {contributionList: [...]}}` — the Perceptronic application node (cockpit URL, pick
// areas, tip, reach) is the entry of our type.

importScripts("pickscript.js");

const P = self.PerceptronicPick;
const NODE_TYPE = P.PICK_TYPE;
const NODE_VERSION = "1.0.0";

function appNodeOf(applicationContext) {
  const list = applicationContext && applicationContext.contributions && applicationContext.contributions.contributionList;
  if (!Array.isArray(list)) return null;
  return list.find((c) => c && (c.type === P.APP_TYPE || c.parentType === P.APP_TYPE)) || null;
}

function builder(lines, indent) {
  return { type: "$$ScriptBuilder", script: lines.join("\n") + "\n", currentIndent: indent };
}

function fresh() {
  return {
    type: NODE_TYPE,
    version: NODE_VERSION,
    allowsChildren: true,
    parameters: {
      nodeId: P.newNodeId(),
      points: [],
      selectedPoint: 0,
      orderFirst: "LR",
      orderRows: "FB",
      gripper: "robotiq",
      popupOnFail: true,
      perPointRoutine: false,
      pickPort: P.DEFAULT_PICK_PORT,
      values: P.defaults(),
      foundVariable: null,
      locVariable: null,
    },
  };
}

const behaviors = {
  factory: async () => fresh(),

  programNodeLabel: async (node) => {
    const p = (node && node.parameters) || {};
    const st = P.settings(p, null);
    const v = st.values;
    const l = Math.max(v.partLengthMm, v.partWidthMm), w = Math.min(v.partLengthMm, v.partWidthMm);
    const np = st.points.length;
    // PolyScope prefixes the tree row with the node's title itself ("Perceptronic Pick: …")
    return [
      { type: "primary", value: `${P.num(l)}×${P.num(w)}×${P.num(v.partHeightMm)} mm` },
      { type: "secondary", value: `${np} picture${np === 1 ? "" : "s"} · ${P.orderText(st.orderFirst, st.orderRows)}` },
    ];
  },

  validator: async (node, context, applicationContext) => {
    const st = P.settings((node && node.parameters) || {}, appNodeOf(applicationContext), null);
    const why = P.problem(st);
    return why ? { isValid: false, errorMessageKey: why } : { isValid: true };
  },

  generateCodeBeforeChildren: async (node, context, applicationContext) => {
    const st = P.settings((node && node.parameters) || {}, appNodeOf(applicationContext), null);
    const sc = P.script(st);
    return builder(sc.before, sc.childDepth);
  },

  generateCodeAfterChildren: async (node, context, applicationContext) => {
    const st = P.settings((node && node.parameters) || {}, appNodeOf(applicationContext), null);
    const sc = P.script(st);
    return builder(sc.after, -sc.childDepth);
  },

  allowsChild: async () => true,

  upgradeNode: async (loaded) => {
    const base = fresh();
    const p = (loaded && loaded.parameters) || {};
    return {
      ...base,
      ...(loaded || {}),
      version: NODE_VERSION,
      allowsChildren: true,
      parameters: {
        ...base.parameters,
        ...p,
        nodeId: /^[0-9a-f]{1,12}$/.test(p.nodeId || "") ? p.nodeId : base.parameters.nodeId,
        values: P.values(p.values),
        points: Array.isArray(p.points) ? p.points.slice(0, P.MAX_POINTS) : [],
      },
    };
  },

  // A pasted copy gets its own identity: the pick server queues parts per node id.
  onLifeCycleHook: async (event, subtree) => {
    if (event === "paste" && subtree && subtree.node && subtree.node.parameters) {
      subtree.node.parameters.nodeId = P.newNodeId();
    }
    return subtree;
  },
};

function serializeError(err) {
  const e = err instanceof Error ? err : new Error(String(err));
  return { __error_marker: "$$error", name: e.name, message: e.message, stack: e.stack || "" };
}

self.addEventListener("message", (event) => {
  const msg = event.data;
  if (!msg || msg.type !== "run" || !msg.method) return;
  const fn = behaviors[msg.method];
  if (typeof fn !== "function") {
    self.postMessage({ type: "error", uid: msg.uid, error: serializeError(new Error(`no behavior ${msg.method}`)) });
    return;
  }
  let out;
  try {
    out = fn(...(msg.args || []));
  } catch (err) {
    self.postMessage({ type: "error", uid: msg.uid, error: serializeError(err) });
    return;
  }
  self.postMessage({ type: "running", uid: msg.uid, resultType: "promise" });
  Promise.resolve(out).then(
    (value) => self.postMessage({ type: "result", uid: msg.uid, complete: true, payload: value }),
    (err) => self.postMessage({ type: "error", uid: msg.uid, error: serializeError(err) }),
  );
});

self.postMessage({ type: "init", exposed: { type: "module", methods: Object.keys(behaviors) } });
