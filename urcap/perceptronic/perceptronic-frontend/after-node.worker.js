// Perceptronic Pick — "After picture N" Program Node behavior: its children run only when
// the enclosing Pick node picked from picture point N (`if <loc variable> == N:`). Same
// protocol notes as pick-node.worker.js.

importScripts("pickscript.js");

const P = self.PerceptronicPick;
const NODE_TYPE = P.AFTER_TYPE;
const NODE_VERSION = "1.0.0";

function builder(lines, indent) {
  return { type: "$$ScriptBuilder", script: lines.join("\n") + "\n", currentIndent: indent };
}

// The enclosing Pick node, from a serialized tree context (`{ancestors: [...]}` — each
// entry a program node, or `{node}` wrapping one). Null when the node sits outside one.
async function pickAncestor(context) {
  const t = context && context.traverse;
  let anc = (t && t.ancestors) || (context && context.ancestors);
  if (!anc) return null;
  if (typeof anc[Symbol.asyncIterator] === "function" && !Array.isArray(anc)) {
    const list = [];
    for await (const a of anc) list.push(a);
    anc = list;
  }
  for (const a of anc) {
    const node = a && a.node ? a.node : a;
    if (node && node.type === P.PICK_TYPE) return node;
  }
  return null;
}

const behaviors = {
  factory: async () => ({ type: NODE_TYPE, version: NODE_VERSION, allowsChildren: true, parameters: { point: 1 } }),

  programNodeLabel: async (node) => [
    { type: "primary", value: String((node && node.parameters && node.parameters.point) || 1) },
    { type: "secondary", value: "the routine for a pick from this picture point" },
  ],

  validator: async (node, context) => {
    const point = node && node.parameters ? node.parameters.point : 0;
    if (!(Number.isInteger(point) && point >= 1 && point <= P.MAX_POINTS)) {
      return { isValid: false, errorMessageKey: `the picture point must be 1..${P.MAX_POINTS}` };
    }
    const pick = await pickAncestor(context);
    if (!pick) return { isValid: false, errorMessageKey: "put this node inside a Perceptronic Pick node" };
    const n = pick.parameters && Array.isArray(pick.parameters.points) ? pick.parameters.points.length : 0;
    if (point > n) return { isValid: false, errorMessageKey: `the Pick node has ${n} picture point${n === 1 ? "" : "s"}, not ${point}` };
    return { isValid: true };
  },

  generateCodeBeforeChildren: async (node, context) => {
    const pick = await pickAncestor(context);
    const loc = pick && pick.parameters && pick.parameters.locVariable && pick.parameters.locVariable.name;
    const sc = P.afterPictureScript(loc, (node && node.parameters && node.parameters.point) || 1);
    return builder(sc.before, sc.childDepth);
  },

  generateCodeAfterChildren: async (node, context) => {
    const pick = await pickAncestor(context);
    const loc = pick && pick.parameters && pick.parameters.locVariable && pick.parameters.locVariable.name;
    const sc = P.afterPictureScript(loc, (node && node.parameters && node.parameters.point) || 1);
    return builder(sc.after, -sc.childDepth);
  },

  allowsChild: async () => true,

  upgradeNode: async (loaded) => ({
    type: NODE_TYPE,
    version: NODE_VERSION,
    allowsChildren: true,
    parameters: { point: (loaded && loaded.parameters && loaded.parameters.point) || 1 },
  }),
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
