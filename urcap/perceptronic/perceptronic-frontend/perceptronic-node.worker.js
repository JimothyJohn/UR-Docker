// Perceptronic — Application Node behavior (the web worker PolyScope X loads
// for the node declared in contribution.json).
//
// The SDK's `registerApplicationBehavior(behaviors)` is `expose(behaviors)` from
// threads.js 1.7 (contribution-api 21.3.266, fesm2022 line ~6205). This file
// speaks that worker protocol directly so the URCap needs no npm build:
//   worker → master  {type:"init", exposed:{type:"module", methods:[...]}}
//   master → worker  {type:"run", uid, method, args}
//   worker → master  {type:"running", uid, resultType:"promise"}
//                    {type:"result", uid, complete:true, payload}
//                    {type:"error", uid, error:{__error_marker:"$$error", name, message, stack}}
// (threads/dist/worker/index.js + dist/types/messages.js + dist/serializers.js).

const NODE_TYPE = "nickarmenta-perceptronic";
const NODE_VERSION = "1.1.0";

// The node's saved state. `cockpitUrl` is where the RealSense cockpit
// (`perceptronics gui --cors …`) answers; empty = the page's own host on :7621.
// `areas` (up to 8: name + the three fingertip touches, base frame, m), `tipMm` (the
// fingertips past the flange), `reachInnerMm` / `reachOuterMm` (the pick ring's margins
// inside the rated reach) and `robotModel` (read from PolyScope) are what the
// Perceptronic Pick program node reads from this node (pickscript.js `settings`).
const fresh = () => ({
  type: NODE_TYPE,
  version: NODE_VERSION,
  cockpitUrl: "",
  areas: [],
  tipMm: 163,
  reachInnerMm: 150,
  reachOuterMm: 150,
  robotModel: "",
});
const behaviors = {
  factory: async () => fresh(),
  upgradeNode: async (loadedNode, defaultNode) => ({ ...fresh(), ...(defaultNode || {}), ...(loadedNode || {}), version: NODE_VERSION }),
  downgradeNode: async (loadedNode, defaultNode) => defaultNode,
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
