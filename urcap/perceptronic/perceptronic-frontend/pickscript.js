// Perceptronic Pick — the program node's settings, the URScript it contributes and the
// pose math both its presenter and its behavior worker need. One file for both: the
// worker `importScripts` it, the page loads it with a <script> tag, tests run it under
// node. It is the PolyScope X port of the PolyScope 5 node's PickScript.java
// (urcap/perceptronic-ps5/src/.../PickScript.java): the same settings table, the same
// request options (`tokens`: what the pick server's `parse_options` reads) and the same
// script — so the cockpit's pick server (`:7622`, protocol 2) serves both robots.
//
// One run of the node picks one part, inside the operator's program (Local mode, no
// Primary client): force the TCP to the flange, open the gripper fully, ask NEXT (a part
// already seen, or the picture point to look from), movej there + FIND, the close look
// (LOOK/REFINE, leaned 0/12/24° while the controller's IK can't solve approach + grip +
// lift), the approach with the fingertips `approachMm` over the top, down to the grip,
// close, lift. The result variable is True only after a held lift; the children — the
// routine after the pick — run with the operator's TCP back. With the gripper set to
// "my nodes" the children run at the grip instead.
(function (root) {
  "use strict";

  const APP_TYPE = "nickarmenta-perceptronic";
  const PICK_TYPE = "nickarmenta-perceptronic-pick";
  const AFTER_TYPE = "nickarmenta-perceptronic-after";
  const VERSION = "0.3.0";
  const DEFAULT_PICK_PORT = 7622;
  const DEFAULT_COCKPIT_PORT = 7621;
  const DEFAULT_TIP_MM = 163; // Hand-E 157 mm + 6 mm adapter
  const SOCKET = "rs_pick";
  const RQ_SOCKET = "rs_rq";
  const MAX_POINTS = 12;
  const MAX_AREAS = 8;
  const ORDERS = ["LR", "RL", "FB", "BF"];
  const ORDER_TILES = [
    ["LR", "FB"], ["RL", "FB"], ["LR", "BF"], ["RL", "BF"],
    ["FB", "LR"], ["FB", "RL"], ["BF", "LR"], ["BF", "RL"],
  ];
  const GRIPPERS = ["robotiq", "digital", "children"];
  const FOUND_VARIABLE = "rs_pick_found";
  const LOC_VARIABLE = "rs_pick_loc";

  // The pick server's status codes, as the operator should read them (protocol 2).
  const REASONS = [
    [0, "no part in view at any picture point"],
    [-1, "the part is wider than the open gripper"],
    [-2, "the part is too close to the robot base"],
    [-3, "the camera computer has no hand-eye calibration"],
    [-4, "no fresh camera frame - is the camera computer's camera streaming?"],
    [-5, "the closer look did not find the part again"],
    [-7, "something is in view, but nothing the size of the part"],
    [-8, "no answer from the camera computer within 10 s"],
    [-9, "the camera computer did not understand the request (update it?)"],
    [-10, "the only parts in view are out of reach"],
    [-11, "no room for the open fingers beside any part"],
    [-12, "the parts in view are outside the pick area"],
    [-13, "the only part in view is cut off by the edge of the picture"],
  ];

  // One number the operator can set: key, section, label, unit, default, min, max, step, help.
  const NUMBERS = [
    ["partLengthMm", "part", "Length", "mm", 50, 5, 500, 1, "long side, as it lies"],
    ["partWidthMm", "part", "Width", "mm", 30, 5, 500, 1, "the fingers close across it"],
    ["partHeightMm", "part", "Height", "mm", 30, 5, 500, 1, "top above the table"],
    ["partTolPct", "part", "Tolerance", "%", 25, 5, 100, 5, "how far off still counts"],
    ["approachMm", "approach", "Approach", "mm", 25, 5, 200, 5, "fingertips over the top, fully open"],
    ["gripBelowTopMm", "approach", "Grip depth", "mm", 15, 0, 60, 1, "fingertips below the top to grip"],
    ["liftMm", "approach", "Lift", "mm", 60, 5, 300, 5, "straight up after the grip"],
    ["lookMm", "approach", "Close look", "mm", 90, 40, 400, 10, "camera over the top, if no better pose"],
    ["strokeMm", "gripper", "Open width", "mm", 50, 10, 300, 1, "the fingers' full opening (Hand-E: 50)"],
    ["gripperForcePct", "gripper", "Grip force", "%", 40, 0, 100, 5, "Robotiq force"],
    ["gripperSpeedPct", "gripper", "Finger speed", "%", 100, 1, 100, 5, "Robotiq speed"],
    ["gripperDo", "gripper", "Digital output", "", 0, 0, 7, 1, "the output that closes the gripper"],
    ["gripperWaitS", "gripper", "Close time", "s", 0.5, 0, 5, 0.1, "how long a digital-output gripper takes"],
    ["speedPct", "motion", "Speed", "%", 60, 10, 100, 10, "of the node's travel speed"],
    ["settleS", "motion", "Settle", "s", 0.2, 0, 2, 0.05, "before each picture: sharp vs fast"],
    ["maxAttempts", "motion", "Grasps per run", "", 3, 1, 10, 1, "tries before the node gives up"],
  ].map(([key, section, label, unit, def, min, max, step, help]) => ({ key, section, label, unit, def, min, max, step, help }));
  const BY_KEY = {};
  NUMBERS.forEach((n) => { BY_KEY[n.key] = n; });

  // The gripper card's fields, per gripper (the rest of the numbers sit on their own cards).
  const GRIPPER_FIELDS = {
    robotiq: ["strokeMm", "gripperForcePct", "gripperSpeedPct"],
    digital: ["strokeMm", "gripperDo", "gripperWaitS"],
    children: ["strokeMm"],
  };

  // -- numbers ----------------------------------------------------------------------------

  /** One decimal at most, no trailing zeros, never a locale's comma (Java's num()). */
  function num(v) {
    const t = Number(v).toFixed(1);
    return t.endsWith(".0") ? t.slice(0, -2) : t;
  }
  const f2 = (v) => Number(v).toFixed(2);
  const m = (mm) => (mm / 1000).toFixed(4);
  const finite = (xs, n) => Array.isArray(xs) && xs.length === n && xs.every((x) => typeof x === "number" && Number.isFinite(x));

  function defaults() {
    const out = {};
    NUMBERS.forEach((n) => { out[n.key] = n.def; });
    return out;
  }

  /** `v` for `key`, clamped to its limits (whole steps round); what would be stored. */
  function clamp(key, v) {
    const n = BY_KEY[key];
    if (!n) throw new Error(`no setting ${key}`);
    let x = Number(v);
    if (!Number.isFinite(x)) x = n.def;
    let c = Math.max(n.min, Math.min(n.max, x));
    if (n.step >= 1) c = Math.round(c / n.step) * n.step;
    return c;
  }

  /** The stored values with every key present and within its limits. */
  function values(stored) {
    const out = defaults();
    if (stored && typeof stored === "object") {
      NUMBERS.forEach((n) => { if (n.key in stored) out[n.key] = clamp(n.key, stored[n.key]); });
    }
    return out;
  }

  // -- the robot ----------------------------------------------------------------------------

  /** {base outer radius, rated reach} (m) by the robot type name — the same table as
   * perceptronics.volume.BASE_RADIUS_M / urctl.safety.MODEL_REACH_M; null when unknown. */
  function modelReach(type) {
    let t = String(type || "").toUpperCase().replace(/[^A-Z0-9]/g, "");
    if (t.endsWith("E")) t = t.slice(0, -1);
    if (t === "UR3") return [0.064, 0.5];
    if (t === "UR5" || t === "UR7") return [0.0745, 0.85];
    if (t === "UR10" || t === "UR12") return [0.095, 1.3];
    if (t === "UR16") return [0.095, 0.9];
    return null;
  }

  /** The pick ring {min, max} in m from the base axis, or null when the model is unknown;
   * max 0 = no outer limit (the margins ate the reach). */
  function reachLimits(model, innerMm, outerMm) {
    const r = modelReach(model);
    if (!r) return null;
    const min = r[0] + Number(innerMm || 0) / 1000;
    const max = Math.max(0, r[1] - Number(outerMm || 0) / 1000);
    return max > min ? { min, max } : { min, max: 0 };
  }

  // -- pose math (UR pose = [x, y, z, rx, ry, rz], rotation vector; 4x4 row-major) -----------

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
  function matToPose(mat) {
    const cos = Math.min(1, Math.max(-1, (mat[0][0] + mat[1][1] + mat[2][2] - 1) / 2));
    const th = Math.acos(cos);
    let r = [0, 0, 0];
    if (th > 1e-9 && Math.PI - th > 1e-6) {
      const k = th / (2 * Math.sin(th));
      r = [(mat[2][1] - mat[1][2]) * k, (mat[0][2] - mat[2][0]) * k, (mat[1][0] - mat[0][1]) * k];
    } else if (th > 1e-9) {
      const ax = [0, 1, 2].map((i) => Math.sqrt(Math.max(0, (mat[i][i] + 1) / 2)));
      const i = ax.indexOf(Math.max(...ax));
      for (let j = 0; j < 3; j++) if (j !== i) ax[j] = Math.sign(mat[i][j] + mat[j][i] || 1) * ax[j];
      r = ax.map((a) => a * th);
    }
    return [mat[0][3], mat[1][3], mat[2][3], ...r];
  }
  function matMul(a, b) {
    return a.map((row) => [0, 1, 2, 3].map((j) => row.reduce((acc, v, k) => acc + v * b[k][j], 0)));
  }
  function matInv(mat) {
    const R = [0, 1, 2].map((i) => [0, 1, 2].map((j) => mat[j][i]));
    const t = [0, 1, 2].map((i) => -(R[i][0] * mat[0][3] + R[i][1] * mat[1][3] + R[i][2] * mat[2][3]));
    return [[...R[0], t[0]], [...R[1], t[1]], [...R[2], t[2]], [0, 0, 0, 1]];
  }
  /** URScript's pose_trans(a, b). */
  const poseTrans = (a, b) => matToPose(matMul(poseToMat(a), poseToMat(b)));
  /** URScript's pose_inv(a). */
  const poseInv = (a) => matToPose(matInv(poseToMat(a)));
  /** Flange pose at joints q from PolyScope's DH table (getKinematicInfo(): [{DHTheta, DHa, DHd, DHAlpha}] × 6). */
  function flangeMat(dh, q) {
    let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    dh.forEach((j, i) => {
      const th = q[i] + (j.DHTheta || 0), ct = Math.cos(th), st = Math.sin(th);
      const ca = Math.cos(j.DHAlpha), sa = Math.sin(j.DHAlpha);
      T = matMul(T, [[ct, -st * ca, st * sa, j.DHa * ct], [st, ct * ca, -ct * sa, j.DHa * st], [0, sa, ca, j.DHd], [0, 0, 0, 1]]);
    });
    return T;
  }
  /** Where the fingertips are for a flange pose: tipMm along the flange's +Z. */
  const fingertip = (flange, tipMm) => poseTrans(flange, [0, 0, tipMm / 1000, 0, 0, 0]).slice(0, 3);

  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => Math.hypot(a[0], a[1], a[2]);
  const unit = (a) => { const n = norm(a); return [a[0] / n, a[1] / n, a[2] / n]; };

  /** The pick area's plane from three fingertip touches (corner, along one edge, the far
   * side): [x, y, z, rx, ry, rz, sizeX_m, sizeY_m] — origin at p0, X along p0→p1, the
   * normal up (n.z ≥ 0), sizes signed along X / Y (the UI shows |size|). Null when the
   * points are too close or in a line (PoseMath.plane in the PolyScope 5 node). */
  function plane(p0, p1, p2) {
    if (![p0, p1, p2].every((p) => finite(p, 3))) return null;
    const e1 = sub(p1, p0), e2 = sub(p2, p0);
    if (norm(e1) < 1e-4) return null;
    const x = unit(e1);
    let n = cross(x, e2);
    if (norm(n) < 1e-6) return null;
    n = unit(n);
    if (n[2] < 0) n = [-n[0], -n[1], -n[2]];
    const y = cross(n, x);
    // Y = n × X; with n flipped, X × Y' must still equal n: re-derive x from y and n
    const xx = cross(y, n);
    const R = [[xx[0], y[0], n[0], p0[0]], [xx[1], y[1], n[1], p0[1]], [xx[2], y[2], n[2], p0[2]], [0, 0, 0, 1]];
    const pose = matToPose(R);
    return [...pose, dot(e1, xx), dot(e2, y)];
  }
  /** How far the plane's normal leans from the base Z (deg): the table is flat, so a tilt is a bad touch. */
  function tiltDeg(pl) {
    const R = poseToMat(pl);
    return (Math.acos(Math.min(1, Math.abs(R[2][2]))) * 180) / Math.PI;
  }

  // -- the application node (cockpit URL, areas, reach) ---------------------------------------

  /** The saved Cockpit field as an absolute base URL (the presenter's page host completes
   * shorthand: ":7621" → http://<pageHost>:7621, "host" → http://host:7621). */
  function cockpitBase(saved, pageHost) {
    const raw = (saved ? String(saved) : "").trim().replace(/\/+$/, "");
    const host = pageHost || "http://127.0.0.1";
    if (!raw) return `${host}:${DEFAULT_COCKPIT_PORT}`;
    const port = /^:?(\d{1,5})$/.exec(raw);
    if (port) return `${host}:${port[1]}`;
    if (/^[a-z][a-z0-9+.-]*:\/\//i.test(raw)) return raw;
    const withScheme = `http://${raw}`;
    try {
      const u = new URL(withScheme);
      if (!u.port && u.pathname === "/") return `${withScheme}:${DEFAULT_COCKPIT_PORT}`;
    } catch (e) { /* problem() names the bad value */ }
    return withScheme;
  }
  /** The host of a base URL ("" if none). */
  function hostOf(base) {
    try { return new URL(base).hostname.replace(/^\[|\]$/g, ""); } catch (e) { return ""; }
  }

  /** The application node's areas, each with its plane (or null) — what the picture points refer to by index. */
  function areasOf(app) {
    const list = app && Array.isArray(app.areas) ? app.areas : [];
    return list.slice(0, MAX_AREAS).map((a, i) => {
      const pl = a ? plane(a.p0, a.p1, a.p2) : null;
      return { name: (a && a.name) || `Area ${i + 1}`, p0: a && a.p0, p1: a && a.p1, p2: a && a.p2, plane: pl };
    });
  }

  function newNodeId() {
    const c = root.crypto && typeof root.crypto.getRandomValues === "function" ? root.crypto : null;
    const bytes = new Uint8Array(3);
    if (c) c.getRandomValues(bytes); else for (let i = 0; i < 3; i++) bytes[i] = Math.floor(Math.random() * 256);
    return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
  }

  // -- the settings one run of the node needs ----------------------------------------------------

  /** Everything the script needs, from the program node's parameters and the application
   * node (cockpit URL, areas, tip, reach, robot model). */
  function settings(params, app, pageHost) {
    const p = params || {};
    const a = app || {};
    const base = cockpitBase(a.cockpitUrl, pageHost);
    const areas = areasOf(a);
    const reach = reachLimits(a.robotModel, a.reachInnerMm, a.reachOuterMm);
    const points = (Array.isArray(p.points) ? p.points : []).map((pt) => {
      const idx = pt && Number.isInteger(pt.area) ? pt.area : -1;
      const area = idx >= 0 && idx < areas.length ? areas[idx] : null;
      const pl = area && area.plane;
      return {
        joints: pt && Array.isArray(pt.q) ? pt.q.map(Number) : null,
        area: idx,
        plane: pl ? pl.slice(0, 6) : null,
        areaXmm: pl ? pl[6] * 1000 : 0,
        areaYmm: pl ? pl[7] * 1000 : 0,
      };
    });
    return {
      host: hostOf(base),
      cockpitSet: !!(a.cockpitUrl && String(a.cockpitUrl).trim()),
      port: Number.isInteger(p.pickPort) ? p.pickPort : DEFAULT_PICK_PORT,
      nodeId: typeof p.nodeId === "string" ? p.nodeId : "",
      points,
      values: values(p.values),
      orderFirst: p.orderFirst || "LR",
      orderRows: p.orderRows || "FB",
      gripper: p.gripper || "robotiq",
      popupOnFail: p.popupOnFail !== false,
      reachMinM: reach ? reach.min : 0,
      reachMaxM: reach ? reach.max : 0,
      foundVariable: (p.foundVariable && p.foundVariable.name) || FOUND_VARIABLE,
      locVariable: (p.locVariable && p.locVariable.name) || LOC_VARIABLE,
    };
  }

  const horizontal = (o) => o === "LR" || o === "RL";
  const isOrder = (first, rows) => ORDERS.includes(first) && ORDERS.includes(rows) && horizontal(first) !== horizontal(rows);
  const words = (o) => (o === "LR" ? "left to right" : o === "RL" ? "right to left" : o === "FB" ? "front to back" : "back to front");
  const orderText = (first, rows) => `${words(first)}, rows ${words(rows)}`;
  const n = (s, key) => s.values[key];
  function partText(s) {
    const l = Math.max(n(s, "partLengthMm"), n(s, "partWidthMm")), w = Math.min(n(s, "partLengthMm"), n(s, "partWidthMm"));
    return `${num(l)} x ${num(w)} x ${num(n(s, "partHeightMm"))} mm +-${num(n(s, "partTolPct"))} %`;
  }
  const ownGripperNodes = (s) => s.gripper === "children";

  /** Why the node cannot generate a program yet, or null when it can. */
  function problem(s) {
    if (!s.cockpitSet || !s.host) return "set the camera computer's address in Application → Perceptronic";
    if (!/^[A-Za-z0-9.:\-]+$/.test(s.host)) return `the camera computer's host "${s.host}" is not an address`;
    if (!(s.port >= 1 && s.port <= 65535)) return "the pick port must be 1..65535";
    if (!/^[0-9a-f]{1,12}$/.test(s.nodeId)) return "the node has no identity yet - open it once";
    if (s.points.length === 0) return "add a picture point: move the arm where the camera sees the parts and tap Add";
    if (s.points.length > MAX_POINTS) return `at most ${MAX_POINTS} picture points`;
    for (let i = 0; i < s.points.length; i++) {
      const p = s.points[i];
      if (!finite(p.joints, 6)) return `picture point ${i + 1} is not a joint position`;
      if (p.area >= 0 && !p.plane) return `picture point ${i + 1}'s pick area is not taught - teach it in Application → Perceptronic`;
      if (p.plane && (!finite(p.plane, 6) || !(Math.abs(p.areaXmm) >= 5 && Math.abs(p.areaYmm) >= 5))) {
        return `picture point ${i + 1}'s pick area is broken - re-teach it in Application → Perceptronic`;
      }
    }
    for (const k of NUMBERS) {
      const v = n(s, k.key);
      if (!(v >= k.min && v <= k.max)) return `${k.label.toLowerCase()} must be ${num(k.min)}..${num(k.max)} ${k.unit}`;
    }
    if (n(s, "gripBelowTopMm") > n(s, "partHeightMm") - 2) {
      return `the grip (${num(n(s, "gripBelowTopMm"))} mm below the top) would put the fingertips on the table: the part is ${num(n(s, "partHeightMm"))} mm tall`;
    }
    const shortSide = Math.min(n(s, "partLengthMm"), n(s, "partWidthMm"));
    if (shortSide > n(s, "strokeMm") - 6) {
      return `the part's short side (${num(shortSide)} mm) needs fingers that open wider than ${num(n(s, "strokeMm"))} mm`;
    }
    if (!isOrder(s.orderFirst, s.orderRows)) return "the pick order must be one horizontal and one vertical direction";
    if (!GRIPPERS.includes(s.gripper)) return `unknown gripper "${s.gripper}"`;
    if (!(s.reachMinM >= 0 && s.reachMinM < 3 && s.reachMaxM >= 0 && s.reachMaxM <= 3 && (s.reachMaxM === 0 || s.reachMaxM > s.reachMinM))) {
      return "the reach limits are not valid (Application → Perceptronic → Reach)";
    }
    if (!/^[A-Za-z_][A-Za-z0-9_]*$/.test(s.foundVariable) || !/^[A-Za-z_][A-Za-z0-9_]*$/.test(s.locVariable)) {
      return "the result variable names are not valid";
    }
    return null;
  }

  function pose(p) {
    return `p[${p.slice(0, 6).map((v) => Number(v).toFixed(5)).join(", ")}]`;
  }
  function joints(q) {
    return `[${q.map((v) => Number(v).toFixed(6)).join(", ")}]`;
  }

  /** " part=50x30x30 tol=25 order=LR,FB …" for picture point i (0-based; -1: none): exactly
   * what the pick server's parse_options reads, and what the teach screen asks with. */
  function tokens(s, i) {
    const l = Math.max(n(s, "partLengthMm"), n(s, "partWidthMm")), w = Math.min(n(s, "partLengthMm"), n(s, "partWidthMm"));
    let b = ` part=${num(l)}x${num(w)}x${num(n(s, "partHeightMm"))}`;
    b += ` tol=${num(n(s, "partTolPct"))}`;
    b += ` order=${s.orderFirst},${s.orderRows}`;
    b += ` grip=${num(n(s, "gripBelowTopMm"))} stroke=${num(n(s, "strokeMm"))}`;
    if (s.reachMinM > 0 || s.reachMaxM > 0) b += ` reach=${s.reachMinM.toFixed(3)},${s.reachMaxM.toFixed(3)}`;
    const p = i >= 0 && i < s.points.length ? s.points[i] : null;
    if (p && p.plane) b += ` plane=${pose(p.plane)} area=${num(p.areaXmm)}x${num(p.areaYmm)}`;
    b += ` node=${s.nodeId}`;
    if (i >= 0) b += ` loc=${i + 1}`;
    b += ` locs=${Math.max(1, s.points.length)} proto=2`;
    return b;
  }

  // -- the script ------------------------------------------------------------------------------

  /** One stage report: textmsg for the Log tab and a LOG line to the pick server. */
  function say(s, indent, text, value) {
    const v = value == null ? '""' : value;
    s.push(`${indent}textmsg("Perceptronic Pick: ${text}", ${v})`);
    const line = value == null ? `"LOG ${text}"` : `str_cat("LOG ${text}", to_str(${value}))`;
    s.push(`${indent}socket_send_line(${line}, "${SOCKET}")`);
  }
  function read16(s, indent) {
    s.push(`${indent}rs_r = socket_read_ascii_float(16, "${SOCKET}", 10)`);
    s.push(`${indent}rs_st = -8`);
    s.push(`${indent}if rs_r[0] == 16:`);
    s.push(`${indent}  rs_st = rs_r[1]`);
    s.push(`${indent}end`);
  }
  function reasons(s, indent) {
    s.push(`${indent}rs_why = str_cat("pick server status ", to_str(rs_st))`);
    REASONS.forEach(([code, text], i) => {
      s.push(`${indent}${i === 0 ? "if" : "elif"} rs_st == ${code}:`);
      s.push(`${indent}  rs_why = "${text}"`);
    });
    s.push(`${indent}end`);
    s.push(`${indent}if (rs_st == -3) or (rs_st == -8) or (rs_st == -9):`);
    s.push(`${indent}  rs_ok = False`);
    s.push(`${indent}end`);
  }
  function rq(s, indent, name, value) {
    s.push(`${indent}socket_send_line("SET ${name} ${value}", "${RQ_SOCKET}")`);
    s.push(`${indent}rs_ack = socket_read_string("${RQ_SOCKET}", timeout=2.0)`);
  }
  function gripperOpen(st, s, indent) {
    if (st.gripper === "robotiq") { rq(s, indent, "POS", 0); rq(s, indent, "GTO", 1); }
    else if (st.gripper === "digital") s.push(`${indent}set_standard_digital_out(${Math.round(n(st, "gripperDo"))}, False)`);
  }
  function gripperStart(st, s, indent) {
    if (st.gripper === "robotiq") {
      s.push(`${indent}rs_ok = socket_open("127.0.0.1", 63352, "${RQ_SOCKET}")`);
      s.push(`${indent}if rs_ok:`);
      s.push(`${indent}  socket_send_line("GET ACT", "${RQ_SOCKET}")`);
      s.push(`${indent}  rs_act = socket_read_string("${RQ_SOCKET}", timeout=2.0)`);
      s.push(`${indent}  if str_find(rs_act, "1") < 0:`);
      say(s, `${indent}    `, "activating the gripper", null);
      rq(s, `${indent}    `, "ACT", 1);
      s.push(`${indent}    rs_t = 0`);
      s.push(`${indent}    while rs_t < 40:`);
      s.push(`${indent}      socket_send_line("GET STA", "${RQ_SOCKET}")`);
      s.push(`${indent}      if str_find(socket_read_string("${RQ_SOCKET}", timeout=2.0), "3") >= 0:`);
      s.push(`${indent}        rs_t = 40`);
      s.push(`${indent}      else:`);
      s.push(`${indent}        sleep(0.25)`);
      s.push(`${indent}        rs_t = rs_t + 1`);
      s.push(`${indent}      end`);
      s.push(`${indent}    end`);
      s.push(`${indent}  end`);
      rq(s, `${indent}  `, "SPE", Math.round(n(st, "gripperSpeedPct") * 2.55));
      rq(s, `${indent}  `, "FOR", Math.round(n(st, "gripperForcePct") * 2.55));
      gripperOpen(st, s, `${indent}  `);
      s.push(`${indent}else:`);
      s.push(`${indent}  rs_why = "no Robotiq gripper on this controller (is its URCap installed?) - or choose another gripper in the node's Options"`);
      s.push(`${indent}end`);
    } else if (st.gripper === "digital") {
      gripperOpen(st, s, indent);
    }
  }
  function gripperOpenWait(st, s, indent) {
    if (st.gripper !== "robotiq") return;
    s.push(`${indent}rs_t = 0`);
    s.push(`${indent}while rs_t < 30:`);
    s.push(`${indent}  socket_send_line("GET OBJ", "${RQ_SOCKET}")`);
    s.push(`${indent}  if str_find(socket_read_string("${RQ_SOCKET}", timeout=2.0), "3") >= 0:`);
    s.push(`${indent}    rs_t = 30`);
    s.push(`${indent}  else:`);
    s.push(`${indent}    sleep(0.1)`);
    s.push(`${indent}    rs_t = rs_t + 1`);
    s.push(`${indent}  end`);
    s.push(`${indent}end`);
  }
  function gripperClose(st, s, indent) {
    if (st.gripper === "robotiq") {
      rq(s, indent, "POS", 255);
      rq(s, indent, "GTO", 1);
      s.push(`${indent}sleep(0.2)`);
      s.push(`${indent}rs_obj = ""`);
      s.push(`${indent}rs_t = 0`);
      s.push(`${indent}while rs_t < 40:`);
      s.push(`${indent}  socket_send_line("GET OBJ", "${RQ_SOCKET}")`);
      s.push(`${indent}  rs_obj = socket_read_string("${RQ_SOCKET}", timeout=2.0)`);
      s.push(`${indent}  if str_find(rs_obj, "0") < 0:`);
      s.push(`${indent}    rs_t = 40`);
      s.push(`${indent}  else:`);
      s.push(`${indent}    sleep(0.1)`);
      s.push(`${indent}    rs_t = rs_t + 1`);
      s.push(`${indent}  end`);
      s.push(`${indent}end`);
      // OBJ 1/2: stopped on contact = holding it; 3: closed all the way = nothing between the fingers
      s.push(`${indent}rs_held = (str_find(rs_obj, "1") >= 0) or (str_find(rs_obj, "2") >= 0)`);
      say(s, indent, "gripper ", "rs_obj");
    } else {
      s.push(`${indent}set_standard_digital_out(${Math.round(n(st, "gripperDo"))}, True)`);
      s.push(`${indent}sleep(${f2(n(st, "gripperWaitS"))})`);
      s.push(`${indent}rs_held = True`);
    }
  }

  /** The lines up to the grip (the PolyScope 5 node's beforeChildren). Throws when problem(). */
  function beforeLines(st) {
    const why = problem(st);
    if (why) throw new Error(why);
    const s = [];
    const np = st.points.length;
    const budget = np + Math.round(n(st, "maxAttempts")) + 1;
    const k = n(st, "speedPct") / 100;
    const jv = f2(1.05 * k), ja = f2(1.4 * k), lv = f2(0.25 * k), la = f2(0.6 * k);
    const found = st.foundVariable, loc = st.locVariable;
    s.push(`# Perceptronic Pick ${VERSION} - camera computer ${st.host}:${st.port} - part ${partText(st)} - ${orderText(st.orderFirst, st.orderRows)} - ${np} picture point${np === 1 ? "" : "s"}`);
    s.push(`global ${found} = False`);
    s.push(`global ${loc} = 0`);
    s.push("rs_tcp0 = get_tcp_offset()");
    s.push("set_tcp(p[0, 0, 0, 0, 0, 0])");
    s.push('rs_why = "no answer from the camera computer within 10 s"');
    s.push("rs_held = False");
    s.push("rs_try = 0");
    s.push("rs_ok = True");
    s.push("rs_loc = 1");
    s.push('rs_tok = ""');
    s.push(`if socket_open("${st.host}", ${st.port}, "${SOCKET}"):`);
    say(s, "  ", "start, flange ", "get_actual_tcp_pose()");
    say(s, "  ", `looking for ${partText(st)}`, null);
    gripperStart(st, s, "  ");
    s.push(`  while (rs_ok) and (rs_try < ${budget}) and (${found} == False):`);
    s.push("    rs_try = rs_try + 1");
    s.push(`    socket_send_line(str_cat("NEXT ", str_cat(to_str(get_actual_tcp_pose()), " node=${st.nodeId} locs=${np} proto=2")), "${SOCKET}")`);
    read16(s, "    ");
    s.push("    if rs_r[0] == 16:");
    s.push("      rs_loc = floor(rs_r[11] + 0.5)");
    s.push("    end");
    s.push(`    if (rs_loc < 1) or (rs_loc > ${np}):`);
    s.push("      rs_loc = 1");
    s.push("    end");
    st.points.forEach((p, i) => {
      s.push(`    ${i === 0 ? "if" : "elif"} rs_loc == ${i + 1}:`);
      s.push(`      rs_tok = "${tokens(st, i)}"`);
    });
    s.push("    end");
    s.push("    if rs_st == 1:");
    say(s, "      ", "next part already seen, #", "rs_r[12]");
    s.push("    else:");
    st.points.forEach((p, i) => {
      s.push(`      ${i === 0 ? "if" : "elif"} rs_loc == ${i + 1}:`);
      s.push(`        movej(${joints(p.joints)}, a=${ja}, v=${jv})`);
    });
    s.push("      end");
    s.push(`      sleep(${f2(n(st, "settleS"))})`);
    say(s, "      ", "picture at point ", "rs_loc");
    s.push(`      socket_send_line(str_cat("FIND ", str_cat(to_str(get_actual_tcp_pose()), rs_tok)), "${SOCKET}")`);
    read16(s, "      ");
    say(s, "      ", "FIND status ", "rs_st");
    s.push("      if rs_st != 1:");
    reasons(s, "        ");
    s.push("      end");
    s.push("    end");
    s.push("    if rs_st == 1:");
    s.push("      rs_c = p[rs_r[2], rs_r[3], rs_r[4], 0, 0, 0]");
    s.push("      rs_top = p[rs_r[5], rs_r[6], rs_r[7], rs_r[8], rs_r[9], rs_r[10]]");
    s.push(`      rs_look = pose_trans(rs_top, p[0, 0, ${m(-n(st, "lookMm"))}, 0, 0, 0])`);
    s.push(`      socket_send_line(str_cat("LOOK ", str_cat(to_str(get_actual_tcp_pose()), str_cat(" ", to_str(rs_c)))), "${SOCKET}")`);
    s.push(`      rs_lk = socket_read_ascii_float(10, "${SOCKET}", 10)`);
    s.push("      if rs_lk[0] == 10:");
    s.push("        if rs_lk[1] == 1:");
    s.push("          rs_look = p[rs_lk[5], rs_lk[6], rs_lk[7], rs_lk[8], rs_lk[9], rs_lk[10]]");
    s.push("        end");
    s.push("      end");
    s.push("      if get_inverse_kin_has_solution(rs_look, get_actual_joint_positions()):");
    s.push(`        movej(get_inverse_kin(rs_look, get_actual_joint_positions()), a=${ja}, v=${jv})`);
    s.push(`        sleep(${f2(n(st, "settleS"))})`);
    say(s, "        ", "close look ", "rs_look");
    s.push("      else:");
    say(s, "        ", "the close look is out of reach - looking again from here", null);
    s.push("      end");
    s.push("      rs_leans = [0, 12, 24]");
    s.push("      rs_lean = 0");
    s.push("      rs_go = False");
    s.push("      rs_rs = -8");
    s.push("      while (rs_lean < 3) and (rs_go == False):");
    s.push(`        socket_send_line(str_cat("REFINE ", str_cat(to_str(get_actual_tcp_pose()), str_cat(" ", str_cat(to_str(rs_c), str_cat(rs_tok, str_cat(" lean=", to_str(rs_leans[rs_lean]))))))), "${SOCKET}")`);
    s.push(`        rs_r = socket_read_ascii_float(16, "${SOCKET}", 10)`);
    s.push("        rs_rs = -8");
    s.push("        if rs_r[0] == 16:");
    s.push("          rs_rs = rs_r[1]");
    s.push("        end");
    say(s, "        ", "REFINE status ", "rs_rs");
    s.push("        if rs_rs == 1:");
    s.push("          rs_c = p[rs_r[2], rs_r[3], rs_r[4], 0, 0, 0]");
    s.push("          rs_top = p[rs_r[5], rs_r[6], rs_r[7], rs_r[8], rs_r[9], rs_r[10]]");
    s.push(`          rs_hover = pose_trans(rs_top, p[0, 0, ${m(-n(st, "approachMm"))}, 0, 0, 0])`);
    s.push(`          rs_grip = pose_trans(rs_top, p[0, 0, ${m(n(st, "gripBelowTopMm"))}, 0, 0, 0])`);
    s.push(`          rs_lift = pose_trans(rs_top, p[0, 0, ${m(-n(st, "liftMm"))}, 0, 0, 0])`);
    s.push("          rs_q = get_actual_joint_positions()");
    s.push("          if get_inverse_kin_has_solution(rs_hover, rs_q) and get_inverse_kin_has_solution(rs_grip, rs_q) and get_inverse_kin_has_solution(rs_lift, rs_q):");
    s.push("            rs_go = True");
    s.push("          else:");
    say(s, "            ", "no IK for approach + grip + lift at lean ", "rs_leans[rs_lean]");
    s.push("          end");
    s.push("          rs_lean = rs_lean + 1");
    s.push("        else:");
    s.push("          rs_lean = 3");
    s.push("        end");
    s.push("      end");
    s.push("      if rs_go:");
    say(s, "        ", "approach ", "rs_hover");
    s.push(`        movel(rs_hover, a=${la}, v=${lv})`);
    gripperOpenWait(st, s, "        ");
    say(s, "        ", "down to the grip ", "rs_grip");
    s.push("        movel(rs_grip, a=0.3, v=0.05)");
    s.push("        rs_held = False");
    if (ownGripperNodes(st)) {
      s.push("        set_tcp(rs_tcp0)");
      say(s, "        ", "your gripper nodes", null);
    } else {
      gripperClose(st, s, "        ");
    }
    return s;
  }

  /** The lines from the lift to the end (the PolyScope 5 node's afterChildren). */
  function afterLines(st) {
    const s = [];
    const own = ownGripperNodes(st);
    const k = n(st, "speedPct") / 100;
    const found = st.foundVariable, loc = st.locVariable;
    if (own) {
      s.push("        set_tcp(p[0, 0, 0, 0, 0, 0])");
      s.push("        rs_held = True");
    }
    say(s, "        ", "lift", null);
    s.push(`        movel(rs_lift, a=0.5, v=${f2(Math.max(0.05, 0.15 * k))})`);
    s.push("        if rs_held:");
    s.push(`          global ${found} = True`);
    s.push(`          global ${loc} = rs_loc`);
    say(s, "          ", "picked at point ", "rs_loc");
    s.push("        else:");
    s.push('          rs_why = "the gripper closed on nothing"');
    say(s, "          ", "closed on nothing - trying the next part", null);
    if (!own) gripperOpen(st, s, "          ");
    s.push("        end");
    s.push("      elif rs_rs != 1:");
    s.push("        rs_st = rs_rs");
    reasons(s, "        ");
    s.push("      else:");
    s.push('        rs_why = "no approach the arm can reach (straight down or leaned to 24 deg)"');
    s.push("      end");
    s.push("    end");
    s.push("  end");
    s.push(`  if ${found} == False:`);
    s.push('    textmsg("Perceptronic Pick: no pick - ", rs_why)');
    s.push(`    socket_send_line(str_cat("LOG no pick - ", rs_why), "${SOCKET}")`);
    s.push("  end");
    s.push(`  socket_close("${SOCKET}")`);
    s.push("else:");
    s.push(`  rs_why = "no camera computer at ${st.host}:${st.port} - is it on, and is the address in Application > Perceptronic right?"`);
    s.push('  textmsg("Perceptronic Pick: ", rs_why)');
    s.push("end");
    if (st.gripper === "robotiq") s.push(`socket_close("${RQ_SOCKET}")`);
    s.push("set_tcp(rs_tcp0)");
    if (st.popupOnFail) {
      s.push(`if ${found} == False:`);
      s.push('  popup(str_cat("Perceptronic Pick: no pick - ", rs_why), "Perceptronic Pick", False, True, blocking=True)');
      s.push("end");
    }
    return s;
  }

  /**
   * What the node contributes, in PolyScope X's shape: the lines before the children, how
   * deep the children sit (PolyScope indents them by it) and the lines after. With a
   * gripper the node drives, the children are the routine after the pick, inside
   * `if <found>:` at the end; with "my nodes" they run at the grip, four blocks deep.
   */
  function script(st) {
    if (ownGripperNodes(st)) {
      return { before: beforeLines(st), childDepth: 4, after: afterLines(st) };
    }
    const before = beforeLines(st).concat(afterLines(st));
    before.push(`if ${st.foundVariable}:`);
    return { before, childDepth: 1, after: ["end"] };
  }

  /** The whole contribution as text with `children` (already indented lines) where the child nodes go — tests. */
  function render(st, children) {
    const sc = script(st);
    const pad = "  ".repeat(sc.childDepth);
    const kids = (children || []).map((l) => (l ? pad + l : l));
    return sc.before.concat(kids, sc.after).join("\n") + "\n";
  }

  /** The "After picture N" node's lines: its children run only for a pick from that picture point. */
  function afterPictureScript(locVariable, point) {
    const v = /^[A-Za-z_][A-Za-z0-9_]*$/.test(locVariable || "") ? locVariable : LOC_VARIABLE;
    return { before: [`if ${v} == ${Math.round(point)}:`], childDepth: 1, after: ["end"] };
  }

  // -- the drawings (the PolyScope 5 node's Diagrams.java, as SVG strings) --------------------------

  const C = { accent: "#1f5fbf", accentSoft: "#e6efff", ink: "#1f2a37", muted: "#5b6b7d", faint: "#c0c8d2", line: "#d5dce5",
    ok: "#1d9a5a", err: "#d64545", jaw: "#f0a500", card: "#ffffff", bg: "#f4f6f9", grey: "#3b4756" };
  const r1 = (v) => Math.round(v * 10) / 10;
  const escXml = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const svgOpen = (w, h, extra) => `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${w} ${h}" width="${w}" height="${h}"${extra || ""}>`;
  const text = (x, y, t, size, color, weight, anchor) =>
    `<text x="${r1(x)}" y="${r1(y)}" font-size="${size}" font-family="system-ui, sans-serif" font-weight="${weight || "normal"}" fill="${color}" text-anchor="${anchor || "middle"}" dominant-baseline="middle">${escXml(t)}</text>`;
  const orderKey = (dir, cell) => (dir === "LR" ? cell[0] : dir === "RL" ? -cell[0] : dir === "FB" ? -cell[1] : cell[1]);

  /** The pick number of each cell of a cols × rows grid (row 0 = the top of the picture) for
   * `first` within a row and `rowsDir` from row to row — perceptronics.volume.order_parts's rule. */
  function orderGrid(first, rowsDir, cols, rows) {
    const cells = [];
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) cells.push([c, r]);
    cells.sort((a, b) => orderKey(rowsDir, a) - orderKey(rowsDir, b) || orderKey(first, a) - orderKey(first, b));
    const out = Array.from({ length: rows }, () => new Array(cols).fill(0));
    cells.forEach((cell, i) => { out[cell[1]][cell[0]] = i + 1; });
    return out;
  }

  /** One pick-order choice: a 3 × 2 grid of parts with the numbers they'd get, and a path through them. */
  function svgOrderTile(first, rows, selected, w, h) {
    w = w || 62; h = h || 50;
    const grid = orderGrid(first, rows, 3, 2);
    const at = [];
    for (let r = 0; r < 2; r++) for (let c = 0; c < 3; c++) at[grid[r][c]] = [12 + (c * (w - 24)) / 2, 15 + r * (h - 30)];
    let out = svgOpen(w, h, ` role="img" aria-label="${escXml(orderText(first, rows))}"`);
    out += `<rect x="1" y="1" width="${w - 3}" height="${h - 3}" rx="12" fill="${selected ? C.accentSoft : C.card}" stroke="${selected ? C.accent : C.line}" stroke-width="${selected ? 2.2 : 1}"/>`;
    out += `<polyline fill="none" stroke="${selected ? C.accent : C.faint}" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round" points="${[1, 2, 3, 4, 5, 6].map((i) => `${r1(at[i][0])},${r1(at[i][1])}`).join(" ")}"/>`;
    for (let i = 1; i <= 6; i++) {
      const fill = i === 1 ? (selected ? C.accent : C.ink) : selected ? "#7ea6ec" : "#a8b3c0";
      out += `<circle cx="${r1(at[i][0])}" cy="${r1(at[i][1])}" r="8" fill="${fill}"/>` + text(at[i][0], at[i][1] + 0.5, String(i), 10, "#fff", "bold");
    }
    return out + "</svg>";
  }

  /** The part, drawn in proportion (isometric), with its length, width and height, and the jaws across the width. */
  function svgPart(lengthMm, widthMm, heightMm, w, h) {
    w = w || 140; h = h || 140;
    const l = Math.max(lengthMm, widthMm), wd = Math.min(lengthMm, widthMm), ht = heightMm;
    const cos30 = Math.cos(Math.PI / 6), sin30 = 0.5;
    const span = (l + wd) * cos30, tall = ht + (l + wd) * sin30;
    const k = Math.min((w - 30) / span, (h - 34) / tall);
    const ox = 15 + wd * cos30 * k, oy = h - 18;
    const base = [[0, 0], [l, 0], [l, wd], [0, wd]];
    const bot = base.map(([x, y]) => [ox + (x - y) * cos30 * k, oy - (x + y) * sin30 * k]);
    const top = bot.map(([x, y]) => [x, y - ht * k]);
    const poly = (pts, fill) => `<polygon points="${pts.map(([x, y]) => `${r1(x)},${r1(y)}`).join(" ")}" fill="${fill}" stroke="${C.accent}" stroke-width="1.6" stroke-linejoin="round"/>`;
    let out = svgOpen(w, h, ` role="img" aria-label="the part: ${num(l)} × ${num(wd)} × ${num(ht)} mm"`);
    out += poly([bot[0], bot[1], top[1], top[0]], "#cfe0fb") + poly([bot[3], bot[0], top[0], top[3]], "#b4cdf5") + poly([top[0], top[1], top[2], top[3]], "#e8f0fd");
    out += text((bot[0][0] + bot[1][0]) / 2 + 8, (bot[0][1] + bot[1][1]) / 2 + 12, num(l), 11.5, C.ink, "bold");
    out += text((bot[3][0] + bot[0][0]) / 2 - 10, (bot[3][1] + bot[0][1]) / 2 + 12, num(wd), 11.5, C.ink, "bold");
    out += text(bot[0][0] + 16, (bot[0][1] + top[0][1]) / 2 + 2, num(ht), 11.5, C.ink, "bold");
    // the jaws: against the two long faces (the fingers close across the width)
    const ex = cos30, ey = -sin30, len = l * k * 0.22;
    const fx = (bot[0][0] + bot[1][0] + top[0][0] + top[1][0]) / 4 + cos30 * 9, fy = (bot[0][1] + bot[1][1] + top[0][1] + top[1][1]) / 4 + sin30 * 9;
    const bx = (top[2][0] + top[3][0]) / 2 - cos30 * 9, by = (top[2][1] + top[3][1]) / 2 - sin30 * 9 - 4;
    const jaw = (x, y) => `<line x1="${r1(x - ex * len)}" y1="${r1(y - ey * len)}" x2="${r1(x + ex * len)}" y2="${r1(y + ey * len)}" stroke="${C.jaw}" stroke-width="4.5" stroke-linecap="round"/>`;
    return out + jaw(fx, fy) + jaw(bx, by) + "</svg>";
  }

  /** The approach from the side: the table, the part, the open fingers over it, the grip and the lift. */
  function svgApproach(approachMm, gripMm, liftMm, heightMm, widthMm, strokeMm, w, h) {
    w = w || 150; h = h || 170;
    const total = heightMm + Math.max(approachMm, liftMm) + 40;
    const k = Math.min((h - 26) / total, (w - 80) / Math.max(strokeMm + 20, widthMm + 20));
    const table = h - 14, cx = w / 2 - 6;
    const pw = widthMm * k, ph = heightMm * k, topY = table - ph;
    const tipY = topY - approachMm * k, half = (strokeMm * k) / 2, gripY = topY + gripMm * k, liftY = topY - liftMm * k;
    let out = svgOpen(w, h, ` role="img" aria-label="approach ${num(approachMm)} mm over the top, grip ${num(gripMm)} mm below it, lift ${num(liftMm)} mm"`);
    out += `<rect x="0" y="${r1(table)}" width="${w}" height="14" fill="#e6eaf0"/><line x1="0" y1="${r1(table)}" x2="${w}" y2="${r1(table)}" stroke="${C.faint}"/>`;
    out += `<rect x="${r1(cx - pw / 2)}" y="${r1(topY)}" width="${r1(pw)}" height="${r1(ph)}" fill="#cfe0fb" stroke="${C.accent}"/>`;
    // fingers, fully open, tips `approach` over the top
    out += `<rect x="${r1(cx - half - 7)}" y="${r1(tipY - 38)}" width="7" height="38" rx="3" fill="${C.grey}"/>`;
    out += `<rect x="${r1(cx + half)}" y="${r1(tipY - 38)}" width="7" height="38" rx="3" fill="${C.grey}"/>`;
    out += `<rect x="${r1(cx - half - 10)}" y="${r1(tipY - 50)}" width="${r1(2 * half + 20)}" height="12" rx="6" fill="${C.grey}"/>`;
    out += `<line x1="${r1(cx - half - 16)}" y1="${r1(gripY)}" x2="${r1(cx + half + 16)}" y2="${r1(gripY)}" stroke="${C.jaw}" stroke-width="1.6" stroke-dasharray="4 4"/>`;
    const dx = cx + half + 14;
    const dim = (y0, y1, t, color) =>
      `<g stroke="${color}" stroke-width="1.2"><line x1="${r1(dx)}" y1="${r1(y0)}" x2="${r1(dx)}" y2="${r1(y1)}"/><line x1="${r1(dx - 4)}" y1="${r1(y0)}" x2="${r1(dx + 4)}" y2="${r1(y0)}"/><line x1="${r1(dx - 4)}" y1="${r1(y1)}" x2="${r1(dx + 4)}" y2="${r1(y1)}"/></g>` +
      text(dx + 6, (y0 + y1) / 2, t, 11, color, "bold", "start");
    out += dim(tipY, topY, num(approachMm), C.accent) + dim(topY, gripY, num(gripMm), "#9a6a00");
    const ax = cx - half - 18;
    out += `<line x1="${r1(ax)}" y1="${r1(gripY)}" x2="${r1(ax)}" y2="${r1(liftY)}" stroke="${C.ok}" stroke-width="1.2"/><polygon points="${r1(ax - 5)},${r1(liftY + 8)} ${r1(ax + 5)},${r1(liftY + 8)} ${r1(ax)},${r1(liftY)}" fill="${C.ok}"/>`;
    out += text(Math.max(2, ax - 12), Math.max(12, liftY - 6), `lift ${num(liftMm)}`, 11, C.ok, "bold", "start");
    return out + "</svg>";
  }

  /** The four corners (x, y; m) of a taught area [pose(6), sizeX, sizeY] on the base XY plane. */
  function areaCorners(pl) {
    const M = poseToMat(pl.slice(0, 6));
    const sx = pl[6], sy = pl[7];
    return [[0, 0], [sx, 0], [sx, sy], [0, sy]].map(([u, v]) => [M[0][3] + u * M[0][0] + v * M[0][1], M[1][3] + u * M[1][0] + v * M[1][1]]);
  }

  /** The cell from above: the base, the ring the parts may be in (the reach limits), and every
   * taught pick area — so an area out of reach shows at once. `areas`: [{name, corners: [[x, y] × 4]}]. */
  function svgReachMap(baseR, minR, maxR, areas, highlight, w, h) {
    w = w || 240; h = h || 240;
    let extent = Math.max(maxR > 0 ? maxR : minR * 2, minR, 0.05) * 1.15;
    (areas || []).forEach((a) => a.corners.forEach(([x, y]) => { extent = Math.max(extent, Math.max(Math.abs(x), Math.abs(y)) * 1.1); }));
    const k = (Math.min(w, h) / 2 - 8) / extent, cx = w / 2, cy = h / 2;
    let out = svgOpen(w, h, ' role="img" aria-label="the cell from above: the reach ring and the pick areas"');
    out += `<rect x="0" y="0" width="${w}" height="${h}" rx="14" fill="${C.bg}"/>`;
    if (maxR > 0) out += `<circle cx="${cx}" cy="${cy}" r="${r1(maxR * k)}" fill="#e2f5eb" stroke="${C.ok}"/>`;
    out += `<circle cx="${cx}" cy="${cy}" r="${r1(minR * k)}" fill="#fde3e3" stroke="${C.err}" stroke-width="1.2" stroke-dasharray="5 4"/>`;
    const b = baseR * k;
    out += `<circle cx="${cx}" cy="${cy}" r="${r1(b)}" fill="${C.grey}"/>`;
    out += `<line x1="${cx}" y1="${cy}" x2="${r1(cx + b + 14)}" y2="${cy}" stroke="${C.err}" stroke-width="1.5"/><line x1="${cx}" y1="${cy}" x2="${cx}" y2="${r1(cy - b - 14)}" stroke="${C.ok}" stroke-width="1.5"/>`;
    out += text(cx + b + 20, cy, "X", 10, C.muted, "bold") + text(cx, cy - b - 20, "Y", 10, C.muted, "bold");
    (areas || []).forEach((a, i) => {
      const pts = a.corners.map(([x, y]) => [cx + x * k, cy - y * k]);
      const mx = pts.reduce((s, p) => s + p[0], 0) / 4, my = pts.reduce((s, p) => s + p[1], 0) / 4;
      out += `<polygon points="${pts.map(([x, y]) => `${r1(x)},${r1(y)}`).join(" ")}" fill="rgba(28,100,216,${i === highlight ? 0.35 : 0.18})" stroke="${C.accent}" stroke-width="${i === highlight ? 2.4 : 1.4}"/>`;
      out += text(mx, my, a.name || String(i + 1), 11, C.ink, "bold");
    });
    out += text(8, h - 8, `pick ${Math.round(minR * 1000)}–${maxR > 0 ? Math.round(maxR * 1000) : "∞"} mm from the base axis`, 10, C.muted, "normal", "start");
    return out + "</svg>";
  }

  root.PerceptronicPick = {
    orderGrid, svgOrderTile, svgPart, svgApproach, areaCorners, svgReachMap,
    APP_TYPE, PICK_TYPE, AFTER_TYPE, VERSION, DEFAULT_PICK_PORT, DEFAULT_COCKPIT_PORT, DEFAULT_TIP_MM,
    SOCKET, RQ_SOCKET, MAX_POINTS, MAX_AREAS, ORDERS, ORDER_TILES, GRIPPERS, GRIPPER_FIELDS, REASONS,
    NUMBERS, BY_KEY, FOUND_VARIABLE, LOC_VARIABLE,
    num, clamp, defaults, values, modelReach, reachLimits,
    poseToMat, matToPose, matMul, matInv, poseTrans, poseInv, flangeMat, fingertip, plane, tiltDeg,
    cockpitBase, hostOf, areasOf, newNodeId, settings, isOrder, horizontal, words, orderText, partText,
    problem, tokens, script, render, afterPictureScript,
  };
})(typeof self !== "undefined" ? self : globalThis);
