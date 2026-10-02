// 3D Pick — the program node's settings, the URScript it contributes and the pose math
// both its presenter and its behavior worker need. One file for both: the worker
// `importScripts` it, the page loads it with a <script> tag, tests run it under node. It is
// the PolyScope X port of the PolyScope 5 node's PickScript.java
// (urcap/perceptronic-ps5/src/.../PickScript.java): the same settings table, the same
// request options (`tokens`: what the pick server's `parse_options` reads) and the same
// script — so the cockpit's pick server (`:7622`, protocol 2) serves both robots.
//
// One run of the node is one move sequence with no children that positions the tool on one
// part and touches nothing else: the node does not drive the gripper (0.6.0, Nick
// 2026-10-01) — the program opens it before the node and closes it after. Inside the
// operator's program (Local mode, no Primary client): force the TCP to the flange, ask NEXT
// (a part already seen, or the picture point to survey from), movej there + FIND, the closer
// look unless it is switched off (LOOK/REFINE, leaned 0/12/24° while the controller's IK
// can't solve approach + grip), the approach with the fingertips `approachMm` over the top,
// and down to the grip. The result variable is True once the tool is there. Speeds have no
// settings.
(function (root) {
  "use strict";

  const APP_TYPE = "advin-perceptronic";
  const PICK_TYPE = "advin-perceptronic-pick";
  const VERSION = "0.6.0";
  const DEFAULT_PICK_PORT = 7622;
  const DEFAULT_COCKPIT_PORT = 7621;
  const DEFAULT_TIP_MM = 163; // Hand-E 157 mm + 6 mm adapter
  const SOCKET = "rs_pick";
  const MAX_POINTS = 12;
  const MAX_AREAS = 8;
  const ORDERS = ["LR", "RL", "FB", "BF"];
  const ORDER_TILES = [
    ["LR", "FB"], ["RL", "FB"], ["LR", "BF"], ["RL", "BF"],
    ["FB", "LR"], ["FB", "RL"], ["BF", "LR"], ["BF", "RL"],
  ];
  const SHAPES = ["box", "cyl"];
  const SPEED = 0.6; // of the node's own joint / linear limits
  const SETTLE_S = 0.2; // before each picture
  const MAX_ATTEMPTS = 3; // grasps per run
  const LOOK_STROKE_MM = 50; // how wide the open fingers are taken to be when the closer look keeps the part clear of them
  const KEEP_OUT_M = 0.15; // past the base's outer radius: perceptronics.volume.REACH_MARGIN_M
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
    [-10, "the only parts in view are out of the arm's reach"],
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
    ["approachMm", "approach", "Approach", "mm", 25, 5, 200, 5, "fingertips over the top"],
    ["gripBelowTopMm", "approach", "Grip depth", "mm", 15, 0, 60, 1, "fingertips below the top to grip"],
    ["fingerRoomMm", "approach", "Finger room", "mm", 20, 0, 100, 5, "clear space on each side of the part"],
  ].map(([key, section, label, unit, def, min, max, step, help]) => ({ key, section, label, unit, def, min, max, step, help }));
  const BY_KEY = {};
  NUMBERS.forEach((n) => { BY_KEY[n.key] = n; });

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

  // -- the application node (cockpit URL, areas, the robot's model) ---------------------------

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
   * node (cockpit URL, areas, tip, robot model). */
  function settings(params, app, pageHost) {
    const p = params || {};
    const a = app || {};
    const base = cockpitBase(a.cockpitUrl, pageHost);
    const areas = areasOf(a);
    // the arm by name: the pick server asks its kinematics which parts are in reach
    const arm = String(a.robotModel || "").trim();
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
      shape: p.shape === "cyl" ? "cyl" : "box",
      gripCheck: p.gripCheck !== false,
      gripLongSide: p.gripLongSide === true,
      closeLook: p.closeLook !== false,
      arm: /^[A-Za-z0-9]{1,8}$/.test(arm) ? arm : "",
      popupOnFail: p.popupOnFail !== false,
      foundVariable: (p.foundVariable && p.foundVariable.name) || FOUND_VARIABLE,
      locVariable: (p.locVariable && p.locVariable.name) || LOC_VARIABLE,
    };
  }

  const horizontal = (o) => o === "LR" || o === "RL";
  const isOrder = (first, rows) => ORDERS.includes(first) && ORDERS.includes(rows) && horizontal(first) !== horizontal(rows);
  const words = (o) => (o === "LR" ? "left to right" : o === "RL" ? "right to left" : o === "FB" ? "front to back" : "back to front");
  const orderText = (first, rows) => `${words(first)}, rows ${words(rows)}`;
  const n = (s, key) => s.values[key];
  const round = (s) => s.shape === "cyl";
  /** The footprint's long side (a cylinder: its diameter). */
  const longSide = (s) => (round(s) ? n(s, "partLengthMm") : Math.max(n(s, "partLengthMm"), n(s, "partWidthMm")));
  /** The side the fingers close across (a cylinder: its diameter). */
  const shortSide = (s) => (round(s) ? n(s, "partLengthMm") : Math.min(n(s, "partLengthMm"), n(s, "partWidthMm")));
  /** ASCII (it goes into the script). */
  function partText(s) {
    const size = round(s) ? `cylinder D${num(longSide(s))}` : `${num(longSide(s))} x ${num(shortSide(s))}`;
    return `${size} x ${num(n(s, "partHeightMm"))} mm +-${num(n(s, "partTolPct"))} %`;
  }
  /** For the screens: "50 × 30 × 30 mm" / "Ø40 × 30 mm". */
  function partWords(s) {
    const size = round(s) ? `Ø${num(longSide(s))}` : `${num(longSide(s))} × ${num(shortSide(s))}`;
    return `${size} × ${num(n(s, "partHeightMm"))} mm`;
  }

  /** Why the node cannot generate a program yet, or null when it can. */
  function problem(s) {
    if (!s.cockpitSet || !s.host) return "set the camera computer's address in Application → Perceptronic";
    if (!/^[A-Za-z0-9.:\-]+$/.test(s.host)) return `the camera computer's host "${s.host}" is not an address`;
    if (!(s.port >= 1 && s.port <= 65535)) return "the pick port must be 1..65535";
    if (!/^[0-9a-f]{1,12}$/.test(s.nodeId)) return "the node has no identity yet - open it once";
    if (s.points.length === 0) return "add a picture point: move the arm where the camera sees the parts and tap +";
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
    if (!SHAPES.includes(s.shape)) return `unknown part shape "${s.shape}"`;
    if (s.arm && !/^[A-Za-z0-9]{1,8}$/.test(s.arm)) return `the robot model "${s.arm}" is not a name`;
    if (!isOrder(s.orderFirst, s.orderRows)) return "the pick order must be one horizontal and one vertical direction";
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
    let b = ` part=${num(longSide(s))}x${num(shortSide(s))}x${num(n(s, "partHeightMm"))}`;
    b += ` tol=${num(n(s, "partTolPct"))}`;
    if (round(s)) b += " shape=cyl";
    b += ` order=${s.orderFirst},${s.orderRows}`;
    b += ` grip=${num(n(s, "gripBelowTopMm"))}`;
    b += ` approach=${num(n(s, "approachMm"))}`;
    b += ` gripcheck=${s.gripCheck ? 1 : 0} room=${num(n(s, "fingerRoomMm"))}`;
    if (s.gripLongSide && !round(s)) b += " across=long";
    if (s.arm) b += ` arm=${s.arm}`;
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
    s.push(`${indent}textmsg("3D Pick: ${text}", ${v})`);
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
  /** The survey: to picture point rs_loc, settle, FIND. */
  function survey(st, s, ind, ja, jv) {
    st.points.forEach((p, i) => {
      s.push(`${ind}${i === 0 ? "if" : "elif"} rs_loc == ${i + 1}:`);
      s.push(`${ind}  movej(${joints(p.joints)}, a=${ja}, v=${jv})`);
    });
    s.push(`${ind}end`);
    s.push(`${ind}sleep(${f2(SETTLE_S)})`);
    say(s, ind, "survey at point ", "rs_loc");
    s.push(`${ind}socket_send_line(str_cat("FIND ", str_cat(to_str(get_actual_tcp_pose()), rs_tok)), "${SOCKET}")`);
    read16(s, ind);
    say(s, ind, "FIND status ", "rs_st");
    s.push(`${ind}if rs_st != 1:`);
    reasons(s, `${ind}  `);
    s.push(`${ind}end`);
  }

  /** The node's whole contribution, line by line (PickScript.lines() in the PolyScope 5 node). Throws when problem(). */
  function lines(st) {
    const why = problem(st);
    if (why) throw new Error(why);
    const s = [];
    const np = st.points.length;
    const budget = np + MAX_ATTEMPTS + 1;
    const jv = f2(1.05 * SPEED), ja = f2(1.4 * SPEED), lv = f2(0.25 * SPEED), la = f2(0.6 * SPEED);
    const found = st.foundVariable, loc = st.locVariable;
    s.push(`# 3D Pick ${VERSION} - camera computer ${st.host}:${st.port} - part ${partText(st)} - ${orderText(st.orderFirst, st.orderRows)} - ${np} picture point${np === 1 ? "" : "s"}${st.closeLook ? "" : " - no closer look"}${st.gripCheck ? ` - finger room ${num(n(st, "fingerRoomMm"))} mm` : " - no grip check"}${st.gripLongSide && !round(st) ? " - across the long side" : ""}`);
    s.push(`global ${found} = False`);
    s.push(`global ${loc} = 0`);
    s.push("rs_tcp0 = get_tcp_offset()");
    s.push("set_tcp(p[0, 0, 0, 0, 0, 0])");
    s.push('rs_why = "no answer from the camera computer within 10 s"');
    s.push("rs_try = 0");
    s.push("rs_ok = True");
    s.push("rs_loc = 1");
    s.push('rs_tok = ""');
    s.push(`if socket_open("${st.host}", ${st.port}, "${SOCKET}"):`);
    say(s, "  ", "start, flange ", "get_actual_tcp_pose()");
    say(s, "  ", `looking for ${partText(st)}`, null);
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
    if (st.closeLook) {
      s.push("    if rs_st == 1:");
      say(s, "      ", "next part already seen, #", "rs_r[12]");
      s.push("    else:");
      survey(st, s, "      ", ja, jv);
      s.push("    end");
    } else {
      // every part is measured from its picture point: the parts queued at the last one were
      // seen from there too, but the arm is no longer where it could look again
      survey(st, s, "    ", ja, jv);
    }
    s.push("    if rs_st == 1:");
    s.push("      rs_c = p[rs_r[2], rs_r[3], rs_r[4], 0, 0, 0]");
    s.push("      rs_top = p[rs_r[5], rs_r[6], rs_r[7], rs_r[8], rs_r[9], rs_r[10]]");
    if (st.closeLook) {
      s.push(`      socket_send_line(str_cat("LOOK ", str_cat(to_str(get_actual_tcp_pose()), str_cat(" ", str_cat(to_str(rs_c), " stroke=${num(LOOK_STROKE_MM)}")))), "${SOCKET}")`);
      s.push(`      rs_lk = socket_read_ascii_float(10, "${SOCKET}", 10)`);
      s.push("      rs_see = False");
      s.push("      rs_look = rs_top");
      s.push("      if rs_lk[0] == 10:");
      s.push("        if rs_lk[1] == 1:");
      s.push("          rs_look = p[rs_lk[5], rs_lk[6], rs_lk[7], rs_lk[8], rs_lk[9], rs_lk[10]]");
      s.push("          rs_see = True");
      s.push("        end");
      s.push("      end");
      s.push("      if rs_see:");
      s.push("        rs_see = get_inverse_kin_has_solution(rs_look, get_actual_joint_positions())");
      s.push("      end");
      s.push("      if rs_see:");
      s.push(`        movej(get_inverse_kin(rs_look, get_actual_joint_positions()), a=${ja}, v=${jv})`);
      s.push(`        sleep(${f2(SETTLE_S)})`);
      say(s, "        ", "closer look ", "rs_look");
      s.push("      else:");
      say(s, "        ", "no closer look from here - measuring again where the arm is", null);
      s.push("      end");
    }
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
    s.push("          rs_q = get_actual_joint_positions()");
    s.push("          if get_inverse_kin_has_solution(rs_hover, rs_q) and get_inverse_kin_has_solution(rs_grip, rs_q):");
    s.push("            rs_go = True");
    s.push("          else:");
    say(s, "            ", "no IK for approach + grip at lean ", "rs_leans[rs_lean]");
    s.push("          end");
    s.push("          rs_lean = rs_lean + 1");
    s.push("        else:");
    s.push("          rs_lean = 3");
    s.push("        end");
    s.push("      end");
    s.push("      if rs_go:");
    say(s, "        ", "approach ", "rs_hover");
    s.push(`        movel(rs_hover, a=${la}, v=${lv})`);
    say(s, "        ", "down to the grip ", "rs_grip");
    s.push("        movel(rs_grip, a=0.3, v=0.05)");
    s.push(`        global ${found} = True`);
    s.push(`        global ${loc} = rs_loc`);
    say(s, "        ", "at the grip, part from point ", "rs_loc");
    s.push("      elif rs_rs != 1:");
    s.push("        rs_st = rs_rs");
    reasons(s, "        ");
    s.push("      else:");
    s.push('        rs_why = "the controller has no joint solution for the approach (straight down or leaned to 24 deg)"');
    s.push("      end");
    s.push("    end");
    s.push("  end");
    s.push(`  if ${found} == False:`);
    s.push('    textmsg("3D Pick: no pick - ", rs_why)');
    s.push(`    socket_send_line(str_cat("LOG no pick - ", rs_why), "${SOCKET}")`);
    s.push("  end");
    s.push(`  socket_close("${SOCKET}")`);
    s.push("else:");
    s.push(`  rs_why = "no camera computer at ${st.host}:${st.port} - is it on, and is the address in Application > Perceptronic right?"`);
    s.push('  textmsg("3D Pick: ", rs_why)');
    s.push("end");
    s.push("set_tcp(rs_tcp0)");
    if (st.popupOnFail) {
      s.push(`if ${found} == False:`);
      s.push('  popup(str_cat("3D Pick: no pick - ", rs_why), "3D Pick", False, True, blocking=True)');
      s.push("end");
    }
    return s;
  }

  /**
   * What the node contributes, in PolyScope X's shape. The node has no children (0.5.0): every
   * line comes before where they would be, at the node's own depth, and nothing after.
   */
  function script(st) {
    return { before: lines(st), childDepth: 0, after: [] };
  }

  /** The whole contribution as one text — tests. */
  function render(st) {
    return lines(st).join("\n") + "\n";
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

  /** The part, drawn in proportion with its dimensions: a box (isometric), or a cylinder standing on its end. */
  function svgPart(lengthMm, widthMm, heightMm, cylinder, w, h) {
    w = w || 260; h = h || 200;
    const ht = heightMm;
    if (cylinder) {
      const d = lengthMm;
      const k = Math.min((w - 130) / d, (h - 50) / (ht + d * 0.35), 4);
      const rx = (d * k) / 2, ry = (d * k * 0.35) / 2, hh = ht * k;
      const cx = w / 2 - 14, top = (h - hh - 2 * ry) / 2 + ry, bot = top + hh;
      let out = svgOpen(w, h, ` role="img" aria-label="the part: a cylinder Ø${num(d)} × ${num(ht)} mm"`);
      out += `<path d="M ${r1(cx - rx)} ${r1(top)} L ${r1(cx - rx)} ${r1(bot)} A ${r1(rx)} ${r1(ry)} 0 0 0 ${r1(cx + rx)} ${r1(bot)} L ${r1(cx + rx)} ${r1(top)} Z" fill="#b4cdf5" stroke="${C.accent}" stroke-width="1.6" stroke-linejoin="round"/>`;
      out += `<ellipse cx="${r1(cx)}" cy="${r1(top)}" rx="${r1(rx)}" ry="${r1(ry)}" fill="#e8f0fd" stroke="${C.accent}" stroke-width="1.6"/>`;
      out += text(cx, top, `Ø ${num(d)}`, 12, C.ink, "bold");
      out += text(cx + rx + 22, (top + bot) / 2, num(ht), 12, C.ink, "bold");
      return out + "</svg>";
    }
    const l = Math.max(lengthMm, widthMm), wd = Math.min(lengthMm, widthMm);
    const cos30 = Math.cos(Math.PI / 6), sin30 = 0.5;
    const span = (l + wd) * cos30, tall = ht + (l + wd) * sin30;
    const k = Math.min((w - 110) / span, (h - 44) / tall, 4);
    const ox = (w - span * k) / 2 + wd * cos30 * k, oy = h - 22 - (h - 44 - tall * k) / 2;
    const base = [[0, 0], [l, 0], [l, wd], [0, wd]];
    const bot = base.map(([x, y]) => [ox + (x - y) * cos30 * k, oy - (x + y) * sin30 * k]);
    const top = bot.map(([x, y]) => [x, y - ht * k]);
    const poly = (pts, fill) => `<polygon points="${pts.map(([x, y]) => `${r1(x)},${r1(y)}`).join(" ")}" fill="${fill}" stroke="${C.accent}" stroke-width="1.6" stroke-linejoin="round"/>`;
    let out = svgOpen(w, h, ` role="img" aria-label="the part: ${num(l)} × ${num(wd)} × ${num(ht)} mm"`);
    out += poly([bot[0], bot[1], top[1], top[0]], "#cfe0fb") + poly([bot[3], bot[0], top[0], top[3]], "#b4cdf5") + poly([top[0], top[1], top[2], top[3]], "#e8f0fd");
    out += text((bot[0][0] + bot[1][0]) / 2 + 10, (bot[0][1] + bot[1][1]) / 2 + 13, num(l), 12, C.ink, "bold");
    out += text((bot[3][0] + bot[0][0]) / 2 - 12, (bot[3][1] + bot[0][1]) / 2 + 13, num(wd), 12, C.ink, "bold");
    out += text(bot[1][0] + 18, (bot[1][1] + top[1][1]) / 2, num(ht), 12, C.ink, "bold");
    return out + "</svg>";
  }

  /** The approach from the side: the table, the part, the open fingers over it and how deep they grip. */
  function svgApproach(approachMm, gripMm, heightMm, widthMm, roomMm, w, h) {
    w = w || 260; h = h || 200;
    roomMm = roomMm || 0;
    const open = widthMm + 2 * Math.max(4, roomMm * 0.5); // the fingers, somewhere inside the room
    const total = heightMm + approachMm + 45;
    const k = Math.min((h - 26) / total, (w - 190) / (Math.max(open, widthMm + 2 * roomMm) + 20), 3);
    const table = h - 14, cx = w / 2 - 50;
    const pw = widthMm * k, ph = heightMm * k, topY = table - ph;
    const tipY = topY - approachMm * k, half = (open * k) / 2, gripY = topY + gripMm * k;
    let out = svgOpen(w, h, ` role="img" aria-label="approach ${num(approachMm)} mm over the top, grip ${num(gripMm)} mm below it"`);
    out += `<rect x="0" y="${r1(table)}" width="${w}" height="14" fill="#e6eaf0"/><line x1="0" y1="${r1(table)}" x2="${w}" y2="${r1(table)}" stroke="${C.faint}"/>`;
    out += `<rect x="${r1(cx - pw / 2)}" y="${r1(topY)}" width="${r1(pw)}" height="${r1(ph)}" fill="#cfe0fb" stroke="${C.accent}"/>`;
    // fingers, fully open, tips `approach` over the top
    out += `<rect x="${r1(cx - half - 7)}" y="${r1(tipY - 38)}" width="7" height="38" rx="3" fill="${C.grey}"/>`;
    out += `<rect x="${r1(cx + half)}" y="${r1(tipY - 38)}" width="7" height="38" rx="3" fill="${C.grey}"/>`;
    out += `<rect x="${r1(cx - half - 10)}" y="${r1(tipY - 50)}" width="${r1(2 * half + 20)}" height="12" rx="6" fill="${C.grey}"/>`;
    out += `<line x1="${r1(cx - half - 16)}" y1="${r1(gripY)}" x2="${r1(cx + half + 16)}" y2="${r1(gripY)}" stroke="${C.jaw}" stroke-width="1.6" stroke-dasharray="4 4"/>`;
    const dx = cx + half + 16;
    const dim = (y0, y1, t, color) =>
      `<g stroke="${color}" stroke-width="1.2"><line x1="${r1(dx)}" y1="${r1(y0)}" x2="${r1(dx)}" y2="${r1(y1)}"/><line x1="${r1(dx - 4)}" y1="${r1(y0)}" x2="${r1(dx + 4)}" y2="${r1(y0)}"/><line x1="${r1(dx - 4)}" y1="${r1(y1)}" x2="${r1(dx + 4)}" y2="${r1(y1)}"/></g>` +
      text(dx + 6, (y0 + y1) / 2, t, 11.5, color, "bold", "start");
    out += dim(tipY, topY, `approach ${num(approachMm)}`, C.accent) + dim(topY, gripY, `grip ${num(gripMm)}`, "#9a6a00");
    if (roomMm > 0) {
      // the finger room: the clear space wanted on each side of the part
      const rw = roomMm * k;
      out += `<rect x="${r1(cx - pw / 2 - rw)}" y="${r1(topY)}" width="${r1(rw)}" height="${r1(ph)}" fill="rgba(61,220,132,.25)"/>`;
      out += `<rect x="${r1(cx + pw / 2)}" y="${r1(topY)}" width="${r1(rw)}" height="${r1(ph)}" fill="rgba(61,220,132,.25)"/>`;
      out += text(cx + pw / 2 + 4, table - 8, `room ${num(roomMm)}`, 11.5, C.ok, "bold", "start");
    }
    return out + "</svg>";
  }

  /** The four corners (x, y; m) of a taught area [pose(6), sizeX, sizeY] on the base XY plane. */
  function areaCorners(pl) {
    const M = poseToMat(pl.slice(0, 6));
    const sx = pl[6], sy = pl[7];
    return [[0, 0], [sx, 0], [sx, sy], [0, sy]].map(([u, v]) => [M[0][3] + u * M[0][0] + v * M[0][1], M[1][3] + u * M[1][0] + v * M[1][1]]);
  }

  /** How far the farthest corner of a taught area is from the base axis (m). */
  const farthest = (corners) => corners.reduce((far, [x, y]) => Math.max(far, Math.hypot(x, y)), 0);

  /** The cell from above: the base, how far the arm reaches (its rated reach, named on its
   * circle), the keep-out round the base, and every taught pick area — where each area sits in
   * the arm's reach, at a glance. `reachR` 0: the model is unknown, no reach circle.
   * `areas`: [{name, corners: [[x, y] × 4]}]. */
  function svgReachMap(model, baseR, keepOutR, reachR, areas, highlight, w, h) {
    w = w || 300; h = h || 300;
    let extent = Math.max(reachR > 0 ? reachR : keepOutR * 2, keepOutR, 0.05) * 1.12;
    (areas || []).forEach((a) => a.corners.forEach(([x, y]) => { extent = Math.max(extent, Math.max(Math.abs(x), Math.abs(y)) * 1.1); }));
    const k = (Math.min(w, h - 34) / 2 - 6) / extent, cx = w / 2, cy = 12 + (h - 34) / 2;
    let out = svgOpen(w, h, ' role="img" aria-label="the cell from above: the arm\'s reach and the pick areas"');
    out += `<rect x="0" y="0" width="${w}" height="${h}" rx="14" fill="${C.bg}"/>`;
    if (reachR > 0) out += `<circle cx="${cx}" cy="${r1(cy)}" r="${r1(reachR * k)}" fill="#e2f5eb" stroke="${C.ok}" stroke-width="1.8"/>`;
    out += `<circle cx="${cx}" cy="${r1(cy)}" r="${r1(keepOutR * k)}" fill="#fde3e3" stroke="${C.err}" stroke-width="1.2" stroke-dasharray="5 4"/>`;
    const b = baseR * k;
    out += `<circle cx="${cx}" cy="${r1(cy)}" r="${r1(b)}" fill="${C.grey}"/>`;
    out += `<line x1="${cx}" y1="${r1(cy)}" x2="${r1(cx + b + 14)}" y2="${r1(cy)}" stroke="${C.err}" stroke-width="1.5"/><line x1="${cx}" y1="${r1(cy)}" x2="${cx}" y2="${r1(cy - b - 14)}" stroke="${C.ok}" stroke-width="1.5"/>`;
    out += text(cx + b + 20, cy, "X", 10, C.muted, "bold") + text(cx, cy - b - 20, "Y", 10, C.muted, "bold");
    (areas || []).forEach((a, i) => {
      const pts = a.corners.map(([x, y]) => [cx + x * k, cy - y * k]);
      const mx = pts.reduce((s, p) => s + p[0], 0) / 4, my = pts.reduce((s, p) => s + p[1], 0) / 4;
      out += `<polygon points="${pts.map(([x, y]) => `${r1(x)},${r1(y)}`).join(" ")}" fill="rgba(28,100,216,${i === highlight ? 0.35 : 0.18})" stroke="${C.accent}" stroke-width="${i === highlight ? 2.4 : 1.4}"/>`;
      out += text(mx, my, a.name || String(i + 1), 11, C.ink, "bold");
    });
    if (reachR > 0) {
      // the reach, named on its own circle (drawn last: an area never hides it)
      const label = `reach ${Math.round(reachR * 1000)} mm`, tw = label.length * 7 + 16, y = cy - reachR * k;
      out += `<rect x="${r1(cx - tw / 2)}" y="${r1(y - 10)}" width="${tw}" height="20" rx="10" fill="${C.ok}"/>` + text(cx, y + 0.5, label, 12, "#fff", "bold");
    }
    out += text(8, h - 9, reachR > 0 ? `green: the ${model || "arm"}'s reach · red: too near its base` : "robot model unknown: no reach to draw", 11, C.muted, "normal", "start");
    return out + "</svg>";
  }

  // -- what went wrong, for the operator (the PolyScope 5 node's Cockpit.advise) -------------------

  const CHECK_CABLES = "Cables: the camera computer is powered and its network cable is plugged in at both ends (link lights on)";
  const CHECK_FIREWALL = `Firewall: the camera computer must let this robot in on TCP ports ${DEFAULT_COCKPIT_PORT} and ${DEFAULT_PICK_PORT}`;
  const CHECK_USB = "Camera cable: the camera's USB cable seated at both ends, in a blue (USB 3) port — unplug it and plug it back in";
  const checkAddress = (base) => `IP address: is ${hostOf(base) || base} the camera computer's, and on the same network as the robot?`;

  /**
   * Why the camera computer gave no picture, as the operator should read it: one plain line,
   * then what to check — most likely first. `kind`: "silent" (no answer / timed out),
   * "refused" (the browser was refused or nothing listens), "cors" (it answers but turns
   * this page away), "nopicture" (HTTP 503: the camera), "outdated" (HTTP 404), "badurl".
   * `detail` is the long story, for the log (console), never for the screen.
   */
  function advise(kind, base, detail) {
    const host = hostOf(base) || base;
    const self = /^(localhost|127\.0\.0\.1|::1)$/.test(host);
    let summary, checks;
    if (kind === "badurl") {
      summary = `"${base}" is not an address.`;
      checks = [`Enter it as  http://<camera computer's IP>:${DEFAULT_COCKPIT_PORT}`];
    } else if (kind === "nopicture") {
      summary = "The camera computer is on, but its camera gives no picture.";
      checks = [CHECK_USB, "The picture comes back by itself a few seconds after the camera does"];
    } else if (kind === "outdated") {
      summary = "The camera computer answers, but its software is older than this URCap.";
      checks = ["Update the camera computer's software and restart it"];
    } else if (kind === "cors") {
      summary = `The camera computer at ${host} is running, but is not set up to serve this robot's screen.`;
      checks = ["Camera program: it must be started for this robot — ask whoever set it up (the detail is in the log)"];
    } else if (kind === "refused") {
      summary = self ? `Nothing on this robot itself answers at ${base}.` : `A computer answers at ${host}, but not the camera program.`;
      checks = [
        self ? "IP address: this address is the robot itself, not the camera computer — enter that computer's IP address" : checkAddress(base),
        "Camera program: is it running on the camera computer? Restart that computer if unsure",
        CHECK_FIREWALL,
      ];
    } else {
      summary = `No answer from the camera computer at ${host}.`;
      checks = [CHECK_CABLES, checkAddress(base), CHECK_FIREWALL];
    }
    return { summary, checks, detail: `${base} — ${detail || kind}`, text: [summary].concat(checks.map((c) => `• ${c}`)).join("\n") };
  }

  /** The rejected candidates the picture draws: nearly the part (a cockpit before 0.7.0 says nothing: all of them). */
  const nearMisses = (scene) => ((scene && scene.rejected) || []).filter((r) => r && r.near !== false);

  /** One line for the status: how many parts will be picked, how many nearly. */
  function sceneSummary(scene) {
    const parts = ((scene && scene.parts) || []).length, near = nearMisses(scene).length;
    if (!parts && !near) return "no part in view";
    const s = `${parts} part${parts === 1 ? "" : "s"} to pick`;
    return near ? `${s} · ${near} not (yellow, with why)` : s;
  }

  root.PerceptronicPick = {
    orderGrid, svgOrderTile, svgPart, svgApproach, areaCorners, farthest, svgReachMap,
    APP_TYPE, PICK_TYPE, VERSION, DEFAULT_PICK_PORT, DEFAULT_COCKPIT_PORT, DEFAULT_TIP_MM,
    SOCKET, MAX_POINTS, MAX_AREAS, ORDERS, ORDER_TILES, SHAPES, REASONS,
    SPEED, SETTLE_S, MAX_ATTEMPTS, LOOK_STROKE_MM, KEEP_OUT_M,
    NUMBERS, BY_KEY, FOUND_VARIABLE, LOC_VARIABLE,
    num, clamp, defaults, values, modelReach,
    poseToMat, matToPose, matMul, matInv, poseTrans, poseInv, flangeMat, fingertip, plane, tiltDeg,
    cockpitBase, hostOf, areasOf, newNodeId, settings, isOrder, horizontal, words, orderText, partText, partWords,
    round, longSide, shortSide, problem, tokens, lines, script, render,
    advise, nearMisses, sceneSummary,
  };
})(typeof self !== "undefined" ? self : globalThis);
