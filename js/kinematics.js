/**
 * kinematics.js — Four-Bar Mechanism Synthesis & Forward Kinematics
 *
 * This module implements:
 *   1. Forward kinematics (position analysis) for a planar four-bar linkage
 *   2. 4-point coupler curve synthesis (Freudenstein + geometric approach)
 *   3. 5-point coupler curve synthesis (Levenberg–Marquardt least-squares)
 *   4. Grashof condition check
 *   5. Newton-Raphson and Levenberg-Marquardt numerical solvers
 *
 * Coordinate convention:
 *   Ground pivot A₀ at origin, ground pivot B₀ at (d, 0).
 *   Link lengths: a = crank (A₀A), b = coupler (AB), c = rocker (B₀B), d = ground (A₀B₀).
 *   Coupler point P is defined by offset (p, q) in the coupler-link frame.
 *   θ₂ = input crank angle measured from A₀ to A.
 */

const Kinematics = (() => {
  "use strict";

  // ─────────────────────────────────────────────
  // CONSTANTS
  // ─────────────────────────────────────────────
  const DEG = Math.PI / 180;
  const TWO_PI = 2 * Math.PI;
  const EPS = 1e-10;

  // ─────────────────────────────────────────────
  // UTILITY: 2D VECTOR OPERATIONS
  // ─────────────────────────────────────────────
  const vec = {
    add: (a, b) => [a[0] + b[0], a[1] + b[1]],
    sub: (a, b) => [a[0] - b[0], a[1] - b[1]],
    scale: (a, s) => [a[0] * s, a[1] * s],
    dot: (a, b) => a[0] * b[0] + a[1] * b[1],
    cross: (a, b) => a[0] * b[1] - a[1] * b[0],
    len: (a) => Math.sqrt(a[0] * a[0] + a[1] * a[1]),
    dist: (a, b) => vec.len(vec.sub(a, b)),
    angle: (a) => Math.atan2(a[1], a[0]),
    rotate: (v, theta) => [
      v[0] * Math.cos(theta) - v[1] * Math.sin(theta),
      v[0] * Math.sin(theta) + v[1] * Math.cos(theta),
    ],
    norm: (a) => {
      const l = vec.len(a);
      return l > EPS ? [a[0] / l, a[1] / l] : [0, 0];
    },
  };

  // ─────────────────────────────────────────────
  // FORWARD KINEMATICS
  // ─────────────────────────────────────────────
  /**
   * Solve the four-bar position problem for a given input angle θ₂.
   *
   * @param {Object} mech - Mechanism parameters {a, b, c, d, p, q, A0, B0}
   *   a  = crank length (A₀A)
   *   b  = coupler length (AB)
   *   c  = rocker length (B₀B)
   *   d  = ground length (A₀B₀)
   *   p  = coupler point offset along AB direction
   *   q  = coupler point offset perpendicular to AB
   *   A0 = ground pivot [x, y]
   *   B0 = ground pivot [x, y]
   * @param {number} theta2 - Input crank angle (radians)
   * @param {number} branch - +1 or -1 for assembly mode selection
   * @returns {Object|null} Joint positions and angles, or null if no solution
   */
  function forwardKinematics(mech, theta2, branch = 1) {
    const { a, b, c, d, p, q, A0, B0 } = mech;

    // Position of moving pivot A
    const A = [A0[0] + a * Math.cos(theta2), A0[1] + a * Math.sin(theta2)];

    // Distance from A to B₀
    const dx = B0[0] - A[0];
    const dy = B0[1] - A[1];
    const f = Math.sqrt(dx * dx + dy * dy);

    // Check triangle inequality for links b, c, and distance f
    if (f > b + c + EPS || f < Math.abs(b - c) - EPS || f < EPS) {
      return null; // No valid configuration
    }

    // Angle from A to B₀
    const alphaAB0 = Math.atan2(dy, dx);

    // Cosine rule: angle at A in triangle A-B-B₀
    let cosGamma = (b * b + f * f - c * c) / (2 * b * f);
    cosGamma = Math.max(-1, Math.min(1, cosGamma));
    const gamma = Math.acos(cosGamma);

    // Two assembly modes
    const theta3 = alphaAB0 + branch * gamma;

    // Position of moving pivot B
    const B = [A[0] + b * Math.cos(theta3), A[1] + b * Math.sin(theta3)];

    // Rocker angle θ₄
    const theta4 = Math.atan2(B[1] - B0[1], B[0] - B0[0]);

    // Coupler point P: offset (p, q) in the coupler frame
    // Coupler frame: x-axis along AB, y-axis perpendicular
    const coupler_angle = theta3;
    const P = [
      A[0] + p * Math.cos(coupler_angle) - q * Math.sin(coupler_angle),
      A[1] + p * Math.sin(coupler_angle) + q * Math.cos(coupler_angle),
    ];

    return {
      A0,
      A,
      B,
      B0,
      P,
      theta2,
      theta3,
      theta4,
    };
  }

  // ─────────────────────────────────────────────
  // GRASHOF CONDITION
  // ─────────────────────────────────────────────
  /**
   * Check the Grashof condition and classify the mechanism.
   *
   * @param {number} a - Crank length
   * @param {number} b - Coupler length
   * @param {number} c - Rocker length
   * @param {number} d - Ground length
   * @returns {Object} { isGrashof, type, shortestLink, longestLink }
   */
  function checkGrashof(a, b, c, d) {
    const links = [a, b, c, d];
    const s = Math.min(...links);
    const l = Math.max(...links);
    const sum = links.reduce((acc, v) => acc + v, 0);
    const pq = sum - s - l;

    const isGrashof = s + l <= pq + EPS;

    // Classify
    let type = "Non-Grashof (triple-rocker)";
    if (isGrashof) {
      const sIndex = links.indexOf(s);
      if (sIndex === 0) type = "Grashof crank-rocker (crank = shortest)";
      else if (sIndex === 3) type = "Grashof double-crank (ground = shortest)";
      else if (sIndex === 1) type = "Grashof rocker-crank (coupler = shortest)";
      else type = "Grashof crank-rocker (rocker = shortest)";

      if (Math.abs(s + l - pq) < EPS) {
        type += " [change-point]";
      }
    }

    return { isGrashof, type, shortestLink: s, longestLink: l };
  }

  // ─────────────────────────────────────────────
  // FIND VALID ANGULAR RANGE
  // ─────────────────────────────────────────────
  /**
   * Sweep θ₂ to find the valid angular range for the mechanism.
   *
   * @param {Object} mech - Mechanism parameters
   * @param {number} branch - Assembly mode
   * @param {number} steps - Number of test angles
   * @returns {Object} { validRanges: [[start, end], ...], isFullRotation }
   */
  function findValidRange(mech, branch = 1, steps = 720) {
    const dt = TWO_PI / steps;
    const ranges = [];
    let inRange = false;
    let start = 0;

    for (let i = 0; i <= steps; i++) {
      const theta = i * dt;
      const result = forwardKinematics(mech, theta, branch);
      if (result) {
        if (!inRange) {
          start = theta;
          inRange = true;
        }
      } else {
        if (inRange) {
          ranges.push([start, (i - 1) * dt]);
          inRange = false;
        }
      }
    }
    if (inRange) {
      ranges.push([start, TWO_PI]);
    }

    // Check if full rotation
    const isFullRotation =
      ranges.length === 1 && Math.abs(ranges[0][1] - ranges[0][0] - TWO_PI) < 2 * dt;

    return { validRanges: ranges, isFullRotation };
  }

  // ─────────────────────────────────────────────
  // GENERATE FULL COUPLER CURVE
  // ─────────────────────────────────────────────
  /**
   * Generate the coupler curve by sweeping through valid angles.
   *
   * @param {Object} mech - Mechanism parameters
   * @param {number} branch - Assembly mode
   * @param {number} steps - Resolution
   * @returns {Array} Array of coupler point coordinates [[x,y], ...]
   */
  function generateCouplerCurve(mech, branch = 1, steps = 720) {
    const dt = TWO_PI / steps;
    const curve = [];
    for (let i = 0; i <= steps; i++) {
      const theta = i * dt;
      const result = forwardKinematics(mech, theta, branch);
      if (result) {
        curve.push(result.P);
      }
    }
    return curve;
  }

  // ─────────────────────────────────────────────
  // LINEAR ALGEBRA HELPERS (small matrices)
  // ─────────────────────────────────────────────

  /** Solve a linear system Ax = b using Gaussian elimination with partial pivoting. */
  function solveLinearSystem(A, b) {
    const n = b.length;
    // Augmented matrix
    const M = A.map((row, i) => [...row, b[i]]);

    for (let col = 0; col < n; col++) {
      // Partial pivoting
      let maxVal = Math.abs(M[col][col]);
      let maxRow = col;
      for (let row = col + 1; row < n; row++) {
        if (Math.abs(M[row][col]) > maxVal) {
          maxVal = Math.abs(M[row][col]);
          maxRow = row;
        }
      }
      if (maxVal < EPS) return null; // Singular

      [M[col], M[maxRow]] = [M[maxRow], M[col]];

      // Eliminate
      for (let row = col + 1; row < n; row++) {
        const factor = M[row][col] / M[col][col];
        for (let j = col; j <= n; j++) {
          M[row][j] -= factor * M[col][j];
        }
      }
    }

    // Back-substitute
    const x = new Array(n);
    for (let i = n - 1; i >= 0; i--) {
      x[i] = M[i][n];
      for (let j = i + 1; j < n; j++) {
        x[i] -= M[i][j] * x[j];
      }
      x[i] /= M[i][i];
    }
    return x;
  }

  /** Compute (Jᵀ J) for an m×n matrix J. */
  function JtJ(J) {
    const n = J[0].length;
    const result = Array.from({ length: n }, () => new Array(n).fill(0));
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        let s = 0;
        for (let k = 0; k < J.length; k++) {
          s += J[k][i] * J[k][j];
        }
        result[i][j] = s;
      }
    }
    return result;
  }

  /** Compute Jᵀ r for Jacobian J (m×n) and residual vector r (m). */
  function JtR(J, r) {
    const n = J[0].length;
    const result = new Array(n).fill(0);
    for (let i = 0; i < n; i++) {
      for (let k = 0; k < J.length; k++) {
        result[i] += J[k][i] * r[k];
      }
    }
    return result;
  }

  // ─────────────────────────────────────────────
  // 4-POINT SYNTHESIS (FREUDENSTEIN + GEOMETRIC)
  // ─────────────────────────────────────────────
  /**
   * Four-point coupler curve synthesis.
   *
   * Strategy:
   *   1. Choose ground pivots A₀, B₀ (initially estimated from point spread).
   *   2. For each precision point Pᵢ, and assumed coupler parameters,
   *      write the closure equations.
   *   3. Use Freudenstein's equation to relate input/output angles.
   *   4. Solve for link lengths using a system of linear equations derived
   *      from Freudenstein's equation for the 4 positions.
   *
   * Freudenstein's equation:
   *   R₁ cos θ₄ - R₂ cos θ₂ + R₃ = cos(θ₂ - θ₄)
   * where R₁ = d/c, R₂ = d/a, R₃ = (a²-b²+c²+d²)/(2ac)
   *
   * @param {Array} points - Array of 4 precision points [[x,y], ...]
   * @returns {Object|null} Mechanism parameters or null if synthesis fails
   */
  function synthesize4Point(points) {
    if (points.length !== 4) return null;

    // Step 1: Estimate mechanism placement from points
    const centroid = points.reduce(
      (acc, p) => [acc[0] + p[0] / points.length, acc[1] + p[1] / points.length],
      [0, 0]
    );

    // Estimate scale from point spread
    let maxDist = 0;
    for (let i = 0; i < points.length; i++) {
      for (let j = i + 1; j < points.length; j++) {
        maxDist = Math.max(maxDist, vec.dist(points[i], points[j]));
      }
    }
    const scale = maxDist;

    if (scale < EPS) return null;

    // We'll use an optimisation-based approach:
    // Parameters: [A0x, A0y, B0x, B0y, a, b, c, p_off, q_off, θ₂₁, θ₂₂, θ₂₃, θ₂₄]
    // For each precision point i, the coupler point must equal points[i]
    // at some crank angle θ₂ᵢ.
    //
    // This gives 8 equations (x,y for each of 4 points) and 13 unknowns.
    // We fix d = |A₀B₀| as a derived quantity, fix A₀ position, and
    // optimise the rest.

    // Better approach: use a constrained Freudenstein method.
    // Pick A0 and B0 below the centroid. Pick initial guesses for 
    // crank angles θ₂ᵢ spread around the points.

    // Initial ground pivot placement
    const spread_x = Math.max(...points.map(p => p[0])) - Math.min(...points.map(p => p[0]));
    const spread_y = Math.max(...points.map(p => p[1])) - Math.min(...points.map(p => p[1]));
    const offset = Math.max(spread_x, spread_y) * 0.6;

    let A0 = [centroid[0] - offset * 0.7, centroid[1] - offset * 0.8];
    let B0 = [centroid[0] + offset * 0.7, centroid[1] - offset * 0.8];

    const pairs = points.map(pt => ({
      point: pt,
      theta: Math.atan2(pt[1] - A0[1], pt[0] - A0[0])
    }));

    // Normalize
    pairs.forEach(p => {
      p.theta = (p.theta + TWO_PI) % TWO_PI;
    });

    // Sort BOTH together
    pairs.sort((a, b) => a.theta - b.theta);

    // Extract back
    const sortedPoints = pairs.map(p => p.point);
    let theta2_init = pairs.map(p => p.theta);

    // Sort angles and ensure monotonic increase
    // (points should be in order of crank rotation)

    // We'll solve this using full nonlinear optimisation (Levenberg-Marquardt).
    // Parameters vector x = [A0x, A0y, B0x, B0y, a, b, c, p_off, q_off, θ₂₁, θ₂₂, θ₂₃, θ₂₄]

    // Initial guess
    const avgDist = sortedPoints.reduce((s, pt) => s + vec.dist(pt, centroid), 0) / sortedPoints.length;
    const a_init = avgDist * 0.8;
    const b_init = avgDist * 1.2;
    const c_init = avgDist * 0.9;
    const p_init = b_init * 0.5;
    const q_init = avgDist * 0.3;
    const d_init = a_init;

    let x0 = [
      A0[0], A0[1],
      B0[0], B0[1],
      a_init, b_init, c_init,
      p_init, q_init,
      ...theta2_init,
    ];

    // Residual function: for each point, compute coupler point and measure distance
    function residuals(x) {
      const _A0 = [x[0], x[1]];
      const _B0 = [x[2], x[3]];
      const _a = x[4];
      const _b = x[5];
      const _c = x[6];

      if (_a <= EPS || _b <= EPS || _c <= EPS) {
        return new Array(12).fill(1e6);
      }
      const _p = x[7];
      const _q = x[8];
      const _d = vec.dist(_A0, _B0);

      if (_a < EPS || _b < EPS || _c < EPS || _d < EPS) {
        return new Array(12).fill(1e6);
      }

      const mech = { a: _a, b: _b, c: _c, d: _d, p: _p, q: _q, A0: _A0, B0: _B0 };
      const r = [];

      for (let i = 0; i < 4; i++) {
        const theta2 = x[9 + i];
        const fk = forwardKinematics(mech, theta2, 1);

        if (!fk) {
          return new Array(12).fill(1e6);
        } else {
          r.push(fk.P[0] - sortedPoints[i][0], fk.P[1] - sortedPoints[i][1]);
        }

        // Penalize large jumps in theta (ensures smooth motion)
        if (i > 0) {
          const diff = x[9 + i] - x[9 + i - 1];
          if (diff < 0) {
            r.push(1* diff);
          }
        }
      }

      // Regularization: prevents the mechanism links from becoming excessively large or complex
      const pen = 1e-4;
      r.push((_a - a_init) * pen);
      r.push((_b - b_init) * pen);
      r.push((_c - c_init) * pen);
      r.push((_d - d_init) * pen);

      return r;
    }

    // Helper to evaluate just the actual euclidean matching error
    function pointMatchingError(x) {
      const _A0 = [x[0], x[1]];
      const _B0 = [x[2], x[3]];
      const _a = x[4];
      const _b = x[5];
      const _c = x[6];

      if (_a <= EPS || _b <= EPS || _c <= EPS) return 1e6;

      const _p = x[7];
      const _q = x[8];
      const _d = vec.dist(_A0, _B0);

      const mech = { a: _a, b: _b, c: _c, d: _d, p: _p, q: _q, A0: _A0, B0: _B0 };

      let sumSq = 0;

      for (let i = 0; i < 4; i++) {
        const theta2 = x[9 + i];
        const fk = forwardKinematics(mech, theta2, 1);

        if (!fk) return 1e6;

        const dx = fk.P[0] - sortedPoints[i][0];
        const dy = fk.P[1] - sortedPoints[i][1];

        sumSq += dx * dx + dy * dy;
      }

      return Math.sqrt(sumSq);
    }

    // Run Levenberg-Marquardt
    const result = levenbergMarquardt(residuals, x0, {
      maxIter: 1000,
      tol: 1e-10,
      lambda: 1e-3,
    });

    let bestResult = result;
    let bestError = pointMatchingError(result.x);

    if (!result.converged || bestError > 1e-4 * scale) {
      // Extensive varying initial configurations
      const configs = [
        { ax: -0.5, ay: -0.9, bx: 0.5, by: -0.9 },
        { ax: -0.8, ay: -0.5, bx: 0.8, by: -0.5 },
        { ax: -0.6, ay: 0.6, bx: 0.6, by: 0.6 },
        { ax: -0.3, ay: -1.2, bx: 0.3, by: -1.2 },
        { ax: -0.9, ay: -0.3, bx: 0.9, by: -0.3 },
        { ax: -0.4, ay: -0.6, bx: 0.8, by: -0.4 },
        { ax: 0.0, ay: -1.0, bx: 0.5, by: -1.5 },
        { ax: -1.0, ay: -1.0, bx: 1.0, by: -1.0 },
      ];
      // Add random restarts to avoid local minima
      for (let i = 0; i < 25; i++) {
        configs.push({
          ax: (Math.random() * 2 - 1) * 1.5,
          ay: (Math.random() * 2 - 1) * 1.5,
          bx: (Math.random() * 2 - 1) * 1.5,
          by: (Math.random() * 2 - 1) * 1.5,
        });
      }

      for (const cfg of configs) {
        const _A0 = [centroid[0] + cfg.ax * offset, centroid[1] + cfg.ay * offset];
        const _B0 = [centroid[0] + cfg.bx * offset, centroid[1] + cfg.by * offset];
        const _pairs = sortedPoints.map(pt => ({
          point: pt,
          theta: Math.atan2(pt[1] - _A0[1], pt[0] - _A0[0])
        }));

        _pairs.forEach(p => p.theta = (p.theta + TWO_PI) % TWO_PI);
        _pairs.sort((a, b) => a.theta - b.theta);

        const _theta2 = _pairs.map(p => p.theta);

        const _x0 = [
          _A0[0], _A0[1], _B0[0], _B0[1],
          a_init, b_init, c_init, p_init, q_init,
          ..._theta2,
        ];

        const _r = levenbergMarquardt(residuals, _x0, {
          maxIter: 1000,
          tol: 1e-10,
          lambda: 1e-3,
        });

        const currentPtError = pointMatchingError(_r.x);
        if (currentPtError < bestError) {
          bestError = currentPtError;
          bestResult = _r;
          if (bestError < 1e-4 * scale) break;
        }
      }

      if (bestError > 1e-4 * scale) {
        return {
          success: false,
          error: "Exact synthesis not achieved. Try adjusting the points.",
          residual: bestError,
        };
      }

      Object.assign(result, bestResult);
    }

    // Extract final parameters
    const xf = result.x;
    const finalA0 = [xf[0], xf[1]];
    const finalB0 = [xf[2], xf[3]];
    const final_a = xf[4];
    const final_b = xf[5];
    const final_c = xf[6];
    const final_d = vec.dist(finalA0, finalB0);
    const final_p = xf[7];
    const final_q = xf[8];

    const grashof = checkGrashof(final_a, final_b, final_c, final_d);

    return {
      success: true,
      mechanism: {
        a: final_a,
        b: final_b,
        c: final_c,
        d: final_d,
        p: final_p,
        q: final_q,
        A0: finalA0,
        B0: finalB0,
      },
      grashof,
      residual: result.error,
      crank_angles: [xf[9], xf[10], xf[11], xf[12]],
    };
  }

  // ─────────────────────────────────────────────
  // 5-POINT SYNTHESIS (LEAST SQUARES)
  // ─────────────────────────────────────────────
  /**
   * Five-point coupler curve synthesis using Levenberg-Marquardt.
   *
   * Same approach as 4-point but with 5 precision points (10 equations,
   * 14 unknowns). The system is still under-determined, but we minimise
   * the squared residuals — effectively finding the mechanism whose
   * coupler curve best approximates all 5 points simultaneously.
   *
   * We seed the optimiser with the 4-point solution for the first 4 points.
   *
   * @param {Array} points - Array of 5 precision points [[x,y], ...]
   * @returns {Object|null} Mechanism parameters or null
   */
  function synthesize5Point(points) {
    if (points.length !== 5) return null;

    // First, get a seed from 4-point synthesis using the first 4 points
    const seed = synthesize4Point(points.slice(0, 4));

    let x0;
    if (seed && seed.success) {
      const m = seed.mechanism;
      // Estimate θ₂ for 5th point from the seed mechanism
      const theta5_est = Math.atan2(
        points[4][1] - m.A0[1],
        points[4][0] - m.A0[0]
      );

      x0 = [
        m.A0[0], m.A0[1], m.B0[0], m.B0[1],
        m.a, m.b, m.c, m.p, m.q,
        ...seed.crank_angles,
        theta5_est,
      ];
    } else {
      // Fallback: estimate from scratch
      const centroid = points.reduce(
        (acc, p) => [acc[0] + p[0] / 5, acc[1] + p[1] / 5],
        [0, 0]
      );
      const maxDist = points.reduce((m, p) => Math.max(m, vec.dist(p, centroid)), 0);
      const offset = maxDist * 0.8;
      const A0 = [centroid[0] - offset, centroid[1] - offset * 1.2];
      const B0 = [centroid[0] + offset, centroid[1] - offset * 1.2];

      x0 = [
        A0[0], A0[1], B0[0], B0[1],
        maxDist * 0.6, maxDist * 1.0, maxDist * 0.7, maxDist * 0.4, maxDist * 0.2,
        ...points.map((pt) => Math.atan2(pt[1] - A0[1], pt[0] - A0[0])),
      ];
    }

    // Residual function for 5 points
    function residuals(x) {
      const _A0 = [x[0], x[1]];
      const _B0 = [x[2], x[3]];
      const _a = Math.abs(x[4]);
      const _b = Math.abs(x[5]);
      const _c = Math.abs(x[6]);
      const _p = x[7];
      const _q = x[8];
      const _d = vec.dist(_A0, _B0);

      if (_a < EPS || _b < EPS || _c < EPS || _d < EPS) {
        return new Array(10).fill(1e6);
      }

      const mech = { a: _a, b: _b, c: _c, d: _d, p: _p, q: _q, A0: _A0, B0: _B0 };
      const r = [];

      for (let i = 0; i < 5; i++) {
        const theta2 = x[9 + i];
        const fk = forwardKinematics(mech, theta2, 1);

        if (!fk) {
          r.push(1e6, 1e6);
        } else {
          r.push(fk.P[0] - points[i][0], fk.P[1] - points[i][1]);
        }
      }
      return r;
    }

    // Helper to evaluate point error
    function pointMatchingError(x) {
      const r = residuals(x);
      let sumSq = 0;
      for (let i = 0; i < 10; i++) sumSq += r[i] * r[i];
      return Math.sqrt(sumSq);
    }

    // Multiple initialisations for robustness
    const centroid = points.reduce(
      (acc, p) => [acc[0] + p[0] / 5, acc[1] + p[1] / 5],
      [0, 0]
    );
    const maxDist = points.reduce((m, p) => Math.max(m, vec.dist(p, centroid)), 0);
    const offset = maxDist * 0.8;

    let bestResult = levenbergMarquardt(residuals, x0, {
      maxIter: 1000,
      tol: 1e-12,
      lambda: 1e-3,
    });
    let bestError = pointMatchingError(bestResult.x);

    // Try alternative initialisations
    const configs = [
      { ax: -0.5, ay: -1.0, bx: 0.5, by: -1.0 },
      { ax: -1.0, ay: -0.5, bx: 1.0, by: -0.5 },
      { ax: -0.7, ay: 0.7, bx: 0.7, by: 0.7 },
      { ax: -0.4, ay: -1.3, bx: 0.4, by: -1.3 },
    ];
    for (let i = 0; i < 25; i++) {
      configs.push({
        ax: (Math.random() * 2 - 1) * 1.5,
        ay: (Math.random() * 2 - 1) * 1.5,
        bx: (Math.random() * 2 - 1) * 1.5,
        by: (Math.random() * 2 - 1) * 1.5,
      });
    }

    for (const cfg of configs) {
      if (bestError < 1e-4 * maxDist) break;

      const _A0 = [centroid[0] + cfg.ax * offset, centroid[1] + cfg.ay * offset];
      const _B0 = [centroid[0] + cfg.bx * offset, centroid[1] + cfg.by * offset];

      const _x0 = [
        _A0[0], _A0[1], _B0[0], _B0[1],
        maxDist * 0.6, maxDist * 1.0, maxDist * 0.7,
        maxDist * 0.4, maxDist * 0.2,
        ...points.map((pt) => Math.atan2(pt[1] - _A0[1], pt[0] - _A0[0])),
      ];

      const _r = levenbergMarquardt(residuals, _x0, {
        maxIter: 1000,
        tol: 1e-10,
        lambda: 1e-3,
      });

      const currentPtError = pointMatchingError(_r.x);
      if (currentPtError < bestError) {
        bestError = currentPtError;
        bestResult = _r;
      }
    }

    if (bestError > 5e-2 * maxDist) {
      return {
        success: false,
        error:
          "5-point synthesis did not converge to a satisfactory solution. " +
          `Residual error: ${bestError.toFixed(4)}. Try adjusting point positions.`,
        residual: bestError,
      };
    }

    const xf = bestResult.x;
    const finalA0 = [xf[0], xf[1]];
    const finalB0 = [xf[2], xf[3]];
    const final_a = Math.abs(xf[4]);
    const final_b = Math.abs(xf[5]);
    const final_c = Math.abs(xf[6]);
    const final_d = vec.dist(finalA0, finalB0);
    const final_p = xf[7];
    const final_q = xf[8];

    const grashof = checkGrashof(final_a, final_b, final_c, final_d);

    return {
      success: true,
      mechanism: {
        a: final_a,
        b: final_b,
        c: final_c,
        d: final_d,
        p: final_p,
        q: final_q,
        A0: finalA0,
        B0: finalB0,
      },
      grashof,
      residual: bestError,
      crank_angles: [xf[9], xf[10], xf[11], xf[12], xf[13]],
    };
  }

  // ─────────────────────────────────────────────
  // LEVENBERG–MARQUARDT SOLVER
  // ─────────────────────────────────────────────
  /**
   * Levenberg–Marquardt algorithm for nonlinear least-squares.
   *
   * Minimizes || f(x) ||² where f: Rⁿ → Rᵐ.
   *
   * @param {Function} residualFn - Function x → [r₁, r₂, ...] residual vector
   * @param {Array} x0 - Initial parameter guess
   * @param {Object} opts - { maxIter, tol, lambda }
   * @returns {Object} { x, error, converged, iterations }
   */
  function levenbergMarquardt(residualFn, x0, opts = {}) {
    const maxIter = opts.maxIter || 300;
    const tol = opts.tol || 1e-10;
    let lambda = opts.lambda || 1e-3;
    const lambdaUp = 10;
    const lambdaDown = 0.1;
    const h = 1e-6; // Finite-difference step size

    let x = [...x0];
    const n = x.length;
    let r = residualFn(x);
    const m = r.length;
    let cost = r.reduce((s, v) => s + v * v, 0);

    let converged = false;
    let iter = 0;

    for (iter = 0; iter < maxIter; iter++) {
      // Compute Jacobian via central finite differences
      const J = Array.from({ length: m }, () => new Array(n).fill(0));

      for (let j = 0; j < n; j++) {
        const xp = [...x];
        const xm = [...x];
        xp[j] += h;
        xm[j] -= h;
        const rp = residualFn(xp);
        const rm = residualFn(xm);
        for (let i = 0; i < m; i++) {
          J[i][j] = (rp[i] - rm[i]) / (2 * h);
        }
      }

      // Normal equations: (JᵀJ + λI) δ = -Jᵀr
      const jtj = JtJ(J);
      const jtr = JtR(J, r);

      // Damped normal equations
      const A_mat = jtj.map((row, i) =>
        row.map((v, j) => (i === j ? v + lambda * (1 + v) : v))
      );
      const b_vec = jtr.map((v) => -v);

      const delta = solveLinearSystem(A_mat, b_vec);
      if (!delta) {
        lambda *= lambdaUp;
        continue;
      }

      // Trial step
      const x_new = x.map((v, i) => v + delta[i]);
      const r_new = residualFn(x_new);
      const cost_new = r_new.reduce((s, v) => s + v * v, 0);

      if (cost_new < cost) {
        x = x_new;
        r = r_new;
        cost = cost_new;
        lambda *= lambdaDown;

        if (cost < tol) {
          converged = true;
          break;
        }
      } else {
        lambda *= lambdaUp;
      }

      // Check for convergence on step size
      const stepNorm = Math.sqrt(delta.reduce((s, v) => s + v * v, 0));
      const xNorm = Math.sqrt(x.reduce((s, v) => s + v * v, 0));
      if (stepNorm < tol * (1 + xNorm) && cost < 1e-12) {
        converged = true;
        break;
      }
    }

    return {
      x,
      error: Math.sqrt(cost),
      converged,
      iterations: iter,
    };
  }

  // ─────────────────────────────────────────────
  // PUBLIC API
  // ─────────────────────────────────────────────
  return {
    forwardKinematics,
    checkGrashof,
    findValidRange,
    generateCouplerCurve,
    synthesize4Point,
    synthesize5Point,
    vec,
    DEG,
    TWO_PI,
  };
})();
