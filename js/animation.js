/**
 * animation.js — Canvas Rendering & Mechanism Animation
 *
 * Handles:
 *   - World-to-screen coordinate transforms (auto-fit)
 *   - Drawing of all four-bar linkage components
 *   - Coupler curve tracing
 *   - Precision point overlay
 *   - Animation loop with speed control
 */

const Animation = (() => {
    "use strict";

    // ─────────────────────────────────────────────
    // STATE
    // ─────────────────────────────────────────────
    let canvas = null;
    let ctx = null;
    let animId = null;
    let isPlaying = false;
    let theta2 = 0;
    let speed = 0.02; // radians per frame
    let mechanism = null;
    let branch = 1;
    let couplerCurve = [];
    let tracePoints = [];
    let precisionPoints = [];
    let showCurve = true;
    let showPoints = true;

    // View transform
    let viewScale = 1;
    let viewOffsetX = 0;
    let viewOffsetY = 0;

    // Manual zoom and pan
    let zoomFactor = 1.0;
    let panX = 0.0;
    let panY = 0.0;

    // ─────────────────────────────────────────────
    // COLOURS (engineering dark theme)
    // ─────────────────────────────────────────────
    const COLORS = {
        bg: "#0f1117",
        grid: "#1a1d27",
        gridMajor: "#252836",
        ground: "#6c7a89",
        groundHatch: "#4a5568",
        crank: "#4fc3f7",
        coupler: "#ff7043",
        rocker: "#66bb6a",
        joint: "#ffffff",
        jointFill: "#1e2130",
        couplerPoint: "#ffeb3b",
        couplerCurve: "#ab47bc",
        couplerCurveGlow: "rgba(171, 71, 188, 0.3)",
        precisionPoint: "#ef5350",
        precisionPointGlow: "rgba(239, 83, 80, 0.4)",
        groundPivot: "#78909c",
        text: "#b0bec5",
        trace: "rgba(171, 71, 188, 0.6)",
    };

    const LINK_WIDTH = 3.5;
    const JOINT_RADIUS = 6;
    const COUPLER_PT_RADIUS = 8;
    const PRECISION_PT_RADIUS = 7;

    // ─────────────────────────────────────────────
    // INITIALISATION
    // ─────────────────────────────────────────────
    function init(canvasElement) {
        canvas = canvasElement;
        ctx = canvas.getContext("2d");
        resizeCanvas();
        window.addEventListener("resize", resizeCanvas);
    }

    function resizeCanvas() {
        const parent = canvas.parentElement;
        const dpr = window.devicePixelRatio || 1;
        canvas.width = parent.clientWidth * dpr;
        canvas.height = parent.clientHeight * dpr;
        canvas.style.width = parent.clientWidth + "px";
        canvas.style.height = parent.clientHeight + "px";
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        if (mechanism) {
            computeViewTransform();
            drawFrame();
        }
    }

    // ─────────────────────────────────────────────
    // VIEW TRANSFORM
    // ─────────────────────────────────────────────
    function computeViewTransform() {
        if (!mechanism) return;

        // Collect all relevant points for bounding box
        const pts = [mechanism.A0, mechanism.B0];

        // Add coupler curve points if available
        if (couplerCurve.length > 0) {
            pts.push(...couplerCurve);
        } else {
            // Estimate bounds from link lengths
            const maxReach = mechanism.a + mechanism.b + Math.sqrt(mechanism.p * mechanism.p + mechanism.q * mechanism.q);
            pts.push(
                [mechanism.A0[0] - maxReach, mechanism.A0[1] - maxReach],
                [mechanism.A0[0] + maxReach, mechanism.A0[1] + maxReach],
                [mechanism.B0[0] - maxReach, mechanism.B0[1] - maxReach],
                [mechanism.B0[0] + maxReach, mechanism.B0[1] + maxReach]
            );
        }

        // Add precision points
        if (precisionPoints.length > 0) {
            pts.push(...precisionPoints);
        }

        if (pts.length === 0) return;

        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        for (const p of pts) {
            minX = Math.min(minX, p[0]);
            minY = Math.min(minY, p[1]);
            maxX = Math.max(maxX, p[0]);
            maxY = Math.max(maxY, p[1]);
        }

        const margin = 60;
        const w = canvas.clientWidth - 2 * margin;
        const h = canvas.clientHeight - 2 * margin;
        const worldW = maxX - minX || 1;
        const worldH = maxY - minY || 1;

        viewScale = Math.min(w / worldW, h / worldH);
        viewOffsetX = margin + (w - worldW * viewScale) / 2 - minX * viewScale;
        viewOffsetY = margin + (h - worldH * viewScale) / 2 + maxY * viewScale; // flip Y
    }

    function worldToScreen(pt) {
        const finalScale = viewScale * zoomFactor;
        const finalOffsetX = viewOffsetX + panX;
        const finalOffsetY = viewOffsetY + panY;
        return [
            finalOffsetX + pt[0] * finalScale,
            finalOffsetY - pt[1] * finalScale, // flip Y axis
        ];
    }

    function screenToWorld(sx, sy) {
        const finalScale = viewScale * zoomFactor;
        const finalOffsetX = viewOffsetX + panX;
        const finalOffsetY = viewOffsetY + panY;
        return [
            (sx - finalOffsetX) / finalScale,
            (finalOffsetY - sy) / finalScale,
        ];
    }

    // ─────────────────────────────────────────────
    // DRAWING PRIMITIVES
    // ─────────────────────────────────────────────

    function drawGrid() {
        const w = canvas.clientWidth;
        const h = canvas.clientHeight;

        // Determine grid spacing in world coords
        let gridStep = 1;
        const desiredPixelStep = 50;
        const actualScale = viewScale * zoomFactor;
        const worldPixelStep = desiredPixelStep / actualScale;

        // Find nice grid step
        const pow10 = Math.pow(10, Math.floor(Math.log10(worldPixelStep)));
        const multiples = [1, 2, 5, 10];
        for (const m of multiples) {
            if (pow10 * m * actualScale >= desiredPixelStep * 0.7) {
                gridStep = pow10 * m;
                break;
            }
        }

        // World bounds visible on screen
        const [wMinX, wMaxY] = screenToWorld(0, 0);
        const [wMaxX, wMinY] = screenToWorld(w, h);

        ctx.lineWidth = 1;

        // Minor grid
        ctx.strokeStyle = COLORS.grid;
        ctx.beginPath();
        for (let x = Math.floor(wMinX / gridStep) * gridStep; x <= wMaxX; x += gridStep) {
            const [sx] = worldToScreen([x, 0]);
            ctx.moveTo(sx, 0);
            ctx.lineTo(sx, h);
        }
        for (let y = Math.floor(wMinY / gridStep) * gridStep; y <= wMaxY; y += gridStep) {
            const [, sy] = worldToScreen([0, y]);
            ctx.moveTo(0, sy);
            ctx.lineTo(w, sy);
        }
        ctx.stroke();

        // Major grid (every 5 steps)
        const majorStep = gridStep * 5;
        ctx.strokeStyle = COLORS.gridMajor;
        ctx.beginPath();
        for (let x = Math.floor(wMinX / majorStep) * majorStep; x <= wMaxX; x += majorStep) {
            const [sx] = worldToScreen([x, 0]);
            ctx.moveTo(sx, 0);
            ctx.lineTo(sx, h);
        }
        for (let y = Math.floor(wMinY / majorStep) * majorStep; y <= wMaxY; y += majorStep) {
            const [, sy] = worldToScreen([0, y]);
            ctx.moveTo(0, sy);
            ctx.lineTo(w, sy);
        }
        ctx.stroke();
    }

    function drawLink(ptA, ptB, color, lineWidth = LINK_WIDTH) {
        const a = worldToScreen(ptA);
        const b = worldToScreen(ptB);

        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth;
        ctx.lineCap = "round";
        ctx.beginPath();
        ctx.moveTo(a[0], a[1]);
        ctx.lineTo(b[0], b[1]);
        ctx.stroke();
    }

    function drawJoint(pt, filled = false) {
        const s = worldToScreen(pt);
        ctx.beginPath();
        ctx.arc(s[0], s[1], JOINT_RADIUS, 0, Math.PI * 2);
        ctx.fillStyle = filled ? COLORS.joint : COLORS.jointFill;
        ctx.fill();
        ctx.strokeStyle = COLORS.joint;
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    function drawGroundPivot(pt) {
        const s = worldToScreen(pt);

        // Triangle base
        const size = 12;
        ctx.beginPath();
        ctx.moveTo(s[0], s[1] + JOINT_RADIUS);
        ctx.lineTo(s[0] - size, s[1] + JOINT_RADIUS + size);
        ctx.lineTo(s[0] + size, s[1] + JOINT_RADIUS + size);
        ctx.closePath();
        ctx.fillStyle = COLORS.groundPivot;
        ctx.globalAlpha = 0.5;
        ctx.fill();
        ctx.globalAlpha = 1.0;

        // Hatching
        ctx.strokeStyle = COLORS.groundHatch;
        ctx.lineWidth = 1.5;
        const y_base = s[1] + JOINT_RADIUS + size;
        ctx.beginPath();
        for (let i = -3; i <= 3; i++) {
            const x = s[0] + i * 4;
            ctx.moveTo(x, y_base);
            ctx.lineTo(x - 5, y_base + 6);
        }
        ctx.stroke();

        // Ground line
        ctx.strokeStyle = COLORS.ground;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(s[0] - size - 4, y_base);
        ctx.lineTo(s[0] + size + 4, y_base);
        ctx.stroke();

        drawJoint(pt, false);
    }

    function drawCouplerPoint(pt) {
        const s = worldToScreen(pt);

        // Glow
        const grd = ctx.createRadialGradient(s[0], s[1], 0, s[0], s[1], COUPLER_PT_RADIUS * 3);
        grd.addColorStop(0, "rgba(255, 235, 59, 0.5)");
        grd.addColorStop(1, "rgba(255, 235, 59, 0)");
        ctx.fillStyle = grd;
        ctx.beginPath();
        ctx.arc(s[0], s[1], COUPLER_PT_RADIUS * 3, 0, Math.PI * 2);
        ctx.fill();

        // Point
        ctx.beginPath();
        ctx.arc(s[0], s[1], COUPLER_PT_RADIUS, 0, Math.PI * 2);
        ctx.fillStyle = COLORS.couplerPoint;
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    function drawPrecisionPoint(pt, index) {
        const s = worldToScreen(pt);

        // Glow
        const grd = ctx.createRadialGradient(s[0], s[1], 0, s[0], s[1], PRECISION_PT_RADIUS * 2.5);
        grd.addColorStop(0, COLORS.precisionPointGlow);
        grd.addColorStop(1, "rgba(239, 83, 80, 0)");
        ctx.fillStyle = grd;
        ctx.beginPath();
        ctx.arc(s[0], s[1], PRECISION_PT_RADIUS * 2.5, 0, Math.PI * 2);
        ctx.fill();

        // Diamond shape
        ctx.beginPath();
        const r = PRECISION_PT_RADIUS;
        ctx.moveTo(s[0], s[1] - r);
        ctx.lineTo(s[0] + r, s[1]);
        ctx.lineTo(s[0], s[1] + r);
        ctx.lineTo(s[0] - r, s[1]);
        ctx.closePath();
        ctx.fillStyle = COLORS.precisionPoint;
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Label
        ctx.fillStyle = "#fff";
        ctx.font = "bold 11px Inter, sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`P${index + 1}`, s[0], s[1] + r + 14);
    }

    // ─────────────────────────────────────────────
    // COUPLER CURVE DRAWING
    // ─────────────────────────────────────────────

    function drawCouplerCurvePath() {
        if (couplerCurve.length < 2) return;

        // Glow layer
        ctx.strokeStyle = COLORS.couplerCurveGlow;
        ctx.lineWidth = 8;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        let first = worldToScreen(couplerCurve[0]);
        ctx.moveTo(first[0], first[1]);
        for (let i = 1; i < couplerCurve.length; i++) {
            const s = worldToScreen(couplerCurve[i]);
            ctx.lineTo(s[0], s[1]);
        }
        ctx.stroke();

        // Main curve
        ctx.strokeStyle = COLORS.couplerCurve;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        first = worldToScreen(couplerCurve[0]);
        ctx.moveTo(first[0], first[1]);
        for (let i = 1; i < couplerCurve.length; i++) {
            const s = worldToScreen(couplerCurve[i]);
            ctx.lineTo(s[0], s[1]);
        }
        ctx.stroke();
    }

    function drawTrace() {
        if (tracePoints.length < 2) return;

        ctx.strokeStyle = COLORS.trace;
        ctx.lineWidth = 1.5;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        const first = worldToScreen(tracePoints[0]);
        ctx.moveTo(first[0], first[1]);
        for (let i = 1; i < tracePoints.length; i++) {
            const s = worldToScreen(tracePoints[i]);
            ctx.lineTo(s[0], s[1]);
        }
        ctx.stroke();
    }

    // ─────────────────────────────────────────────
    // GROUND LINK
    // ─────────────────────────────────────────────
    function drawGroundLink() {
        if (!mechanism) return;
        const a = worldToScreen(mechanism.A0);
        const b = worldToScreen(mechanism.B0);

        // Dashed line
        ctx.setLineDash([6, 4]);
        ctx.strokeStyle = COLORS.ground;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(a[0], a[1]);
        ctx.lineTo(b[0], b[1]);
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // ─────────────────────────────────────────────
    // FULL MECHANISM DRAW
    // ─────────────────────────────────────────────
    function drawMechanism(fk) {
        if (!fk) return;

        // Ground link (dashed)
        drawGroundLink();

        // Links
        drawLink(fk.A0, fk.A, COLORS.crank);
        drawLink(fk.A, fk.B, COLORS.coupler, 4);
        drawLink(fk.B0, fk.B, COLORS.rocker);

        // Coupler triangle (A → B → P)
        const sa = worldToScreen(fk.A);
        const sb = worldToScreen(fk.B);
        const sp = worldToScreen(fk.P);
        ctx.beginPath();
        ctx.moveTo(sa[0], sa[1]);
        ctx.lineTo(sb[0], sb[1]);
        ctx.lineTo(sp[0], sp[1]);
        ctx.closePath();
        ctx.fillStyle = "rgba(255, 112, 67, 0.08)";
        ctx.fill();
        ctx.strokeStyle = "rgba(255, 112, 67, 0.4)";
        ctx.lineWidth = 1;
        ctx.stroke();

        // Ground pivots
        drawGroundPivot(fk.A0);
        drawGroundPivot(fk.B0);

        // Moving joints
        drawJoint(fk.A, true);
        drawJoint(fk.B, true);

        // Coupler point
        drawCouplerPoint(fk.P);
    }

    // ─────────────────────────────────────────────
    // FRAME DRAW
    // ─────────────────────────────────────────────
    function drawFrame() {
        if (!ctx) return;

        const w = canvas.clientWidth;
        const h = canvas.clientHeight;

        // Clear
        ctx.fillStyle = COLORS.bg;
        ctx.fillRect(0, 0, w, h);

        // Grid
        drawGrid();

        if (!mechanism) {
            // Draw only precision points if no mechanism yet
            if (showPoints) {
                precisionPoints.forEach((pt, i) => drawPrecisionPoint(pt, i));
            }
            return;
        }

        // Coupler curve
        if (showCurve) {
            drawCouplerCurvePath();
        }

        // Live trace
        drawTrace();

        // Current mechanism position
        const fk = Kinematics.forwardKinematics(mechanism, theta2, branch);
        if (fk) {
            drawMechanism(fk);
            tracePoints.push([...fk.P]);
        }

        // Precision points
        if (showPoints) {
            precisionPoints.forEach((pt, i) => drawPrecisionPoint(pt, i));
        }

        // Angle display
        if (fk) {
            drawAngleInfo(fk);
        }
    }

    function drawAngleInfo(fk) {
        const margin = 12;
        ctx.fillStyle = "rgba(15, 17, 23, 0.85)";
        ctx.fillRect(margin, margin, 160, 50);
        ctx.strokeStyle = COLORS.gridMajor;
        ctx.lineWidth = 1;
        ctx.strokeRect(margin, margin, 160, 50);

        ctx.fillStyle = COLORS.text;
        ctx.font = "12px 'JetBrains Mono', monospace";
        ctx.textAlign = "left";
        ctx.textBaseline = "top";

        const deg = (rad) => ((rad * 180) / Math.PI).toFixed(1);
        ctx.fillText(`θ₂ = ${deg(fk.theta2)}°`, margin + 10, margin + 10);
        ctx.fillText(`θ₃ = ${deg(fk.theta3)}°  θ₄ = ${deg(fk.theta4)}°`, margin + 10, margin + 30);
    }

    // ─────────────────────────────────────────────
    // ANIMATION LOOP
    // ─────────────────────────────────────────────
    function animate() {
        if (!isPlaying) return;

        theta2 += speed;
        if (theta2 > Kinematics.TWO_PI) theta2 -= Kinematics.TWO_PI;

        drawFrame();
        animId = requestAnimationFrame(animate);
    }

    function play() {
        if (isPlaying) return;
        isPlaying = true;
        animate();
    }

    function pause() {
        isPlaying = false;
        if (animId) {
            cancelAnimationFrame(animId);
            animId = null;
        }
    }

    function reset() {
        pause();
        theta2 = 0;
        tracePoints = [];
        drawFrame();
    }

    function setSpeed(s) {
        speed = s;
    }

    // ─────────────────────────────────────────────
    // MECHANISM & POINTS SETTERS
    // ─────────────────────────────────────────────
    function setMechanism(mech) {
        mechanism = mech;
        tracePoints = [];
        zoomFactor = 1.0;
        panX = 0;
        panY = 0;

        // Generate full coupler curve
        couplerCurve = Kinematics.generateCouplerCurve(mech, branch, 720);

        // Determine best assembly branch
        const range1 = Kinematics.findValidRange(mech, 1);
        const range2 = Kinematics.findValidRange(mech, -1);

        if (range1.isFullRotation) {
            branch = 1;
        } else if (range2.isFullRotation) {
            branch = -1;
            couplerCurve = Kinematics.generateCouplerCurve(mech, branch, 720);
        } else {
            // Pick branch with larger valid range
            const totalRange = (ranges) =>
                ranges.validRanges.reduce((s, r) => s + (r[1] - r[0]), 0);
            branch = totalRange(range1) >= totalRange(range2) ? 1 : -1;
            couplerCurve = Kinematics.generateCouplerCurve(mech, branch, 720);
        }

        computeViewTransform();
        theta2 = 0;
        drawFrame();
    }

    function setPrecisionPoints(pts) {
        precisionPoints = pts;
        if (!mechanism) {
            computeViewTransform();
        }
        drawFrame();
    }

    function setShowCurve(v) {
        showCurve = v;
        drawFrame();
    }

    function setShowPoints(v) {
        showPoints = v;
        drawFrame();
    }

    function clear() {
        pause();
        mechanism = null;
        couplerCurve = [];
        tracePoints = [];
        precisionPoints = [];
        zoomFactor = 1.0;
        panX = 0;
        panY = 0;
        theta2 = 0;
        if (ctx) {
            const w = canvas.clientWidth;
            const h = canvas.clientHeight;
            ctx.fillStyle = COLORS.bg;
            ctx.fillRect(0, 0, w, h);
        }
    }

    function redraw() {
        if (mechanism) {
            computeViewTransform();
        }
        drawFrame();
    }

    // ─────────────────────────────────────────────
    // ZOOM AND PAN METHODS
    // ─────────────────────────────────────────────

    function applyZoom(deltaY, mouseX, mouseY) {
        // Find world coordinates of mouse before zoom
        const [worldX, worldY] = screenToWorld(mouseX, mouseY);

        // Adjust zoom
        const zoomStep = 1.1;
        if (deltaY < 0) {
            zoomFactor *= zoomStep;
        } else {
            zoomFactor /= zoomStep;
        }

        // constrain zoomFactor so it doesn't get crazy
        zoomFactor = Math.max(0.1, Math.min(zoomFactor, 50.0));

        const newScale = viewScale * zoomFactor;
        panX = mouseX - viewOffsetX - worldX * newScale;
        panY = mouseY - viewOffsetY + worldY * newScale;

        drawFrame();
    }

    function applyPan(dx, dy) {
        panX += dx;
        panY += dy;
        drawFrame();
    }

    // ─────────────────────────────────────────────
    // PUBLIC API
    // ─────────────────────────────────────────────
    return {
        init,
        play,
        pause,
        reset,
        setSpeed,
        setMechanism,
        setPrecisionPoints,
        setShowCurve,
        setShowPoints,
        clear,
        redraw,
        drawFrame,
        applyZoom,
        applyPan,
        screenToWorld,
        worldToScreen,
        computeViewTransform,
        get isPlaying() { return isPlaying; },
        get currentAngle() { return theta2; },
        get mechanism() { return mechanism; },
        get precisionPoints() { return precisionPoints; },
    };
})();
