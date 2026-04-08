/**
 * ui.js — User Interface Controller
 *
 * Handles:
 *   - Mode switching (4-point / 5-point)
 *   - Graphical input (click-to-place, drag-to-adjust)
 *   - Manual coordinate input fields
 *   - Synthesis triggering
 *   - Animation controls (play/pause/reset/speed)
 *   - Display toggles (coupler curve, precision points)
 *   - Info panel updates (link lengths, angles, Grashof, error)
 *   - Export / Import mechanism data as JSON
 *   - Error display
 */

const UI = (() => {
    "use strict";

    // ─────────────────────────────────────────────
    // STATE
    // ─────────────────────────────────────────────
    let mode = 4; // 4 or 5 point synthesis
    let points = [];
    let draggingIndex = -1;
    let isDragging = false;
    let canvas = null;
    let currentResult = null;
    let hoveredPointIndex = -1;
    let isPanning = false;
    let lastPanX = 0;
    let lastPanY = 0;

    // ─────────────────────────────────────────────
    // DOM REFERENCES
    // ─────────────────────────────────────────────
    const $ = (id) => document.getElementById(id);

    // ─────────────────────────────────────────────
    // INITIALISATION
    // ─────────────────────────────────────────────
    function init() {
        canvas = $("mainCanvas");
        Animation.init(canvas);

        bindModeControls();
        bindCanvasInteraction();
        bindAnimationControls();
        bindToggles();
        bindManualInput();
        bindSynthesizeButton();
        bindExportImport();

        updateManualInputFields();
        updateStatus("Click on the canvas to place precision points.");
        Animation.redraw();
    }

    // ─────────────────────────────────────────────
    // MODE SWITCHING
    // ─────────────────────────────────────────────
    function bindModeControls() {
        $("mode4").addEventListener("click", () => setMode(4));
        $("mode5").addEventListener("click", () => setMode(5));
    }

    function setMode(m) {
        mode = m;
        $("mode4").classList.toggle("active", m === 4);
        $("mode5").classList.toggle("active", m === 5);
        $("modeLabel").textContent = `${m}-Point Synthesis`;

        // Trim or keep points
        if (points.length > m) {
            points = points.slice(0, m);
        }

        updateManualInputFields();
        updatePointsList();
        Animation.setPrecisionPoints([...points]);
        clearResult();
        updateStatus(`${m}-point mode. Place ${m - points.length} more point(s).`);
    }

    // ─────────────────────────────────────────────
    // CANVAS INTERACTION (GRAPHICAL INPUT)
    // ─────────────────────────────────────────────
    function bindCanvasInteraction() {
        canvas.addEventListener("mousedown", onMouseDown);
        canvas.addEventListener("mousemove", onMouseMove);
        canvas.addEventListener("mouseup", onMouseUp);
        canvas.addEventListener("mouseleave", onMouseUp);
        canvas.addEventListener("wheel", onWheel, { passive: false });
        canvas.addEventListener("contextmenu", (e) => e.preventDefault());

        // Touch support
        canvas.addEventListener("touchstart", onTouchStart, { passive: false });
        canvas.addEventListener("touchmove", onTouchMove, { passive: false });
        canvas.addEventListener("touchend", onTouchEnd);
    }

    function getCanvasPos(e) {
        const rect = canvas.getBoundingClientRect();
        return [e.clientX - rect.left, e.clientY - rect.top];
    }

    function findPointNear(sx, sy, threshold = 15) {
        for (let i = 0; i < points.length; i++) {
            const sp = Animation.worldToScreen(points[i]);
            const dx = sp[0] - sx;
            const dy = sp[1] - sy;
            if (Math.sqrt(dx * dx + dy * dy) < threshold) {
                return i;
            }
        }
        return -1;
    }

    function onWheel(e) {
        e.preventDefault();
        const [sx, sy] = getCanvasPos(e);
        Animation.applyZoom(e.deltaY, sx, sy);
    }

    function onMouseDown(e) {
        if (e.button === 1 || e.button === 2) { // Middle or Right click
            e.preventDefault();
            isPanning = true;
            const [sx, sy] = getCanvasPos(e);
            lastPanX = sx;
            lastPanY = sy;
            canvas.style.cursor = "grabbing";
            return;
        }

        e.preventDefault();
        const [sx, sy] = getCanvasPos(e);
        const idx = findPointNear(sx, sy);

        if (idx >= 0) {
            // Start dragging existing point
            draggingIndex = idx;
            isDragging = true;
            canvas.style.cursor = "grabbing";
        } else if (points.length < mode) {
            // Add new point
            const world = Animation.screenToWorld(sx, sy);
            points.push(world);
            updateManualInputFields();
            updatePointsList();
            Animation.setPrecisionPoints([...points]);

            if (points.length >= mode) {
                updateStatus(`All ${mode} points placed. Click "Synthesize" or adjust points.`);
            } else {
                updateStatus(`Point ${points.length} placed. ${mode - points.length} more to go.`);
            }
        }
    }

    function onMouseMove(e) {
        const [sx, sy] = getCanvasPos(e);

        if (isPanning) {
            const dx = sx - lastPanX;
            const dy = sy - lastPanY;
            Animation.applyPan(dx, dy);
            lastPanX = sx;
            lastPanY = sy;
            return;
        }

        if (isDragging && draggingIndex >= 0) {
            const world = Animation.screenToWorld(sx, sy);
            points[draggingIndex] = world;
            updateManualInputFields();
            updatePointsList();
            Animation.setPrecisionPoints([...points]);
            return;
        }

        // Hover cursor
        const idx = findPointNear(sx, sy);
        if (idx >= 0) {
            canvas.style.cursor = "grab";
            hoveredPointIndex = idx;
        } else if (points.length < mode) {
            canvas.style.cursor = "crosshair";
            hoveredPointIndex = -1;
        } else {
            canvas.style.cursor = "default";
            hoveredPointIndex = -1;
        }

        // Show tooltip with world coordinates
        const world = Animation.screenToWorld(sx, sy);
        $("coordTooltip").textContent = `(${world[0].toFixed(1)}, ${world[1].toFixed(1)})`;
    }

    function onMouseUp() {
        if (isPanning) {
            isPanning = false;
            canvas.style.cursor = points.length < mode ? "crosshair" : "default";
        }
        if (isDragging) {
            isDragging = false;
            draggingIndex = -1;
            canvas.style.cursor = points.length < mode ? "crosshair" : "default";
        }
    }

    // Touch handlers
    function onTouchStart(e) {
        e.preventDefault();
        const touch = e.touches[0];
        const rect = canvas.getBoundingClientRect();
        const sx = touch.clientX - rect.left;
        const sy = touch.clientY - rect.top;

        const idx = findPointNear(sx, sy, 25);
        if (idx >= 0) {
            draggingIndex = idx;
            isDragging = true;
        } else if (points.length < mode) {
            const world = Animation.screenToWorld(sx, sy);
            points.push(world);
            updateManualInputFields();
            updatePointsList();
            Animation.setPrecisionPoints([...points]);
        }
    }

    function onTouchMove(e) {
        e.preventDefault();
        if (isDragging && draggingIndex >= 0) {
            const touch = e.touches[0];
            const rect = canvas.getBoundingClientRect();
            const sx = touch.clientX - rect.left;
            const sy = touch.clientY - rect.top;
            const world = Animation.screenToWorld(sx, sy);
            points[draggingIndex] = world;
            updateManualInputFields();
            updatePointsList();
            Animation.setPrecisionPoints([...points]);
        }
    }

    function onTouchEnd() {
        isDragging = false;
        draggingIndex = -1;
    }

    // ─────────────────────────────────────────────
    // MANUAL INPUT FIELDS
    // ─────────────────────────────────────────────
    function bindManualInput() {
        $("applyManual").addEventListener("click", applyManualInput);
    }

    function updateManualInputFields() {
        const container = $("manualInputs");
        container.innerHTML = "";

        for (let i = 0; i < mode; i++) {
            const row = document.createElement("div");
            row.className = "input-row";

            const label = document.createElement("span");
            label.className = "input-label";
            label.textContent = `P${i + 1}`;

            const xInput = document.createElement("input");
            xInput.type = "number";
            xInput.step = "0.1";
            xInput.placeholder = "x";
            xInput.id = `pt_x_${i}`;
            xInput.value = points[i] ? points[i][0].toFixed(2) : "";

            const yInput = document.createElement("input");
            yInput.type = "number";
            yInput.step = "0.1";
            yInput.placeholder = "y";
            yInput.id = `pt_y_${i}`;
            yInput.value = points[i] ? points[i][1].toFixed(2) : "";

            row.appendChild(label);
            row.appendChild(xInput);
            row.appendChild(yInput);
            container.appendChild(row);
        }
    }

    function applyManualInput() {
        const newPoints = [];
        let valid = true;

        for (let i = 0; i < mode; i++) {
            const x = parseFloat($(`pt_x_${i}`).value);
            const y = parseFloat($(`pt_y_${i}`).value);

            if (isNaN(x) || isNaN(y)) {
                valid = false;
                break;
            }
            newPoints.push([x, y]);
        }

        if (!valid) {
            showError("Please enter valid coordinates for all points.");
            return;
        }

        points = newPoints;
        updatePointsList();
        Animation.setPrecisionPoints([...points]);
        clearResult();
        updateStatus(`${mode} points applied from manual input.`);
    }

    // ─────────────────────────────────────────────
    // POINTS LIST DISPLAY
    // ─────────────────────────────────────────────
    function updatePointsList() {
        const list = $("pointsList");
        list.innerHTML = "";

        points.forEach((pt, i) => {
            const item = document.createElement("div");
            item.className = "point-item";

            const coords = document.createElement("span");
            coords.textContent = `P${i + 1}: (${pt[0].toFixed(2)}, ${pt[1].toFixed(2)})`;

            const removeBtn = document.createElement("button");
            removeBtn.className = "btn-remove";
            removeBtn.textContent = "×";
            removeBtn.title = "Remove point";
            removeBtn.addEventListener("click", () => {
                points.splice(i, 1);
                updateManualInputFields();
                updatePointsList();
                Animation.setPrecisionPoints([...points]);
                clearResult();
            });

            item.appendChild(coords);
            item.appendChild(removeBtn);
            list.appendChild(item);
        });

        $("pointCount").textContent = `${points.length} / ${mode}`;
    }

    // ─────────────────────────────────────────────
    // SYNTHESIS
    // ─────────────────────────────────────────────
    function bindSynthesizeButton() {
        $("synthesizeBtn").addEventListener("click", runSynthesis);
        $("clearBtn").addEventListener("click", clearAll);
    }

    function runSynthesis() {
        if (points.length !== mode) {
            showError(`Please place exactly ${mode} precision points before synthesizing.`);
            return;
        }

        hideError();
        updateStatus("Running synthesis...");
        $("synthesizeBtn").disabled = true;

        // Use a timeout to allow UI to update
        setTimeout(() => {
            try {
                let result;
                if (mode === 4) {
                    result = Kinematics.synthesize4Point(points);
                } else {
                    result = Kinematics.synthesize5Point(points);
                }

                if (!result || !result.success) {
                    const msg = result ? result.error : "Synthesis returned null.";
                    showError(msg);
                    updateStatus("Synthesis failed.");
                    $("synthesizeBtn").disabled = false;
                    return;
                }

                currentResult = result;

                // Set mechanism in animation
                Animation.setMechanism(result.mechanism);

                // Update info panel
                updateInfoPanel(result);

                updateStatus("Synthesis complete! Use controls to animate.");
            } catch (err) {
                showError("Synthesis error: " + err.message);
                console.error(err);
                updateStatus("Synthesis error.");
            }

            $("synthesizeBtn").disabled = false;
        }, 50);
    }

    function updateInfoPanel(result) {
        const m = result.mechanism;
        const g = result.grashof;

        $("linkA").textContent = m.a.toFixed(3);
        $("linkB").textContent = m.b.toFixed(3);
        $("linkC").textContent = m.c.toFixed(3);
        $("linkD").textContent = m.d.toFixed(3);
        $("couplerP").textContent = m.p.toFixed(3);
        $("couplerQ").textContent = m.q.toFixed(3);

        $("grashofType").textContent = g.type;
        $("grashofType").className = g.isGrashof ? "value grashof-yes" : "value grashof-no";

        $("residualError").textContent = result.residual.toFixed(6);
        $("residualRow").style.display = mode === 5 ? "flex" : "none";

        $("groundA0").textContent = `(${m.A0[0].toFixed(2)}, ${m.A0[1].toFixed(2)})`;
        $("groundB0").textContent = `(${m.B0[0].toFixed(2)}, ${m.B0[1].toFixed(2)})`;

        // Show info panel
        $("infoPanel").classList.add("visible");
    }

    // ─────────────────────────────────────────────
    // ANIMATION CONTROLS
    // ─────────────────────────────────────────────
    function bindAnimationControls() {
        $("playBtn").addEventListener("click", () => {
            if (!Animation.mechanism) {
                showError("Synthesize a mechanism first.");
                return;
            }
            if (Animation.isPlaying) {
                Animation.pause();
                $("playBtn").innerHTML = '<span class="icon">▶</span> Play';
                $("playBtn").classList.remove("active");
            } else {
                Animation.play();
                $("playBtn").innerHTML = '<span class="icon">⏸</span> Pause';
                $("playBtn").classList.add("active");
            }
        });

        $("resetBtn").addEventListener("click", () => {
            Animation.reset();
            $("playBtn").innerHTML = '<span class="icon">▶</span> Play';
            $("playBtn").classList.remove("active");
        });

        $("speedSlider").addEventListener("input", (e) => {
            const val = parseFloat(e.target.value);
            Animation.setSpeed(val);
            $("speedValue").textContent = `${(val / 0.02).toFixed(1)}×`;
        });
    }

    // ─────────────────────────────────────────────
    // TOGGLES
    // ─────────────────────────────────────────────
    function bindToggles() {
        $("toggleCurve").addEventListener("change", (e) => {
            Animation.setShowCurve(e.target.checked);
        });
        $("togglePoints").addEventListener("change", (e) => {
            Animation.setShowPoints(e.target.checked);
        });
    }

    // ─────────────────────────────────────────────
    // EXPORT / IMPORT
    // ─────────────────────────────────────────────
    function bindExportImport() {
        $("exportBtn").addEventListener("click", exportJSON);
        $("importBtn").addEventListener("click", () => $("importFile").click());
        $("importFile").addEventListener("change", importJSON);
    }

    function exportJSON() {
        if (!currentResult || !currentResult.success) {
            showError("Nothing to export. Synthesize a mechanism first.");
            return;
        }

        const data = {
            mode,
            precisionPoints: points,
            mechanism: currentResult.mechanism,
            grashof: currentResult.grashof,
            residual: currentResult.residual,
            crankAngles: currentResult.crank_angles,
            exportDate: new Date().toISOString(),
        };

        const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `four_bar_mechanism_${mode}pt.json`;
        a.click();
        URL.revokeObjectURL(url);

        updateStatus("Mechanism data exported as JSON.");
    }

    function importJSON(e) {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (evt) => {
            try {
                const data = JSON.parse(evt.target.result);

                if (!data.mechanism || !data.precisionPoints) {
                    showError("Invalid file format.");
                    return;
                }

                // Restore state
                mode = data.mode || 4;
                setMode(mode);
                points = data.precisionPoints;

                updateManualInputFields();
                updatePointsList();
                Animation.setPrecisionPoints([...points]);

                // Restore mechanism
                currentResult = {
                    success: true,
                    mechanism: data.mechanism,
                    grashof: data.grashof || Kinematics.checkGrashof(
                        data.mechanism.a, data.mechanism.b,
                        data.mechanism.c, data.mechanism.d
                    ),
                    residual: data.residual || 0,
                    crank_angles: data.crankAngles || [],
                };

                Animation.setMechanism(data.mechanism);
                updateInfoPanel(currentResult);

                hideError();
                updateStatus("Mechanism loaded from file.");
            } catch (err) {
                showError("Failed to parse file: " + err.message);
            }
        };
        reader.readAsText(file);
        e.target.value = ""; // Reset file input
    }

    // ─────────────────────────────────────────────
    // ERROR & STATUS DISPLAY
    // ─────────────────────────────────────────────
    function showError(msg) {
        const el = $("errorDisplay");
        el.textContent = msg;
        el.classList.add("visible");
    }

    function hideError() {
        const el = $("errorDisplay");
        el.classList.remove("visible");
    }

    function updateStatus(msg) {
        $("statusText").textContent = msg;
    }

    // ─────────────────────────────────────────────
    // CLEAR ALL
    // ─────────────────────────────────────────────
    function clearAll() {
        points = [];
        currentResult = null;
        Animation.clear();
        updateManualInputFields();
        updatePointsList();
        hideError();

        $("infoPanel").classList.remove("visible");
        $("playBtn").innerHTML = '<span class="icon">▶</span> Play';
        $("playBtn").classList.remove("active");

        updateStatus("All cleared. Click to place points.");
        Animation.redraw();
    }

    function clearResult() {
        currentResult = null;
        Animation.clear();
        Animation.setPrecisionPoints([...points]);
        $("infoPanel").classList.remove("visible");
        $("playBtn").innerHTML = '<span class="icon">▶</span> Play';
        $("playBtn").classList.remove("active");
    }

    // ─────────────────────────────────────────────
    // PUBLIC API
    // ─────────────────────────────────────────────
    return { init };
})();

// ─────────────────────────────────────────────
// LAUNCH
// ─────────────────────────────────────────────
document.addEventListener("DOMContentLoaded", UI.init);
