/**
 * SRO - Soldering Reflow Oven
 * Copyright (c) 2025-2030 Intector
 *
 * SRO Tuning Manager Module - Simplified Version
 * Single 3-phase auto-tune: HEAT → COAST → COOL → ANALYZE
 * Uses Cohen-Coon formulas for PID calculation
 *
 */
// -----------------------------------------------------------------------------
// class TuningManager
// -----------------------------------------------------------------------------
class TuningManager {
    constructor() {
        this.isInitialized = false;
        this.TuningChart = new SRO_Chart();
        this.tuningChartCanvas = null;

        // Tuning process state
        this.isTuningActive = false;
        this.tuningStartTime = null;
        this.dataLoggerTimer = null;
        this.loggedSeconds = 0;
        this.lastPhase = null;

        this.isCollectOvenDataActive = false;

        // Target temperature (draggable line)
        this.setTemperature = 150;
        this.isDragging = false;
        this.dragOffset = 0;

        // Calculated PID parameters
        this.calculatedPID = {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0
        };

        // Performance metrics tracking
        this.performanceMetrics = {
            overshoot: {
                value: 0,
                peakTemp: 0,
                peakTime: 0,
                targetTemp: 0
            },
            settlingTime: {
                value: 0,
                tolerance: 2,
                settled: false
            },
            steadyStateError: {
                value: 0,
                samples: [],
                sampleSize: 30
            },
            riseTime: {
                value: 0,
                startTime: 0,
                endTime: 0,
                calculated: false
            }
        };

        // Temperature data logging
        this.currentTemperature = 25;
        this.currentPidOutput = 0;
        this.pidOutputData = [];

        // Chart update control (like Dashboard)
        this.chartLocked = false;
        this.chartUpdateTimer = null;

        this.pidFieldsInitialized = false;
    }

    // ------------------------------------------------------------------------
    // Initialize
    // ------------------------------------------------------------------------
    async init() {
        if (this.isInitialized) return;

        console.log('Initializing Tuning Manager');

        this.tuningChartCanvas = document.getElementById('TuningChart');
        this.TuningChart.Init(this.tuningChartCanvas);
        this.TuningChartInit();

        // Setup event listeners
        this.setupEventListeners();

        // Setup WebSocket handlers
        this.setupWebSocketHandlers();

        // Setup draggable temperature line
        this.setupSetTemperatureLineDrag();

        this.isInitialized = true;
        console.log('Tuning Manager initialized');
    }

    // ------------------------------------------------------------------------
    // Chart Initialization
    // ------------------------------------------------------------------------
    TuningChartInit() {
        if (!this.TuningChart.ReflowChart) return;

        this.TuningChart.ReflowChart.options.plugins.title.text = 'PID Auto-Tuning Chart';
        this.TuningChart.ReflowChart.options.plugins.subtitle.text = 'automatic temperature run to calculate the PID parameter';
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.display = true;
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.display = true;
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.display = true;

        this.TuningChart.setTemperature = this.setTemperature;
        this.updateSetTemperatureLine();

        // Right Y-axis for PID output
        this.TuningChart.ReflowChart.options.scales.y1 = {
            type: 'linear',
            position: 'right',
            display: true,
            title: {
                display: true,
                text: 'PID Output %',
                color: this.TuningChart.ChartColors.Y_Axis.FontColor,
                font: {
                    family: "'Ubuntu Condensed', sans-serif",
                    size: 14,
                    weight: "normal",
                }
            },
            ticks: {
                color: this.TuningChart.ChartColors.Y_Axis.Stroke,
                font: {
                    family: "'Ubuntu Mono', monospace",
                    size: 12,
                    weight: "normal",
                }
            },
            grid: { display: false },
            min: 0,
            max: 100
        };

        // Overshoot line annotation
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.overshootLine = {
            display: false,
            type: "line",
            drawTime: "afterDraw",
            yMin: 0,
            yMax: 0,
            borderColor: "#FF4081",
            borderWidth: 2,
            borderDash: [10, 5],
            label: {
                position: "end",
                backgroundColor: "#FF4081AA",
                color: "#FFFFFF",
                content: "Overshoot: 0%",
                font: { family: "'Ubuntu Condensed', sans-serif", size: 11, weight: "normal" },
                borderRadius: 5,
                yAdjust: -14,
                display: true,
            },
        };

        // Settling zone annotation
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.settlingZone = {
            display: false,
            type: "box",
            drawTime: "beforeDraw",
            xMin: 0, xMax: 0, yMin: 0, yMax: 0,
            borderColor: "#00E676AA",
            backgroundColor: "#00E67620",
            borderWidth: 1,
            borderDash: [3, 3],
            label: {
                position: "center",
                backgroundColor: "#00E676AA",
                color: "#FFFFFF",
                content: "Settling Zone (±2°C)",
                font: { family: "'Ubuntu Condensed', sans-serif", size: 10, weight: "normal" },
                display: true
            }
        };

        // Settling time line annotation
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.settlingTimeLine = {
            display: false,
            type: "line",
            drawTime: "afterDraw",
            xMin: 0, xMax: 0,
            borderColor: "#00E676",
            borderWidth: 2,
            borderDash: [5, 5],
            label: {
                position: "end",
                backgroundColor: "#00E676AA",
                color: "#FFFFFF",
                content: "Settling Time: 0s",
                font: { family: "'Ubuntu Condensed', sans-serif", size: 11, weight: "normal" },
                borderRadius: 5,
                yAdjust: 50,
                display: true,
            },
        };

        // Steady-state error annotation
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.steadyStateErrorLine = {
            display: false,
            type: "line",
            drawTime: "afterDraw",
            yMin: 0, yMax: 0,
            borderColor: "#FFC107",
            borderWidth: 2,
            label: {
                position: "start",
                backgroundColor: "#FFC107AA",
                color: "#000000",
                content: "Steady-State Error: 0°C",
                font: { family: "'Ubuntu Condensed', sans-serif", size: 11, weight: "normal" },
                borderRadius: 5,
                yAdjust: -14,
                display: true,
            },
        };

        // Rise time zone annotation
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.riseTimeZone = {
            display: false,
            type: "box",
            drawTime: "beforeDraw",
            xMin: 0, xMax: 0, yMin: 0, yMax: 0,
            borderColor: "#2196F3AA",
            backgroundColor: "#2196F320",
            borderWidth: 1,
            borderDash: [2, 2],
            label: {
                position: "center",
                backgroundColor: "#2196F3AA",
                color: "#FFFFFF",
                content: "Rise Time: 0s",
                font: { family: "'Ubuntu Condensed', sans-serif", size: 10, weight: "normal" },
                display: true
            }
        };

        // Dataset setup
        this.TuningChart.ReflowChart.data = {
            datasets: [
                {
                    label: "temperature",
                    data: this.TuningChart.ActValues,
                    backgroundColor: this.TuningChart.ChartColors.AutoActValues.Fill,
                    borderColor: this.TuningChart.ChartColors.AutoActValues.Stroke,
                    pointBackgroundColor: this.TuningChart.ChartColors.AutoActValues.Stroke,
                    fill: false,
                    tension: 0.3,
                    pointRadius: 1,
                    borderWidth: 2,
                    xAxisID: 'x',
                    yAxisID: 'y',
                    hidden: false,
                },
                {
                    label: "PID output",
                    data: [],
                    backgroundColor: this.TuningChart.ChartColors.SoakZone.Fill,
                    borderColor: this.TuningChart.ChartColors.SoakZone.Stroke,
                    pointBackgroundColor: this.TuningChart.ChartColors.SoakZone.Stroke,
                    fill: false,
                    tension: 0.3,
                    pointRadius: 1,
                    borderWidth: 2,
                    xAxisID: 'x',
                    yAxisID: 'y1',
                    hidden: false,
                }
            ]
        };

        // update chart with actual temperature
        const initialYMin = Math.floor(this.currentTemperature) - 10;
        this.TuningChart.ReflowChart.options.scales.y.min = initialYMin;

        this.TuningChart.ReflowChart.update();

        // Start 5-second update timer
        this.startChartUpdates();

        // Add drag functionality
        this.setupSetTemperatureLineDrag();

    }

    // ------------------------------------------------------------------------
    // Start periodic chart updates (every 5 seconds)
    // ------------------------------------------------------------------------
    startChartUpdates() {
        if (this.chartUpdateTimer) {
            clearInterval(this.chartUpdateTimer);
        }

        this.chartLocked = false;
        console.log('Starting periodic tuning chart updates');

        // Update immediately
        this.updateChartWithCurrentTemp();

        // Update every 5 seconds
        this.chartUpdateTimer = setInterval(() => {
            if (!this.chartLocked) {
                this.updateChartWithCurrentTemp();
            }
        }, 5000);
    }

    // ------------------------------------------------------------------------
    // Stop periodic chart updates and lock chart
    // ------------------------------------------------------------------------
    stopChartUpdates() {
        if (this.chartUpdateTimer) {
            clearInterval(this.chartUpdateTimer);
            this.chartUpdateTimer = null;
        }

        this.chartLocked = true;
        console.log('Stopped tuning chart updates and locked chart');
    }

    // ------------------------------------------------------------------------
    // Update chart Y-axis with current temperature
    // ------------------------------------------------------------------------
    updateChartWithCurrentTemp() {
        if (!this.TuningChart.ReflowChart) {
            console.log('No TuningChart - returning');
            return;
        }

        const tempToUse = Math.round(this.currentTemperature);

        console.log('Updating tuning chart Y-axis to:', Math.floor(tempToUse / 10) * 10);
        this.TuningChart.ReflowChart.options.scales.y.min = Math.floor(tempToUse / 10) * 10;
        this.TuningChart.ReflowChart.update('none');
    }

    // ------------------------------------------------------------------------
    // Update current temperature (called from main module)
    // ------------------------------------------------------------------------
    updateTemperature(temperature) {
        this.currentTemperature = temperature;
    }

    // ------------------------------------------------------------------------
    // Setup Event Listeners
    // ------------------------------------------------------------------------
    setupEventListeners() {
        // Start button
        const startBtn = document.getElementById('btnAutoTuneStart');
        if (startBtn) {
            startBtn.addEventListener('click', () => this.startAutoTuning());
        }

        // Stop button
        const stopBtn = document.getElementById('btnAutoTuneStop');
        if (stopBtn) {
            stopBtn.addEventListener('click', () => this.stopTuning());
        }

        // Save button
        const saveBtn = document.getElementById('btnAutoTuneSave');
        if (saveBtn) {
            saveBtn.addEventListener('click', () => this.saveTuningResults());
        }

        // collect oven data button
        const collectOvenDataBtn = document.getElementById('btnCollectOvenDataStart');
        if (collectOvenDataBtn) {
            collectOvenDataBtn.addEventListener('click', () => this.startCollectOvenData());
        }

        // Target temperature input
        const targetInput = document.getElementById('InputAutoTuneTargetTemp');
        if (targetInput) {
            targetInput.value = this.setTemperature;
            targetInput.addEventListener('change', () => {
                this.setTemperature = parseInt(targetInput.value) || 150;
                this.updateSetTemperatureLine();
                this.TuningChart.ReflowChart.update('none');
            });
        }
    }

    // ------------------------------------------------------------------------
    // Setup Draggable Temperature Line
    // ------------------------------------------------------------------------
    setupSetTemperatureLineDrag() {
        setTimeout(() => {
            const canvas = this.tuningChartCanvas;
            if (!canvas) {
                console.log('Tuning canvas not found for drag setup');
                return;
            }

            console.log('Setting up tuning chart drag functionality');

            // Remove any existing listeners to prevent duplicates
            canvas.removeEventListener('mousedown', this.handleMouseDown);
            canvas.removeEventListener('mousemove', this.handleMouseMove);
            canvas.removeEventListener('mouseup', this.handleMouseUp);
            canvas.removeEventListener('mouseleave', this.handleMouseUp);

            this.boundMouseDown = (e) => this.handleMouseDown(e);
            this.boundMouseMove = (e) => this.handleMouseMove(e);
            this.boundMouseUp = (e) => this.handleMouseUp(e);

            canvas.addEventListener('mousedown', this.boundMouseDown);
            canvas.addEventListener('mousemove', this.boundMouseMove);
            canvas.addEventListener('mouseup', this.boundMouseUp);
            canvas.addEventListener('mouseleave', this.boundMouseUp);

            console.log('Tuning chart drag setup complete');
        }, 100);
    }

    handleMouseDown(e) {
        if (this.isTuningActive) return;
        if (!this.TuningChart.ReflowChart) return;

        const canvas = this.tuningChartCanvas;
        const rect = canvas.getBoundingClientRect();
        const y = e.clientY - rect.top;

        const chartArea = this.TuningChart.ReflowChart.chartArea;
        if (!chartArea || y < chartArea.top || y > chartArea.bottom) return;

        const lineYPixel = this.TuningChart.ReflowChart.scales.y.getPixelForValue(this.setTemperature);

        if (Math.abs(y - lineYPixel) <= 15) {
            this.isDragging = true;
            this.dragOffset = y - lineYPixel;
            canvas.style.cursor = 'ns-resize';
            e.preventDefault();
        }
    }

    handleMouseMove(e) {
        if (!this.TuningChart.ReflowChart) return;

        const canvas = this.tuningChartCanvas;
        const rect = canvas.getBoundingClientRect();
        const y = e.clientY - rect.top;

        if (this.isDragging) {
            const newTemp = this.TuningChart.ReflowChart.scales.y.getValueForPixel(y - this.dragOffset);
            const clampedTemp = Math.max(50, Math.min(280, newTemp));

            this.setTemperature = Math.round(clampedTemp);
            this.updateSetTemperatureLine();
            this.TuningChart.ReflowChart.update('none');

            // Update the input field
            const targetInput = document.getElementById('InputAutoTuneTargetTemp');
            if (targetInput) {
                targetInput.value = this.setTemperature;
            }

            e.preventDefault();
        } else {
            // Hover cursor change
            const chartArea = this.TuningChart.ReflowChart.chartArea;
            if (chartArea && y >= chartArea.top && y <= chartArea.bottom) {
                const lineYPixel = this.TuningChart.ReflowChart.scales.y.getPixelForValue(this.setTemperature);
                canvas.style.cursor = Math.abs(y - lineYPixel) <= 15 ? 'ns-resize' : 'default';
            }
        }
    }

    handleMouseUp(e) {
        if (this.isDragging) {
            this.isDragging = false;
            this.tuningChartCanvas.style.cursor = 'default';
        }
    }

    updateSetTemperatureLine() {
        if (!this.TuningChart.ReflowChart) return;

        // Update the annotation line position
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.yMin = this.setTemperature;
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.yMax = this.setTemperature;

        // Update the label content to show current temperature
        this.TuningChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.label.content = `Set-Temp: ${this.setTemperature}°C`;
    }

    // ------------------------------------------------------------------------
    // WebSocket Handlers
    // ------------------------------------------------------------------------
    setupWebSocketHandlers() {
        if (!window.SroWebSocket) {
            console.warn('SroWebSocket not available for TuningManager');
            return;
        }

        console.log('Setting up TuningManager WebSocket handlers');

        window.SroWebSocket.on('tuningResults', (data) => {
            this.handleTuningResults(data);
        });

        window.SroWebSocket.on('tuningProgress', (data) => {
            this.handleTuningProgress(data);
        });

        console.log('TuningManager WebSocket handlers setup complete');
    }

    // ------------------------------------------------------------------------
    // Handle Autotune Status (from regular broadcasts)
    // ------------------------------------------------------------------------
    handleAutotuneStatus(autotune, pid) {
        if (!autotune) return;

        console.log('Autotune status:', autotune.phase, autotune.progress + '%');

        // Track tuning state - initialize on first non-idle phase
        if (autotune.phase !== 'idle' && !this.isTuningActive) {
            this.isTuningActive = true;
            this.tuningStartTime = Date.now();
            // this.lastPhase = null;

            // Update Y-axis to current temp before locking
            this.updateChartWithCurrentTemp();

            // Now lock the chart
            this.stopChartUpdates();

            this.clearChartData();
            this.resetPerformanceMetrics();
        }

        // Show phase change notifications
        if (this.isTuningActive && autotune.phase !== this.lastPhase) {
            console.log('Phase changed:', this.lastPhase, '→', autotune.phase, 'message:', autotune.message);
            this.lastPhase = autotune.phase;

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Auto Tuning', autotune.message, 'info');
            } else {
                console.log('SroApp.showNotification not available');
            }
        }

        // Update chart with live data
        if (this.TuningChart && this.TuningChart.ReflowChart && this.isTuningActive && this.tuningStartTime) {
            const elapsedSec = (Date.now() - this.tuningStartTime) / 1000;

            // Temperature dataset
            this.TuningChart.ReflowChart.data.datasets[0].data.push({
                x: elapsedSec,
                y: autotune.currentTemp
            });

            // PID output dataset
            if (pid && pid.output !== undefined) {
                this.TuningChart.ReflowChart.data.datasets[1].data.push({
                    x: elapsedSec,
                    y: pid.output
                });
            }

            // Calculate performance metrics
            this.calculatePerformanceMetrics(elapsedSec, autotune.currentTemp, this.setTemperature);

            this.updateChartTimeAxis(elapsedSec);
            this.TuningChart.ReflowChart.update('none');
        }

        // Check for completion
        if (autotune.phase === 'idle' && this.isTuningActive) {
            this.isTuningActive = false;

            console.log('Auto-tuning complete - chart remains locked');

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Auto Tuning Complete',
                    autotune.message || 'Tuning finished', 'success');
            }
        }
    }

    // ------------------------------------------------------------------------
    // Handle Tuning Results
    // ------------------------------------------------------------------------
    handleTuningResults(data) {
        console.log('Received tuning results:', data);

        if (data.kp !== undefined) {
            this.calculatedPID = {
                kp: data.kp,
                ki: data.ki,
                kd: data.kd
            };

            // Update display
            const kpEl = document.getElementById('AutoTune-Kp');
            const kiEl = document.getElementById('AutoTune-Ki');
            const kdEl = document.getElementById('AutoTune-Kd');

            if (kpEl) kpEl.value = data.kp.toFixed(6);
            if (kiEl) kiEl.value = data.ki.toFixed(6);
            if (kdEl) kdEl.value = data.kd.toFixed(6);

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Tuning Complete',
                    `Kp=${data.kp.toFixed(6)}, Ki=${data.ki.toFixed(6)}, Kd=${data.kd.toFixed(6)}`, 'success');
            }
        } else if (data.success === false) {
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Tuning Failed',
                    data.message || 'Auto-tuning failed', 'danger');
            }
        }

        this.isTuningActive = false;
        this.stopDataLogging();
    }

    updateCurrentPID(kp, ki, kd) {
        // Only update if not actively tuning (don't overwrite calculated results)
        if (this.isTuningActive || this.pidFieldsInitialized) return;

        const kpEl = document.getElementById('AutoTune-Kp');
        const kiEl = document.getElementById('AutoTune-Ki');
        const kdEl = document.getElementById('AutoTune-Kd');

        if (kpEl && !kpEl.value) kpEl.value = kp.toFixed(6);
        if (kiEl && !kiEl.value) kiEl.value = ki.toFixed(6);
        if (kdEl && !kdEl.value) kdEl.value = kd.toFixed(6);

        this.pidFieldsInitialized = true;
    }

    // ------------------------------------------------------------------------
    // Handle Tuning Progress
    // ------------------------------------------------------------------------
    handleTuningProgress(data) {
        console.log('Received tuning progress:', data);

        if (data.liveData && this.TuningChart.ReflowChart) {
            const tempDataPoint = {
                x: data.liveData.timestamp,
                y: data.liveData.temperature
            };
            this.TuningChart.ReflowChart.data.datasets[0].data.push(tempDataPoint);

            if (data.liveData.pid_output !== undefined) {
                const pidDataPoint = {
                    x: data.liveData.timestamp,
                    y: data.liveData.pid_output
                };
                this.TuningChart.ReflowChart.data.datasets[1].data.push(pidDataPoint);
                this.pidOutputData.push(pidDataPoint);
            }

            this.TuningChart.ReflowChart.update('none');
        }
    }

    // ------------------------------------------------------------------------
    // Start Auto Tuning
    // ------------------------------------------------------------------------
    startAutoTuning() {
        if (this.isTuningActive) {
            console.log('Tuning already active');
            return;
        }

        console.log('Starting auto tuning');

        // Use the draggable line temperature directly
        const targetTemp = this.setTemperature;

        // Clear chart data
        this.clearChartData();
        this.tuningStartTime = Date.now();

        // Send command to ESP32
        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            window.SroWebSocket.sendCommand('startAutoTuning', {
                targetTemp: targetTemp
            });
        }

        this.isTuningActive = true;
        this.stopChartUpdates();

        if (window.SroApp && window.SroApp.showNotification) {
            window.SroApp.showNotification('Auto Tuning Started',
                'Heating to ' + targetTemp + '°C', 'info');
        }
    }

    // ------------------------------------------------------------------------
    // Stop Tuning
    // ------------------------------------------------------------------------
    stopTuning() {
        if (!this.isTuningActive) {
            console.log('No tuning process active');
            return;
        }

        console.log('Stopping tuning process');

        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            window.SroWebSocket.sendCommand('stopTuning');
        }

        this.isTuningActive = false;
        this.stopDataLogging();

        if (window.SroApp && window.SroApp.showNotification) {
            window.SroApp.showNotification('Tuning Stopped',
                'Auto-tuning aborted', 'warning');
        }
    }

    // ------------------------------------------------------------------------
    // Save Tuning Results
    // ------------------------------------------------------------------------
    saveTuningResults() {
        console.log('Saving tuning results');

        // Read from input fields instead of this.calculatedPID
        const kpEl = document.getElementById('AutoTune-Kp');
        const kiEl = document.getElementById('AutoTune-Ki');
        const kdEl = document.getElementById('AutoTune-Kd');

        const kp = parseFloat(kpEl?.value) || 0;
        const ki = parseFloat(kiEl?.value) || 0;
        const kd = parseFloat(kdEl?.value) || 0;

        if (kp === 0 && ki === 0 && kd === 0) {
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('No Results',
                    'No tuning results to save. Enter values or run auto-tune first.', 'warning');
            }
            return;
        }

        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            window.SroWebSocket.sendCommand('saveTuningParams', {
                kp: kp,
                ki: ki,
                kd: kd
            });
        }

        if (window.SroApp && window.SroApp.showNotification) {
            window.SroApp.showNotification('PID Saved',
                'Tuning results saved to device', 'success');
        }
    }

    // ------------------------------------------------------------------------
    // Collect Oven Data
    // ------------------------------------------------------------------------
    startCollectOvenData() {
        if (this.isCollectOvenDataActive) {
            console.log('Collecting oven data already active');
            return;
        }

        console.log('Starting collecting oven data');

        // Use the draggable line temperature directly
        const targetTemp = this.setTemperature;

        // Clear chart data
        this.clearChartData();
        this.tuningStartTime = Date.now();

        // Send command to ESP32
        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            window.SroWebSocket.sendCommand('startCollectingOvenData', {
                targetTemp: targetTemp
            });
        }

        this.isCollectOvenDataActive = true;
        this.stopChartUpdates();

        if (window.SroApp && window.SroApp.showNotification) {
            window.SroApp.showNotification('Collecting Oven Data started',
                'Heating to ' + targetTemp + '°C', 'info');
        }
    }


    // ------------------------------------------------------------------------
    // Data Logging
    // ------------------------------------------------------------------------
    startDataLogging() {
        console.log('Starting tuning data logging');

        this.clearChartData();
        this.tuningStartTime = Date.now();
        this.loggedSeconds = 0;
        this.resetPerformanceMetrics();

        this.dataLoggerTimer = setInterval(() => {
            this.logTuningData();
        }, 1000);
    }

    stopDataLogging() {
        if (this.dataLoggerTimer) {
            clearInterval(this.dataLoggerTimer);
            this.dataLoggerTimer = null;
            console.log('Tuning data logging stopped');
        }
    }

    logTuningData() {
        if (!this.TuningChart.ReflowChart) return;

        const tempDataPoint = { x: this.loggedSeconds, y: this.currentTemperature };
        this.TuningChart.ReflowChart.data.datasets[1].data.push(tempDataPoint);

        const pidDataPoint = { x: this.loggedSeconds, y: this.currentPidOutput };
        this.TuningChart.ReflowChart.data.datasets[2].data.push(pidDataPoint);

        const targetTemp = this.setTemperature;
        if (targetTemp > 0) {
            this.calculatePerformanceMetrics(this.loggedSeconds, this.currentTemperature, targetTemp);
        }

        this.updateChartTimeAxis(this.loggedSeconds);
        this.TuningChart.ReflowChart.update('none');

        this.loggedSeconds++;
    }

    updateChartTimeAxis(currentTime) {
        if (!this.TuningChart.ReflowChart) return;

        const currentMax = this.TuningChart.ReflowChart.options.scales.x.max || 300;

        if (currentTime > (currentMax - 10)) {
            const newMax = currentMax + 60;
            this.TuningChart.ReflowChart.options.scales.x.max = newMax;
            console.log(`Chart X-axis extended to ${newMax} seconds`);
        }
    }

    clearChartData() {
        if (!this.TuningChart.ReflowChart) return;

        if (this.TuningChart.ReflowChart.data.datasets.length > 1) {
            this.TuningChart.ReflowChart.data.datasets[0].data = [];
        }
        if (this.TuningChart.ReflowChart.data.datasets.length > 2) {
            this.TuningChart.ReflowChart.data.datasets[1].data = [];
        }

        const annotations = this.TuningChart.ReflowChart.options.plugins.annotation.annotations;
        if (annotations) {
            if (annotations.overshootLine) annotations.overshootLine.display = false;
            if (annotations.settlingZone) annotations.settlingZone.display = false;
            if (annotations.settlingTimeLine) annotations.settlingTimeLine.display = false;
            if (annotations.riseTimeZone) annotations.riseTimeZone.display = false;
            if (annotations.steadyStateErrorLine) annotations.steadyStateErrorLine.display = false;
        }

        this.pidOutputData = [];
        this.TuningChart.ReflowChart.update('none');
        console.log('Tuning chart data cleared');
    }

    // ------------------------------------------------------------------------
    // Hardware Status Update (from centralized app)
    // ------------------------------------------------------------------------
    updateHardwareStatus(data) {
        if (data.temp !== undefined) {
            this.currentTemperature = data.temp;
        }
        if (data.pid && data.pid.output !== undefined) {
            this.currentPidOutput = data.pid.output;
        }
    }

    // ------------------------------------------------------------------------
    // Performance Metrics
    // ------------------------------------------------------------------------
    calculatePerformanceMetrics(timeStamp, currentTemp, targetTemp) {
        if (!targetTemp || targetTemp === 0) return;

        const metrics = this.performanceMetrics;
        metrics.overshoot.targetTemp = targetTemp;

        // Overshoot
        if (currentTemp > targetTemp && currentTemp > metrics.overshoot.peakTemp) {
            metrics.overshoot.peakTemp = currentTemp;
            metrics.overshoot.peakTime = timeStamp;
            metrics.overshoot.value = ((currentTemp - targetTemp) / targetTemp) * 100;
            this.updateOvershootAnnotation(currentTemp, metrics.overshoot.value);
        }

        // Rise time (10% to 90%)
        if (!metrics.riseTime.calculated) {
            const tempRange = targetTemp - 25;
            const tenPercent = 25 + (tempRange * 0.1);
            const ninetyPercent = 25 + (tempRange * 0.9);

            if (currentTemp >= tenPercent && metrics.riseTime.startTime === 0) {
                metrics.riseTime.startTime = timeStamp;
            }
            if (currentTemp >= ninetyPercent && metrics.riseTime.startTime > 0 && metrics.riseTime.endTime === 0) {
                metrics.riseTime.endTime = timeStamp;
                metrics.riseTime.value = metrics.riseTime.endTime - metrics.riseTime.startTime;
                metrics.riseTime.calculated = true;
                this.updateRiseTimeAnnotation(metrics.riseTime.startTime, metrics.riseTime.endTime, tenPercent, ninetyPercent);
            }
        }

        // Settling time
        const tolerance = metrics.settlingTime.tolerance;
        const withinTolerance = Math.abs(currentTemp - targetTemp) <= tolerance;

        if (withinTolerance && !metrics.settlingTime.settled) {
            metrics.settlingTime.value = timeStamp;
            metrics.settlingTime.settled = true;
            this.updateSettlingTimeAnnotation(timeStamp, targetTemp, tolerance);
        } else if (!withinTolerance && metrics.settlingTime.settled) {
            metrics.settlingTime.settled = false;
        }

        // Steady-state error
        metrics.steadyStateError.samples.push(currentTemp);
        if (metrics.steadyStateError.samples.length > metrics.steadyStateError.sampleSize) {
            metrics.steadyStateError.samples.shift();
        }
        if (metrics.steadyStateError.samples.length >= 10) {
            const avgTemp = metrics.steadyStateError.samples.reduce((sum, t) => sum + t, 0) / metrics.steadyStateError.samples.length;
            metrics.steadyStateError.value = avgTemp - targetTemp;
            this.updateSteadyStateErrorAnnotation(avgTemp, metrics.steadyStateError.value);
        }
    }

    updateOvershootAnnotation(peakTemp, overshootPercent) {
        if (!this.TuningChart.ReflowChart) return;
        const annotation = this.TuningChart.ReflowChart.options.plugins.annotation.annotations.overshootLine;
        annotation.display = true;
        annotation.yMin = peakTemp;
        annotation.yMax = peakTemp;
        annotation.label.content = `Overshoot: ${overshootPercent.toFixed(1)}%`;
    }

    updateSettlingTimeAnnotation(settlingTime, targetTemp, tolerance) {
        if (!this.TuningChart.ReflowChart) return;

        const zoneAnnotation = this.TuningChart.ReflowChart.options.plugins.annotation.annotations.settlingZone;
        zoneAnnotation.display = true;
        zoneAnnotation.xMin = settlingTime;
        zoneAnnotation.xMax = this.TuningChart.ChartMaxTicks;
        zoneAnnotation.yMin = targetTemp - tolerance;
        zoneAnnotation.yMax = targetTemp + tolerance;

        const lineAnnotation = this.TuningChart.ReflowChart.options.plugins.annotation.annotations.settlingTimeLine;
        lineAnnotation.display = true;
        lineAnnotation.xMin = settlingTime;
        lineAnnotation.xMax = settlingTime;
        lineAnnotation.label.content = `Settling Time: ${settlingTime.toFixed(1)}s`;
    }

    updateRiseTimeAnnotation(startTime, endTime, tenPercent, ninetyPercent) {
        if (!this.TuningChart.ReflowChart) return;
        const annotation = this.TuningChart.ReflowChart.options.plugins.annotation.annotations.riseTimeZone;
        annotation.display = true;
        annotation.xMin = startTime;
        annotation.xMax = endTime;
        annotation.yMin = tenPercent;
        annotation.yMax = ninetyPercent;
        annotation.label.content = `Rise Time: ${(endTime - startTime).toFixed(1)}s`;
    }

    updateSteadyStateErrorAnnotation(avgTemp, error) {
        if (!this.TuningChart.ReflowChart) return;
        const annotation = this.TuningChart.ReflowChart.options.plugins.annotation.annotations.steadyStateErrorLine;
        annotation.display = true;
        annotation.yMin = avgTemp;
        annotation.yMax = avgTemp;
        annotation.label.content = `Steady-State Error: ${error.toFixed(1)}°C`;
    }

    resetPerformanceMetrics() {
        this.performanceMetrics = {
            overshoot: { value: 0, peakTemp: 0, peakTime: 0, targetTemp: 0 },
            settlingTime: { value: 0, tolerance: 2, settled: false },
            steadyStateError: { value: 0, samples: [], sampleSize: 30 },
            riseTime: { value: 0, startTime: 0, endTime: 0, calculated: false }
        };

        if (this.TuningChart.ReflowChart) {
            const annotations = this.TuningChart.ReflowChart.options.plugins.annotation.annotations;
            if (annotations.overshootLine) annotations.overshootLine.display = false;
            if (annotations.settlingZone) annotations.settlingZone.display = false;
            if (annotations.settlingTimeLine) annotations.settlingTimeLine.display = false;
            if (annotations.steadyStateErrorLine) annotations.steadyStateErrorLine.display = false;
            if (annotations.riseTimeZone) annotations.riseTimeZone.display = false;
        }
    }

    getPerformanceMetricsSummary() {
        const m = this.performanceMetrics;
        return {
            overshoot: { percentage: m.overshoot.value.toFixed(1), peakTemp: m.overshoot.peakTemp.toFixed(1) },
            settlingTime: { time: m.settlingTime.value.toFixed(1), tolerance: m.settlingTime.tolerance },
            steadyStateError: { error: m.steadyStateError.value.toFixed(2) },
            riseTime: { time: m.riseTime.value.toFixed(1), calculated: m.riseTime.calculated }
        };
    }

    // ------------------------------------------------------------------------
    // Cleanup
    // ------------------------------------------------------------------------
    cleanup() {
        console.log('TuningManager cleanup started');

        if (this.isTuningActive) {
            this.stopTuning();
        }
        this.stopDataLogging();
        this.clearChartData();

        console.log('TuningManager cleanup completed');
    }

    destroy() {
        console.log('Destroying TuningManager');
        this.cleanup();

        if (this.TuningChart && this.TuningChart.ReflowChart) {
            this.TuningChart.Destroy();
        }

        this.isInitialized = false;
        console.log('TuningManager destroyed');
    }
}

// Create global instance
window.TuningManager = new TuningManager();
