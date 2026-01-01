/**
 * SRO - Soldering Reflow Oven
 * Copyright (c) 2025-2030 Intector
 *
 * SRO Manual Control
 *
 */

// -----------------------------------------------------------------------------
// class ManualControl
// -----------------------------------------------------------------------------

class ManualControl {
    constructor() {
        this.isInitialized = false;
        this.ManualChart = new SRO_Chart();
        this.manualControlChartCanvas = null;
        this.setTemperature = 200;
        this.scanLine = 0;

        this.isDragging = false;
        this.dragOffset = 0;

        this.heaterState = false;
        this.fanState = false;
        this.selectedDoorPreset = null;
        this.currentDoorPosition = 0;

        // temperature data logging variables
        this.currentTemperature = 25;
        this.dataLoggerTimer = null;
        this.startTime = null;
        this.loggedSeconds = 0;

        // Chart update control (like Dashboard)
        this.chartLocked = false;
        this.chartUpdateTimer = null;
    }

    // ------------------------------------------------------------------------
    // ManualControl init
    // ------------------------------------------------------------------------
    async init() {
        if (this.isInitialized) return;

        console.log('Initializing Manual Control');

        this.manualControlChartCanvas = document.getElementById('ManualChart');
        this.ManualChart.Init(this.manualControlChartCanvas);

        this.setupDoorSlider();
        this.ChartInit();

        // Setup all event listeners
        this.setupEventListeners();

        this.isInitialized = true;
        console.log('Manual Control initialized');
    }

    // ------------------------------------------------------------------------
    // Manual Chart init
    // ------------------------------------------------------------------------
    ChartInit() {
        if (this.ManualChart.ReflowChart) {
            this.ManualChart.ReflowChart.options.plugins.title.text = 'Manual Chart';
            this.ManualChart.ReflowChart.options.plugins.subtitle.text = 'manual temperature testing chart';
            this.ManualChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.display = true;
            this.ManualChart.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.display = true;
            this.ManualChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.display = true;

            this.ManualChart.setTemperature = this.setTemperature;
            this.updateSetTemperatureLine();

            this.ManualChart.ReflowChart.data = {
                datasets: [
                    {
                        label: "actual Temperature",
                        data: this.ManualChart.ActValues,
                        backgroundColor: this.ManualChart.ChartColors.AutoActValues.Fill,
                        borderColor: this.ManualChart.ChartColors.AutoActValues.Stroke,
                        pointBackgroundColor: this.ManualChart.ChartColors.AutoActValues.Stroke,
                        pointHoverBackgroundColor: this.ManualChart.ChartColors.AutoActValues.Stroke,
                        fill: false,
                        tension: 0,
                        pointRadius: 2,
                        xAxisID: 'x',
                        hidden: false,
                    }
                ]
            }

            this.ManualChart.ReflowChart.update();

            // Start 5-second update timer
            this.startChartUpdates();

            // Add drag functionality
            this.setupSetTemperatureLineDrag();

        }

    }

    // ------------------------------------------------------------------------
    // Start periodic chart updates (every 5 seconds)
    // ------------------------------------------------------------------------
    startChartUpdates() {
        if (this.chartUpdateTimer) {
            clearInterval(this.chartUpdateTimer);
        }

        this.chartLocked = false;
        console.log('Starting periodic manual chart updates');

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
        console.log('Stopped manual chart updates and locked chart');
    }

    // ------------------------------------------------------------------------
    // Update chart Y-axis with current temperature
    // ------------------------------------------------------------------------
    updateChartWithCurrentTemp() {
        if (!this.ManualChart.ReflowChart) {
            console.log('No ManualChart - returning');
            return;
        }

        const tempToUse = Math.round(this.currentTemperature);

        console.log('Updating manual chart Y-axis to:', Math.floor(tempToUse / 10) * 10);
        this.ManualChart.ReflowChart.options.scales.y.min = Math.floor(tempToUse / 10) * 10;
        this.ManualChart.ReflowChart.update('none');
    }

    // ------------------------------------------------------------------------
    // setup temperature line drag
    // ------------------------------------------------------------------------
    setupSetTemperatureLineDrag() {
        // Add a small delay to ensure chart is fully rendered
        setTimeout(() => {
            const canvas = this.manualControlChartCanvas;
            if (!canvas) {
                console.log('Canvas not found for drag setup');
                return;
            }

            console.log('Setting up drag functionality on canvas:', canvas.id);

            // Remove any existing listeners to prevent duplicates
            canvas.removeEventListener('mousedown', this.handleMouseDown);
            canvas.removeEventListener('mousemove', this.handleMouseMove);
            canvas.removeEventListener('mouseup', this.handleMouseUp);
            canvas.removeEventListener('mouseleave', this.handleMouseUp);

            // Add bound event listeners
            this.boundMouseDown = (e) => this.handleMouseDown(e);
            this.boundMouseMove = (e) => this.handleMouseMove(e);
            this.boundMouseUp = (e) => this.handleMouseUp(e);

            canvas.addEventListener('mousedown', this.boundMouseDown);
            canvas.addEventListener('mousemove', this.boundMouseMove);
            canvas.addEventListener('mouseup', this.boundMouseUp);
            canvas.addEventListener('mouseleave', this.boundMouseUp);

            console.log('Drag functionality setup complete');
        }, 100);
    }

    // ------------------------------------------------------------------------
    // mouse down event handler
    // ------------------------------------------------------------------------
    handleMouseDown(e) {
        // Don't allow dragging when heater is ON
        if (this.heaterState) return;

        console.log('Mouse down event triggered');

        if (!this.ManualChart.ReflowChart) {
            console.log('Chart not available');
            return;
        }

        const canvas = this.manualControlChartCanvas;
        const rect = canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;

        console.log(`Mouse position: x=${x}, y=${y}`);

        // Check if click is within chart area
        const chartArea = this.ManualChart.ReflowChart.chartArea;
        if (!chartArea || x < chartArea.left || x > chartArea.right || y < chartArea.top || y > chartArea.bottom) {
            console.log('Click outside chart area');
            return;
        }

        // Get line position
        const lineYPixel = this.ManualChart.ReflowChart.scales.y.getPixelForValue(this.setTemperature);
        console.log(`Line Y pixel: ${lineYPixel}, setTemperature: ${this.setTemperature}`);

        // Check if click is near the setTemperatureLine (within 15 pixels for easier targeting)
        if (Math.abs(y - lineYPixel) <= 15) {
            console.log('Starting drag operation');
            this.isDragging = true;
            this.dragOffset = y - lineYPixel;
            canvas.style.cursor = 'ns-resize';

            // Prevent default to avoid text selection
            e.preventDefault();
        }
    }

    // ------------------------------------------------------------------------
    // mouse move event handler
    // ------------------------------------------------------------------------
    handleMouseMove(e) {
        if (!this.ManualChart.ReflowChart) return;

        const canvas = this.manualControlChartCanvas;
        const rect = canvas.getBoundingClientRect();
        const y = e.clientY - rect.top;

        if (this.isDragging) {
            console.log('Dragging, y position:', y);

            // Convert pixel position to temperature value
            const newTemp = this.ManualChart.ReflowChart.scales.y.getValueForPixel(y - this.dragOffset);

            // Clamp temperature to reasonable bounds
            const clampedTemp = Math.max(25, Math.min(400, newTemp));

            // Update setTemperature
            this.setTemperature = Math.round(clampedTemp);
            console.log('New setTemperature:', this.setTemperature);

            // Update the line position
            this.updateSetTemperatureLine();

            // Update chart
            this.ManualChart.ReflowChart.update('none');

            // Prevent default
            e.preventDefault();
        } else {
            // Check if hovering over line for cursor change
            const chartArea = this.ManualChart.ReflowChart.chartArea;
            if (chartArea && y >= chartArea.top && y <= chartArea.bottom) {
                const lineYPixel = this.ManualChart.ReflowChart.scales.y.getPixelForValue(this.setTemperature);
                if (Math.abs(y - lineYPixel) <= 15) {
                    canvas.style.cursor = 'ns-resize';
                } else {
                    canvas.style.cursor = 'default';
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // mouse up event handler
    // ------------------------------------------------------------------------
    handleMouseUp(e) {
        if (this.isDragging) {
            console.log('Ending drag operation');
            this.isDragging = false;
            this.manualControlChartCanvas.style.cursor = 'default';
        }
    }

    // ------------------------------------------------------------------------
    // helper methods
    // ------------------------------------------------------------------------
    updateSetTemperatureLine() {
        if (!this.ManualChart.ReflowChart) return;

        const btn_label = document.getElementById('heater_btn_label');

        // Update the annotation line position
        this.ManualChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.yMin = this.setTemperature;
        this.ManualChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.yMax = this.setTemperature;

        // Update the label content to show current temperature
        this.ManualChart.ReflowChart.options.plugins.annotation.annotations.setTemperatureLine.label.content = `Set-Temp: ${this.setTemperature}°C`;
        btn_label.innerText = `Heating to ${this.setTemperature}°C`;
    }

    // ------------------------------------------------------------------------
    // Door Slider init
    // ------------------------------------------------------------------------
    setupDoorSlider() {
        const doorSlider = document.getElementById('rangeDoor');
        const doorValueDisplay = document.getElementById('doorValueDisplay');

        if (doorSlider && doorValueDisplay) {
            doorSlider.addEventListener('input', (e) => {
                const value = parseInt(e.target.value);

                // Update display
                doorValueDisplay.textContent = value + '%';

                // Update preset buttons
                this.updateDoorPresetButtons(value);

                // Send WebSocket command
                window.SroWebSocket.sendCommand('cmdDoor', { position: value });
            });

            // Set initial state
            const initialValue = parseInt(doorSlider.value);
            doorValueDisplay.textContent = initialValue + '%';
            this.updateDoorPresetButtons(initialValue);
        }
    }

    // ------------------------------------------------------------------------
    // toggle heater button
    // ------------------------------------------------------------------------
    toggleHeater() {
        this.heaterState = !this.heaterState;

        const btn = document.getElementById('btnHeatingElements');

        if (this.heaterState) {
            // Turn ON
            btn.classList.remove('btn-secondary');
            btn.classList.add('btn-success', 'active');
            btn.textContent = 'ON';

            // Clear chart and start data logging
            this.clearChartData();
            this.stopChartUpdates()
            this.updateChartWithCurrentTemp();
            this.startDataLogging();

            // Send single command to ESP32 with target temperature
            window.SroWebSocket.sendCommand('cmdHeater', {
                state: 'on',
                targetTemperature: this.setTemperature
            });

        } else {
            // Turn OFF
            btn.classList.remove('btn-success', 'active');
            btn.classList.add('btn-secondary');
            btn.textContent = 'OFF';

            // Stop data logging
            this.stopDataLogging();

            // Send single command to ESP32 with target temperature
            window.SroWebSocket.sendCommand('cmdHeater', {
                state: 'off',
                targetTemperature: 0
            });
        }
    }

    // ------------------------------------------------------------------------
    // toggle fan button
    // ------------------------------------------------------------------------
    toggleFan() {
        this.fanState = !this.fanState;

        const btn = document.getElementById('btnConvectionFan');

        if (this.fanState) {
            // Turn ON
            btn.classList.remove('btn-secondary');
            btn.classList.add('btn-success', 'active');
            btn.textContent = 'ON';
            window.SroWebSocket.sendCommand('cmdFan', { state: 'on' });
        } else {
            // Turn OFF
            btn.classList.remove('btn-success', 'active');
            btn.classList.add('btn-secondary');
            btn.textContent = 'OFF';
            window.SroWebSocket.sendCommand('cmdFan', { state: 'off' });
        }
    }

    // ------------------------------------------------------------------------
    // Door Preset Implementation
    // ------------------------------------------------------------------------
    setDoorPreset(position) {
        // Clear all door preset buttons
        ['btnDoor0', 'btnDoor25', 'btnDoor50', 'btnDoor75', 'btnDoor100'].forEach(id => {
            const btn = document.getElementById(id);
            btn.classList.remove('btn-success', 'active');
            btn.classList.add('btn-secondary');
        });

        // Set the clicked button as active
        const activeBtn = document.getElementById(`btnDoor${position}`);
        activeBtn.classList.remove('btn-secondary');
        activeBtn.classList.add('btn-success', 'active');

        // Update slider position
        const slider = document.getElementById('rangeDoor');
        slider.value = position;

        // Update display
        document.getElementById('doorValueDisplay').textContent = position + '%';

        // Send WebSocket command
        window.SroWebSocket.sendCommand('cmdDoor', { position: position });

        // Track state
        this.selectedDoorPreset = position;
    }

    // ------------------------------------------------------------------------
    // update heater button (for hardware status sync)
    // ------------------------------------------------------------------------
    updateHeaterButton() {
        const btn = document.getElementById('btnHeatingElements');
        if (!btn) return;

        if (this.heaterState) {
            // Turn ON - visual state only, no WebSocket command
            btn.classList.remove('btn-secondary');
            btn.classList.add('btn-success', 'active');
            btn.textContent = 'ON';
        } else {
            // Turn OFF - visual state only, no WebSocket command
            btn.classList.remove('btn-success', 'active');
            btn.classList.add('btn-secondary');
            btn.textContent = 'OFF';
        }
    }

    // ------------------------------------------------------------------------
    // update fan button (for hardware status sync)
    // ------------------------------------------------------------------------
    updateFanButton() {
        const btn = document.getElementById('btnConvectionFan');
        if (!btn) return;

        if (this.fanState) {
            // Turn ON - visual state only, no WebSocket command
            btn.classList.remove('btn-secondary');
            btn.classList.add('btn-success', 'active');
            btn.textContent = 'ON';
        } else {
            // Turn OFF - visual state only, no WebSocket command
            btn.classList.remove('btn-success', 'active');
            btn.classList.add('btn-secondary');
            btn.textContent = 'OFF';
        }
    }

    // ------------------------------------------------------------------------
    // Slider-to-Preset Button Synchronization
    // ------------------------------------------------------------------------
    updateDoorPresetButtons(sliderValue) {
        // Clear all door preset buttons first
        ['btnDoor0', 'btnDoor25', 'btnDoor50', 'btnDoor75', 'btnDoor100'].forEach(id => {
            const btn = document.getElementById(id);
            btn.classList.remove('btn-success', 'active');
            btn.classList.add('btn-secondary');
        });

        // Check if slider value matches a preset button
        const presetButtons = {
            0: 'btnDoor0',
            25: 'btnDoor25',
            50: 'btnDoor50',
            75: 'btnDoor75',
            100: 'btnDoor100'
        };

        // If slider value matches a preset, activate that button
        if (presetButtons[sliderValue]) {
            const activeBtn = document.getElementById(presetButtons[sliderValue]);
            activeBtn.classList.remove('btn-secondary');
            activeBtn.classList.add('btn-success', 'active');
            this.selectedDoorPreset = sliderValue;
        } else {
            // Slider is at a non-preset value (10%, 20%, 30%, etc.)
            this.selectedDoorPreset = null; // No preset is active
        }
    }

    // ------------------------------------------------------------------------
    // setup event listener
    // ------------------------------------------------------------------------
    setupEventListeners() {
        // Heater button
        const heaterBtn = document.getElementById('btnHeatingElements');
        if (heaterBtn) {
            heaterBtn.addEventListener('click', () => this.toggleHeater());
        }

        // Fan button
        const fanBtn = document.getElementById('btnConvectionFan');
        if (fanBtn) {
            fanBtn.addEventListener('click', () => this.toggleFan());
        }

        // Door preset buttons
        document.getElementById('btnDoor0')?.addEventListener('click', () => this.setDoorPreset(0));
        document.getElementById('btnDoor25')?.addEventListener('click', () => this.setDoorPreset(25));
        document.getElementById('btnDoor50')?.addEventListener('click', () => this.setDoorPreset(50));
        document.getElementById('btnDoor75')?.addEventListener('click', () => this.setDoorPreset(75));
        document.getElementById('btnDoor100')?.addEventListener('click', () => this.setDoorPreset(100));

        // Enhanced door slider
        this.setupDoorSlider();
    }

    // ------------------------------------------------------------------------
    // hardware status update
    // ------------------------------------------------------------------------
    updateHardwareStatus(data) {
        // Store current temperature for data logging
        if (data.temp !== undefined) {
            this.currentTemperature = data.temp;
        }

        // Update door position if present
        if (data.door && data.door.position !== undefined) {
            const doorSlider = document.getElementById('rangeDoor');
            const doorValueDisplay = document.getElementById('doorValueDisplay');

            if (doorSlider && doorValueDisplay) {
                doorSlider.value = data.door.position;
                doorValueDisplay.textContent = data.door.position + '%';
                this.updateDoorPresetButtons(data.door.position);
            }
        }

        // Update fan state if present
        if (data.fan !== undefined) {
            this.fanState = data.fan;
            this.updateFanButton();
        }

        // Update heater state if present
        // if (data.heater !== undefined) {
            // this.heaterState = data.heater;
            // this.updateHeaterButton();
        // }
    }

    // ------------------------------------------------------------------------
    // Data Logging (1 second interval)
    // ------------------------------------------------------------------------
    startDataLogging() {
        console.log('Starting data logging every 1 second');

        // Clear existing data and reset counters
        this.clearChartData();
        this.startTime = Date.now();
        this.loggedSeconds = 0;

        // Start 1-second timer
        this.dataLoggerTimer = setInterval(() => {
            this.logTemperatureData();
        }, 1000);
    }

    stopDataLogging() {
        if (this.dataLoggerTimer) {
            clearInterval(this.dataLoggerTimer);
            this.dataLoggerTimer = null;
            console.log('Data logging stopped');
        }
    }

    logTemperatureData() {
        if (!this.ManualChart.ReflowChart) return;

        // Check if we've reached the maximum ticks
        if (this.loggedSeconds >= this.ManualChart.ChartMaxTicks) {
            console.log('Reached maximum chart ticks, stopping data logging');
            this.stopDataLogging();
            return;
        }

        // Add data point: {x: seconds since start, y: current temperature}
        const dataPoint = {
            x: this.loggedSeconds,
            y: this.currentTemperature
        };

        // Add to the actual temperature dataset (first dataset index 0)
        this.ManualChart.ReflowChart.data.datasets[0].data.push(dataPoint);

        // Update chart
        this.ManualChart.ReflowChart.update('none');

        console.log(`Logged data point: ${this.loggedSeconds}s = ${this.currentTemperature}°C`);

        this.loggedSeconds++;
    }

    clearChartData() {
        if (!this.ManualChart.ReflowChart) return;

        // Clear all data from the actual temperature dataset
        this.ManualChart.ReflowChart.data.datasets[0].data = [];
        this.ManualChart.ReflowChart.update('none');

        console.log('Chart data cleared');
    }

    // ------------------------------------------------------------------------
    // Cleanup method (call this when switching sections or destroying)
    // ------------------------------------------------------------------------
    cleanup() {
        this.stopDataLogging();
        console.log('ManualControl cleanup completed');
    }

}

// Create global instance
window.ManualControl = new ManualControl();
