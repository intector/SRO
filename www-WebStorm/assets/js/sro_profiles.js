/**
 * SRO - Soldering Reflow Oven
 * Copyright (c) 2025-2030 Intector
 *
 * SRO Soldering Profiles
 *
 */

// -----------------------------------------------------------------------------
// class ProfileManager
// -----------------------------------------------------------------------------

class ProfileManager {
    constructor() {
        this.currentSelectedId = 0;  // Currently selected in combo box
        this.profiles = [];
        this.workProfile = null;     // Currently loaded profile data
        this.isInitialized = false;
        this.ProfileChartCanvas = null;
        this.ProfileChart = new SRO_Chart();
        this.isProfileRunning = false;
        this.SolderMeltingLine = 195;

        this.dataLoggerTimer = null;
        this.loggedSeconds = 0;
        this.currentTemperature = 25.0;
    }

    // ------------------------------------------------------------------------
    // ProfileManager init
    // ------------------------------------------------------------------------
    async init() {
        if (this.isInitialized) return;

        console.log('Initializing Profile Manager');

        // Setup event listeners
        this.setupEventListeners();

        // Setup WebSocket handlers for ESP32 communication (for profile execution only)
        this.setupWebSocketHandlers();

        // Initialize chart canvas
        this.ProfileChartCanvas = document.getElementById('ReflowChart');
        this.ProfileChart.Init(this.ProfileChartCanvas);
        this.ProfileChartInit();

        // -------------------------------------------------------------------------
        // Load profile index from ESP32 via HTTP
        // -------------------------------------------------------------------------
        try {
            console.log('Fetching profile names from ESP32...');
            const namesResponse = await fetch('/api/profiles/get?name=profile_names');

            if (!namesResponse.ok) {
                throw new Error(`HTTP ${namesResponse.status}: ${namesResponse.statusText}`);
            }

            const profileNames = await namesResponse.json();
            console.log('Received profile names:', profileNames);

            // Check if we got valid data
            if (!profileNames || typeof profileNames !== 'object') {
                throw new Error('Invalid profile names data structure');
            }

            // Build profiles array with metadata only
            this.profiles = Object.values(profileNames)
                .map(p => {
                    console.log('Processing profile:', p);

                    return {
                        id: p.id,
                        name: p.name,
                        description: p.description,
                        fileName: p.file_name
                    };
                })
                .sort((a, b) => a.id - b.id);  // Sort by id

            console.log('Built profiles array:', this.profiles);

            // Validate we have at least one profile
            if (this.profiles.length === 0) {
                throw new Error('No profiles found in profile_names.json');
            }

            // Populate dropdown with profile names
            this.updateProfileSelector();

            // Set first profile as selected
            this.currentSelectedId = 0;

            // Automatically load first profile data from ESP32
            await this.loadSelectedProfile();

        } catch (error) {
            console.error('Failed to load profile index:', error);

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification(
                    'Profile Index Error',
                    `Failed to load profile list from ESP32: ${error.message}. Please refresh the page.`,
                    'danger',
                    0
                );
            }

            return;
        }

        this.isInitialized = true;
        console.log('Profile Manager initialized');
    }

    // ------------------------------------------------------------------------
    // Profile Chart init
    // ------------------------------------------------------------------------
    ProfileChartInit() {
        if (this.ProfileChart.ReflowChart) {
            this.ProfileChart.ReflowChart.options.plugins.title.text = 'Reflow Profile Chart';
            this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.solderMeltingLine.display = true;
            this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.peakTemperatureLine.display = true;
            this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.display = true;
            this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.soakTemperatureBox.display = true;
            this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.reflowTemperatureBox.display = true;

            this.ProfileChart.ReflowChart.data = {
                datasets: [
                    {
                        label: "set value",
                        data: this.ProfileChart.SetValues,
                        backgroundColor: this.ProfileChart.ChartColors.SetValues.Fill,
                        borderColor: this.ProfileChart.ChartColors.SetValues.Stroke,
                        pointBackgroundColor: this.ProfileChart.ChartColors.SetValues.Stroke,
                        pointHoverBackgroundColor: this.ProfileChart.ChartColors.SetValues.Stroke,
                        fill: false,
                        tension: 0,
                        pointRadius: 2,
                        borderWidth: 1,
                        xAxisID: 'x',
                        hidden: false,
                    },
                    {
                        label: "act values",
                        data: this.ProfileChart.ActValues,
                        backgroundColor: this.ProfileChart.ChartColors.AutoActValues.Fill,
                        borderColor: this.ProfileChart.ChartColors.AutoActValues.Stroke,
                        pointBackgroundColor: this.ProfileChart.ChartColors.AutoActValues.Stroke,
                        pointHoverBackgroundColor: this.ProfileChart.ChartColors.AutoActValues.Stroke,
                        fill: false,
                        tension: 0,
                        pointRadius: 2,
                        borderWidth: 1,
                        xAxisID: 'x',
                        hidden: false,
                    }
                ]
            }

            const tempToUse = this.sensorError ? 25 : Math.round(this.currentTemperature);
            this.ProfileChart.UpdateProfileChartData(this.workProfile, tempToUse);

            // this.ProfileChart.UpdateProfileChartData(this.workProfile);
            // this.ProfileChart.ReflowChart.update();
        }

    }

    // ------------------------------------------------------------------------
    // Setup WebSocket event handlers for ESP32 integration (ENHANCED)
    // ------------------------------------------------------------------------
    setupWebSocketHandlers() {
        if (window.SroWebSocket) {
            console.log('Setting up ESP32 profile WebSocket handlers');

            // Handle profile started event from ESP32
            window.SroWebSocket.on('profileStarted', (data) => {
                console.log('Profile started event received:', data);
                this.handleProfileStarted(data);
            });

            // Handle profile stopped event from ESP32
            window.SroWebSocket.on('profileStopped', (data) => {
                console.log('Profile stopped event received:', data);
                this.handleProfileStopped(data);
            });

            // Handle profile progress updates from ESP32
            window.SroWebSocket.on('profileProgress', (data) => {
                console.log('Profile progress event received:', data);
                this.handleProfileProgress(data);
            });

            // Handle profile completed event from ESP32
            window.SroWebSocket.on('profileCompleted', (data) => {
                console.log('Profile completed event received:', data);
                this.handleProfileCompleted(data);
            });

            // 🆕 NEW: Add temperature update handler for chart integration
            window.SroWebSocket.on('tempUpdate', (data) => {
                this.handleTemperatureUpdate(data);
            });

            // 🆕 NEW: Also handle alternative event name if it exists
            window.SroWebSocket.on('temperatureUpdate', (data) => {
                this.handleTemperatureUpdate(data);
            });

            console.log('ESP32 profile WebSocket handlers setup complete');
        } else {
            console.warn('SroWebSocket not available, ESP32 profile integration disabled');
        }
    }

    // ------------------------------------------------------------------------
    // setup event listeners
    // ------------------------------------------------------------------------
    setupEventListeners() {
        // Profile selector change - just updates selection, doesn't load
        const profileSelector = document.getElementById('idSRO_ParSetSel');
        if (profileSelector) {
            profileSelector.addEventListener('change', (e) => {
                this.currentSelectedId = parseInt(e.target.value) - 1; // Convert 1-based to 0-based
                console.log('Profile selected:', this.currentSelectedId, this.profiles[this.currentSelectedId]?.name);
            });
        }

        // Load profile button (btnProfileLoadSet = "load set")
        const loadBtn = document.getElementById('btnProfileLoadSet');
        if (loadBtn) {
            loadBtn.addEventListener('click', () => {
                this.loadSelectedProfile();
            });
        }

        // Save profile button (btnProfileSaveSet = "save set")
        const saveBtn = document.getElementById('btnProfileSaveSet');
        if (saveBtn) {
            saveBtn.addEventListener('click', () => {
                this.saveToSelectedProfile();
            });
        }

        // RUN button for profile execution (start/stop toggle)
        const runBtn = document.getElementById('btnProfileRun');
        if (runBtn) {
            runBtn.addEventListener('click', () => {
                this.toggleProfileExecution();
            });
        }

        // Setup form field listeners for real-time work profile updates
        this.setupFormFieldListeners();

        // window.SroWebSocket.on('profileStarted', (data) => this.handleProfileStarted(data));

    }

    // ------------------------------------------------------------------------
    // setup form field listeners
    // ------------------------------------------------------------------------
    setupFormFieldListeners() {
        const fieldIds = [
            'InputPreheatTempMax', 'InputPreheatTime',
            'InputSoakTempMax', 'InputSoakTime',
            'InputReflowTempMin', 'InputReflowTempMax', 'InputReflowTime',
            'InputCoolingTempRamp', 'InputSolderMeltingPoint', 'InputPeakTemperature',
            'InputProfileName', 'InputProfileDescription'
        ];

        fieldIds.forEach(fieldId => {
            const element = document.getElementById(fieldId);
            if (element) {
                // Update work profile on Enter key or focus change
                element.addEventListener('keypress', (e) => {
                    if (e.key === 'Enter') {
                        this.updateWorkProfileFromForm();
                    }
                });

                element.addEventListener('blur', () => {
                    this.updateWorkProfileFromForm();
                });
            }
        });
    }

    // ------------------------------------------------------------------------
    // Load the currently selected profile into work profile and display in form
    // ------------------------------------------------------------------------
    async loadSelectedProfile() {
        const selectedMeta = this.profiles[this.currentSelectedId];

        if (!selectedMeta) {
            console.error('No profile metadata at index:', this.currentSelectedId);
            return;
        }

        try {
            // Extract filename without .json extension
            const profileName = selectedMeta.fileName.replace('.json', '');

            console.log('Loading profile from ESP32:', profileName);

            // Fetch full profile data from ESP32
            const response = await fetch(`/api/profiles/get?name=${profileName}`);

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const profileData = await response.json();

            // Load into work area
            this.workProfile = profileData;
            this.updateFormFields(profileData);

            // Update chart with work profile
            if (this.ProfileChart.ReflowChart && this.ProfileChart.ReflowChart.data) {
                const tempToUse = this.sensorError ? 25 : Math.round(this.currentTemperature);
                this.ProfileChart.UpdateProfileChartData(profileData, tempToUse);

                // Start periodic updates if not running
                if (!this.isProfileRunning) {
                    this.startChartUpdates();
                }
            }

            console.log('Profile loaded successfully:', profileData.name);

            // Show success notification
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification(
                    'Profile Loaded',
                    `Loaded: ${profileData.name}`,
                    'success'
                );
            }

        } catch (error) {
            console.error('Failed to load profile:', selectedMeta.fileName, error);

            // Keep dropdown intact, show error for this specific profile
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification(
                    'Profile Load Error',
                    `Failed to load ${selectedMeta.name} from ESP32`,
                    'danger',
                    10000  // Show for 10 seconds
                );
            }
        }
    }

    // ------------------------------------------------------------------------
    // Save current work profile data to the selected profile
    // ------------------------------------------------------------------------
    async saveToSelectedProfile() {
        if (!this.workProfile) {
            console.warn('No work profile to save');
            return;
        }

        const selectedMeta = this.profiles[this.currentSelectedId];
        if (!selectedMeta) {
            console.error('Target profile not found:', this.currentSelectedId);
            return;
        }

        // Update work profile with latest form data
        this.updateWorkProfileFromForm();

        try {
            // Extract filename without .json extension (e.g., "profile_0")
            const profileFileName = selectedMeta.fileName.replace('.json', '');

            // Build complete profile data to save
            const profileDataToSave = {
                fileName: profileFileName,  // For ESP32 routing (will be removed before saving)
                id: this.workProfile.id,
                name: this.workProfile.name || selectedMeta.name,  // Display name
                description: this.workProfile.description || selectedMeta.description,
                solder_melting_point: this.workProfile.solder_melting_point,
                absolute_peak_temp: this.workProfile.absolute_peak_temp,
                estimated_duration_sec: this.workProfile.estimated_duration_sec || 0,
                active: this.workProfile.active || false,
                preheat: { ...this.workProfile.preheat },
                soak: { ...this.workProfile.soak },
                reflow: { ...this.workProfile.reflow },
                cooling: { ...this.workProfile.cooling }
            };

            console.log('Saving profile to ESP32:', profileFileName, profileDataToSave);

            // Save to ESP32 via HTTP POST
            const response = await fetch('/api/profiles/save', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(profileDataToSave)
            });

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const result = await response.json();
            console.log('Profile saved successfully:', result);

            // Update local metadata if name/description changed
            selectedMeta.name = this.workProfile.name;
            selectedMeta.description = this.workProfile.description;
            this.updateProfileSelector();

            // Show success notification
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification(
                    'Profile Saved',
                    `Saved: ${selectedMeta.name}`,
                    'success'
                );
            }

        } catch (error) {
            console.error('Failed to save profile:', selectedMeta.fileName, error);

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification(
                    'Profile Save Error',
                    `Failed to save ${selectedMeta.name} to ESP32`,
                    'danger',
                    10000
                );
            }
        }
    }

    // ------------------------------------------------------------------------
    // Update work profile with current form field values
    // ------------------------------------------------------------------------
    updateWorkProfileFromForm() {
        if (!this.workProfile) return;

        try {
            // Update work profile with form data using correct IDs
            this.workProfile.name = document.getElementById('InputProfileName')?.value || this.workProfile.name;
            this.workProfile.description = document.getElementById('InputProfileDescription')?.value || this.workProfile.description;

            this.workProfile.preheat.max_temp = parseFloat(document.getElementById('InputPreheatTempMax')?.value || 0);
            this.workProfile.preheat.time_sec = parseInt(document.getElementById('InputPreheatTime')?.value || 0);

            this.workProfile.soak.max_temp = parseFloat(document.getElementById('InputSoakTempMax')?.value || 0);
            this.workProfile.soak.time_sec = parseInt(document.getElementById('InputSoakTime')?.value || 0);

            this.workProfile.reflow.min_temp = parseFloat(document.getElementById('InputReflowTempMin')?.value || 0);
            this.workProfile.reflow.max_temp = parseFloat(document.getElementById('InputReflowTempMax')?.value || 0);
            this.workProfile.reflow.time_sec = parseInt(document.getElementById('InputReflowTime')?.value || 0);

            this.workProfile.cooling.max_rate = parseFloat(document.getElementById('InputCoolingTempRamp')?.value || 0);
            this.workProfile.solder_melting_point = parseFloat(document.getElementById('InputSolderMeltingPoint')?.value || 0);
            this.workProfile.absolute_peak_temp = parseFloat(document.getElementById('InputPeakTemperature')?.value || 0);

            // Update chart with work profile
            if (this.ProfileChart.ReflowChart) {
                // this.ProfileChart.UpdateProfileChartData(this.workProfile);
                if (this.ProfileChart.ReflowChart) {
                    const tempToUse = this.sensorError ? 25 : Math.round(this.currentTemperature);
                    this.ProfileChart.UpdateProfileChartData(this.workProfile, tempToUse);
                }
            }

            console.log('Work profile updated from form');

        } catch (error) {
            console.error('Error updating work profile from form:', error);
        }
    }

    // ------------------------------------------------------------------------
    // Update profile selector dropdown
    // ------------------------------------------------------------------------
    updateProfileSelector() {
        const selector = document.getElementById('idSRO_ParSetSel');
        if (!selector) return;

        // Clear existing options
        selector.innerHTML = '';

        // Add profile options
        this.profiles.forEach((profile, index) => {
            const option = document.createElement('option');
            option.value = index + 1; // 1-based value for display
            option.textContent = profile.name;
            // if (index === 0) option.selected = true; // Select first profile by default
            if (index === this.currentSelectedId) option.selected = true;
            selector.appendChild(option);
        });

        console.log('Profile selector updated with', this.profiles.length, 'profiles');
    }

    // ------------------------------------------------------------------------
    // Update form fields with profile data
    // ------------------------------------------------------------------------
    updateFormFields(profile) {

        console.log(`********************************************`);
        console.log(profile);
        console.log(`--------------------------------------------`);
        console.log(`profile raw data: ${profile}`);
        console.log(`********************************************`);


        const fieldMappings = {
            // Profile name and description
            'InputProfileName': profile.name,
            'InputProfileDescription': profile.description,

            // Preheat phase (no min temp field, no ramp rate field)
            'InputPreheatTempMax': profile.preheat.max_temp,
            'InputPreheatTime': profile.preheat.time_sec,

            // Soak phase (no ramp rate field)
            'InputSoakTempMax': profile.soak.max_temp,
            'InputSoakTime': profile.soak.time_sec,

            // Reflow phase (no ramp rate field)
            'InputReflowTempMin': profile.reflow.min_temp,
            'InputReflowTempMax': profile.reflow.max_temp,
            'InputReflowTime': profile.reflow.time_sec,

            // Cooling phase
            'InputCoolingTempRamp': profile.cooling.max_rate,

            // Information data
            'InputSolderMeltingPoint': profile.solder_melting_point,
            'InputPeakTemperature': profile.absolute_peak_temp
        };

        Object.entries(fieldMappings).forEach(([elementId, value]) => {
            const element = document.getElementById(elementId);
            if (element) {
                element.value = value;
                console.log(`Set ${elementId} = ${value}`);
            } else {
                console.warn('Element not found:', elementId);
            }
        });
    }

    // ------------------------------------------------------------------------
    // Toggle profile execution (start/stop based on current state)
    // ------------------------------------------------------------------------
    toggleProfileExecution() {
        // Check current profile state to determine action
        if (this.isProfileRunning) {
            this.stopProfile();
        } else {
            this.startProfile();
        }
    }

    // ------------------------------------------------------------------------
    // Get current work profile
    // ------------------------------------------------------------------------
    getCurrentWorkProfile() {
        return this.workProfile;
    }

    // ------------------------------------------------------------------------
    // Get selected profile ID
    // ------------------------------------------------------------------------
    getSelectedProfileId() {
        return this.currentSelectedId;
    }

    // ------------------------------------------------------------------------
    // Get all profiles
    // ------------------------------------------------------------------------
    getAllProfiles() {
        return this.profiles;
    }

    // 🆕 🆕 🆕 🆕
    // ============================================================================
    // 2. ADD NEW METHODS - Insert these methods in the ProfileManager class
    // ============================================================================

    // ------------------------------------------------------------------------
    // Handle temperature update events for chart integration
    // ------------------------------------------------------------------------
    /*
    handleTemperatureUpdate(data) {
        if (this.isProfileRunning && data.temp !== undefined) {
            this.currentTemperature = data.temp;
            // Note: actual chart update happens in the data logging timer
        }
    }
    */
    handleTemperatureUpdate(data) {
        // ALWAYS capture temperature, not just when running
        if (data.temp !== undefined) {
            this.currentTemperature = data.temp;
        }

        // Check for sensor error
        if (data.diagnostics && data.diagnostics.temp_get_result) {
            const hasError = (data.diagnostics.temp_get_result !== 'ESP_OK');
            this.handleSensorError(hasError);
        }
    }

    // ------------------------------------------------------------------------
    // Update chart time axis (copied from sro_tuning.js pattern)
    // ------------------------------------------------------------------------
    updateProfileChartTimeAxis(currentTime) {
        if (!this.ProfileChart.ReflowChart) return;

        const currentMax = this.ProfileChart.ReflowChart.options.scales.x.max || 300;

        // Check if we need to extend the chart (with 10-second buffer)
        if (currentTime > (currentMax - 10)) {
            const newMax = currentMax + 60; // Extend by 1 minute

            // Update the chart scale
            this.ProfileChart.ReflowChart.options.scales.x.max = newMax;

            // this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.xMin = this.ProfileChart.ChartMinTicks + 1;
            // this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.yMin = this.workProfile.absolute_peak_temp;
            this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.xMax = newMax;
            // this.ProfileChart.ReflowChart.options.plugins.annotation.annotations.overTemperatureBox.yMax = tmpY_Scale_Max - 1;

            console.log(`📊 Profile Chart X-axis auto-scaled from ${currentMax} to ${newMax} seconds at time ${currentTime}s`);

            // Note: Chart will be updated by the next update('none') call in logProfileData()
        }
    }

    // ------------------------------------------------------------------------
    // Log profile data to chart (adapted from sro_tuning.js logTuningData pattern)
    // ------------------------------------------------------------------------
    logProfileData() {
        if (!this.ProfileChart.ReflowChart) return;

        // Add data point for actual temperature (dataset index 1 - "act values")
        const tempDataPoint = {
            x: this.loggedSeconds,
            y: this.currentTemperature
        };

        this.ProfileChart.ReflowChart.data.datasets[1].data.push(tempDataPoint);

        this.updateProfileChartTimeAxis(this.loggedSeconds);

        this.ProfileChart.ReflowChart.update('none');

        console.log(`📈 Logged profile data: ${this.loggedSeconds}s = ${this.currentTemperature}°C`);
        this.loggedSeconds++;
    }

    // ------------------------------------------------------------------------
    // Start data logging for profile execution (adapted from sro_tuning.js)
    // ------------------------------------------------------------------------
    startDataLogging() {
        console.log('🟢 Starting profile data logging');

        // Clear existing actual temperature data (preserve profile curve)
        this.clearActualTemperatureData();
        this.loggedSeconds = 0;

        // Start 1-second timer
        this.dataLoggerTimer = setInterval(() => {
            this.logProfileData();
        }, 1000);
    }

    // ------------------------------------------------------------------------
    // Stop data logging (adapted from sro_tuning.js)
    // ------------------------------------------------------------------------
    stopDataLogging() {
        if (this.dataLoggerTimer) {
            clearInterval(this.dataLoggerTimer);
            this.dataLoggerTimer = null;
            console.log('🔴 Profile data logging stopped');
        }
    }

    // ------------------------------------------------------------------------
    // Clear actual temperature data only (preserve profile curve)
    // ------------------------------------------------------------------------
    clearActualTemperatureData() {
        if (!this.ProfileChart.ReflowChart) return;

        // Clear only the actual temperature dataset (index 1), preserve profile curve (index 0)
        this.ProfileChart.ReflowChart.data.datasets[1].data = [];
        this.ProfileChart.ReflowChart.update('none');

        console.log('🧹 Profile chart actual temperature data cleared');
    }

    // ------------------------------------------------------------------------
    // Start periodic chart updates (every 5 seconds before profile starts)
    // ------------------------------------------------------------------------
    startChartUpdates() {
        if (this.chartUpdateTimer) {
            clearInterval(this.chartUpdateTimer);
        }

        this.chartLocked = false;
        console.log('Starting periodic chart updates');

        // Update immediately
        this.updateChartWithCurrentTemp();

        // Update every 5 seconds
        this.chartUpdateTimer = setInterval(() => {
            // if (!this.chartLocked && this.workProfile) {
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
        console.log('Chart locked at:', Math.round(this.currentTemperature), '°C');
    }
    // ------------------------------------------------------------------------
    // Redraw chart with current sensor temperature
    // ------------------------------------------------------------------------
    /*
    updateChartWithCurrentTemp() {
        // if (!this.workProfile || !this.ProfileChart.ReflowChart) {
            // return;
        // }
        if (!this.ProfileChart.ReflowChart) {
            return;
        }

        const tempToUse = this.sensorError ? 25 : Math.round(this.currentTemperature);

        if (this.workProfile) {
            console.log('Updating chart with temp:', tempToUse, '°C');
            this.ProfileChart.UpdateProfileChartData(this.workProfile, tempToUse);
        } else {
            // Just update Y-axis min without profile data
            this.ProfileChart.ReflowChart.options.scales.y.min = Math.floor(tempToUse / 10) * 10;
            this.ProfileChart.ReflowChart.update('none');
        }

        // Use 25°C if sensor error, otherwise use actual temp
        // const tempToUse = this.sensorError ? 25 : Math.round(this.currentTemperature);
        // console.log('Updating chart with temp:', tempToUse, '°C');
        // this.ProfileChart.UpdateProfileChartData(this.workProfile, tempToUse);
    }
    */
    updateChartWithCurrentTemp() {
        if (!this.ProfileChart.ReflowChart) {
            console.log('No ReflowChart - returning');
            return;
        }

        const tempToUse = this.sensorError ? 25 : Math.round(this.currentTemperature);

        if (this.workProfile) {
            console.log('Updating with profile, temp:', tempToUse);
            this.ProfileChart.UpdateProfileChartData(this.workProfile, tempToUse);
        } else {
            console.log('No profile - updating Y-axis only to:', Math.floor(tempToUse / 10) * 10);
            this.ProfileChart.ReflowChart.options.scales.y.min = Math.floor(tempToUse / 10) * 10;
            this.ProfileChart.ReflowChart.update('none');
        }
    }

    // ------------------------------------------------------------------------
    // Handle sensor error state changes
    // ------------------------------------------------------------------------
    handleSensorError(hasError) {
        const wasError = this.sensorError;
        this.sensorError = hasError;

        if (hasError && !wasError) {
            console.error('Temperature sensor error detected');
            this.lastGoodTemp = this.currentTemperature;
            this.updateSensorErrorUI(true);

            if (!this.chartLocked) {
                this.updateChartWithCurrentTemp();
            }
        } else if (!hasError && wasError) {
            console.log('Temperature sensor recovered');
            this.updateSensorErrorUI(false);

            if (!this.chartLocked && this.workProfile && !this.isProfileRunning) {
                this.startChartUpdates();  // ← Restart periodic updates
            }
        }
    }

    // ------------------------------------------------------------------------
    // Update UI elements for sensor error
    // ------------------------------------------------------------------------
    updateSensorErrorUI(isError) {
        const tempDisplay = document.getElementById('navbar-actual-temp');
        const statusBar = document.getElementById('sro-status-bar');
        const runButton = document.getElementById('btnProfileRun');

        if (isError) {
            if (tempDisplay) {
                tempDisplay.textContent = '--°C';
                tempDisplay.style.color = 'red';
            }

            if (statusBar) {
                statusBar.textContent = 'Temperature sensor error';
                statusBar.style.color = 'red';
            }

            if (runButton && !this.isProfileRunning) {
                runButton.disabled = true;
            }
        } else {
            if (tempDisplay) {
                tempDisplay.textContent = Math.round(this.currentTemperature) + '°C';
                tempDisplay.style.color = '';
            }

            if (statusBar) {
                statusBar.textContent = '';
                statusBar.style.color = '';
            }

            if (runButton && !this.isProfileRunning) {
                runButton.disabled = false;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Start profile execution using current work profile
    // ------------------------------------------------------------------------
    startProfile() {
        if (!this.workProfile) {
            console.warn('No work profile loaded');
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('No Profile', 'Load a profile first', 'warning');
            }
            return;
        }

        // Stop chart updates and lock to current temperature
        this.stopChartUpdates();
        this.updateChartWithCurrentTemp();

        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            // Send only the ID - firmware loads profile from file
            window.SroWebSocket.sendCommand('startProfile', {
                id: this.currentSelectedId
            });
            console.log('Starting profile ID:', this.currentSelectedId);

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Profile Starting', `Starting: ${this.workProfile.name}...`, 'info');
            }
        } else {
            console.warn('WebSocket not connected');
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Connection Error', 'WebSocket not connected', 'warning');
            }
        }
    }
    
    // ------------------------------------------------------------------------
    // Stop profile execution
    // ------------------------------------------------------------------------
    stopProfile() {
        console.log('Stopping profile execution');

        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            window.SroWebSocket.sendCommand('stopProfile');
            console.log('Stop profile command sent');

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Profile Stopped', 'Profile execution stopped', 'warning');
            }
        } else {
            console.warn('WebSocket not connected');
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Connection Error', 'WebSocket not connected', 'warning');
            }
        }
    }

    // ------------------------------------------------------------------------
    // Handle profile started event
    // ------------------------------------------------------------------------
    handleProfileStarted(data) {
        this.isProfileRunning = true;
        this.updateRunButtonState();

        // Ensure chart stays locked
        this.chartLocked = true;

        if (data.success) {
            console.log(`Profile ${data.profileId} started successfully`);

            // 🆕 NEW: Start data logging for chart updates
            this.startDataLogging();

            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Profile Started',
                    data.message || 'Profile execution started', 'success');
            }
        } else {
            console.error('Profile start failed:', data.message);
            this.isProfileRunning = false;
            // Profile failed to start - restart chart updates
            this.chartLocked = false;
            this.startChartUpdates();
            this.updateRunButtonState();
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Profile Start Failed',
                    data.message || 'Failed to start profile', 'error');
            }
        }
    }

    // ------------------------------------------------------------------------
    // Handle profile stopped event
    // ------------------------------------------------------------------------
    handleProfileStopped(data) {
        this.isProfileRunning = false;
        this.updateRunButtonState();

        // Stop data logging
        this.stopDataLogging();

        // Chart stays locked after profile stops - do NOT call startChartUpdates()
        console.log('Profile stopped - chart remains locked');

        if (data.success) {
            console.log('Profile stopped successfully');
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Profile Stopped',
                    data.message || 'Profile execution stopped', 'info');
            }
        } else {
            console.error('Profile stop failed:', data.message);
            if (window.SroApp && window.SroApp.showNotification) {
                window.SroApp.showNotification('Profile Stop Failed',
                    data.message || 'Failed to stop profile', 'error');
            }
        }
    }

    // ------------------------------------------------------------------------
    // Handle profile progress updates
    // ------------------------------------------------------------------------
    handleProfileProgress(data) {
        // Update progress display if you have progress indicators
        console.log(`Profile progress: ${data.progress}% - Phase: ${data.phase}`);
        // You can add progress bar updates here if needed
    }

    // ------------------------------------------------------------------------
    // Handle profile completed event
    // ------------------------------------------------------------------------
    handleProfileCompleted(data) {
        this.isProfileRunning = false;
        this.updateRunButtonState();

        // 🆕 NEW: Stop data logging
        this.stopDataLogging();

        console.log('Profile completed successfully');
        if (window.SroApp && window.SroApp.showNotification) {
            window.SroApp.showNotification('Profile Complete',
                'Reflow profile completed successfully!', 'success');
        }
    }

    // ------------------------------------------------------------------------
    // Update RUN button state based on profile running status
    // ------------------------------------------------------------------------
    updateRunButtonState() {
        const runBtn = document.getElementById('btnProfileRun'); // Check actual button ID
        if (runBtn) {
            if (this.isProfileRunning) {
                runBtn.textContent = 'STOP';
                runBtn.className = 'btn btn-lg btn-danger w-100'; // Red for stop
            } else {
                runBtn.textContent = 'RUN';
                runBtn.className = 'btn btn-lg btn-success w-100'; // Green for start
            }
        }
    }

    // ------------------------------------------------------------------------
    // Cleanup method (call when switching sections or destroying)
    // ------------------------------------------------------------------------
    cleanup() {
        this.stopDataLogging();
        console.log('ProfileManager cleanup completed');
    }

}

// Create global instance
window.ProfileManager = new ProfileManager();

