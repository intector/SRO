/**
 * SRO Main Application Module - Centralized Version
 * Main application controller and centralized coordinator
 */

class SroApp {
    constructor() {
        this.currentSection = 'not-connected';
        this.isInitialized = false;
        this.modules = {};
        this.connectionState = 'disconnected';
        this.offlineMode = false;
        
        this.handleWebSocketEvent = this.handleWebSocketEvent.bind(this);
        this.updateConnectionStatus = this.updateConnectionStatus.bind(this);
    }
    
    /**
     * Initialize the application - centralized entry point
     */
    async init() {
        if (this.isInitialized) return;
        
        console.log('Initializing SRO Application (Centralized)');
        
        try {
            // Setup navigation first
            this.setupNavigation();

            // Setup offline mode button
            this.setupOfflineModeButton();
            
            // Setup global event listeners
            this.setupGlobalEventListeners();
            
            // Initialize and setup WebSocket as central coordinator
            this.initializeWebSocket();
            
            // Initialize modules (but don't let them setup their own WebSocket handlers)
            await this.initializeModules();
            
            // Start with not-connected section and disabled navbar
            this.showSection('not-connected');
            this.setNavbarState(false);
            
            // Don't call updateConnectionStatus here - let the WebSocket events handle it
            
            // Attempt WebSocket connection
            this.connectWebSocket();
            
            this.isInitialized = true;
            console.log('SRO Application initialized successfully (Centralized)');
            
        } catch (error) {
            console.error('Failed to initialize SRO Application:', error);
            this.showNotification('Initialization Error', 'Failed to start application', 'danger');
        }
    }
    
    /**
     * Initialize and setup WebSocket as central communication hub
     */
    initializeWebSocket() {
        if (!window.SroWebSocket) {
            console.error('SroWebSocket not available');
            return;
        }
        
        console.log('Setting up centralized WebSocket management');
        
        // Clear any existing handlers to prevent duplicates
        window.SroWebSocket.eventHandlers.clear();
        
        // Setup centralized WebSocket event handlers
        window.SroWebSocket.on('connected', () => {
            console.log('WebSocket connected - centralizing connection handling');
            this.connectionState = 'connected';
            this.offlineMode = false;
            this.updateConnectionStatus('connected');
            this.setNavbarState(true);
            this.showNotification('Connected', 'WebSocket connection established', 'success');
            
            // Switch to dashboard once connected
            this.showSection('dashboard');
            
            // Request initial system status
            this.requestSystemStatus();
        });
        
        window.SroWebSocket.on('disconnected', (data) => {
            console.log('WebSocket disconnected - centralizing disconnection handling');
            this.connectionState = 'disconnected';
            this.updateConnectionStatus('disconnected');
            this.showNotification('Disconnected', 'Lost connection to controller', 'warning');
            
            // Only switch back to not-connected if not in offline mode
            if (!this.offlineMode) {
                this.setNavbarState(false);
                this.showSection('not-connected');
            }
        });
        
        window.SroWebSocket.on('connecting', () => {
            console.log('WebSocket connecting - updating status');
            this.connectionState = 'connecting';
            this.updateConnectionStatus('connecting');
        });
        
        // Setup centralized data routing for all incoming WebSocket events
        this.setupWebSocketDataRouter();
    }
    
    /**
     * Setup centralized WebSocket data routing
     */
    setupWebSocketDataRouter() {
        // Temperature updates - route to dashboard
        window.SroWebSocket.on('temperatureUpdate', (data) => {
            this.handleWebSocketEvent('temperatureUpdate', data);
        });
        
        window.SroWebSocket.on('tempUpdate', (data) => {
            this.handleWebSocketEvent('temperatureUpdate', data);
        });

        window.SroWebSocket.on('hardwareUpdate', (data) => {
            this.handleWebSocketEvent('hardwareUpdate', data);
        });

        // System status updates - route to dashboard
        window.SroWebSocket.on('systemStatus', (data) => {
            this.handleWebSocketEvent('systemStatus', data);
        });
        
        window.SroWebSocket.on('statusUpdate', (data) => {
            this.handleWebSocketEvent('statusUpdate', data);
        });
        
        // Profile-related events - route to profile manager
        window.SroWebSocket.on('profileStatus', (data) => {
            this.handleWebSocketEvent('profileStatus', data);
        });
        
        window.SroWebSocket.on('profilesData', (data) => {
            this.handleWebSocketEvent('profilesData', data);
        });
        
        window.SroWebSocket.on('profileSaved', (data) => {
            this.handleWebSocketEvent('profileSaved', data);
        });
        
        window.SroWebSocket.on('hardwareStatus', (data) => {
            this.handleWebSocketEvent('hardwareStatus', data);
        });

        window.SroWebSocket.on('tuningData', (data) => {
            this.handleWebSocketEvent('tuningData', data);
        });

        window.SroWebSocket.on('tuningResults', (data) => {
            this.handleWebSocketEvent('tuningResults', data);
        });

        window.SroWebSocket.on('tuningProgress', (data) => {
            this.handleWebSocketEvent('tuningProgress', data);
        });

        window.SroWebSocket.on('tuningComplete', (data) => {
            this.handleWebSocketEvent('tuningComplete', data);
        });

        console.log('Centralized WebSocket data router setup complete');
    }
    
    /**
     * Centralized WebSocket event handler and router
     */
    handleWebSocketEvent(eventType, data) {
        console.log(`Routing WebSocket event: ${eventType}`, data);

        // Round target temperature to 2 decimal places
        if (data.target !== undefined) {
            data.target = Math.round(data.target * 100) / 100;
        }

        try {
            switch (eventType) {
                case 'temperatureUpdate':
                    // Update the dynamic system status area directly
                    this.updateSystemStatus(data);

                    // Route to manual control module if it exists
                    if (this.modules.manualControl && this.modules.manualControl.updateHardwareStatus) {
                        this.modules.manualControl.updateHardwareStatus(data);
                    }

                    // Handle autotune status if present in broadcast
                    if (data.autotune && this.modules.tuning) {
                        this.modules.tuning.handleAutotuneStatus(data.autotune, data.pid);
                    }

                    // Update tuning module with current temperature
                    if (this.modules.tuning && this.modules.tuning.updateTemperature && data.temp !== undefined) {
                        this.modules.tuning.updateTemperature(data.temp);
                    }

                    // Update tuning module with current PID values
                    if (data.pid && this.modules.tuning && this.modules.tuning.updateCurrentPID) {
                        this.modules.tuning.updateCurrentPID(data.pid.kp, data.pid.ki, data.pid.kd);
                    }

                    break;

                case 'hardwareUpdate':
                    this.updateSystemStatus(data);
                    break;

                case 'systemStatus':
                    // Handle system status updates
                    this.updateSystemStatus(data);
                    break;

                case 'statusUpdate':
                    // Handle combined status updates
                    this.updateSystemStatus(data);
                    break;

                case 'profileStatus':
                case 'profilesData':
                case 'profileSaved':
                    // Route to profile manager
                    if (this.modules.profiles) {
                        if (eventType === 'profilesData') {
                            this.modules.profiles.handleProfilesDataFromESP32(data);
                        } else {
                            console.log(`Profile event ${eventType} received:`, data);
                        }
                    }
                    break;

                case 'tuningData':
                case 'tuningResults':
                case 'tuningProgress':
                    // Route to tuning manager
                    if (this.modules.tuning) {
                        if (eventType === 'tuningResults') {
                            this.modules.tuning.handleTuningResults(data);
                        } else if (eventType === 'tuningProgress') {
                            this.modules.tuning.handleTuningProgress(data);
                        } else if (eventType === 'tuningData') {
                            this.modules.tuning.handleTuningData(data);
                        }
                        console.log(`Tuning event ${eventType} received:`, data);
                    }
                    break;

                case 'tuningComplete':
                    if (this.modules.tuning) {
                        this.modules.tuning.handleAutotuneStatus(data.autotune, null);
                    }
                    break;

                default:
                    console.log(`Unhandled WebSocket event: ${eventType}`, data);
            }
        } catch (error) {
            console.error(`Error handling WebSocket event ${eventType}:`, error);
        }
    }

    /**
     * Connect WebSocket with centralized management
     */
    connectWebSocket() {
        if (!window.SroWebSocket) {
            console.error('SroWebSocket not available');
            this.updateConnectionStatus('error');
            return;
        }
        
        console.log('Starting centralized WebSocket connection');
        this.updateConnectionStatus('connecting');
        window.SroWebSocket.connect();
    }
    
    /**
     * Centralized connection status management
     */
    updateConnectionStatus(status) {
        console.log(`Updating connection status to: ${status}`);

        // Update hardware connection status (icon + text)
        const hwStatusText = document.getElementById('hardware-connection-status');
        const hwStatusIcon = hwStatusText?.previousElementSibling; // The <i> icon

        if (hwStatusText) {
            // Update text
            const statusText = {
                'connecting': 'connecting...',
                'connected': 'connected',
                'disconnected': 'disconnected',
                'offline': 'offline',
                'error': 'error'
            };
            hwStatusText.textContent = statusText[status] || status;
        }

        if (hwStatusIcon) {
            // Remove old status classes, add connection-status-icon for animations
            hwStatusIcon.classList.remove('connecting', 'connected', 'disconnected', 'error');
            hwStatusIcon.classList.add('connection-status-icon'); // Enable the CSS animations
            hwStatusIcon.classList.add(status === 'offline' ? 'disconnected' : status);
        }

        this.connectionState = status;
        console.log(`Connection status updated to: ${status}`);
    }

    /**
     * Setup offline mode button monitoring
     */
    setupOfflineModeButton() {
        console.log('Setting up offline mode button monitoring');
        
        const offlineBtn = document.getElementById('btnWorkOffline');
        if (offlineBtn) {
            offlineBtn.addEventListener('click', (e) => {
                e.preventDefault();
                this.activateOfflineMode();
            });
            console.log('Offline mode button monitoring active');
        } else {
            console.warn('Offline mode button not found');
        }
    }
    
    /**
     * Activate offline mode for testing
     */
    activateOfflineMode() {
        console.log('Activating offline mode');
        
        this.offlineMode = true;
        this.connectionState = 'offline';
        this.updateConnectionStatus('offline');
        this.setNavbarState(true);
        
        // Show success notification
        this.showNotification('Offline Mode', 'Working offline for testing', 'info');
        
        // Switch to dashboard
        this.showSection('dashboard');
        
        // Initialize modules for offline testing
        this.initializeOfflineMode();
    }
    
    /**
     * Initialize modules for offline testing
     */
    initializeOfflineMode() {
        console.log('Initializing offline testing mode');
        
        // Start dashboard updates if available
        if (this.modules.dashboard && this.modules.dashboard.startUpdates) {
            // Override dashboard's WebSocket requests to prevent errors
            const originalRequestStatus = this.modules.dashboard.requestSystemStatus;
            this.modules.dashboard.requestSystemStatus = () => {
                console.log('Dashboard status request blocked (offline mode)');
            };
            
            this.modules.dashboard.startUpdates();
        }
        
        // Could add mock data injection here for testing
        this.injectMockData();
    }
    
    /**
     * Inject mock data for offline testing
     */
    injectMockData() {
        console.log('Injecting mock data for offline testing');
        
        // Mock temperature data
        const mockTempData = {
            temp: 25.0,
            target: 25.0,
            heater: false
        };
        
        // Mock system status data
        const mockSystemData = {
            safe: true,
            uptime: 123456,
            doorPosition: 10,
            fanState: false
        };
        
        // Inject data after a short delay to simulate real updates
        setTimeout(() => {
            if (this.modules.dashboard) {
                this.modules.dashboard.updateTemperatureDisplay(mockTempData);
                this.modules.dashboard.updateSystemStatus(mockSystemData);
            }
        }, 1000);
    }
    
    /**
     * Set navbar navigation buttons enabled/disabled state
     */
    setNavbarState(enabled) {
        console.log(`Setting navbar state: ${enabled ? 'enabled' : 'disabled'}`);
        
        const navLinks = document.querySelectorAll('.navbar-nav .nav-link[data-section]');
        
        navLinks.forEach(link => {
            if (enabled) {
                link.classList.remove('disabled');
                link.style.pointerEvents = 'auto';
                link.style.opacity = '1';
            } else {
                link.classList.add('disabled');
                link.style.pointerEvents = 'none';
                link.style.opacity = '0.5';
            }
        });
        
        console.log(`Navbar buttons ${enabled ? 'enabled' : 'disabled'}`);
    }

    /**
     * Initialize all modules with centralized coordination
     */
    async initializeModules() {
        console.log('Initializing modules under centralized management...');
        
        // Initialize dashboard (but prevent it from setting up its own WebSocket handlers)
        if (window.SroDashboard) {
            // Override dashboard's WebSocket setup to prevent conflicts
            const originalSetupWebSocketHandlers = window.SroDashboard.setupWebSocketHandlers;
            window.SroDashboard.setupWebSocketHandlers = () => {
                console.log('Dashboard WebSocket setup bypassed - using centralized management');
            };
            
            window.SroDashboard.init();
            this.modules.dashboard = window.SroDashboard;
            
            // Restore original method in case it's needed later
            window.SroDashboard.setupWebSocketHandlers = originalSetupWebSocketHandlers;
        }

        console.log('Checking for ProfileManager:', !!window.ProfileManager);
        if (window.ProfileManager) {
            console.log('Initializing ProfileManager...');
            await window.ProfileManager.init();
            this.modules.profiles = window.ProfileManager;
            console.log('ProfileManager initialized');
        } else {
            console.warn('ProfileManager not found on window object');
        }

            console.log('Checking for ManualControl:', !!window.ManualControl);
        if (window.ManualControl) {
            console.log('Initializing ManualControl...');
            await window.ManualControl.init();
            this.modules.manualControl = window.ManualControl;
            console.log('ManualControl initialized');
        } else {
            console.warn('ManualControl not found on window object');
        }

        console.log('Checking for TuningManager:', !!window.TuningManager);
        if (window.TuningManager) {
            console.log('Initializing TuningManager...');
            await window.TuningManager.init();
            this.modules.tuning = window.TuningManager;
            console.log('TuningManager initialized');
        } else {
            console.warn('TuningManager not found on window object');
        }

            console.log('Modules initialized under centralized management:', Object.keys(this.modules));
    }
    
    /**
     * Request system status through centralized WebSocket
     */
    requestSystemStatus() {
        if (window.SroWebSocket && window.SroWebSocket.isConnected) {
            console.log('Requesting system status (centralized)');
            window.SroWebSocket.sendCommand('getStatus');
        } else {
            console.log('Cannot request system status - WebSocket not connected');
        }
    }
    
    /**
     * Setup navigation handling
     */
    setupNavigation() {
        // Handle navigation clicks
        document.querySelectorAll('[data-section]').forEach(link => {
            link.addEventListener('click', (e) => {
                e.preventDefault();
                const section = e.target.closest('[data-section]').dataset.section;
                this.showSection(section);
            });
        });
        
        // Handle browser back/forward
        window.addEventListener('popstate', (e) => {
            if (e.state && e.state.section) {
                this.showSection(e.state.section, false);
            }
        });
        
        // Set initial state
        const hash = window.location.hash.slice(1);
        if (hash && ['dashboard', 'manual-control', 'tuning', 'settings', 'not-connected'].includes(hash)) {
            this.currentSection = hash;
        }
    }
    
    /**
     * Show specific section with centralized management
     */
    showSection(sectionName, updateHistory = true) {
        console.log('Showing section:', sectionName);
        
        // Prevent showing other sections if not connected and not in offline mode
        if (!this.offlineMode && sectionName !== 'not-connected' && this.connectionState !== 'connected') {
            console.log(`Cannot show ${sectionName} - not connected and not in offline mode`);
            sectionName = 'not-connected';
        }
        
        // Hide all sections
        document.querySelectorAll('.content-section').forEach(section => {
            section.classList.remove('active');
        });
        
        // Show target section
        const targetSection = document.getElementById(`${sectionName}-section`);
        if (targetSection) {
            targetSection.classList.add('active');
        }
        
        // Update navigation
        document.querySelectorAll('[data-section]').forEach(link => {
            link.classList.remove('active');
            if (link.dataset.section === sectionName) {
                link.classList.add('active');
            }
        });
        
        // Update URL and history
        if (updateHistory) {
            const url = `#${sectionName}`;
            window.history.pushState({ section: sectionName }, '', url);
        }
        
        this.currentSection = sectionName;
        
        // Trigger section-specific initialization
        this.handleSectionChange(sectionName);
    }

    /**
     * Create dynamic system status area for current section
     */
    createSystemStatusArea(sectionType) {
        const containerId = `${sectionType}-status-container`;
        const container = document.getElementById(containerId);
        if (!container) {
            console.warn(`System status container not found: ${containerId}`);
            return;
        }

        // Clear any existing content
        container.innerHTML = '';

        // Section-aware styling classes
        let sectionClass = 'sro-border-automatic';
        let bgClass = 'bg-primary';

        switch (sectionType) {
            case 'dashboard':
                sectionClass = 'sro-border-automatic';
                bgClass = 'bg-primary';
                break;

            case 'manual-control':
                sectionClass = 'sro-border-manual';
                bgClass = 'bg-primary';
                break;

            case 'tuning':
                sectionClass = 'sro-border-tuning';
                bgClass = 'bg-primary';
                break;
        }
        // const sectionClass = sectionType === 'dashboard' ? 'sro-border-automatic' : 'sro-border-manual';
        // const bgClass = sectionType === 'dashboard' ? 'bg-primary' : 'bg-primary border-secondary';

        // Create the status area HTML
        const statusAreaHTML = `
        <div class="${bgClass} border ${sectionClass} rounded d-flex align-items-center" style="min-height: 100px;">
            <!-- Temperature Display -->
            <div class="col-xl-2 sro-font-size-xl d-flex">
                <div class="vstack gap-1">
                    <div class="hstack gap-2 mx-auto">
                        <div class="bi bi-thermometer-half"></div>
                        <div>Temp</div>
                    </div>
                    <div class="mx-auto sro-font-size-xxl" id="current-temp">--°C</div>
                </div>
            </div>
            <!-- Target Temperature -->
            <div class="col-xl-2 sro-font-size-xl d-flex">
                <div class="vstack gap-1">
                    <div class="hstack gap-2 mx-auto">
                        <div class="bi bi-bullseye"></div>
                        <div>Target</div>
                    </div>
                    <div class="mx-auto sro-font-size-xxl" id="target-temp">--°C</div>
                </div>
            </div>
            <!-- Heater Status -->
            <div class="col-xl-2 sro-font-size-xl d-flex">
                <div class="vstack gap-1">
                    <div class="hstack gap-2 mx-auto">
                        <div class="bi bi-fire"></div>
                        <div>Heater</div>
                    </div>
                    <div class="badge bg-secondary mx-2" id="heater-status">OFF</div>
                </div>
            </div>
            <!-- Fan Status -->
            <div class="col-xl-2 sro-font-size-xl d-flex">
                <div class="vstack gap-1">
                    <div class="hstack gap-2 mx-auto">
                        <div class="bi bi-fan"></div>
                        <div>Fan</div>
                    </div>
                    <div class="badge bg-secondary mx-2" id="fan-status">OFF</div>
                </div>
            </div>
            <!-- Door Status -->
            <div class="col-xl-2 sro-font-size-xl d-flex">
                <div class="vstack gap-1">
                    <div class="hstack gap-2 mx-auto">
                        <div class="bi bi-door-open"></div>
                        <div>Door</div>
                    </div>
                    <div class="mx-auto sro-font-size-xxl" id="door-status">0%</div>
                </div>
            </div>
        </div>
    `;

        container.innerHTML = statusAreaHTML;
        console.log(`Created system status area for ${sectionType} section`);
    }

    /**
     * Update system status with real-time data
     */
    updateSystemStatus(data) {
        // Temperature updates
        if (data.temp !== undefined) {
            const tempElement = document.getElementById('navbar-actual-temp');
            if (tempElement) {
                tempElement.textContent = `${data.temp}°C`;
            }
        }

        if (this.modules.tuning && this.modules.tuning.updateHardwareStatus) {
            this.modules.tuning.updateHardwareStatus(data);
        }

        if (data.target !== undefined) {
            const targetElement = document.getElementById('navbar-set-temp');
            if (targetElement) {
                targetElement.textContent = `${data.target}°C`;
            }
        }

        // Heater status
        if (data.heater !== undefined) {
            const heaterElement = document.getElementById('hardware-heater-status');
            if (heaterElement) {
                heaterElement.textContent = data.heater ? 'ON' : 'OFF';
                heaterElement.className = data.heater ? 'badge bg-danger mx-2' : 'badge bg-secondary mx-2';
            }
        }

        // Fan status
        if (data.fan !== undefined) {
            const fanElement = document.getElementById('hardware-fan-status');
            if (fanElement) {
                fanElement.textContent = data.fan ? 'ON' : 'OFF';
                fanElement.className = data.fan ? 'badge bg-info mx-2' : 'badge bg-secondary mx-2';
            }
        }

        // Door position
        if (data.door && data.door.position !== undefined) {
            const doorElement = document.getElementById('hardware-door-status');
            if (doorElement) {
                doorElement.textContent = `${data.door.position}%`;
            }
        }

        // RSSI signal strength with dynamic icon and colors
        if (data.diagnostics && data.diagnostics.rssi !== undefined) {
            const rssi = data.diagnostics.rssi;
            const rssiElement = document.getElementById('rssi-strength-status');
            const rssiIcon = document.getElementById('rssi-icon');

            if (rssiElement) {
                rssiElement.textContent = `${rssi} dBm`;
            }

            if (rssiIcon) {
                let iconClass, colorVar;

                if (rssi === 0 || rssi < -90) {
                    // No connection or very weak
                    iconClass = 'bi-wifi-off';
                    colorVar = 'var(--sro-red-500)';           // #F44336
                } else if (rssi < -70) {
                    // Weak signal
                    iconClass = 'bi-wifi-1';
                    colorVar = 'var(--sro-yellow-500)';        // #FFEB3B
                } else if (rssi < -50) {
                    // Medium signal
                    iconClass = 'bi-wifi-2';
                    colorVar = 'var(--sro-light-green-500)';   // #8BC34A
                } else {
                    // Strong signal
                    iconClass = 'bi-wifi';
                    colorVar = 'var(--sro-connected)';         // #00C853
                }

                rssiIcon.className = `bi ${iconClass} hardware-status-icon`;
                rssiIcon.style.color = colorVar;
                rssiElement.style.color = colorVar;
            }
        }

        // PID values
        if (data.pid) {
            const kpElement = document.getElementById('used-kp');
            const kiElement = document.getElementById('used-ki');
            const kdElement = document.getElementById('used-kd');

            if (kpElement && data.pid.kp !== undefined) {
                kpElement.textContent = `Kp: ${data.pid.kp.toFixed(6)}`;
            }
            if (kiElement && data.pid.ki !== undefined) {
                kiElement.textContent = `Ki: ${data.pid.ki.toFixed(6)}`;
            }
            if (kdElement && data.pid.kd !== undefined) {
                kdElement.textContent = `Kd: ${data.pid.kd.toFixed(6)}`;
            }

            /*
            // AutoTune PID fields
            const autoKpEl = document.getElementById('AutoTune-Kp');
            const autoKiEl = document.getElementById('AutoTune-Ki');
            const autoKdEl = document.getElementById('AutoTune-Kd');

            if (autoKpEl) autoKpEl.value = data.pid.kp.toFixed(6);
            if (autoKiEl) autoKiEl.value = data.pid.ki.toFixed(6);
            if (autoKdEl) autoKdEl.value = data.pid.kd.toFixed(6);
            */
        }
    }

    /**
     * Clear system status area
     */
    clearSystemStatusArea() {
        const container = document.getElementById('system-status-container');
        if (container) {
            container.innerHTML = '<!-- Dynamic system status will be created here -->';
        }
    }

    /**
     * Clear all system status areas to prevent duplicate IDs
     */
    clearAllSystemStatusAreas() {
        const dashboardContainer = document.getElementById('dashboard-status-container');
        const manualContainer = document.getElementById('manual-control-status-container');

        if (dashboardContainer) {
            dashboardContainer.innerHTML = '<!-- Dynamic system status will be created here -->';
        }

        if (manualContainer) {
            manualContainer.innerHTML = '<!-- Dynamic system status will be created here -->';
        }

        console.log('Cleared all system status areas');
    }

    /**
     * Handle section change events
     */
    handleSectionChange(sectionName) {
        // Cleanup previous section if leaving manual-control
        if (this.currentSection === 'manual-control' && sectionName !== 'manual-control') {
            if (this.modules.manualControl && this.modules.manualControl.cleanup) {
                this.modules.manualControl.cleanup();
            }
        }

        // Clear any existing status areas before creating new ones
        this.clearAllSystemStatusAreas();

        switch (sectionName) {
            case 'dashboard':
                // this.createSystemStatusArea('dashboard');
                break;

            case 'manual-control':
                // this.createSystemStatusArea('manual-control');
                break;

            case 'tuning':
                // this.createSystemStatusArea('tuning');
                break;

            case 'settings':
                // this.clearSystemStatusArea();
                break;

            case 'not-connected':
                // this.clearSystemStatusArea();
                break;
        }
    }
    
    /**
     * Setup global event listeners
     */
    setupGlobalEventListeners() {
        // Handle page visibility changes
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) {
                console.log('Page hidden, reducing update frequency');
            } else {
                console.log('Page visible, resuming normal updates');
                if (this.connectionState === 'connected') {
                    this.requestSystemStatus();
                }
            }
        });
        
        // Handle keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            this.handleKeyboardShortcuts(e);
        });
        
        // Handle critical errors
        window.addEventListener('error', (e) => {
            console.error('Global error:', e.error);
            this.showNotification('Application Error', 'An unexpected error occurred', 'danger');
        });
    }
    
    /**
     * Handle keyboard shortcuts
     */
    handleKeyboardShortcuts(e) {
        if (this.isTypingInInput(e.target)) {
            return;
        }
        
        // Section navigation with number keys (only if connected or offline mode)
        if (e.key >= '1' && e.key <= '4' && !e.ctrlKey && !e.altKey) {
            const sections = ['dashboard', 'manual-control', 'tuning', 'settings'];
            const sectionIndex = parseInt(e.key) - 1;
            if (sections[sectionIndex] && (this.connectionState === 'connected' || this.offlineMode)) {
                this.showSection(sections[sectionIndex]);
            }
        }
        
        // Refresh with F5 or Ctrl+R
        if (e.key === 'F5' || (e.ctrlKey && e.key === 'r')) {
            e.preventDefault();
            this.refresh();
        }
    }
    
    /**
     * Check if user is typing in an input field
     */
    isTypingInInput(target) {
        return target.tagName === 'INPUT' || target.tagName === 'TEXTAREA';
    }
    
    /**
     * Show notification toast
     */
    showNotification(title, message, type = 'info', duration = 5000) {
        const statusBar = document.getElementById('sro-status-bar');
        const statusIcon = document.getElementById('status-icon');
        const statusMessage = document.getElementById('status-message');

        if (!statusBar || !statusIcon || !statusMessage) {
            console.log(`${type.toUpperCase()}: ${title} - ${message}`);
            return;
        }

        // Set message and icon
        statusMessage.textContent = `${title}: ${message}`;
        statusMessage.className = `flex-grow-1 text-${type}`;

        // Set alert type and icon
        // statusBar.className = `alert alert-${type} alert-dismissible m-0`;
        statusBar.className = `alert bg-primary border rounded-top-0 border-dark py-1`;
        statusIcon.className = `bi bi-${this.getIconForType(type)} text-${type} me-2`;

        // Status bar is always visible now - no show/hide logic

        console.log(`Status: ${title} - ${message}`);
    }

    /**
     * status bar helpr function getIconForType
     */
    getIconForType(type) {
        switch (type) {
            case 'success': return 'check-circle';
            case 'danger': return 'exclamation-triangle';
            case 'warning': return 'exclamation-circle';
            case 'info': return 'info-circle';
            default: return 'info-circle';
        }
    }

    /**
     * Refresh the application
     */
    refresh() {
        console.log('Refreshing application...');
        
        if (this.connectionState === 'connected') {
            this.requestSystemStatus();
        } else if (this.offlineMode) {
            // Refresh mock data in offline mode
            this.injectMockData();
        }
        
        this.showNotification('Refreshed', 'Application data refreshed', 'info');
    }
    
    /**
     * Handle application shutdown/cleanup
     */
    shutdown() {
        console.log('Shutting down SRO Application...');
        
        // Disconnect WebSocket
        if (window.SroWebSocket) {
            window.SroWebSocket.disconnect();
        }
        
        // Clean up modules
        Object.values(this.modules).forEach(module => {
            if (module && typeof module.destroy === 'function') {
                module.destroy();
            }
        });
        
        this.isInitialized = false;
    }
    
    /**
     * Get application status
     */
    getStatus() {
        return {
            initialized: this.isInitialized,
            currentSection: this.currentSection,
            connectionState: this.connectionState,
            offlineMode: this.offlineMode,
            modules: Object.keys(this.modules),
            websocketStatus: window.SroWebSocket ? window.SroWebSocket.getStatus() : 'not available'
        };
    }
}

// Create global app instance
window.SroApp = new SroApp();

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM loaded, initializing centralized SRO Application...');
    window.SroApp.init();
});

// Handle page unload
window.addEventListener('beforeunload', () => {
    window.SroApp.shutdown();
});