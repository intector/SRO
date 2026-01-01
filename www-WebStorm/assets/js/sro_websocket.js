/**
 * SRO WebSocket Communication Module
 * Handles real-time communication with the ESP32-S3 controller
 */

class SroWebSocket {
    constructor() {
        this.ws = null;
        this.reconnectTimer = null;
        this.reconnectAttempts = 0;
        // this.maxReconnectAttempts = 5;
        this.reconnectDelay = 3000;
        this.isConnected = false;
        this.messageQueue = [];
        this.eventHandlers = new Map();
        
        // Bind methods
        this.connect = this.connect.bind(this);
        this.onOpen = this.onOpen.bind(this);
        this.onMessage = this.onMessage.bind(this);
        this.onClose = this.onClose.bind(this);
        this.onError = this.onError.bind(this);
    }
    
    /**
     * Initialize WebSocket connection
     */
    connect() {
        try {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;
            
            console.log('Connecting to WebSocket:', wsUrl);
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = this.onOpen;
            this.ws.onmessage = this.onMessage;
            this.ws.onclose = this.onClose;
            this.ws.onerror = this.onError;
            
        } catch (error) {
            console.error('WebSocket connection error:', error);
            this.scheduleReconnect();
        }
    }
    
    /**
     * WebSocket opened event handler
     */
    onOpen(event) {
        console.log('WebSocket connected');
        this.isConnected = true;
        this.reconnectAttempts = 0;
        
        // Update UI connection status
        this.updateConnectionStatus('connected');
        
        // Send any queued messages
        this.flushMessageQueue();
        
        // Trigger connected event
        this.emit('connected', {});
        
        // Show success notification
        window.SroApp?.showNotification('Connected', 'WebSocket connection established', 'success');
    }
    
    /**
     * WebSocket message received event handler
     */
    onMessage(event) {
        try {
            const data = JSON.parse(event.data);
            console.log('WebSocket message received:', data);
            
            // Handle different message types
            if (data.event) {
                this.handleEvent(data);
            } else if (data.response) {
                this.handleResponse(data);
            }
            
        } catch (error) {
            console.error('Error parsing WebSocket message:', error, event.data);
        }
    }
    
    /**
     * WebSocket closed event handler
     */
    onClose(event) {
        console.log('WebSocket closed:', event.code, event.reason);
        this.isConnected = false;
        
        // Update UI connection status
        this.updateConnectionStatus('disconnected');
        
        // Trigger disconnected event
        this.emit('disconnected', { code: event.code, reason: event.reason });
        
        // Schedule reconnection if not intentional close
        if (event.code !== 1000) {
            this.scheduleReconnect();
        }
    }
    
    /**
     * WebSocket error event handler
     */
    onError(error) {
        console.error('WebSocket error:', error);
        this.updateConnectionStatus('error');
        
        // Show error notification
        window.SroApp?.showNotification('Connection Error', 'WebSocket connection failed', 'danger');
    }
    
    /**
     * Handle incoming events from the server
     */
    handleEvent(data) {
        const { event } = data;

        switch (event) {
            case 'tempUpdate':
                this.emit('temperatureUpdate', data);
                break;

            case 'hardwareUpdate':
                this.emit('hardwareUpdate', data);
                break;

            case 'systemStatus':
                this.emit('systemStatus', data);
                break;

            case 'profileStatus':
                this.emit('profileStatus', data);
                break;

            case 'wifiStatus':
                this.emit('wifiStatus', data);
                break;

            case 'profileProgress':
                this.emit('profileProgress', data);
                break;

            case 'hardwareStatus':
                this.emit('hardwareStatus', data);
                break;

            // *** Profile start, stop and complete
            case 'profileStarted':
                this.emit('profileStarted', data);
                break;

            case 'profileStopped':
                this.emit('profileStopped', data);
                break;

            case 'profileCompleted':
                this.emit('profileCompleted', data);
                break;

            // *** Handle profile data from ESP32 ***
            case 'profilesData':
                this.emit('profilesData', data);
                break;

            case 'profileSaved':
                this.emit('profileSaved', data);
                break;

            // *** TUNING EVENT HANDLERS ***
            case 'tuningStarted':
                this.emit('tuningStarted', data);
                break;

            case 'tuningProgress':
                this.emit('tuningProgress', data);
                break;

            case 'tuningResults':
                this.emit('tuningResults', data);
                break;

            case 'tuningData':
                this.emit('tuningData', data);
                break;

            case 'tuningCompleted':
                this.emit('tuningCompleted', data);
                break;

            default:
                console.log('Unknown event:', event, data);
                this.emit('unknownEvent', data);
        }
    }

    /**
     * Handle command responses from the server
     */
    handleResponse(data) {
        this.emit('response', data);
    }
    
    /**
     * Send a command to the server
     */
    sendCommand(command, parameters = {}) {
        const message = {
            cmd: command,
            timestamp: Date.now(),
            ...parameters
        };
        
        return this.send(message);
    }
    
    /**
     * Send a message to the server
     */
    send(message) {
        if (this.isConnected && this.ws.readyState === WebSocket.OPEN) {
            try {
                const messageStr = JSON.stringify(message);
                this.ws.send(messageStr);
                console.log('Sent WebSocket message:', message);
                return true;
            } catch (error) {
                console.error('Error sending WebSocket message:', error);
                return false;
            }
        } else {
            // Queue message for later sending
            this.messageQueue.push(message);
            console.log('WebSocket not connected, message queued');
            return false;
        }
    }
    
    /**
     * Send queued messages
     */
    flushMessageQueue() {
        while (this.messageQueue.length > 0) {
            const message = this.messageQueue.shift();
            this.send(message);
        }
    }
    
    /**
     * Schedule automatic reconnection
     */
    scheduleReconnect() {
        // if (this.reconnectAttempts >= this.maxReconnectAttempts) {
        //     console.log('Max reconnection attempts reached');
        //     this.updateConnectionStatus('failed');
        //     window.SroApp?.showNotification('Connection Failed', 'Unable to reconnect to controller', 'danger');
        //     return;
        // }
        
        this.reconnectAttempts++;
        console.log(`Scheduling reconnection attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts}`);
        
        this.updateConnectionStatus('connecting');
        
        this.reconnectTimer = setTimeout(() => {
            this.connect();
        }, this.reconnectDelay);
    }
    
    /**
     * Update connection status in the UI
     */
    updateConnectionStatus(status) {
        const wsStatusElement = document.getElementById('ws-status');
        const connectionStatusElement = document.getElementById('connection-status');
        
        if (wsStatusElement) {
            switch (status) {
                case 'connected':
                    wsStatusElement.textContent = 'Connected';
                    wsStatusElement.className = 'text-success';
                    break;
                case 'connecting':
                    wsStatusElement.textContent = 'Connecting...';
                    wsStatusElement.className = 'text-warning';
                    break;
                case 'disconnected':
                    wsStatusElement.textContent = 'Disconnected';
                    wsStatusElement.className = 'text-danger';
                    break;
                case 'error':
                    wsStatusElement.textContent = 'Error';
                    wsStatusElement.className = 'text-danger';
                    break;
                case 'failed':
                    wsStatusElement.textContent = 'Failed';
                    wsStatusElement.className = 'text-danger';
                    break;
            }
        }
        
        if (connectionStatusElement) {
            connectionStatusElement.className = 'bi bi-circle-fill';
            switch (status) {
                case 'connected':
                    connectionStatusElement.classList.add('text-success');
                    break;
                case 'connecting':
                    connectionStatusElement.classList.add('text-warning');
                    break;
                default:
                    connectionStatusElement.classList.add('text-danger');
            }
        }
    }
    
    /**
     * Register event handler
     */
    on(event, handler) {
        if (!this.eventHandlers.has(event)) {
            this.eventHandlers.set(event, []);
        }
        this.eventHandlers.get(event).push(handler);
    }
    
    /**
     * Remove event handler
     */
    off(event, handler) {
        if (this.eventHandlers.has(event)) {
            const handlers = this.eventHandlers.get(event);
            const index = handlers.indexOf(handler);
            if (index > -1) {
                handlers.splice(index, 1);
            }
        }
    }
    
    /**
     * Emit event to all registered handlers
     */
    emit(event, data) {
        if (this.eventHandlers.has(event)) {
            this.eventHandlers.get(event).forEach(handler => {
                try {
                    handler(data);
                } catch (error) {
                    console.error(`Error in event handler for ${event}:`, error);
                }
            });
        }
    }
    
    /**
     * Disconnect WebSocket
     */
    disconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
        
        if (this.ws) {
            this.ws.close(1000, 'Client disconnect');
            this.ws = null;
        }
        
        this.isConnected = false;
        this.updateConnectionStatus('disconnected');
    }
    
    /**
     * Get connection status
     */
    getStatus() {
        return {
            connected: this.isConnected,
            readyState: this.ws ? this.ws.readyState : WebSocket.CLOSED,
            reconnectAttempts: this.reconnectAttempts
        };
    }
}

// Export WebSocket instance
window.SroWebSocket = new SroWebSocket();