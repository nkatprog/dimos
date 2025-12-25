#!/usr/bin/env node
/**
 * Minimal LCM to HTTP Bridge Server for Unitree Go2 Mobile App
 * 
 * This standalone server:
 * - Subscribes to LCM channels for video and lidar data
 * - Re-streams them as HTTP/MJPEG for React Native mobile app
 * - Provides control endpoint for joystick commands
 * 
 * No DimOS dependencies required!
 */

const express = require('express');
const cors = require('cors');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 5555;

// Try to load LCM, but provide a mock if it fails
let lcm;
let USE_MOCK_LCM = false;

try {
  lcm = require('lcm');
  // Test if LCM is properly loaded
  if (!lcm || !lcm.LCM) {
    throw new Error('LCM module not properly loaded');
  }
} catch (err) {
  console.warn('⚠️  LCM module not available, using mock mode');
  console.warn('   Install LCM from: https://github.com/lcm-proj/lcm');
  console.warn('   Or use Python backend for full LCM support');
  console.warn('');
  USE_MOCK_LCM = true;

  // Mock LCM client for development without real LCM
  lcm = {
    LCM: class MockLCM {
      constructor(url) {
        this.url = url;
        this.subscriptions = new Map();
      }

      subscribe(channel, callback) {
        this.subscriptions.set(channel, callback);
        console.log(`[MOCK] Subscribed to channel: ${channel}`);
      }

      publish(channel, buffer) {
        console.log(`[MOCK] Published to ${channel}: ${buffer.length} bytes`);
      }

      handle() {
        // Mock - does nothing
      }

      destroy() {
        console.log('[MOCK] LCM client destroyed');
      }
    }
  };
}

// Middleware
// Configure CORS with explicit origins for security
// TODO: Update allowed origins for production deployment
const corsOptions = {
  origin: process.env.ALLOWED_ORIGINS ? process.env.ALLOWED_ORIGINS.split(',') : '*',
  credentials: true,
};
app.use(cors(corsOptions));
app.use(express.json());

// LCM Configuration (can be updated via API)
let config = {
  LCM_URL: process.env.LCM_URL || 'udpm://239.255.76.67:7667?ttl=1',
  VIDEO_CHANNEL: process.env.VIDEO_CHANNEL || 'UNITREE_VIDEO',
  LIDAR_CHANNEL: process.env.LIDAR_CHANNEL || 'UNITREE_LIDAR',
  CONTROL_CHANNEL: process.env.CONTROL_CHANNEL || 'UNITREE_CONTROL'
};

// Load config from file if it exists
const CONFIG_FILE = path.join(__dirname, 'config.json');
if (fs.existsSync(CONFIG_FILE)) {
  try {
    const savedConfig = JSON.parse(fs.readFileSync(CONFIG_FILE, 'utf8'));
    config = { ...config, ...savedConfig };
    console.log('✓ Loaded configuration from config.json');
  } catch (err) {
    console.warn('⚠ Could not load config.json, using defaults');
  }
}

// Extract for backward compatibility
const LCM_URL = config.LCM_URL;
const VIDEO_CHANNEL = config.VIDEO_CHANNEL;
const LIDAR_CHANNEL = config.LIDAR_CHANNEL;
const CONTROL_CHANNEL = config.CONTROL_CHANNEL;

// Initialize LCM
let lcmClient;
try {
  lcmClient = new lcm.LCM(LCM_URL);
  if (USE_MOCK_LCM) {
    console.log(`✓ Mock LCM client created (development mode)`);
    console.log(`  Real robot connection requires LCM installation`);
  } else {
    console.log(`✓ Connected to LCM at ${LCM_URL}`);
  }
} catch (err) {
  console.error('❌ Failed to create LCM client:', err.message);
  console.error('Tip: Use mock data mode in mobile app for testing without LCM');
  process.exit(1);
}

// Stream buffers (store latest frame for each client)
const videoClients = new Set();
const lidarClients = new Set();
let latestVideoFrame = null;
let latestLidarFrame = null;

// Subscribe to LCM channels
lcmClient.subscribe(VIDEO_CHANNEL, (channel, message) => {
  try {
    latestVideoFrame = message;

    // Broadcast to all connected video clients
    const disconnectedClients = [];
    videoClients.forEach(client => {
      if (client && !client.destroyed) {
        try {
          client.write('--frame\r\n');
          client.write('Content-Type: image/jpeg\r\n\r\n');
          client.write(message);
          client.write('\r\n');
        } catch (err) {
          console.warn(`Video client write failed: ${err.message}`);
          disconnectedClients.push(client);
        }
      } else {
        disconnectedClients.push(client);
      }
    });

    // Clean up disconnected clients
    disconnectedClients.forEach(client => videoClients.delete(client));
  } catch (err) {
    console.error('Error processing video frame:', err.message);
  }
});

lcmClient.subscribe(LIDAR_CHANNEL, (channel, message) => {
  try {
    latestLidarFrame = message;

    // Broadcast to all connected lidar clients
    const disconnectedClients = [];
    lidarClients.forEach(client => {
      if (client && !client.destroyed) {
        try {
          client.write('--frame\r\n');
          client.write('Content-Type: image/jpeg\r\n\r\n');
          client.write(message);
          client.write('\r\n');
        } catch (err) {
          console.warn(`Lidar client write failed: ${err.message}`);
          disconnectedClients.push(client);
        }
      } else {
        disconnectedClients.push(client);
      }
    });

    // Clean up disconnected clients
    disconnectedClients.forEach(client => lidarClients.delete(client));
  } catch (err) {
    console.error('Error processing lidar frame:', err.message);
  }
});

// Routes
app.get('/', (req, res) => {
  res.json({
    service: 'Unitree Mobile Bridge Server',
    version: '1.0.0',
    status: 'running',
    lcm_url: LCM_URL,
    channels: {
      video: VIDEO_CHANNEL,
      lidar: LIDAR_CHANNEL,
      control: CONTROL_CHANNEL
    },
    endpoints: {
      streams: '/streams',
      video: '/video_feed/unitree_video',
      lidar: '/video_feed/unitree_costmap',
      control: '/control (POST)'
    }
  });
});

app.get('/streams', (req, res) => {
  res.json({
    streams: ['unitree_video', 'unitree_costmap']
  });
});

// Video stream endpoint (MJPEG)
app.get('/video_feed/unitree_video', (req, res) => {
  res.writeHead(200, {
    'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
    'Cache-Control': 'no-cache',
    'Connection': 'keep-alive'
  });

  // Add this client to the set
  videoClients.add(res);
  console.log(`Video client connected (${videoClients.size} total)`);

  // Send latest frame immediately if available
  if (latestVideoFrame) {
    try {
      res.write('--frame\r\n');
      res.write('Content-Type: image/jpeg\r\n\r\n');
      res.write(latestVideoFrame);
      res.write('\r\n');
    } catch (err) {
      // Client disconnected
    }
  }

  // Handle client disconnect
  req.on('close', () => {
    videoClients.delete(res);
    console.log(`Video client disconnected (${videoClients.size} remaining)`);
  });
});

// Lidar/Costmap stream endpoint (MJPEG)
app.get('/video_feed/unitree_costmap', (req, res) => {
  res.writeHead(200, {
    'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
    'Cache-Control': 'no-cache',
    'Connection': 'keep-alive'
  });

  // Add this client to the set
  lidarClients.add(res);
  console.log(`Lidar client connected (${lidarClients.size} total)`);

  // Send latest frame immediately if available
  if (latestLidarFrame) {
    try {
      res.write('--frame\r\n');
      res.write('Content-Type: image/jpeg\r\n\r\n');
      res.write(latestLidarFrame);
      res.write('\r\n');
    } catch (err) {
      // Client disconnected
    }
  }

  // Handle client disconnect
  req.on('close', () => {
    lidarClients.delete(res);
    console.log(`Lidar client disconnected (${lidarClients.size} remaining)`);
  });
});

// Control endpoint (joystick commands)
app.post('/control', (req, res) => {
  try {
    const { linear_x, linear_y, angular_z } = req.body;

    // Validate input ranges (safety limits)
    const MAX_LINEAR = 1.5;  // m/s
    const MAX_ANGULAR = 2.0;  // rad/s

    const clamp = (val, max) => Math.max(-max, Math.min(max, val || 0));

    const vx = clamp(linear_x, MAX_LINEAR);
    const vy = clamp(linear_y, MAX_LINEAR);
    const vz = clamp(angular_z, MAX_ANGULAR);

    // Create control message for LCM
    // NOTE: This sends a JSON string to LCM, which may not work with all robots.
    // For production use with Unitree Go2, you should:
    // 1. Define proper LCM message types (e.g., twist_t.lcm)
    // 2. Use lcm-gen to generate encoders
    // 3. Encode messages properly using the generated code
    //
    // For now, this works with the Python DimOS backend which accepts JSON.
    const controlMsg = {
      timestamp: Date.now(),
      linear: { x: vx, y: vy, z: 0 },
      angular: { x: 0, y: 0, z: vz }
    };

    const msgBuffer = Buffer.from(JSON.stringify(controlMsg));
    lcmClient.publish(CONTROL_CHANNEL, msgBuffer);

    console.log(`Control: vx=${vx.toFixed(2)}, vy=${vy.toFixed(2)}, vz=${vz.toFixed(2)}`);

    res.json({
      success: true,
      velocities: {
        linear_x: vx,
        linear_y: vy,
        angular_z: vz
      }
    });
  } catch (err) {
    console.error('Error sending control command:', err.message);
    res.status(500).json({
      success: false,
      message: err.message
    });
  }
});

// Health check
app.get('/health', (req, res) => {
  res.json({
    status: 'healthy',
    uptime: process.uptime(),
    video_clients: videoClients.size,
    lidar_clients: lidarClients.size,
    latest_video: latestVideoFrame !== null,
    latest_lidar: latestLidarFrame !== null
  });
});

// Configuration endpoints
app.get('/config', (req, res) => {
  res.json({
    serverIP: 'localhost',
    serverPort: PORT.toString(),
    videoChannel: config.VIDEO_CHANNEL,
    lidarChannel: config.LIDAR_CHANNEL,
    controlChannel: config.CONTROL_CHANNEL,
    lcmUrl: config.LCM_URL
  });
});

app.post('/config', (req, res) => {
  try {
    const { videoChannel, lidarChannel, controlChannel, lcmUrl } = req.body;
    
    // Update config
    if (videoChannel) config.VIDEO_CHANNEL = videoChannel;
    if (lidarChannel) config.LIDAR_CHANNEL = lidarChannel;
    if (controlChannel) config.CONTROL_CHANNEL = controlChannel;
    if (lcmUrl) config.LCM_URL = lcmUrl;
    
    // Save to file
    fs.writeFileSync(CONFIG_FILE, JSON.stringify(config, null, 2));
    
    res.json({
      success: true,
      message: 'Configuration saved. Server restart required for changes to take effect.',
      config: {
        videoChannel: config.VIDEO_CHANNEL,
        lidarChannel: config.LIDAR_CHANNEL,
        controlChannel: config.CONTROL_CHANNEL,
        lcmUrl: config.LCM_URL
      }
    });
  } catch (err) {
    res.status(500).json({
      success: false,
      message: err.message
    });
  }
});

app.post('/restart', (req, res) => {
  res.json({
    success: true,
    message: 'Server restarting...'
  });
  
  setTimeout(() => {
    process.exit(0); // Exit and let process manager restart
  }, 1000);
});

app.post('/detect-channels', async (req, res) => {
  try {
    const { lcmUrl, timeout } = req.body;
    const detectedChannels = new Set();
    
    // Create a temporary LCM instance for detection
    const testLcm = new lcm.LCM(lcmUrl || config.LCM_URL);
    
    // Listen to all channels for a short time
    const channelRegex = /.*/; // Match all channels
    testLcm.subscribe(channelRegex, (channel) => {
      detectedChannels.add(channel);
    });
    
    // Run detection for specified timeout
    const detectionTimeout = timeout || 5000;
    const startTime = Date.now();
    
    const detectionInterval = setInterval(() => {
      testLcm.handle();
    }, 1);
    
    setTimeout(() => {
      clearInterval(detectionInterval);
      testLcm.destroy();
      
      res.json({
        success: true,
        channels: Array.from(detectedChannels),
        duration: Date.now() - startTime
      });
    }, detectionTimeout);
    
  } catch (err) {
    res.status(500).json({
      success: false,
      message: err.message,
      channels: []
    });
  }
});

// Start LCM event loop
// Use 10ms interval for better CPU efficiency (was 1ms = 1000 calls/sec)
setInterval(() => {
  try {
    lcmClient.handle();
  } catch (err) {
    console.error('LCM handle error:', err.message);
  }
}, 10);

// Start server
app.listen(PORT, '0.0.0.0', () => {
  console.log('');
  console.log('='.repeat(60));
  console.log('  Unitree Mobile Bridge Server');
  console.log('='.repeat(60));
  console.log('');
  console.log(`✓ Server running on http://0.0.0.0:${PORT}`);

  if (USE_MOCK_LCM) {
    console.log(`⚠️  Running in MOCK MODE (no real LCM)`);
    console.log(`   Use "Mock Data" mode in mobile app for testing`);
    console.log(`   For real robot: Install LCM or use Python backend`);
  } else {
    console.log(`✓ LCM URL: ${LCM_URL}`);
    console.log('');
    console.log('Subscribed to LCM channels:');
    console.log(`  - ${VIDEO_CHANNEL} (video stream)`);
    console.log(`  - ${LIDAR_CHANNEL} (lidar/costmap)`);
  }

  console.log('');
  console.log('Available endpoints:');
  console.log(`  - GET  http://0.0.0.0:${PORT}/streams`);
  console.log(`  - GET  http://0.0.0.0:${PORT}/video_feed/unitree_video`);
  console.log(`  - GET  http://0.0.0.0:${PORT}/video_feed/unitree_costmap`);
  console.log(`  - POST http://0.0.0.0:${PORT}/control`);
  console.log('');

  if (USE_MOCK_LCM) {
    console.log('💡 To connect to real robot:');
    console.log('   1. Install LCM: https://lcm-proj.github.io');
    console.log('   2. OR use Python backend (see ../mobile_stream_server.py)');
  } else {
    console.log('Mobile app can now connect to robot!');
  }

  console.log('='.repeat(60));
  console.log('');
});

// Graceful shutdown
process.on('SIGTERM', () => {
  console.log('Shutting down gracefully...');
  videoClients.forEach(client => client.end());
  lidarClients.forEach(client => client.end());
  lcmClient.destroy();
  process.exit(0);
});

process.on('SIGINT', () => {
  console.log('\nShutting down...');
  videoClients.forEach(client => client.end());
  lidarClients.forEach(client => client.end());
  lcmClient.destroy();
  process.exit(0);
});

