#!/usr/bin/env node
/**
 * Wait for Bridge Server to be Ready
 *
 * This script waits for the bridge server to respond to health checks
 * before declaring the system ready.
 */

const http = require('http');

const BRIDGE_PORT = process.env.PORT || 5555;
const MAX_ATTEMPTS = 30;
const RETRY_DELAY = 1000; // 1 second

const GREEN = '\x1b[32m';
const YELLOW = '\x1b[33m';
const BLUE = '\x1b[36m';
const RESET = '\x1b[0m';

function checkBridgeHealth(attempt = 1) {
  return new Promise((resolve, reject) => {
    const options = {
      hostname: 'localhost',
      port: BRIDGE_PORT,
      path: '/health',
      method: 'GET',
      timeout: 2000
    };

    const req = http.request(options, (res) => {
      if (res.statusCode === 200) {
        resolve(true);
      } else {
        reject(new Error(`Bridge returned status ${res.statusCode}`));
      }
    });

    req.on('error', (err) => {
      reject(err);
    });

    req.on('timeout', () => {
      req.destroy();
      reject(new Error('Request timeout'));
    });

    req.end();
  });
}

async function waitForBridge() {
  console.log(`${BLUE}Waiting for bridge server to start...${RESET}`);

  for (let attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    try {
      await checkBridgeHealth(attempt);
      console.log(`${GREEN}✓ Bridge server is ready on port ${BRIDGE_PORT}!${RESET}`);
      console.log('');
      return true;
    } catch (err) {
      if (attempt < MAX_ATTEMPTS) {
        process.stdout.write(`${YELLOW}  Attempt ${attempt}/${MAX_ATTEMPTS}...${RESET}\r`);
        await new Promise(resolve => setTimeout(resolve, RETRY_DELAY));
      } else {
        console.log('');
        console.log(`${YELLOW}⚠ Bridge server health check timed out${RESET}`);
        console.log(`${YELLOW}  The server may still be starting...${RESET}`);
        console.log('');
        return false;
      }
    }
  }
}

// Run if executed directly
if (require.main === module) {
  waitForBridge()
    .then(() => process.exit(0))
    .catch((err) => {
      console.error('Error:', err.message);
      process.exit(1);
    });
}

module.exports = { waitForBridge, checkBridgeHealth };
