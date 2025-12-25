#!/usr/bin/env node
/**
 * Check Bridge Server Setup
 * 
 * This script validates that the bridge-server is properly configured
 * before starting the mobile app.
 */

const fs = require('fs');
const path = require('path');

const GREEN = '\x1b[32m';
const YELLOW = '\x1b[33m';
const RED = '\x1b[31m';
const BLUE = '\x1b[36m';
const RESET = '\x1b[0m';

console.log('');
console.log('='.repeat(70));
console.log(`${BLUE}  DimOS Mobile App - Startup Check${RESET}`);
console.log('='.repeat(70));
console.log('');

const bridgeDir = path.join(__dirname, '..', 'bridge-server');
const envFile = path.join(bridgeDir, '.env');
const envExample = path.join(bridgeDir, 'env.example');
const packageJson = path.join(bridgeDir, 'package.json');

let hasErrors = false;

// Check 1: Bridge server directory exists
console.log(`${BLUE}[1/4]${RESET} Checking bridge-server directory...`);
if (!fs.existsSync(bridgeDir)) {
  console.log(`${RED}  ✗ Bridge server directory not found!${RESET}`);
  console.log(`${RED}  Expected: ${bridgeDir}${RESET}`);
  hasErrors = true;
} else {
  console.log(`${GREEN}  ✓ Bridge server directory found${RESET}`);
}

// Check 2: package.json exists
console.log(`${BLUE}[2/4]${RESET} Checking bridge server package.json...`);
if (!fs.existsSync(packageJson)) {
  console.log(`${RED}  ✗ package.json not found!${RESET}`);
  hasErrors = true;
} else {
  console.log(`${GREEN}  ✓ package.json found${RESET}`);
}

// Check 3: .env file
console.log(`${BLUE}[3/4]${RESET} Checking configuration...`);
if (!fs.existsSync(envFile)) {
  console.log(`${YELLOW}  ⚠ No .env file found${RESET}`);
  
  if (fs.existsSync(envExample)) {
    console.log(`${YELLOW}  Creating .env from env.example...${RESET}`);
    try {
      fs.copyFileSync(envExample, envFile);
      console.log(`${GREEN}  ✓ Created .env file${RESET}`);
      console.log('');
      console.log(`${YELLOW}  ⚠ IMPORTANT: Edit bridge-server/.env with your LCM channel names!${RESET}`);
      console.log(`${YELLOW}     Run 'lcm-spy' to find your channel names${RESET}`);
      console.log('');
    } catch (err) {
      console.log(`${RED}  ✗ Failed to create .env: ${err.message}${RESET}`);
      hasErrors = true;
    }
  } else {
    console.log(`${RED}  ✗ env.example not found!${RESET}`);
    hasErrors = true;
  }
} else {
  console.log(`${GREEN}  ✓ .env file exists${RESET}`);
  
  // Check if .env has been customized
  const envContent = fs.readFileSync(envFile, 'utf8');
  const exampleContent = fs.existsSync(envExample) ? fs.readFileSync(envExample, 'utf8') : '';
  
  if (envContent === exampleContent) {
    console.log(`${YELLOW}  ⚠ .env appears to be using default example values${RESET}`);
    console.log(`${YELLOW}    Consider customizing bridge-server/.env for your robot${RESET}`);
  }
}

// Check 4: Node version
console.log(`${BLUE}[4/4]${RESET} Checking Node.js version...`);
const nodeVersion = process.version;
const majorVersion = parseInt(nodeVersion.slice(1).split('.')[0]);

if (majorVersion < 18) {
  console.log(`${RED}  ✗ Node.js ${nodeVersion} detected${RESET}`);
  console.log(`${RED}    Requires Node.js 18 or higher${RESET}`);
  hasErrors = true;
} else {
  console.log(`${GREEN}  ✓ Node.js ${nodeVersion} (compatible)${RESET}`);
}

console.log('');
console.log('='.repeat(70));

if (hasErrors) {
  console.log(`${RED}Setup incomplete! Please fix the errors above.${RESET}`);
  console.log('');
  process.exit(1);
} else {
  console.log(`${GREEN}✓ All checks passed!${RESET}`);
  console.log('');
  console.log(`${BLUE}Starting services:${RESET}`);
  console.log(`  ${GREEN}1.${RESET} Bridge Server (Node.js) → Port 5555`);
  console.log(`  ${GREEN}2.${RESET} Mobile App (Expo/RN) → Port 8081`);
  console.log('');
  console.log(`${YELLOW}💡 Tips:${RESET}`);
  console.log(`  • Press ${GREEN}i${RESET} for iOS simulator`);
  console.log(`  • Press ${GREEN}a${RESET} for Android emulator`);
  console.log(`  • Scan QR code with Expo Go app on your phone`);
  console.log(`  • Press ${GREEN}Ctrl+C${RESET} to stop all services`);
  console.log('');
  console.log('='.repeat(70));
  console.log('');
}

