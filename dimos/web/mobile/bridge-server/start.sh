#!/bin/bash
# Quick start script for Unitree Mobile Bridge Server

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========================================================================"
echo "  Unitree Mobile Bridge Server Launcher"
echo "========================================================================"
echo ""

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo -e "${YELLOW}❌ Node.js not found!${NC}"
    echo "Please install Node.js 18+ first:"
    echo "  curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -"
    echo "  sudo apt install -y nodejs"
    exit 1
fi

echo -e "${GREEN}✓ Node.js $(node --version) found${NC}"

# Check if in bridge-server directory
if [ ! -f "package.json" ]; then
    echo -e "${YELLOW}❌ Not in bridge-server directory${NC}"
    echo "Please run from: dimos/web/mobile/bridge-server/"
    exit 1
fi

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}📦 Installing dependencies...${NC}"
    npm install
fi

# Check if .env exists
if [ ! -f ".env" ]; then
    echo -e "${YELLOW}⚠️  No .env file found${NC}"
    echo "Creating from .env.example..."
    cp .env.example .env
    echo ""
    echo -e "${YELLOW}Please edit .env with your LCM channel names!${NC}"
    echo "  nano .env"
    echo ""
    read -p "Continue with default settings? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Load .env
export $(cat .env | grep -v '^#' | xargs)

echo ""
echo "Configuration:"
echo "  Port: ${PORT:-5555}"
echo "  LCM URL: ${LCM_URL:-udpm://239.255.76.67:7667?ttl=1}"
echo "  Video Channel: ${VIDEO_CHANNEL:-UNITREE_VIDEO}"
echo "  Lidar Channel: ${LIDAR_CHANNEL:-UNITREE_LIDAR}"
echo ""

# Start server
echo -e "${GREEN}🚀 Starting server...${NC}"
echo ""
npm start

