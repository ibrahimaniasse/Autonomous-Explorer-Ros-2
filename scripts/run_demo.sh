#!/usr/bin/env bash
set -e

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}====================================================${NC}"
echo -e "${BLUE}   Autonomous Explorer ROS 2 - Demo Launcher        ${NC}"
echo -e "${BLUE}====================================================${NC}"

# Ensure we are in the root directory
if [ ! -f "docker/docker-compose.yml" ]; then
    echo -e "${RED}[ERROR] Please run this script from the root of the repository:${NC}"
    echo -e "        ./scripts/run_demo.sh"
    exit 1
fi

# Check Docker
if ! command -v docker &> /dev/null; then
    echo -e "${RED}[ERROR] Docker is not installed or not in PATH.${NC}"
    exit 1
fi

# Select World
WORLD="warehouse"
if [ "$1" == "--office" ]; then
    WORLD="office"
    echo -e "${YELLOW}Mode: Office environment selected.${NC}"
else
    echo -e "${YELLOW}Mode: Warehouse environment selected. (Use --office to switch)${NC}"
fi

echo -e "\n${GREEN}[1/3] Building Docker Containers...${NC}"
docker compose -f docker/docker-compose.yml build

echo -e "\n${GREEN}[2/3] Launching Simulation ($WORLD)...${NC}"
# Use environment variables so docker-compose passes it to the container if needed.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
docker compose -f docker/docker-compose.yml up -d

echo -e "\n${GREEN}[3/3] Demo is running!${NC}"
echo -e "Wait a few seconds for Gazebo, Nav2, and RViz to initialize."
echo -e "The robot will automatically spin up and begin exploring."
echo -e "${YELLOW}To view logs dynamically:${NC} docker logs -f autonomous-explorer-ros2-ros2_app-1"
echo -e "${YELLOW}To stop the demo:${NC} docker compose -f docker/docker-compose.yml down"

# Wait gracefully (optional)
echo -e "\n${BLUE}Press Ctrl+C to stop and teardown the simulation.${NC}"
trap "echo -e '\n${RED}Stopping Demo...${NC}'; docker compose -f docker/docker-compose.yml down; exit 0" SIGINT SIGTERM
sleep infinity
