#!/bin/bash

# RobotLab Cloud Server Startup Script

echo "🤖 Starting RobotLab Cloud Server..."
echo "=================================="

# Check if conda is available
if ! command -v conda &> /dev/null; then
    echo "❌ Conda not found. Please install Anaconda or Miniconda first."
    echo "   Download from: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

# Check if pin_env environment exists
if ! conda env list | grep -q "pin_env"; then
    echo "📦 Creating pin_env conda environment..."
    conda create -n pin_env python=3.10 -y
    echo "🔧 Installing Pinocchio (this may take a few minutes)..."
    conda activate pin_env
    conda install -c conda-forge pinocchio -y
    echo "📚 Installing other dependencies..."
    pip install -r requirements.txt
else
    echo "✅ Found existing pin_env environment"
fi

# Activate environment and start server
echo "🚀 Activating environment and starting server..."
conda activate pin_env

# Check if port 8000 is in use
if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  Port 8000 is already in use. Stopping existing processes..."
    pkill -f rcm_server.py
    sleep 2
fi

echo "🌐 Starting RobotLab Cloud server on http://localhost:8000"
echo "   Press Ctrl+C to stop the server"
echo ""

python3 rcm_server.py
