# AGV Control Panel

A modern web interface for controlling Autonomous Ground Vehicles (AGV) using ROS2 and MAVROS.

## Features

- Launch and monitor MAVROS connections
- Control AGV bringup processes
- Arm and disarm vehicle
- Set flight modes (MANUAL, AUTO, RTL, etc.)
- Real-time system logging

## Setup

### Backend

1. Install dependencies:

cd backend
pip install -r requirements.txt

2. RUN THE BACKEND SERVER:

uvicorn server:app --reload

### Frontend

1. Open `frontend/index.html` in your web browser
2. Make sure your backend server is running at http://localhost:8000

## Requirements

- Python 3.8+
- ROS2 (Foxy or later)
- MAVROS installed and configured
- Web browser (Chrome, Firefox, Edge, Safari)

## Usage

1. Launch the backend server
2. Open the frontend in your browser
3. Click "Launch MAVROS" to establish the connection
4. Use the control buttons to operate your AGV


