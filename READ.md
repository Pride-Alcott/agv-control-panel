# AGV Web Dashboard

A lightweight web dashboard for controlling an AGV using ROS 2 and MAVROS via FastAPI.

## 📦 Features

- Set flight/control modes (AUTO, MANUAL, HOLD, SMART_RTL)
- Arm/Disarm vehicle
- Fully web-accessible from local network
- FastAPI + HTML frontend
- Mock support for testing without real MAVROS

## 🚀 Installation


## ⚙️ systemd Autostart

To run this on boot on a Pi or Linux system:

```bash
sudo cp agv_web_dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable agv_web_dashboard
sudo systemctl start agv_web_dashboard
```

## 📁 Files

- `control.html` – Frontend UI
- `agv_web_control.py` – FastAPI ROS backend
- `agv_web_dashboard.service` – Optional auto-start unit

---

Built for AGV deployments on Raspberry Pi + ROS 2 (Humble)



