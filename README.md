# CPS4042 – Real-Time Embedded Systems Project

## 📌 Overview
This project simulates communication protocols in a Cyber-Physical System environment, including:

- I2C Protocol (Sensor Communication)
- USART Full-Duplex Communication
- I2C Multiplexer (Multi-Sensor Management)
- Future Extension: Java/Spring Backend Integration

Developed as part of CPS4042 – Real-Time Embedded Systems.

---

## 👥 Team Members

| Name   | Responsibility |
|--------|---------------|
| Amir   | Integration, Documentation, CMake, Final Testing |
| Negin  | I2C Protocol Simulation |
| Armin  | USART Full Duplex Communication |
| Saba   | I2C Multiplexer |

---

## 🏗️ Project Structure
CPS4042-RTES-Project/

│

├── src/

│ ├── i2c/

│ ├── usart/

│ ├── mux/

│ └── main.cpp

│

├── include/

├── tests/

├── docs/

├── CMakeLists.txt

└── README.md

text

---

## 🌳 Branching Strategy

- `main` → Stable, production-ready
- `dev` → Integration branch
- `feature/<name>` → Individual work branches

Example:
feature/negin

feature/armin

feature/saba

feature/amir

text

---


## ⚙️ Build Instructions
```bash
mkdir build
cd build
cmake ..
make
./app


## 🔁 Development Workflow
Work on your feature branch
Pull latest dev
Commit changes
Push branch
Open Pull Request → dev
After review → Merge


## 📊 Communication Architecture
Sensor(s) --> I2C --> I2CMux --> USART --> MCU --> Storage


## ✅ Code Standards
Follow C++17
Clear commit messages
One feature per branch
Pull Request required before merging
