# 🌀 Cam Profile Generator — Desktop Application

A **desktop-based Cam Profile Generator** developed in **Python** to design and visualize cam mechanisms using standard kinematic parameters and motion laws.

This application is intended for **mechanical engineering students** and serves as an educational and analytical tool for cam profile generation.

---

## 📌 Features

- Runs locally as a **desktop application**
- User-defined cam design parameters:
  - Stroke
  - Base circle radius
  - Rise angle
  - Return angle
  - Dwell angles
- Independent motion law selection for:
  - Rise
  - Return
- Automatic generation of:
  - Displacement diagram
  - Pitch curve
  - Cam profile
- Modular computation logic for easy extension

---

## 🛠️ Tech Stack

- **Python 3**
- **Flask** (used internally for local UI rendering)
- **HTML / CSS**
- **NumPy**
- **Matplotlib / Plotly**

> Note: Flask is used only for local interface rendering.  
> This project is **not intended for web deployment**.

---

## 📂 Project Structure

```
cam-profile-generator/
│
├── cam_profile_generator.py
├── webapp/
├   ├──
├── requirements.txt
└── README.md
```

---

## ⚙️ Installation

### 1. Clone the repository
```bash
git clone https://github.com/your-username/cam-profile-generator.git
cd cam-profile-generator
```

### 2. Create and activate a virtual environment (recommended)
```bash
python -m venv venv
source venv/bin/activate        # Linux / macOS
venv\Scripts\activate           # Windows
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

---

## ▶️ Running the Application

```bash
python cam_profile_generator.py
```

The application runs locally and opens in your default browser.  
It behaves like a **desktop engineering tool** and does not require an internet connection.

---

## 🧭 Usage Instructions

1. Launch the application
2. Enter cam parameters:
   - Stroke
   - Base circle radius
   - Rise angle
   - Dwell angles
   - Return angle
3. Select motion laws for **rise** and **return**
4. Submit the inputs
5. Analyze the generated:
   - Displacement diagram
   - Pitch curve
   - Cam profile

---

## 📐 Engineering Overview

Follower displacement is calculated using selected **motion laws**, from which the pitch curve is generated.  
The pitch curve is then transformed into the final cam profile coordinates.

The separation between UI and computational logic makes the system extensible for advanced cam analysis.

---

## 🚀 Future Scope

- Pressure angle analysis
- Velocity, acceleration, and jerk plots
- Multiple follower types (roller, flat-faced, knife-edge)
- Offset follower support
- Export cam profiles as **DXF / CSV**
- Packaging as a standalone executable using **PyInstaller**

---

## 🎓 Intended Use

- Mechanical Engineering mini / major project
- Kinematics of Machinery coursework
- Educational visualization of cam mechanisms

---

## 📜 License

This project is licensed under the **MIT License**.  
Free to use, modify, and distribute for academic and educational purposes.
