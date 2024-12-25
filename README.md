# competition-ros2

## AutoRace Competition

This repository contains the setup and codebase for participating in the AutoRace competition.

---

### **Usage**

Follow these steps to set up and run the project.

---

### **1. Install dependencies**

Before proceeding, ensure you have the required dependencies installed. Run the following command:

```bash
pip install -r requirements.txt
```

---

### **2. Build the project**

Build the required packages:

```bash
colcon build --packages-select \
    autorace_camera \
    comp_nedorosl \
    robot_bringup \
    referee_console
```

---

### **3. Source the environment**

Once the build is complete, source the environment:

```bash
source install/setup.sh
```

---

### **4. Run the project**

Launch the following nodes for the competition:

1. Launch the primary robot setup:

    ```bash
    ros2 launch robot_bringup autorace_2023.launch.py
    ```

2. Launch the competition-specific node:

    ```bash
    ros2 launch comp_nedorosl autorace_2024.launch.py
    ```

3. Run the referee console:

    ```bash
    ros2 run referee_console mission_autorace_2023_referee
    ```

---




