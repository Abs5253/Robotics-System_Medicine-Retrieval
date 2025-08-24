# 🤖 Robotics Project Workspace (Showcase Version)

This src folder contains a **sanitised showcase** of my robotics project workspace.

⚠️ **Note**:  
For privacy/integrity, size, licensing, and dependency reasons, this repo does not contain the full complete workspace (some things omitted here), Singularity/Apptainer `.sif` container, or large datasets (custom gazebo models, darknet weights).  
If you are interested in obtaining the complete project (including the full workspace & container image), please **contact Abs (me) directly**.

---

## 📂 Contents
- src/ → ROS packages (core implementation & project code)

- ../README.md → Documentation overview
- ../Diagrams/ → Figures and diagrams (shown in README)


---

## 🚀 About Src
This src contains packages for a robotic system capable of navigation, speech, computer vision & ocr to retrieve medicines.

My own package (combining all aspects of robot behaviour):
- **medicine_retrieval** - Performing **OCR on medicine labels** in simulation
  - Executing state transitions (SMACH - included in my medicine_retrieval package)

Adapted public packages for full system behaviour:
- darknet - Recognising and detecting medicine bottles - (apdapted public package for yolo integration)
- navigation, depthimage_to_laserscan - Navigating autonomously & safely in Gazebo (adapted ROS public packages)
- dialogflow - Natural Speech Understanding and communication (adapted public package for Dialogflow & audio input/output integration)

---

## 📩 Full Access - Request-only Distribution
For those who want to view the complete workspace (including .sif container and full ROS environment), please reach out to **Abs** (me) on GitHub.
