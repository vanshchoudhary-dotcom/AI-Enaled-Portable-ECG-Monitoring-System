AI-Enabled Portable ECG Monitoring System for Early Cardiac Risk Detection

📌 Project Overview

The AI-Enabled Portable ECG Monitoring System for Early Cardiac Risk Detection is a compact, low-cost and portable healthcare monitoring prototype designed to acquire ECG signals and provide real-time cardiac status information.

The system combines ECG signal acquisition, digital signal processing, machine learning, physiological sensing, ESP32-based embedded processing, MQTT communication, and a Flutter mobile dashboard into a single monitoring platform.

The primary objective is to develop an affordable portable system that can monitor ECG signals, extract useful features, classify the detected heartbeat pattern, and provide an understandable status to the user.

🎯 Objectives

Acquire ECG signals using the AD8232 ECG sensor.
Process and filter ECG signals using the ESP32.
Detect important ECG waveform characteristics such as R-peaks.
Calculate heart rate from the ECG signal.
Extract numerical features from ECG heartbeat segments.
Use a Machine Learning model for heartbeat classification.
Display ECG and physiological information on an OLED display.
Measure pulse rate and SpO₂ using the MAX30102 sensor.
Transmit processed data wirelessly using Wi-Fi and MQTT.
Display monitoring information through a Flutter mobile application.
Develop a portable and expandable embedded healthcare platform.

🔧 Hardware Components

Component	Purpose
ESP32 Dev Module	Main microcontroller and wireless communication
AD8232	ECG signal acquisition
MAX30102	Heart rate and SpO₂ measurement
OLED Display	Local real-time information display
LEDs	System/status indication
TP4056	Li-ion/Li-Po battery charging
Battery	Portable power supply
Push Button/Switch	Power/control
Pref-Board	Circuit assembly

⚙️ Working Principle

1. ECG Signal Acquisition

The AD8232 is used as the ECG analog front-end.

Electrodes attached to the body detect the electrical activity of the heart. The AD8232 amplifies and conditions the weak ECG signal and provides an analog output to the ESP32 ADC.

Electrodes
    ↓
AD8232
    ↓
Analog ECG Signal
    ↓
ESP32 ADC

2. ECG Signal Processing

The raw ECG signal can contain:

Baseline wandering
Power-line interference
High-frequency noise
Motion artifacts
Digital filtering is therefore applied before feature extraction.

The project uses filtering concepts such as a Butterworth filter to reduce unwanted frequency components while preserving important ECG waveform characteristics.

3. R-Peak Detection
 
The processed ECG waveform is analyzed to identify R-peaks.

The time difference between consecutive R-peaks can be used to estimate heart rate.

The basic relationship is:

Heart Rate (BPM) = 60 / RR interval (seconds)
For example, if:

RR interval = 1 second
then:

Heart Rate = 60 BPM

4. Heartbeat Segmentation

After identifying an R-peak, a fixed window around the peak is extracted to represent an individual heartbeat.

For the current dataset processing:

90 samples before R-peak
+
162 samples after R-peak
=
252 samples per heartbeat
The MIT-BIH ECG data used in the initial model uses a sampling frequency of:

360 Hz

🤖 Artificial Intelligence / Machine Learning

The project uses Machine Learning to classify ECG heartbeat patterns.

Initially, a Random Forest classifier was used for classification.

Instead of directly feeding the entire ECG waveform into the classifier, numerical features are extracted from each heartbeat.

Features used
Mean
Standard deviation
Maximum value
Minimum value
Peak-to-peak value
RMS (Root Mean Square)
These features form the input vector for the Random Forest model.

Example:

ECG Beat
   ↓
Feature Extraction
   ↓
[Mean, Std, Max, Min, P2P, RMS]
   ↓
Random Forest
   ↓
Classification

📊 Dataset

The initial model development used ECG data from the MIT-BIH Arrhythmia Database.

The ECG recordings were processed into individual heartbeat segments.

The initial prototype used:

33 Normal beats
+
33 Abnormal beats
for model development and testing.

The initial prototype achieved approximately:

Accuracy: 85.71%
with the available small dataset.

🚀 Dataset Upgrade

To improve the reliability of the model, the dataset is being expanded.

Target dataset:

~500 Normal beats
+
~500 Abnormal beats
The upgraded approach also aims to use samples from multiple ECG records and perform a record-wise train/test split.

This is important because randomly splitting beats from the same patient/record can cause data leakage and produce overly optimistic results.

The objective of the dataset upgrade is to obtain a more representative evaluation of the model.

🧩 ESP32 AI Integration

After training and evaluation, the trained model can be converted into a format suitable for embedded deployment.

Example:

ecg_model.h
The model is then integrated into the ESP32 firmware.

The intended embedded workflow is:

ECG Acquisition
      ↓
Filtering
      ↓
R-Peak Detection
      ↓
Heartbeat Segmentation
      ↓
Feature Extraction
      ↓
Random Forest Model
      ↓
Normal / Abnormal
This allows the ESP32 to perform the main processing locally rather than sending raw ECG data to the mobile application for AI processing.

❤️ MAX30102 Integration

The MAX30102 is used as an additional physiological sensing module.

It can provide:

Heart rate
SpO₂ estimation
The sensor communicates with the ESP32 using the I²C interface.

The MAX30102 is intended to complement ECG monitoring rather than replace ECG analysis.

🖥️ OLED Display

An OLED display provides local feedback without requiring a smartphone.

The display can show information such as:

SMART ECG

HR: 72 BPM
SpO2: 98%

STATUS:
NORMAL
For an abnormal classification, the system can provide an alert through the display, LED and buzzer.

📡 IoT Communication
The ESP32 provides Wi-Fi connectivity and communicates with the Flutter application using the MQTT protocol.

System flow:

ESP32
  ↓
Wi-Fi
  ↓
MQTT Broker
  ↓
Flutter Application
The project uses MQTT for lightweight publish/subscribe communication.

Example topic:

smart_ecg/demo_esp32_01/data
The mobile application receives processed monitoring information rather than performing the primary ECG AI processing.

📱 Flutter Mobile Application

A Flutter-based mobile application is used as the monitoring dashboard.

The application can display:

ECG monitoring information
Heart rate
SpO₂
Classification status
Device connection status
Real-time monitoring information
The application communicates with the ESP32 through the MQTT broker.

Technology Stack
Flutter
Dart
MQTT
ESP32
Wi-Fi

🔔 Alert System

The embedded system can provide alerts through:

Red LED
Buzzer
OLED warning message
Mobile dashboard status
Example:

⚠ ABNORMAL

Heart Rate: 55 BPM

Please consult a healthcare professional.
The alert mechanism is intended as a prototype notification feature and should not be interpreted as a medical diagnosis.

🔋 Power System

The portable version is designed around a rechargeable battery system.

Basic power architecture:

Rechargeable Battery
        ↓
      TP4056
   Charging/Protection
        ↓
     Power Switch
        ↓
   Voltage Regulation
        ↓
      ESP32
        ├── AD8232
        ├── MAX30102
        └── OLED
The final PCB design will include appropriate power regulation and decoupling.

🔌 PCB Development

A custom PCB is planned for the final hardware version.

Target PCB specification:

Board Size: 70 × 90 mm
Layers: 2
Material: FR-4
Thickness: ~1.6 mm
Copper: ~1 oz
The PCB will integrate/connect:

ESP32
AD8232
MAX30102
OLED
LEDs
Buzzer
Power management
Connectors
The ECG analog signal routing will be kept short and separated from noisy digital/power traces wherever practical.

🛠️ Software & Tools

Embedded
Arduino IDE / ESP32 development environment
C/C++
ESP32 framework
AI / Data Processing
Python
NumPy
Matplotlib
Scikit-learn
WFDB
Random Forest
Mobile Application
Flutter
Dart
MQTT Client
Hardware Design
EasyEDA / KiCad
PCB prototyping

📁 Project Structure

AI-Enabled-Portable-ECG/
│
├── README.md
│
├── ESP32/
│   └── smart_ecg.ino
│
├── AI_Model/
│   ├── training_code/
│   ├── dataset_processing/
│   └── ecg_model.h
│
├── Flutter_App/
│   └── smart_ecg/
│
├── Hardware/
│   ├── circuit_diagram/
│   ├── wiring/
│   └── PCB/
│
├── Dataset/
│   └── README.md
│
├── Images/
│   ├── prototype.jpg
│   ├── circuit.jpg
│   └── app_dashboard.jpg
│
└── LICENSE

🔮 Future Scope

Future improvements may include:

Larger and more diverse ECG datasets
Multi-class arrhythmia classification
Improved ECG feature extraction
Lightweight deep-learning models for embedded devices
Better motion-artifact rejection
Custom PCB and compact enclosure
Cloud-based historical monitoring
Secure patient-data storage
Additional physiological sensors
Clinical validation under appropriate medical supervision
👨‍💻 Project
Project Name: AI-Enabled Portable ECG Monitoring System for Early Cardiac Risk Detection

Core Technologies:

ESP32
AD8232
MAX30102
OLED
Machine Learning
Random Forest
Python
MQTT
Flutter
Wi-Fi
PCB

⭐ Key Highlights

Portable ECG monitoring
Embedded signal processing
Machine-learning-based classification
ESP32-based edge processing
Heart-rate monitoring
SpO₂ integration
OLED local display
MQTT IoT communication
Flutter mobile dashboard
Rechargeable portable power
Future custom PCB integration
