# EKF_DOCKING

A simple C++ Extended Kalman Filter (EKF) project that simulates noisy robot pose data, filters it, rejects outliers, and visualizes the result.

The robot state contains only:

```text
x, y, theta
```

where:

* `x` = X position
* `y` = Y position
* `theta` = orientation

## How It Works

The project has three main parts:

### 1. Simulator

The simulator creates a robot pose and generates noisy measurements.

It can simulate:

* Normal Gaussian noise
* Outliers
* Sensor dropouts

The simulator publishes the data using MQTT.

### 2. EKF Filter

The EKF receives the noisy `x, y, theta` measurements.

For every measurement it:

1. Predicts the state using the previous state.
2. Calculates the difference between the measurement and prediction.
3. Checks the measurement using Mahalanobis distance.
4. Rejects the measurement if it is an outlier.
5. Updates the state if the measurement is valid.
6. Publishes the filtered result.

The current EKF uses:

```text
State:
[x, y, theta]

Prediction model:
F = Identity Matrix

Measurement model:
H = Identity Matrix
```

There is currently no velocity or acceleration in the state.

### 3. Visualizer

The Qt visualizer displays:

* Ground truth pose
* Noisy measurement
* EKF filtered pose
* Residual
* Covariance
* Accepted/rejected measurement status

## MQTT Topics

| Topic              | Purpose              |
| ------------------ | -------------------- |
| `ekf/ground_truth` | True robot pose      |
| `ekf/measurement`  | Noisy measurement    |
| `ekf/config`       | Change Q/R scale     |
| `ekf/filtered`     | EKF filtered pose    |
| `ekf/residual`     | Measurement residual |
| `ekf/covariance`   | EKF covariance       |
| `ekf/status`       | Accepted/Rejected    |

## Outlier Rejection

The EKF uses Mahalanobis distance to determine whether a measurement is valid.

```text
Mahalanobis distance > 11.34
        → Reject

Mahalanobis distance <= 11.34
        → Accept
```

This prevents large measurement errors from significantly affecting the filtered state.

## Q and R

The filter has two important noise parameters:

### Q - Process Noise

Controls how much the filter trusts its prediction.

```text
Q ↑ → Trust prediction less
Q ↓ → Trust prediction more
```

### R - Measurement Noise

Controls how much the filter trusts the sensor.

```text
R ↑ → Trust measurement less
R ↓ → Trust measurement more
```

They can be changed at runtime through:

```text
ekf/config
```

Example:

```text
2.0,1.0
```

means:

```text
Q scale = 2.0
R scale = 1.0
```

## Project Structure

```text
EKF_DOCKING/
│
├── ekf_filter/       # EKF implementation
├── simulator/        # Noisy pose simulator
├── visualizer_ui/    # Qt visualization
├── docker-compose.yml
└── README.md
```

## Requirements

* Docker
* Docker Compose

The project uses C++17, Eigen, MQTT and Qt6.

## Running the Project

Clone the repository:

```bash
git clone https://github.com/Gowrish666/EKF_DOCKING.git
cd EKF_DOCKING
```

Build and run:

```bash
docker compose up --build
```

To stop:

```bash
docker compose down
```

## Purpose

This project is mainly intended to demonstrate how an EKF handles noisy `x, y, theta` measurements, including measurement noise, outliers, and sensor dropouts.
