# Sensor Modeling: Simulating Robot Perception

---

## Introduction

Robots perceive the world through sensors: cameras capture images, LiDAR measures distances, IMUs track orientation. For simulation to be useful, these virtual sensors must produce data that resembles their physical counterparts. Sensor modeling is the art of making simulated sensors behave realistically.

**In This Section**:
- Understand the challenges of modeling physical sensors in simulation
- Learn how cameras, LiDAR, and IMUs are simulated
- Explore noise models that make simulated data more realistic
- See why sensor fidelity is critical for sim-to-real transfer

---

## The Sensor Modeling Challenge

### Ideal vs Real Sensors

Simulators naturally produce "perfect" data:
- **Perfect camera**: No noise, blur, or distortion
- **Perfect LiDAR**: Exact distances with no error
- **Perfect IMU**: True acceleration and rotation

But real sensors are imperfect:
- **Real camera**: Noise, motion blur, lens distortion, exposure issues
- **Real LiDAR**: Range noise, beam divergence, multipath reflections
- **Real IMU**: Bias drift, noise, temperature sensitivity

If AI models train on perfect simulated data, they may fail on imperfect real data.

### Sensor Simulation Pipeline

**Architecture Overview**:
This diagram shows the pipeline for simulating a sensor, from ideal measurement to realistic output.

**Components**:

**Ideal Measurement**:
- **Ground Truth**: Perfect world state from simulation
- **Ray Casting/Rendering**: Compute what sensor would "see"
- **Raw Signal**: Ideal measurement (no errors)

**Noise Injection**:
- **Sensor Noise Model**: Gaussian, salt-and-pepper, structured noise
- **Systematic Errors**: Bias, scale factor, misalignment
- **Environmental Effects**: Temperature, vibration, interference

**Post-Processing**:
- **Quantization**: Discrete output values (8-bit images, discrete ranges)
- **Sampling/Timing**: Realistic frame rates and timestamps
- **Message Formatting**: Convert to ROS 2 message types

**Output**:
- **Simulated Sensor Data**: Looks like real sensor output
- **Published to ROS 2**: `/camera/image`, `/scan`, `/imu/data`

**Data Flow**:
1. Query world state → compute ideal measurement
2. Apply sensor-specific noise model
3. Add systematic errors (bias, drift)
4. Quantize and format output
5. Publish at realistic rate with proper timestamps

**Takeaway**: Good sensor simulation isn't about perfect measurements—it's about realistic imperfections.

---

## Camera Simulation

### What Cameras Measure

Cameras capture light reflected from the environment:
- **Color (RGB)**: Light intensity per color channel
- **Depth**: Distance to surfaces (depth cameras)
- **Infrared**: Thermal signatures (IR cameras)

### Camera Modeling Components

| Component | Ideal Simulation | Realistic Simulation |
|-----------|-----------------|---------------------|
| Projection | Perfect pinhole | Lens distortion, field of view |
| Color | Perfect rendering | White balance, color response |
| Exposure | Perfect brightness | Over/under exposure, HDR limits |
| Noise | None | Gaussian noise, shot noise |
| Motion | Sharp frames | Motion blur during movement |
| Focus | Everything sharp | Depth of field, out-of-focus blur |

### Camera Noise Model

```python
# PURPOSE: Illustrate camera noise model for realistic simulation
# NOTE: This is conceptual code, not production-ready

import numpy as np

class CameraNoisModel:
    """Add realistic noise to simulated camera images"""

    def __init__(self, config):
        # Noise parameters (calibrated from real camera)
        self.gaussian_sigma = config.gaussian_sigma  # e.g., 5.0
        self.salt_pepper_prob = config.salt_pepper_prob  # e.g., 0.001
        self.exposure_variation = config.exposure_variation  # e.g., 0.1

    def apply_noise(self, clean_image):
        """Transform ideal rendered image to realistic noisy image"""
        image = clean_image.copy().astype(float)

        # 1. Gaussian noise (sensor read noise)
        noise = np.random.normal(0, self.gaussian_sigma, image.shape)
        image += noise

        # 2. Salt and pepper noise (dead/hot pixels)
        salt_mask = np.random.random(image.shape[:2]) < self.salt_pepper_prob / 2
        pepper_mask = np.random.random(image.shape[:2]) < self.salt_pepper_prob / 2
        image[salt_mask] = 255
        image[pepper_mask] = 0

        # 3. Exposure variation (global brightness shift)
        exposure_factor = 1.0 + np.random.uniform(
            -self.exposure_variation,
            self.exposure_variation
        )
        image *= exposure_factor

        # 4. Clip to valid range and convert back
        image = np.clip(image, 0, 255).astype(np.uint8)

        return image

    def apply_lens_distortion(self, image, camera_matrix, dist_coeffs):
        """Apply radial and tangential lens distortion"""
        # Real cameras have barrel/pincushion distortion
        # This is typically handled by OpenCV undistort in real systems
        # We simulate what the raw image looks like BEFORE undistortion
        return cv2.distort(image, camera_matrix, dist_coeffs)
```

**Explanation**: This code shows how noise is added to clean rendered images. Real cameras have Gaussian read noise, occasional dead pixels, and exposure variations. By applying these effects, simulated images better match what real cameras produce.

**Key Concepts**:
- **Gaussian noise**: Random variations in pixel values
- **Salt and pepper**: Stuck pixels (dead or hot)
- **Lens distortion**: Radial and tangential distortion from optics

---

## LiDAR Simulation

### What LiDAR Measures

LiDAR (Light Detection and Ranging) measures distances using laser pulses:
- **Range**: Distance to reflection point
- **Intensity**: Strength of return signal
- **Point Cloud**: 3D representation of environment

### LiDAR Modeling Components

| Component | Ideal Simulation | Realistic Simulation |
|-----------|-----------------|---------------------|
| Range | Exact distance | Range noise, quantization |
| Beam | Infinitely thin | Beam divergence, footprint |
| Returns | Single return | Multiple returns (foliage, glass) |
| Materials | Perfect reflection | Material-dependent reflectivity |
| Environment | Clean returns | Rain, fog, dust interference |

### LiDAR Noise Model

```python
# PURPOSE: Illustrate LiDAR noise model for realistic point clouds
# NOTE: This is conceptual code, not production-ready

import numpy as np

class LiDARNoiseModel:
    """Add realistic noise to simulated LiDAR data"""

    def __init__(self, config):
        # Noise parameters (from sensor datasheet)
        self.range_noise_std = config.range_noise_std  # e.g., 0.02 meters
        self.min_range = config.min_range  # e.g., 0.1 meters
        self.max_range = config.max_range  # e.g., 100 meters
        self.dropout_prob = config.dropout_prob  # e.g., 0.01

    def apply_noise(self, clean_points, intensities):
        """Transform ideal LiDAR scan to realistic noisy scan"""
        noisy_points = clean_points.copy()
        ranges = np.linalg.norm(noisy_points, axis=1)

        # 1. Range noise (increases with distance)
        range_noise = np.random.normal(
            0,
            self.range_noise_std * (1 + ranges / self.max_range),
            len(ranges)
        )

        # Apply noise along ray direction
        directions = noisy_points / ranges[:, np.newaxis]
        noisy_points += directions * range_noise[:, np.newaxis]

        # 2. Invalid returns (dropouts due to material, angle, distance)
        dropout_mask = np.random.random(len(ranges)) < self.dropout_prob
        # Points at extreme angles more likely to drop
        angle_factor = np.abs(noisy_points[:, 2]) / ranges  # vertical angle
        dropout_mask |= np.random.random(len(ranges)) < angle_factor * 0.1

        # 3. Range limits
        out_of_range = (ranges < self.min_range) | (ranges > self.max_range)
        dropout_mask |= out_of_range

        # Mark invalid points (NaN or special value)
        noisy_points[dropout_mask] = np.nan

        # 4. Intensity noise
        noisy_intensities = intensities + np.random.normal(0, 5, len(intensities))
        noisy_intensities = np.clip(noisy_intensities, 0, 255)

        return noisy_points, noisy_intensities
```

**Explanation**: This code adds realistic noise to simulated LiDAR point clouds. Real LiDARs have range measurement noise that increases with distance, dropout returns from certain materials or angles, and intensity variations. These effects are critical for training LiDAR-based perception.

---

## IMU Simulation

### What IMUs Measure

Inertial Measurement Units measure motion:
- **Accelerometer**: Linear acceleration (m/s²)
- **Gyroscope**: Angular velocity (rad/s)
- **Magnetometer**: Magnetic field direction (optional)

### IMU Noise Characteristics

IMUs have complex noise properties:

| Noise Type | Description | Effect |
|------------|-------------|--------|
| **White noise** | Random fluctuations | Immediate measurement error |
| **Bias** | Constant offset | Drift over time |
| **Bias instability** | Slowly varying offset | Long-term drift |
| **Scale factor error** | Gain error | Proportional measurement error |
| **Cross-axis sensitivity** | Axis coupling | Errors when moving on multiple axes |

### IMU Noise Model

```python
# PURPOSE: Illustrate IMU noise model with bias and drift
# NOTE: This is conceptual code, not production-ready

import numpy as np

class IMUNoiseModel:
    """Add realistic noise to simulated IMU data"""

    def __init__(self, config):
        # Accelerometer parameters
        self.accel_noise_density = config.accel_noise_density  # m/s²/√Hz
        self.accel_bias = np.zeros(3)  # Initial bias
        self.accel_bias_instability = config.accel_bias_instability  # m/s²

        # Gyroscope parameters
        self.gyro_noise_density = config.gyro_noise_density  # rad/s/√Hz
        self.gyro_bias = np.zeros(3)  # Initial bias
        self.gyro_bias_instability = config.gyro_bias_instability  # rad/s

        self.dt = 1.0 / config.sample_rate  # e.g., 200 Hz

    def apply_noise(self, true_accel, true_gyro):
        """Transform ideal IMU readings to realistic noisy readings"""
        # Update bias (random walk)
        self.accel_bias += np.random.normal(
            0, self.accel_bias_instability * np.sqrt(self.dt), 3
        )
        self.gyro_bias += np.random.normal(
            0, self.gyro_bias_instability * np.sqrt(self.dt), 3
        )

        # Accelerometer output
        accel_noise = np.random.normal(
            0, self.accel_noise_density / np.sqrt(self.dt), 3
        )
        noisy_accel = true_accel + self.accel_bias + accel_noise

        # Gyroscope output
        gyro_noise = np.random.normal(
            0, self.gyro_noise_density / np.sqrt(self.dt), 3
        )
        noisy_gyro = true_gyro + self.gyro_bias + gyro_noise

        return noisy_accel, noisy_gyro

    def reset(self):
        """Reset bias (e.g., for new episode)"""
        # Initialize with random bias (within spec)
        self.accel_bias = np.random.normal(0, 0.01, 3)  # ~0.01 m/s² typical
        self.gyro_bias = np.random.normal(0, 0.001, 3)  # ~0.001 rad/s typical
```

**Explanation**: IMU simulation is challenging because of bias drift—a slowly varying offset that causes position/orientation estimates to diverge over time. This model includes white noise (immediate errors) and bias instability (long-term drift), both essential for realistic state estimation testing.

---

## Why Sensor Fidelity Matters

### The Training Data Gap

Consider training an object detector:
- **Clean simulated images**: Model learns to detect objects easily
- **Deployed on real camera**: Noise causes missed detections

The model never learned to handle noise because simulation was "too perfect."

### Fidelity Requirements by Application

| Application | Fidelity Requirement | Key Parameters |
|-------------|---------------------|----------------|
| Control algorithm testing | Medium | Physics, timing |
| Perception model training | High | Visual noise, sensor characteristics |
| State estimation | Very High | IMU bias, noise spectra |
| Safety validation | Very High | Edge cases, failure modes |

### Calibrating Noise Models

Realistic noise models require calibration:
1. **Collect real sensor data** in controlled conditions
2. **Analyze noise characteristics** (power spectrum, bias drift)
3. **Fit noise model parameters** to match real data
4. **Validate** that simulated data statistics match real data

This is ongoing work—no simulation perfectly matches reality.

---

## Summary

**Key Takeaways**:
- **Ideal simulation is insufficient**: Perfect sensors don't exist in reality
- **Noise models** add realistic imperfections to simulated sensor data
- **Camera noise**: Gaussian noise, lens distortion, exposure variation
- **LiDAR noise**: Range errors, dropouts, intensity variation
- **IMU noise**: White noise plus slowly drifting bias (critical for state estimation)
- **Calibration**: Match noise model parameters to real sensor characteristics
- **Fidelity matters**: Models trained on unrealistic data fail in the real world

**Connection to Next Section**: Even with good sensor models, simulation doesn't perfectly match reality. We'll explore the **sim-to-real gap** and strategies to bridge it.

---

**[Continue to Section 5: Sim-to-Real Transfer →](./05-sim-to-real.md)**
