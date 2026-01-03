# AI Inference Pipelines: From Training to Deployment

---

## Introduction

Having AI models is only half the battle—getting them to run fast enough on robot hardware is the other half. AI inference pipelines transform trained neural networks into optimized, deployable systems that meet the strict latency requirements of real-time robotics.

**In This Section**:
- Understand the AI model lifecycle from training to robot deployment
- Learn about TensorRT optimization and why it's essential
- Explore multi-model inference orchestration with Triton
- See how perception, reasoning, and control connect in a unified pipeline

---

## The AI Model Lifecycle for Robots

### From Lab to Robot

AI models in robotics follow a distinct lifecycle:

```
Training (Cloud/DGX) → Optimization (TensorRT) → Deployment (Jetson)
      |                       |                        |
      v                       v                        v
   PyTorch/TF             ONNX/TRT              Isaac ROS
   FP32 weights          INT8/FP16            Real-time inference
   Hours/days            Minutes              Milliseconds
```

Each stage has different requirements:

| Stage | Hardware | Precision | Speed Priority |
|-------|----------|-----------|----------------|
| Training | DGX/Cloud GPUs | FP32 | Accuracy first |
| Optimization | Workstation | FP32 → FP16/INT8 | Balance accuracy & speed |
| Deployment | Jetson | FP16/INT8 | Speed first |

### Why Optimization Matters

A model trained in PyTorch cannot run efficiently on Jetson without optimization:

**Unoptimized Model**:
- Large memory footprint (FP32 weights)
- Inefficient GPU utilization
- Framework overhead (Python, eager execution)
- Result: 100-500ms inference time

**Optimized Model (TensorRT)**:
- Reduced precision (FP16/INT8)
- Fused operations (fewer GPU kernel launches)
- Optimized memory layout
- Result: 10-50ms inference time

**The difference**: 10x faster inference, enabling real-time robot perception.

---

## TensorRT: The Optimization Engine

### What is TensorRT?

**TensorRT** is NVIDIA's SDK for optimizing deep learning inference. It transforms models into highly efficient engines for deployment:

**Key Optimizations**:
1. **Precision Reduction**: FP32 → FP16 → INT8 (smaller, faster, minimal accuracy loss)
2. **Layer Fusion**: Combine multiple operations into single GPU kernels
3. **Kernel Auto-Tuning**: Select best GPU kernels for target hardware
4. **Memory Optimization**: Reduce memory allocations and copies
5. **Dynamic Tensor Memory**: Reuse memory between layers

### The Optimization Process

```python
# PURPOSE: Illustrate TensorRT optimization workflow for robot AI models
# NOTE: This is conceptual code, not production-ready

import tensorrt as trt

class ModelOptimizer:
    """Optimize AI models for robot deployment"""

    def optimize_for_jetson(self, onnx_model_path, target_device="orin"):
        # Step 1: Create TensorRT builder
        logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(logger)

        # Step 2: Parse ONNX model
        network = builder.create_network()
        parser = trt.OnnxParser(network, logger)
        with open(onnx_model_path, 'rb') as f:
            parser.parse(f.read())

        # Step 3: Configure optimization settings
        config = builder.create_builder_config()

        # Enable FP16 for speed (Jetson has fast FP16)
        config.set_flag(trt.BuilderFlag.FP16)

        # For even more speed, enable INT8 (requires calibration)
        # config.set_flag(trt.BuilderFlag.INT8)
        # config.int8_calibrator = MyCalibrator(calibration_data)

        # Set memory limit for Jetson (8GB example)
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)

        # Step 4: Build optimized engine (this takes a few minutes)
        # Engine is hardware-specific - must rebuild for each Jetson model
        engine = builder.build_serialized_network(network, config)

        # Step 5: Save engine for deployment
        with open("model_optimized.engine", "wb") as f:
            f.write(engine)

        return engine

    def measure_speedup(self, original_ms, optimized_ms):
        """Typical speedups from TensorRT optimization"""
        # YOLOv5 example:
        #   Original (PyTorch FP32): 150ms
        #   Optimized (TensorRT FP16): 25ms
        #   Speedup: 6x

        # ResNet50 example:
        #   Original: 45ms
        #   Optimized: 5ms
        #   Speedup: 9x

        speedup = original_ms / optimized_ms
        return speedup
```

**Explanation**: This code demonstrates the TensorRT optimization workflow. The key insight is that optimization is **hardware-specific**—an engine built for Jetson Orin won't work on Jetson Xavier. This is why Isaac ROS provides pre-optimized models and tools for building your own engines.

**Key Concepts Demonstrated**:
- **ONNX as interchange format**: Train in any framework, optimize with TensorRT
- **Precision selection**: FP16 for speed, INT8 for maximum performance
- **Hardware-specific engines**: Optimization targets a specific GPU model

---

## Multi-Model Orchestration

### The Challenge: Multiple AI Models

Modern robots run multiple AI models simultaneously:

- **Object Detection**: What objects are visible?
- **Segmentation**: What are the boundaries?
- **Depth Estimation**: How far away is everything?
- **Pose Estimation**: What is the 6-DoF pose of each object?
- **Human Pose**: Where are people and what are they doing?

Running all these models efficiently requires orchestration.

### Triton Inference Server

**Triton** is NVIDIA's inference serving solution, adapted for robotics:

**Capabilities**:
- **Model Ensemble**: Chain multiple models together
- **Dynamic Batching**: Combine requests for efficiency
- **Model Versioning**: A/B testing, rollbacks
- **Multi-Framework**: TensorRT, PyTorch, TensorFlow, ONNX
- **Metrics and Monitoring**: Performance visibility

### Diagram: Multi-Model Inference Pipeline

### Robot Perception Multi-Model Pipeline

**Architecture Overview**:
This diagram shows how multiple AI models are orchestrated in a robot perception pipeline, from raw sensor data to actionable information.

**Components**:

**Input Stage**:
- **RGB Camera**: 1920x1080 @ 30fps
- **Depth Camera**: 640x480 @ 30fps
- **Stereo Cameras**: For visual odometry

**Preprocessing (GPU)**:
- **Image Resize**: Scale for model input sizes
- **Normalization**: Convert to model-expected format
- **Debayer/Rectify**: Raw sensor processing

**Model Ensemble (Triton/Isaac ROS)**:
- **Model 1: Object Detector** (YOLOv8)
  - Input: 640x640 RGB
  - Output: Bounding boxes, class labels, confidence
  - Latency: 15ms
- **Model 2: Instance Segmentation** (Mask R-CNN)
  - Input: 640x640 RGB
  - Output: Per-object masks
  - Latency: 25ms
- **Model 3: Depth Completion** (DNN)
  - Input: Sparse depth + RGB
  - Output: Dense depth map
  - Latency: 12ms
- **Model 4: Pose Estimator** (FoundationPose)
  - Input: RGB + depth + detection
  - Output: 6-DoF object poses
  - Latency: 20ms

**Post-Processing**:
- **NMS (Non-Max Suppression)**: Filter overlapping detections
- **Temporal Filtering**: Smooth across frames
- **Coordinate Transforms**: Camera → robot → world frames

**Output Stage**:
- **Detection2DArray**: ROS 2 message with 2D detections
- **PointCloud2**: Segmented 3D point cloud
- **PoseArray**: 6-DoF poses for manipulation
- **TF Transforms**: Object positions in world frame

**Execution Flow**:
1. Camera captures frame → GPU memory (NITROS)
2. Preprocessing runs on GPU → normalized tensors
3. Object Detector runs → bounding boxes
4. Segmentation runs (parallel to detector possible) → masks
5. Pose Estimator runs (needs detector output) → 6-DoF poses
6. Post-processing combines results → ROS 2 messages published

**Pipeline Parallelism**:
- Models without dependencies run in parallel
- GPU scheduler interleaves execution for efficiency
- Target: Full pipeline under 50ms for 20+ fps perception

**Key Interfaces**:
- **NITROS**: GPU-to-GPU data transfer between stages
- **ROS 2 Topics**: Output to downstream planning/control nodes
- **Triton gRPC**: Model management and inference requests

**Takeaway**: Modern robot perception requires orchestrating multiple specialized models. The pipeline must balance accuracy (more/larger models) against latency (faster response). GPU acceleration and careful orchestration enable both.

---

## Perception Latency Budget

### Understanding the Latency Budget

Every robot has a **latency budget**—the maximum time from sensor input to action:

| Robot Type | Latency Budget | Example Task |
|------------|---------------|--------------|
| Slow mobile robot | 500ms | Warehouse navigation |
| Fast mobile robot | 100ms | Person following |
| Manipulation | 50ms | Object grasping |
| Dynamic (humanoid) | 20ms | Balance recovery |

Perception must fit within this budget alongside planning and control.

### Budget Allocation Example

**Task**: Pick up moving object with humanoid arm

**Total budget**: 100ms (10Hz control loop)

| Stage | Allocation | Typical Latency |
|-------|------------|-----------------|
| Camera capture | 5ms | Fixed by sensor |
| Object detection | 20ms | TensorRT-optimized |
| Pose estimation | 25ms | TensorRT-optimized |
| Motion planning | 30ms | GPU-accelerated |
| Control + comm | 20ms | Safety margins |
| **Total** | 100ms | Meets budget |

**Key Insight**: Optimization at every stage enables responsive robots.

---

## Where Learning Fits

### AI Model Types in Robots

Different learning approaches serve different purposes:

**Supervised Learning (Most Common for Perception)**:
- Object detection, segmentation, classification
- Trained on labeled datasets
- Deployed as frozen models in Isaac ROS

**Reinforcement Learning (Isaac Lab Focus)**:
- Locomotion policies (walking, running)
- Manipulation skills (grasping, insertion)
- Trained in simulation (Isaac Sim), deployed on robot
- Billions of samples needed—simulation enables this

**Imitation Learning (Emerging)**:
- Learn from human demonstrations
- Teleoperation → policy learning
- Vision-language-action (VLA) models (covered in Module 5)

### The Training-Deployment Gap

Models trained in the lab must work on real robots:

**Training Environment**:
- Clean data, controlled conditions
- High-end GPUs (A100, H100)
- No latency constraints
- FP32 precision for accuracy

**Deployment Environment**:
- Noisy sensors, uncontrolled conditions
- Edge hardware (Jetson)
- Strict latency requirements
- FP16/INT8 for speed

**Bridging the Gap**:
- Domain randomization in training (Isaac Sim)
- Robust architectures that tolerate noise
- Careful quantization to preserve accuracy
- Real-world fine-tuning when needed

---

## End-to-End Example: Object Manipulation

### Putting It Together

```python
# PURPOSE: Illustrate end-to-end AI pipeline for robot manipulation
# NOTE: This is conceptual code, not production-ready

class ManipulationPipeline:
    """Complete perception → planning → action pipeline"""

    def __init__(self):
        # Perception models (TensorRT-optimized, via Isaac ROS)
        self.detector = IsaacROSObjectDetector()    # ~20ms
        self.pose_estimator = IsaacROSPoseEstimator()  # ~25ms

        # Planning (GPU-accelerated)
        self.motion_planner = CuMotionPlanner()    # ~30ms

        # Control (runs at 1kHz on CPU)
        self.arm_controller = JointController()

    def pick_object(self, target_class="mug"):
        """Full pipeline: see object → plan path → pick it up"""

        # STEP 1: PERCEPTION (GPU, ~45ms total)
        # Camera image → Isaac ROS detector → bounding boxes
        image = self.camera.capture()
        detections = self.detector.detect(image)

        # Find target object
        target = self.find_by_class(detections, target_class)
        if not target:
            return False

        # Get precise 6-DoF pose
        depth = self.depth_camera.capture()
        pose = self.pose_estimator.estimate(image, depth, target)

        # STEP 2: PLANNING (GPU, ~30ms)
        # Current pose → target pose → collision-free trajectory
        current_joints = self.arm.get_joint_positions()
        grasp_pose = self.compute_grasp_pose(pose)

        trajectory = self.motion_planner.plan(
            start=current_joints,
            goal=grasp_pose,
            obstacles=detections  # Avoid other objects
        )

        # STEP 3: EXECUTION (CPU, 1kHz loop)
        # Follow trajectory with real-time control
        for waypoint in trajectory:
            self.arm_controller.move_to(waypoint)

        # Grasp
        self.gripper.close()

        return True
```

**Explanation**: This code shows the complete pipeline from perception to action. GPU-accelerated perception (~45ms) and planning (~30ms) run fast enough to support responsive manipulation. The control loop runs on CPU at high frequency (1kHz) for safety.

**Key Concepts Demonstrated**:
- **Layered Latencies**: Each stage has its latency budget
- **GPU for Parallel Work**: Perception and planning leverage GPU
- **CPU for Control**: Real-time control runs on CPU for determinism
- **Isaac ROS Integration**: GEMs handle perception complexity

---

## Summary

**Key Takeaways**:
- **AI Model Lifecycle**: Train → Optimize (TensorRT) → Deploy (Jetson)
- **TensorRT Optimization**: 5-10x speedup through precision reduction and kernel fusion
- **Multi-Model Orchestration**: Triton/Isaac ROS manage multiple models efficiently
- **Latency Budgets**: Every robot has timing constraints that shape the pipeline
- **Learning in Robotics**: Supervised for perception, RL for policies, emerging VLA models
- **End-to-End Integration**: Perception, planning, and control must work as a unified system

**Connection to Next Section**: We've seen how AI models are optimized and deployed. But how does Isaac actually connect to ROS 2? Next, we'll explore the **ROS 2 Integration** in detail.

---

**[Continue to Section 5: ROS 2 Integration →](./05-ros2-integration.md)**
