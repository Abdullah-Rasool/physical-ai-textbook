# Action Generation: From Understanding to Movement

**Estimated Reading Time**: 8 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Explain** how VLA models represent and generate robot actions
2. **Describe** action tokenization and why it enables transformer-based action prediction
3. **Understand** action chunking and its benefits for smooth, coherent motion
4. **Compare** different action representations (joint space, Cartesian, velocity, position)

---

## The Action Generation Problem

Given visual understanding of the scene and grounded language intent, the VLA model must now produce actual robot movements. This final stage—action generation—bridges the gap between neural network outputs and physical motor commands.

### What Does "Action" Mean?

In VLA systems, an "action" typically refers to:

- **Control command**: What signal to send to the robot
- **Time step**: Actions are generated at regular intervals (e.g., 10 Hz)
- **Dimensionality**: Vector matching robot's degrees of freedom

For a 7-DOF robot arm with gripper:
```
Action = [j1, j2, j3, j4, j5, j6, j7, gripper]
         └────────── 8 values per timestep ──────────┘
```

---

## Action Representations

VLA systems use various representations for robot actions:

### Joint Space Actions

Direct joint angles or velocities:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Joint Space Representation                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Robot Arm Configuration                                           │
│                                                                     │
│          [J1: shoulder_pan]                                         │
│                │                                                    │
│          [J2: shoulder_lift]                                        │
│                │                                                    │
│          [J3: elbow]                                                │
│                │                                                    │
│          [J4: wrist_1]                                              │
│                │                                                    │
│          [J5: wrist_2]                                              │
│                │                                                    │
│          [J6: wrist_3]                                              │
│                │                                                    │
│          [gripper]                                                  │
│                                                                     │
│   Action Vector: [0.1, -0.05, 0.2, 0.0, 0.15, -0.1, 0.8]           │
│                  └─── delta joint positions (radians) ───┘         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Advantages**: Direct control, no inverse kinematics needed
**Disadvantages**: Non-intuitive, harder to learn across robot morphologies

### Cartesian Space Actions

End-effector position and orientation:

```
Action = [x, y, z, roll, pitch, yaw, gripper]
         └── 6D pose ──┘  └─ orientation ─┘
```

**Advantages**: Intuitive (move hand to position), transfers better across robots
**Disadvantages**: Requires inverse kinematics to execute

### Velocity vs. Position

Actions can specify:
- **Velocity**: Change rate (move at 0.1 m/s in +x direction)
- **Position**: Target state (move to x=0.5, y=0.3, z=0.2)
- **Delta position**: Relative change (move +0.01 in x)

Most VLA systems use **delta position** or **delta joint angles**—small incremental changes from current state.

---

## Action Tokenization

A key innovation enabling transformer-based VLA is treating actions like language tokens.

### The Insight

Transformers excel at predicting sequences of discrete tokens (words, subwords). If we can convert continuous robot actions into discrete tokens, we can use the same architectures.

### Discretization Process

```
┌─────────────────────────────────────────────────────────────────────┐
│              Action Tokenization                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Continuous Action Space (per dimension):                          │
│   ◀─────────────────────────────────────────────────────────────▶   │
│   -1.0                        0.0                        +1.0       │
│                                                                     │
│   Discretized (256 bins per dimension):                             │
│   [0] [1] [2] ... [127] [128] [129] ... [254] [255]                │
│    │                      │                      │                  │
│   -1.0                   0.0                   +1.0                 │
│                                                                     │
│   7-DOF arm → 7 tokens per timestep                                 │
│   Token vocabulary: 256 action tokens + language tokens             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Diagram Explanation**: Each continuous action dimension is divided into discrete bins. An action value maps to the nearest bin, which becomes a token the transformer can predict.

### Why Discretization Works

Despite losing precision, discretization:
- Enables use of powerful language model architectures
- Works well for many manipulation tasks
- Can use 256+ bins for sufficient resolution
- Allows the model to predict action distributions, not just point estimates

### RT-2's Approach

RT-2 represents actions as text strings:

```
Action output: "1 128 64 200 180 90 255"
               └─ 7 tokens representing joint positions ─┘
```

The language model generates action tokens just like it generates words.

---

## Action Chunking

Rather than predicting one action at a time, modern VLA systems predict **chunks** of actions.

### Single-Step vs. Chunked Prediction

```
┌─────────────────────────────────────────────────────────────────────┐
│              Action Prediction Approaches                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   SINGLE-STEP (reactive)                                            │
│                                                                     │
│   t=0: Observe → Predict action[0] → Execute                        │
│   t=1: Observe → Predict action[1] → Execute                        │
│   t=2: Observe → Predict action[2] → Execute                        │
│   ...                                                               │
│                                                                     │
│   ─────────────────────────────────────────────────────────────     │
│                                                                     │
│   CHUNKED (planned)                                                 │
│                                                                     │
│   t=0: Observe → Predict [action[0], action[1], ..., action[k]]     │
│         └── Execute chunk over k timesteps ──┘                      │
│   t=k: Observe → Predict next chunk                                 │
│   ...                                                               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Benefits of Action Chunking

1. **Temporal consistency**: Predicted trajectory is coherent over time
2. **Reduced compounding error**: Fewer inference-execution cycles
3. **Smoother motion**: Actions form natural motion primitives
4. **Better for contact**: Planned approach-grasp-lift sequences

### Chunk Implementation

```python
# Purpose: Illustrate action chunk prediction
# Note: Pseudocode, not production code

def predict_action_chunk(model, image, language, chunk_size=16):
    """
    Predict a chunk of future actions.

    Args:
        model: VLA model
        image: Current camera observation
        language: Task instruction
        chunk_size: Number of future actions to predict

    Returns:
        List of action vectors [a_0, a_1, ..., a_{k-1}]
    """
    # Encode inputs
    features = model.encode(image, language)

    # Predict chunk_size actions at once
    action_chunk = model.decode_actions(features, num_actions=chunk_size)

    return action_chunk  # Shape: [chunk_size, action_dim]
```

### Execution Strategies

How to use predicted chunks:

- **Execute all**: Run entire chunk, then re-predict
- **Receding horizon**: Execute first few, re-predict with overlap
- **MPC-style**: Re-predict every step, use first action only (falls back to single-step)

---

## From Tokens to Motor Commands

The final step connects VLA outputs to physical robot control:

```
┌─────────────────────────────────────────────────────────────────────┐
│              Action Pipeline                                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   VLA Model Output                                                  │
│        │                                                            │
│        ▼                                                            │
│   ┌─────────────┐                                                   │
│   │ Detokenize  │  Action tokens → continuous values               │
│   └──────┬──────┘                                                   │
│          │                                                          │
│          ▼                                                          │
│   ┌─────────────┐                                                   │
│   │ Denormalize │  [-1, 1] → robot-specific ranges                 │
│   └──────┬──────┘                                                   │
│          │                                                          │
│          ▼                                                          │
│   ┌─────────────┐                                                   │
│   │   Safety    │  Clip to joint limits, velocity bounds           │
│   │   Checks    │                                                   │
│   └──────┬──────┘                                                   │
│          │                                                          │
│          ▼                                                          │
│   ┌─────────────┐                                                   │
│   │ ROS 2 Msg   │  Package as JointTrajectory or similar           │
│   └──────┬──────┘                                                   │
│          │                                                          │
│          ▼                                                          │
│     Motor Controllers                                               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Output Heads and Architectures

VLA models use different architectures for action prediction:

### Autoregressive Prediction

Predict action tokens one at a time:

```
Input:  [vision] [language] [action_0]
Output:                     [action_1]

Input:  [vision] [language] [action_0] [action_1]
Output:                                [action_2]
...
```

Used by: RT-2, OpenVLA

### Diffusion-Based Prediction

Predict entire trajectory through iterative denoising:

```
Start:  Random noise trajectory
Step 1: Denoise → Less noisy trajectory
Step 2: Denoise → Cleaner trajectory
...
Final:  Clean action trajectory
```

Used by: Diffusion Policy

### Direct Regression

Predict action vector directly from features:

```
Features → MLP → Action vector
```

Simpler but may not capture multimodal action distributions.

---

## Handling Continuous Control

VLA inference runs at discrete timesteps, but robots need smooth motion:

### Interpolation

Smooth between predicted waypoints:
- Linear interpolation
- Spline fitting
- Trajectory optimization

### Control Frequency Mismatch

VLA models often run slower (10-20 Hz) than robot controllers (100-1000 Hz):
- Use predicted trajectory as reference
- Low-level controller tracks reference
- Re-plan at VLA inference rate

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: Action generation completes the perception-action loop, translating understanding to physical movement
- **Module 2 (ROS 2)**: Generated actions become ROS 2 JointTrajectory messages or similar, published to controller topics/actions
- **Module 4 (Isaac)**: Isaac Sim provides the simulation environment where action generation is tested before real deployment

---

## Key Takeaways

- VLA models generate **discrete or continuous action outputs** representing robot movements
- **Action tokenization** converts continuous actions to discrete tokens, enabling transformer architectures
- **Action chunking** predicts sequences of actions for smoother, more coherent motion
- Actions go through **detokenization, denormalization, and safety checks** before reaching motors
- Different architectures (**autoregressive, diffusion, direct regression**) offer different tradeoffs

---

**Next**: [VLA Architectures →](./05-architectures.md)
