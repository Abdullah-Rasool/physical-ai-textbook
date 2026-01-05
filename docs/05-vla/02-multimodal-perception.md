# Multimodal Perception: Vision Encoders and Cross-Modal Fusion

**Estimated Reading Time**: 8 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Explain** how vision encoders convert raw images into useful representations
2. **Describe** the role of transformer architectures in processing visual information
3. **Understand** cross-modal fusion—how visual and language information are combined
4. **Identify** different strategies for integrating multimodal inputs in VLA systems

---

## The Perception Challenge

For a VLA model to act appropriately, it must first *see* and *understand* the world. This perception challenge is non-trivial:

- Raw camera images are high-dimensional (millions of pixels)
- Relevant information (objects, relationships, affordances) is implicit
- The same scene can be described in many ways depending on the task
- Visual understanding must connect to language concepts

VLA models solve this through **vision encoders** and **cross-modal fusion**.

---

## Vision Encoders

A **vision encoder** is a neural network that transforms raw images into compact, meaningful representations called **embeddings** or **features**.

### From Pixels to Features

```
┌─────────────────────────────────────────────────────────────────────┐
│              Vision Encoder Pipeline                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Raw Image                Features                                 │
│   ┌─────────┐             ┌─────────┐                              │
│   │░░░░░░░░░│             │ Vector  │                              │
│   │░░░░░░░░░│   Encoder   │ [0.2,   │   "Table with               │
│   │░░███░░░░│ ──────────▶ │  0.8,   │    red cup,                  │
│   │░░░░░░░░░│             │  0.1,   │    near edge"                │
│   │░░░░░░░░░│             │  ...]   │                              │
│   └─────────┘             └─────────┘                              │
│                                                                     │
│   224x224x3 pixels        768-dimensional vector                    │
│   (~150K values)          (semantic meaning)                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Diagram Explanation**: The vision encoder compresses a high-dimensional image into a compact feature vector that captures semantic meaning. This compressed representation enables efficient reasoning.

### Common Vision Encoder Architectures

#### Convolutional Neural Networks (CNNs)

Traditional approach using learned filters:
- **ResNet**: Deep residual networks with skip connections
- **EfficientNet**: Optimized for compute efficiency
- Local receptive fields capture spatial patterns

#### Vision Transformers (ViT)

Modern approach treating images as sequences:
- Image split into patches (e.g., 16x16 pixel regions)
- Patches treated as tokens, processed by transformer
- Global attention enables long-range reasoning

```
┌─────────────────────────────────────────────────────────────────────┐
│              Vision Transformer (ViT) Architecture                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Input Image          Patch Embedding       Transformer            │
│   ┌───┬───┬───┐       ┌───┬───┬───┐        ┌─────────────┐         │
│   │ 1 │ 2 │ 3 │       │ E1│ E2│ E3│        │ Self-       │         │
│   ├───┼───┼───┤  ──▶  ├───┼───┼───┤  ──▶   │ Attention   │         │
│   │ 4 │ 5 │ 6 │       │ E4│ E5│ E6│        │ Layers      │         │
│   ├───┼───┼───┤       ├───┼───┼───┤        └─────────────┘         │
│   │ 7 │ 8 │ 9 │       │ E7│ E8│ E9│              │                  │
│   └───┴───┴───┘       └───┴───┴───┘              ▼                  │
│                                              Feature                │
│   9 patches           9 embeddings           Vectors                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Diagram Explanation**: ViT divides the image into patches, embeds each patch as a vector, then processes all patches through transformer layers that enable each patch to attend to all others.

### Pre-trained Vision Encoders

VLA models typically use **pre-trained** vision encoders:
- **CLIP vision encoder**: Trained on image-text pairs (400M examples)
- **DINOv2**: Self-supervised training on images only
- **SigLIP**: Sigmoid-based contrastive learning

Pre-training provides rich visual understanding before any robot data is seen.

---

## Cross-Modal Fusion

Once visual and language inputs are encoded, they must be **fused** to enable reasoning that spans both modalities.

### Why Fusion is Challenging

Visual features represent:
- Spatial relationships (positions, sizes, distances)
- Object properties (colors, shapes, textures)
- Scene context (indoor/outdoor, clutter level)

Language features represent:
- Semantic concepts (verbs, nouns, relations)
- Intent (what the human wants)
- Referential expressions ("the red one", "that thing")

Fusion must align these different representational spaces.

### Fusion Strategies

#### Early Fusion

Combine inputs at the start, process together:

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Early Fusion                                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Image Tokens:    [I1] [I2] [I3] ... [In]                         │
│   Language Tokens: [L1] [L2] [L3] ... [Lm]                         │
│                          │                                          │
│                          ▼                                          │
│   Combined:        [I1] [I2] ... [In] [L1] [L2] ... [Lm]           │
│                          │                                          │
│                          ▼                                          │
│                   Unified Transformer                               │
│                          │                                          │
│                          ▼                                          │
│                   Output Features                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

- **Advantage**: Maximum interaction between modalities
- **Disadvantage**: High computational cost, harder to train

#### Late Fusion

Process separately, combine at output:

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Late Fusion                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Image ──▶ Vision Encoder ──▶ Visual Features ──┐                 │
│                                                   │                 │
│                                                   ├──▶ Combined     │
│                                                   │     Output      │
│   Language ──▶ Language Encoder ──▶ Lang Features─┘                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

- **Advantage**: Computationally efficient, modular
- **Disadvantage**: Limited cross-modal reasoning

#### Cross-Attention Fusion

Allow modalities to attend to each other:

```
┌─────────────────────────────────────────────────────────────────────┐
│                   Cross-Attention Fusion                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Visual Features                    Language Features              │
│        │                                    │                       │
│        ▼                                    ▼                       │
│   ┌─────────┐                          ┌─────────┐                  │
│   │  Self-  │                          │  Self-  │                  │
│   │Attention│                          │Attention│                  │
│   └────┬────┘                          └────┬────┘                  │
│        │                                    │                       │
│        └──────────┐          ┌──────────────┘                       │
│                   │          │                                      │
│                   ▼          ▼                                      │
│              ┌───────────────────┐                                  │
│              │  Cross-Attention  │                                  │
│              │  (V attends to L) │                                  │
│              │  (L attends to V) │                                  │
│              └─────────┬─────────┘                                  │
│                        │                                            │
│                        ▼                                            │
│                  Fused Features                                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

- **Advantage**: Flexible, allows selective attention
- **Disadvantage**: More complex architecture design

---

## Multimodal Representations in VLA

Different VLA architectures make different fusion choices:

### RT-2 Approach

RT-2 (Google) uses a pre-trained Vision-Language Model (PaLI) with visual tokens directly concatenated with text:

1. Image encoded to visual tokens
2. Language instruction tokenized
3. Both fed to unified transformer
4. Action tokens predicted as text output

### OpenVLA Approach

OpenVLA builds on open-source VLMs (Llama + SigLIP):

1. Visual features extracted by SigLIP encoder
2. Projected to language model embedding space
3. Combined with instruction tokens
4. Action tokens predicted autoregressively

### Key Design Choice: Visual Token Count

- **More visual tokens**: Higher resolution, more detail, slower inference
- **Fewer visual tokens**: Faster, but may miss important details

Typical ranges: 256-576 visual tokens per image frame.

---

## Temporal Perception

Robots operate over time, requiring perception that considers history:

### Current Frame Only

Simplest approach—decide based on single observation:
- Fast inference
- No memory of past states
- Struggles with occlusion, ambiguity

### Frame Stacking

Include recent frames in input:
- [Frame t-2] [Frame t-1] [Frame t]
- Captures recent motion
- Fixed history window

### Recurrent/Memory Approaches

Maintain hidden state across time:
- LSTM/GRU components
- Memory transformers
- Longer-term context
- More complex training

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: Perception is the first stage of the perception-action loop; VLA enhances it with language understanding
- **Module 2 (ROS 2)**: Camera data arrives via ROS 2 image topics (sensor_msgs/Image), providing the raw input for vision encoders
- **Module 4 (Isaac)**: Isaac ROS provides GPU-accelerated image processing that can preprocess inputs for VLA models

---

## Key Takeaways

- **Vision encoders** transform raw images into compact, meaningful feature representations
- **Vision Transformers (ViT)** treat images as patch sequences, enabling global reasoning
- **Cross-modal fusion** combines visual and language features for unified understanding
- Fusion strategies include **early fusion** (unified processing), **late fusion** (separate then combine), and **cross-attention** (selective interaction)
- VLA systems must also handle **temporal perception** for continuous robot operation

---

**Next**: [Language Grounding →](./03-language-grounding.md)
