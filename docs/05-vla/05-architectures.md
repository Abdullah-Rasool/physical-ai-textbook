# VLA Architectures: End-to-End vs. Modular Approaches

**Estimated Reading Time**: 10 minutes

---

## Learning Objectives

After completing this section, you will be able to:

1. **Compare** end-to-end and modular VLA architectural approaches
2. **Describe** specific VLA systems (RT-2, OpenVLA, PaLM-E, SayCan, Code-as-Policies)
3. **Articulate** tradeoffs between architectural choices
4. **Determine** when to use each approach based on requirements

---

## Two Architectural Philosophies

VLA systems can be organized in fundamentally different ways:

1. **End-to-End**: Single model from pixels + language to actions
2. **Modular**: Separate components with defined interfaces

Each approach has distinct characteristics affecting training, generalization, debugging, and deployment.

```
┌─────────────────────────────────────────────────────────────────────┐
│              End-to-End vs. Modular VLA                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  END-TO-END                           MODULAR                       │
│                                                                     │
│  ┌─────────────────────────┐         ┌──────────┐                  │
│  │   Single Neural Model    │         │ Language │                  │
│  │                          │         │  Model   │                  │
│  │  Image ─┐                │         └────┬─────┘                  │
│  │         ├─▶ Actions     │              │ subtasks                │
│  │  Lang ──┘                │              ▼                        │
│  │                          │         ┌──────────┐  ┌──────────┐   │
│  └─────────────────────────┘         │ Skill 1  │  │ Skill 2  │   │
│                                       └────┬─────┘  └────┬─────┘   │
│  Learned end-to-end                        │             │         │
│  from robot data                           └──────┬──────┘         │
│                                                   │                 │
│                                                   ▼                 │
│                                              Robot Actions          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Diagram Explanation**: End-to-end models learn direct mappings from inputs to outputs. Modular systems decompose the problem into components that can be developed separately.

---

## End-to-End Architectures

End-to-end VLA models use a single neural network to process visual input and language, directly outputting robot actions.

### RT-2 (Robotics Transformer 2)

**Developed by**: Google DeepMind (2023)

**Architecture**:
- Built on PaLI-X (55B parameter vision-language model)
- Images tokenized via ViT encoder
- Language and actions tokenized as text
- Single autoregressive transformer generates action tokens

```
┌─────────────────────────────────────────────────────────────────────┐
│              RT-2 Architecture                                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Image                    Language                                 │
│     │                         │                                     │
│     ▼                         │                                     │
│  ┌──────────┐                 │                                     │
│  │   ViT    │                 │                                     │
│  │ Encoder  │                 │                                     │
│  └────┬─────┘                 │                                     │
│       │                       │                                     │
│       ▼                       ▼                                     │
│  [img tokens] + [lang tokens] + [action tokens]                     │
│                       │                                             │
│                       ▼                                             │
│            ┌─────────────────────┐                                  │
│            │   PaLI-X Backbone   │                                  │
│            │  (55B parameters)   │                                  │
│            └──────────┬──────────┘                                  │
│                       │                                             │
│                       ▼                                             │
│              Predicted Action Tokens                                │
│              "1 128 64 200 180 90 255"                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Key Innovation**: Actions represented as text tokens, enabling web-scale VLM to be finetuned for robotics.

**Strengths**:
- Leverages massive pre-training on internet data
- Strong semantic reasoning and generalization
- Can follow novel instructions

**Limitations**:
- Requires significant compute for inference
- Large models are slow for real-time control
- Closed-source

### OpenVLA

**Developed by**: Stanford/Berkeley collaboration (2024)

**Architecture**:
- Built on Llama 2 (7B) + SigLIP vision encoder
- Open-source and reproducible
- Trained on Open X-Embodiment dataset

```
┌─────────────────────────────────────────────────────────────────────┐
│              OpenVLA Architecture                                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Image ──▶ SigLIP ──▶ [visual tokens]                             │
│                              │                                      │
│                              ▼                                      │
│   Language ──▶ Tokenizer ──▶ [text tokens]                         │
│                              │                                      │
│                              ▼                                      │
│            ┌─────────────────────────────────┐                      │
│            │  Llama 2 Backbone (7B params)   │                      │
│            │  with visual token projection   │                      │
│            └───────────────┬─────────────────┘                      │
│                            │                                        │
│                            ▼                                        │
│                   Action Token Prediction                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Key Innovation**: Open-source, efficient (7B vs 55B), trained on large multi-robot dataset.

**Strengths**:
- Reproducible research
- Reasonable inference speed
- Cross-embodiment training

**Limitations**:
- Lower performance than largest models
- Still requires significant GPU memory

### PaLM-E (Embodied Language Model)

**Developed by**: Google (2023)

**Architecture**:
- 562B parameter multimodal model
- Integrates robot state, images, and language
- Can output plans as well as low-level actions

**Key Innovation**: Largest embodied model, combines high-level reasoning with physical grounding.

---

## Modular Architectures

Modular approaches decompose VLA into separate components that communicate through defined interfaces.

### SayCan

**Developed by**: Google (2022)

**Architecture**:
- Large Language Model (LLM) proposes candidate actions
- Learned affordance functions score action feasibility
- Skills execute selected actions

```
┌─────────────────────────────────────────────────────────────────────┐
│              SayCan Architecture                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   "Bring me a snack from the kitchen"                               │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │      Large Language Model           │                          │
│   │   (proposes action candidates)      │                          │
│   │                                     │                          │
│   │   Candidates:                       │                          │
│   │   - "go to kitchen"                 │                          │
│   │   - "find snack"                    │                          │
│   │   - "pick up snack"                 │                          │
│   │   - "bring to user"                 │                          │
│   └─────────────────┬───────────────────┘                          │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │      Affordance Functions           │                          │
│   │   (score: can robot do this now?)   │                          │
│   │                                     │                          │
│   │   "go to kitchen": 0.9 (can do)     │                          │
│   │   "pick up snack": 0.1 (not near)   │                          │
│   └─────────────────┬───────────────────┘                          │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │      Select Best Action             │                          │
│   │   score = P(useful) × P(feasible)   │                          │
│   └─────────────────┬───────────────────┘                          │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │      Execute Skill                  │                          │
│   │   (learned manipulation/nav skill)  │                          │
│   └─────────────────────────────────────┘                          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Key Innovation**: Separate "what to do" (LLM) from "can I do it" (affordance) from "how to do it" (skills).

**Strengths**:
- Leverages powerful LLM reasoning
- Skills can be trained independently
- Transparent decision making

**Limitations**:
- Requires pre-defined skill library
- Can't generalize beyond known skills
- Multiple models to maintain

### Code-as-Policies

**Developed by**: Google (2022)

**Architecture**:
- LLM generates Python code representing robot policy
- Code calls perception and control APIs
- Executed code produces robot behavior

```
┌─────────────────────────────────────────────────────────────────────┐
│              Code-as-Policies Architecture                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   "Stack the red block on the blue block"                           │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │      Large Language Model           │                          │
│   │   (generates Python code)           │                          │
│   └─────────────────┬───────────────────┘                          │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │  Generated Code:                    │                          │
│   │                                     │                          │
│   │  red_pos = detect_object("red")     │                          │
│   │  blue_pos = detect_object("blue")   │                          │
│   │  pick(red_pos)                      │                          │
│   │  place(blue_pos + [0, 0, 0.05])     │                          │
│   │                                     │                          │
│   └─────────────────┬───────────────────┘                          │
│                     │                                               │
│                     ▼                                               │
│   ┌─────────────────────────────────────┐                          │
│   │      Code Execution Engine          │                          │
│   │   (calls perception + control APIs) │                          │
│   └─────────────────────────────────────┘                          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Key Innovation**: LLM's code generation ability repurposed for robot control.

**Strengths**:
- Compositional: can combine primitives in novel ways
- Interpretable: generated code shows reasoning
- No robot training data needed for LLM

**Limitations**:
- Requires well-defined API
- Code errors crash execution
- Limited to API capabilities

---

## Architecture Comparison

| Aspect | End-to-End | Modular |
|--------|-----------|---------|
| **Training data** | Robot demonstrations | Skills + LLM (separate) |
| **Generalization** | Novel objects/scenes | Novel task compositions |
| **Debugging** | Opaque (neural network) | Transparent (inspect components) |
| **Compute** | High (large models) | Distributed across components |
| **Flexibility** | Requires retraining | Swap components independently |
| **Long-horizon** | Challenging | Natural decomposition |

---

## Tradeoff Analysis

### When to Use End-to-End

Choose end-to-end when:
- **Rich demonstration data available**: You have large robot datasets
- **Novel manipulation required**: Tasks requiring fine-grained motor control
- **Scene variation matters**: Need to generalize across visual appearances
- **Latency is acceptable**: Can afford ~100ms inference

### When to Use Modular

Choose modular when:
- **Limited robot data**: Can leverage web-trained LLMs
- **Long-horizon tasks**: Need high-level planning
- **Debuggability critical**: Must understand failures
- **Skill library exists**: Have reliable primitive skills
- **Rapid iteration needed**: Want to update components independently

---

## Hybrid Approaches

Recent work combines both philosophies:

- **VLA for primitives + LLM for sequencing**: End-to-end skills with modular composition
- **LLM plans + VLA executes**: High-level from LLM, low-level from VLA
- **Hierarchical VLA**: Multiple VLA models at different abstraction levels

---

## Connection to Previous Modules

- **Module 1 (Foundations)**: Both approaches implement the perception-action loop; the difference is internal organization
- **Module 2 (ROS 2)**: Modular approaches naturally map to ROS 2 nodes; end-to-end may run as single node
- **Module 3 (Digital Twin)**: Both benefit from simulation training; end-to-end needs more sim data
- **Module 4 (Isaac)**: Isaac perception can feed either architecture

---

## Key Takeaways

- **End-to-end VLA** (RT-2, OpenVLA) learns direct mapping from inputs to actions in a single model
- **Modular VLA** (SayCan, Code-as-Policies) decomposes the problem into separate components
- End-to-end offers better **generalization to novel scenes** but requires **more robot data**
- Modular offers better **debuggability** and **long-horizon planning** but is **limited by predefined skills**
- Real systems increasingly use **hybrid approaches** combining strengths of both

---

**Next**: [Integration Patterns →](./06-integration.md)
