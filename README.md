# TClient Sovereign Centaur (Phase 68) 🏺🛡️🌌

Welcome to the **Edge-Sovereign** Singularity. This project is a high-performance, neural-physical autonomous navigation bot for DDNet, optimized for finishing complex maps like **AiP-Gores**.

## 🧠 The Singularity Architecture
The system operates as a "Sovereign Centaur": a fusion of a **Neural Action Prior** (Python Brain) and a **Deterministic Forward-Simulator** (C++ Oracle).

### 1. LPSD (Latent Phase-Space Diffusion)
The primary intelligence core. It uses a **Consistency Model** to hallucinate 100-tick action trajectories based on a 7D input manifold:
- **State**: [Pos_x, Pos_y, Vel_x, Vel_y, Shadow_x, Shadow_y, Risk]
- **Output**: 100 predicted actions + 200 spatial target coordinates.

### 2. Sovereign Sync Bridge (Phase 68)
A zero-latency, wait-free synchronization protocol between C++ and Python:
- **Seqlock Protocol**: Parity-check spinlocks ensure atomic snapshot retrieval without mutex-induced jitters.
- **Memory Integrity**: Forced `#pragma pack(push, 1)` and `std::atomic` fences guarantee 100% telemetry consistency and bypass CPU L1/L2 cache lag.
- **Struct Size**: 1244 Bytes (Exactly aligned).

### 3. Edge-Sovereign Telemetry
- **Phantom Tether**: Latency-compensated shadow-state projection. The AI sees where the character *will be* on the server, not where it is on the client.
- **Risk-Jacobian Sonar**: A Euclidean threat gradient that detects lethal tiles (death/freeze) in a 5x5 radius, allowing the brain to prune hazardous hallucinations.

---

## 📈 The Road to Singularity (Versions)

### Phase 44: The Curiosity Engine
Implemented **Curiosity Entropy Injection** and Anti-Looping guards to prevent the bot from getting stuck in rhythmic local minima.

### Phase 62: LPSD Deployment
Traditional A* pathfinding was replaced with the **Latent Phase-Space Diffusion** manifold, enabling momentum-aware "hallucinogenic" navigation.

### Phase 65: The Sync Collapse Resolution
Identified a fundamental failure where the C++ core and Python brain were out of sync due to compiler padding. Resolved through a global architectural debate (AI Studio vs Standard Gemini).

### Phase 68: Edge-Sovereign Singularity
Final hardening. Implementation of the **Sovereign Sync** (Seqlock) and **Risk Sonar**. Zero-latency state transfer achieved.

---

## 🚀 Deployment
1. **Compile**: `ninja -C build` (MSYS2 MinGW64).
2. **Launch Client**: `./DDNet.exe` (Ensure `cl_tas_playback 2` for LPSD mode).
3. **Launch Brain**: `python Global_AI_Toolkit/diffuser.py` (Requires PyTorch).

*Architected by Lexzyy* 🏛️🛡️
