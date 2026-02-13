# Aquila

Aquila is a small fixed-wing UAS “digital twin” and flight software stack.

- 6-DOF nonlinear dynamics model of a small fixed-wing UAV
- Modular C++ flight software core (estimation, guidance, control, modes)
- Python-based simulation and analysis tools
- Containerized dev environment and CI

See [`docs/design_overview.md`](docs/design_overview.md) for the full architecture.

## Quickstart

### Native (host) build

```bash
mkdir -p build
cmake -S . -B build
cmake --build build
ctest --test-dir build
pytest sim/tests