# Bridge Layer Scaffold

This folder contains a concrete, non-invasive scaffold for an EMQX-to-RMF
bridge service.

Goals:
- Keep current bridge scripts untouched.
- Provide clear interfaces (contracts) for future implementation.
- Make Python-to-C++ migration easier by stabilizing boundaries first.

Current status:
- Interface contracts and method signatures are defined.
- Service wiring skeleton exists.
- No existing console script entry points are modified.
