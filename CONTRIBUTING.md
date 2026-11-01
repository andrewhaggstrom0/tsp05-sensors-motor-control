# Workflow

1. Never commit directly to `main`.
2. Branch from `main`: `git checkout -b <area>/<short-desc>`
   e.g. `firmware/scheduler`, `dc/pid`, `sensors/thermistor`, `ros/bridge`, `gui/foxglove`
3. Commit small and often with messages that say what changed:
   `Add encoder ISR with 2x decoding on INT0`
4. Push and open a PR. Fill in every section of the template.
5. One teammate reviews and merges.

Commit from your own GitHub account. Individual grades come from commit and PR history.
