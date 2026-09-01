## What

<!-- One or two sentences: what changes and why. -->

## Checklist

- [ ] `make lint` passes (ruff check + format, shellcheck)
- [ ] `uv run pytest -m "not integration"` passes
- [ ] New robot capability? Added in all three places: `Robot` method → `urctl/tools.py` Tool → `urctl/cli.py` subcommand (they must stay in lockstep — see CLAUDE.md)
- [ ] Touches simulator wiring, motion, or the URP schema? Add the **`run-integration`** label so CI boots URSim and runs the integration suite
- [ ] Changed the `.urp`/`.installation` schema? Unit test added and `tests/test_integration_ursim.py::test_generated_urp_loads` still passes

## Robot impact

<!-- Does this change what gets sent to a controller (motion, IO, Dashboard
     commands)? If yes, note how it was verified (URSim / real e-Series) and
     whether the safety envelope covers it. "None" is a fine answer. -->
