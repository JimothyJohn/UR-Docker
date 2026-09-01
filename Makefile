# Common dev tasks for UR-Docker. Run `make help` for a list.

DOCKER ?= sudo docker
COMPOSE ?= $(DOCKER) compose
# Python env + deps are managed with uv (see pyproject.toml).
UV ?= uv
PYTHON ?= $(UV) run python
PYTEST ?= $(UV) run pytest
RUFF ?= $(UV) run ruff

CONTAINER := ur-docker-ursim-1
PX_CONTAINER := ur-docker-ursim-px-1

# PolyScope X sim knobs (consumed by the `ursim-px` compose service).
# ROBOT_TYPE: UR3 UR5 UR8L UR10 UR16 UR18 UR20 UR30. HOST_ARCH: amd64 | arm64.
ROBOT_TYPE ?= UR10
HOST_ARCH ?= amd64
export ROBOT_TYPE HOST_ARCH

# Controller connection target. Defaults to the local URSim container; override
# to drive a real robot, e.g. `make sim-poweron UR_HOST=10.0.0.5`. Exported so
# the host-side scripts (poweron.sh, e2e_drive.py, urctl) pick it up.
UR_HOST ?= localhost
export UR_HOST

.PHONY: help sim-up sim-down sim-logs sim-shell sim-poweron \
        simx-up simx-down simx-logs simx-shell \
        test test-unit test-integration test-all \
        lint lint-py lint-sh fmt regen-urps install-dev

help:  ## Show this help.
	@awk 'BEGIN{FS=":.*##"} /^[a-zA-Z_-]+:.*##/ { printf "  \033[1m%-22s\033[0m %s\n", $$1, $$2 }' $(MAKEFILE_LIST)

# ---- Simulator lifecycle ----------------------------------------------------

sim-up:  ## Start URSim in the background.
	$(COMPOSE) up -d
	@echo "URSim coming up — VNC at http://localhost:6080/vnc.html"

sim-down:  ## Stop URSim and remove the container.
	$(COMPOSE) down

sim-logs:  ## Tail URSim container logs.
	$(DOCKER) logs -f $(CONTAINER)

sim-shell:  ## Open a shell inside the URSim container.
	$(DOCKER) exec -it $(CONTAINER) /bin/bash

sim-poweron:  ## Power on the robot (POWER_OFF -> RUNNING).
	./scripts/poweron.sh

sim-e2e:  ## Drive the robot end to end (poweron -> motion -> load -> play).
	./scripts/poweron.sh && ./scripts/e2e_drive.py

# ---- PolyScope X simulator (separate product, web UI) -----------------------

simx-up:  ## Start the PolyScope X sim (ROBOT_TYPE=UR10 by default).
	$(COMPOSE) --profile polyscopex up -d
	@echo "PolyScope X coming up ($(ROBOT_TYPE)) — open http://localhost:8000 in Chrome"

simx-down:  ## Stop the PolyScope X sim and remove the container.
	$(COMPOSE) --profile polyscopex down

simx-logs:  ## Tail the PolyScope X container logs.
	$(DOCKER) logs -f $(PX_CONTAINER)

simx-shell:  ## Open a shell inside the PolyScope X container.
	$(DOCKER) exec -it $(PX_CONTAINER) /bin/bash

# urctl against the PolyScope X sim: REST Robot-API on :8000, Primary on :31001.
# (Mutating actions need the robot switched to Remote in the PolyScope X UI;
#  motion additionally needs Primary enabled under Settings -> Security -> Services.)
PX_ENV := UR_PLATFORM=polyscopex UR_ROBOT_API_PORT=8000 UR_PRIMARY_PORT=31001

simx-state:  ## Read PolyScope X robot state via the Robot-API (JSON).
	$(PX_ENV) $(PYTHON) -m urctl state

simx-bring-up:  ## Power on + brake release the PolyScope X robot (needs Remote mode).
	$(PX_ENV) $(PYTHON) -m urctl bring-up

# ---- Tests ------------------------------------------------------------------

test: test-unit  ## Alias for `test-unit` (the default fast path).

test-unit:  ## Run unit tests (no simulator required).
	$(PYTEST) -m "not integration"

test-integration:  ## Run integration tests against a running URSim.
	$(PYTEST) -m integration

test-all:  ## Run every test.
	$(PYTEST)

# ---- Lint / format ----------------------------------------------------------

lint: lint-py lint-sh  ## Run all linters.

lint-py:  ## Lint Python with ruff.
	$(RUFF) check urctl perception scripts tests

fmt:  ## Format Python with ruff.
	$(RUFF) format urctl perception scripts tests

lint-sh:  ## Lint shell scripts (skipped silently if shellcheck not installed).
	@if command -v shellcheck >/dev/null; then \
		shellcheck scripts/*.sh; \
	else \
		echo "shellcheck not installed — skipping shell lint"; \
	fi

# ---- Sample programs --------------------------------------------------------

regen-urps:  ## Rebuild every <name>.urp from its build.py (node tree) or sibling <name>.script.
	@for d in programs/*/; do \
		name=$$(basename $$d); \
		if [ -f "$$d/build.py" ]; then \
			echo "regenerating $$d$$name.urp (node tree via build.py)"; \
			$(PYTHON) "$$d/build.py"; \
		elif [ -f "$$d/$$name.script" ]; then \
			echo "regenerating $$d/$$name.urp"; \
			$(PYTHON) scripts/urp_convert.py to-urp \
				"$$d/$$name.script" "$$d/$$name.urp" \
				--installation "$$name" \
				--directory "/programs/$$name"; \
		fi; \
	done

# ---- One-time setup ---------------------------------------------------------

install-dev:  ## Create/refresh the uv venv with dev + optional extras.
	$(UV) sync --extra perception --extra mcp
