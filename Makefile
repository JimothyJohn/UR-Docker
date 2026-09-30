# Common dev tasks for perceptronics. Run `make help` for a list.

DOCKER ?= sudo docker
COMPOSE ?= $(DOCKER) compose
# Python env + deps are managed with uv (see pyproject.toml).
UV ?= uv
PYTHON ?= $(UV) run python
PYTEST ?= $(UV) run pytest
RUFF ?= $(UV) run ruff

CONTAINER := perceptronics-ursim-1
PX_CONTAINER := perceptronics-ursim-px-1

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
        simx-up simx-down simx-logs simx-shell urcap-package urcap-install urcap-cockpit \
        urcap-track urcap-compat urcap-e2e urcapx-matrix \
        urcap5-sdk urcap5-package urcap5-install \
        rs-info rs-gui rs-gui-fake rs-test perceptronics-build perceptronics-up perceptronics-down \
        doctor cockpit cockpit-dry mcp \
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

# ---- PolyScope X URCap (urcap/: README.md to install, DEVELOPING.md to work on it) --
urcap-package:  ## Rebuild the downloadable urcap/dist/perceptronic-<ver>.urcapx (no npm; commit it).
	$(PYTHON) urcap/urcapx.py package urcap/perceptronic --out urcap/dist

urcap-install: urcap-package  ## Install (or replace) it in the PolyScope X sim on :8000; then refresh the page.
	$(PYTHON) urcap/urcapx.py install urcap/dist/perceptronic-*.urcapx --port 8000 --replace

urcap-cockpit:  ## A synthetic cockpit on :7621 (the normal port) the URCap page may call from the sim's origin.
	$(PYTHON) -m perceptronics gui --fake --no-browser --port 7621 --cors http://localhost:8000,http://127.0.0.1:8000

urcap-track:  ## Is urcap/target.json still UR's newest PolyScope X release? (exit 1 + why when not)
	$(PYTHON) urcap/track.py check

urcap-compat:  ## The URCap against the pinned SDK: contribution-api members, manifest spec, worker protocol.
	$(PYTHON) urcap/track.py compat

urcap-e2e:  ## Boot target.json's simulator, install a fresh build, load + click the node headlessly.
	$(UV) run --with playwright==1.63.0 python urcap/e2e.py

urcapx-matrix:  ## The e2e on the ten newest PolyScope X releases (PSX_VERSION=10.14.0 / all; images removed after each run).
	$(UV) run --with playwright==1.63.0 python urcap/psx_matrix.py run --version $(PSX_VERSION) --rmi --artifacts target/psx-matrix

# ---- PolyScope 5 (e-Series) URCap (urcap/perceptronic-ps5, urcap/urcap5.py) --------
URCAP5_CONTAINER ?= ur-utils-ursim-e-ur3e
urcap5-sdk:  ## The URCap API jars of the oldest supported PolyScope (+ compat.since) into target/ (registry; never committed).
	$(PYTHON) urcap/urcap5.py sdk

urcap5-package:  ## Rebuild the downloadable urcap/dist/perceptronic-ps5-<ver>.urcap (JDK; commit it).
	$(PYTHON) urcap/urcap5.py package urcap/perceptronic-ps5 --out urcap/dist

urcap5-install: urcap5-package  ## Install it in the e-Series sim container $(URCAP5_CONTAINER) (restarts it).
	$(PYTHON) urcap/urcap5.py install urcap/dist/perceptronic-ps5-*.urcap --container $(URCAP5_CONTAINER)

# The URCap on every PolyScope 5 minor from 5.4 (urcap/ps5_matrix.py MATRIX; amd64 host).
PSX_VERSION ?= all
PS5_VERSION ?= all
.PHONY: urcap5-matrix urcap5-matrix-down urcap5-matrix-compose
urcap5-matrix:  ## Per PS5 URSim: API check, boot, URCap starts, pick e2e, down -v (PS5_VERSION=5.4 / 5.26 / all).
	DOCKER="$(DOCKER)" $(PYTHON) urcap/ps5_matrix.py run --version $(PS5_VERSION) --artifacts target/ps5-matrix/artifacts

urcap5-matrix-down:  ## Tear the PS5 matrix sims down with their volumes.
	DOCKER="$(DOCKER)" $(PYTHON) urcap/ps5_matrix.py down --version $(PS5_VERSION)

urcap5-matrix-compose:  ## Regenerate docker-compose.ps5-matrix.yml from ps5_matrix.py's MATRIX (commit it).
	$(PYTHON) urcap/ps5_matrix.py compose > docker-compose.ps5-matrix.yml

# ---- RealSense perception (docs/realsense.md) ----------------------------------
# On macOS librealsense needs root to claim the camera's USB interface, hence
# the `sudo` on the hardware targets; `rs-gui-fake` needs no camera at all.

rs-info:  ## List attached RealSense cameras + SDK version (macOS: needs sudo).
	sudo $(PYTHON) -m perceptronics rs-info

rs-gui:  ## RGB-D cockpit on the RealSense (browser, loopback).
	sudo $(PYTHON) -m perceptronics gui

rs-gui-fake:  ## RGB-D cockpit on a synthetic scene (no camera).
	$(PYTHON) -m perceptronics gui --fake


rs-test:  ## Hardware-in-the-loop RealSense tests (skips without a camera).
	sudo $(PYTEST) -m realsense -q

perceptronics-build:  ## Build the perceptronics service image (Jetson / Linux; compiles librealsense).
	$(COMPOSE) --profile perceptronics build

perceptronics-up:  ## Run the perceptronics service (privileged, USB, cockpit on :7621).
	$(COMPOSE) --profile perceptronics up -d

perceptronics-down:  ## Stop the perceptronics service.
	$(COMPOSE) --profile perceptronics down

# ---- The pilot's seat (docs/realsense-cell.html) ------------------------------------
# One cell profile (sim | ur3 | ur20, perceptronics/cells/*.env) selects robot host/ports + bracket.
CELL ?= sim

doctor:  ## Pre-flight checklist for CELL (SDK, camera, robot reachability + state).
	$(PYTHON) -m perceptronics --cell $(CELL) doctor

cockpit:  ## The RGB-D cockpit for CELL (sudo on macOS for a real camera; scripts/cockpit.sh does that).
	./scripts/cockpit.sh $(CELL)

cockpit-dry:  ## Cockpit for CELL with robot actions validated + audited but not sent.
	./scripts/cockpit.sh $(CELL) --robot-dry-run

mcp:  ## The combined robot + camera MCP server for CELL over stdio (what .mcp.json runs).
	$(PYTHON) -m perceptronics.mcp_server --cell $(CELL)

# ---- Tests ------------------------------------------------------------------

test: test-unit  ## Alias for `test-unit` (the default fast path).

test-unit:  ## Run unit tests (no simulator required).
	$(PYTEST) -m "not integration and not sam"

test-integration:  ## Run integration tests against a running URSim.
	$(PYTEST) -m integration

test-all:  ## Run every test.
	$(PYTEST)

# ---- Lint / format ----------------------------------------------------------

lint: lint-py lint-sh  ## Run all linters.

lint-py:  ## Lint Python with ruff.
	$(RUFF) check urctl perceptronics scripts urcap tests

fmt:  ## Format Python with ruff.
	$(RUFF) format urctl perceptronics scripts urcap tests

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
	$(UV) sync --extra vision
