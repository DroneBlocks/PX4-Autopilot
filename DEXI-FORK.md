# DEXI fork reference (PX4 1.17)

What this fork changes vs upstream PX4, why, and what to re-check on every PX4 bump.
Target remote: **`dbaldwin`** (github.com/dbaldwin/PX4-Autopilot). Base tag: **v1.17.0**.
Primary target: **DEXI-3 / H743-AIO, indoor GPS-denied optical-flow.**

## DEXI deltas (OURS — must survive a rebase)

| # | change | file | type | notes |
|---|--------|------|------|-------|
| 1 | Freeze horizontal accel bias when optical flow is the *only* horizontal aiding source | `src/modules/ekf2/EKF/ekf_helper.cpp` (`updateIMUBiasInhibit`) | **upstream candidate** | Fixes flow-only accel-bias walk-off (~0.15/flight → plateaus ~0.016). GPS/vision drones unaffected — gated on `isOnlyActiveSourceOfHorizontalAiding(opt_flow)`. Flight-validated over 5 flights, no EKF reset needed. |
| 2 | Relax `AUTO_TAKEOFF` to `mode_req_local_position_relaxed` | `src/modules/commander/ModeUtil/mode_requirements.cpp` | **upstream candidate** (likely wants param-gating) | Lets discrete `NAV_TAKEOFF` accept on a flow-only drone on the ground. Mirrors AUTO_LAND/POSCTL. Without this, takeoff is `COMMAND_DENIED` until flow fuses in the air — a deadlock. |
| 3 | DEXI-3 indoor airframe + board config | `ROMFS/px4fmu_common/init.d/airframes/4701_dexi3_indoor`, `boards/droneblocks/h743-aio/default.px4board` | **fork-only** | `SENS_BOARD_ROT=1` (Yaw45 mount) correct out of the box; validated tune (airmode off, 70% rate-K) replacing inherited QAV250 gains; SIH enabled for desk sim-to-flight. |

Plus the pre-existing fork board/battery commits (`droneblocks_h743-aio` board, V_DIV/3S defaults) already on the branch.

## NOT ours — do NOT re-carry as custom patches

- **Optical-flow range-height bootstrap** (`optical_flow_control.cpp`, `54ab3158c5`) — authored by PX4 maintainer J. Dahl, already in upstream **`release/1.17`** and **`stable`**. It rides along with the 1.17 base for free. (Previously mis-documented as a DEXI custom branch `fix/optical-flow-range-height-bootstrap` — it is not.)

## Guardrails on every PX4 bump

- **Stay on PX4 1.17 — do NOT drop to 1.16.2.** 1.16.2's NuttX STM32H7 serial driver garbles the 420000-baud CRSF stream (`rc_input` stuck "searching", 0 valid frames). 1.17's NuttX **`fb2fadf6`** decodes it cleanly.
- **NuttX submodule must stay ≥ `fb2fadf6`** (`platforms/nuttx/NuttX/nuttx`). This is the CRSF fix; our current pin is exactly it.
- **Re-check deltas #1 and #2 survive the rebase** — both touch files upstream actively edits (`ekf_helper.cpp`, `mode_requirements.cpp`). A silent drop reintroduces the takeoff drift / `COMMAND_DENIED`.
- **px4_msgs skew is a companion issue, not firmware.** A 1.17 FC publishes versioned `_v1` topics; ROS2 consumers must remap. This takeoff path is MAVLink-only (no drone compute), so it's unaffected — do not "fix" it by rolling the FC back to 1.16.2 (see CRSF guardrail).

## Not committed here (out of scope)

Untracked experimental dirs left alone on purpose: `boards/droneblocks/h743/`, `src/lib/rl_tools/`, `src/modules/mc_raptor/`.

## Build / flash (Apple Silicon)

Host build (Docker amd64 fails under Rosetta): Python 3.12 venv, `setuptools<81`, `git clean -fdx` the NuttX submodules first if a prior Docker build left x86 host tools. `make droneblocks_h743-aio_default`. Flash via DFU (`dfu-util`, app at `0x08020000`). Full build/flash playbook lives in memory `dexi-autotakeoff-flow-ekf-degrades`.
