# Wirebonder Operator's Manual

LaTeX source for the end-user manual. Audience is the operator at the machine:
controls, menus, bonding modes, parameters, calibration and troubleshooting.
It deliberately does **not** cover the GDB debug bridge, the
`FIRMWARE_MODE_DEBUG_*` images, or the `AnalysisScripts/` tuning workflow —
those are service material for a separate document.

## Scope: this manual vs. the K&S service manual

The machine is a **Kulicke & Soffa 4523AD** with the control card and firmware
replaced. That split defines what belongs here:

- **This manual** — anything under software control: panel, display, menus,
  parameters, bonding sequences, calibration, messages.
- **The K&S 4523AD Service Manual** — the mechanics: bond head and lever, tool
  fitting, wire threading and clamps, installation, consumables.

Appendix C maps the two and lists where the K&S manual is *wrong* about this
machine (electrical documentation, panel, messages, storage, calibration).

Use `\seeks{...}` to point the reader at the K&S manual — it renders as
"*See the K&S 4523AD Service Manual for …*". The base-machine name is defined
once, in `product.tex` (`\basemachine`, `\ksmanual`).

> The firmware comment at `configuration_parameter_catalog.cpp:22` calls it the
> **4523AD**, which is the K&S product designation, so that is what the manual
> uses. Change `\basemachine` in `product.tex` if a different form is wanted.

## Building

```sh
make            # build main.pdf
make watch      # rebuild on save
make todo       # list every section still to be written
make clean      # remove build artefacts, keep the PDF
```

Needs a TeX Live install with `latexmk` and `lualatex` (both ship with MacTeX
and TeX Live full). No external fonts or network access required.

## Layout

| Path | What it is |
| --- | --- |
| `main.tex` | Document skeleton — title page, part structure, `\include` list |
| `product.tex` | **The only place naming is defined.** Product name, model, version |
| `wirebonder-manual.cls` | Document class: `\keycap`, `lcdscreen`, callouts, `\TODO` |
| `chapters/` | One file per chapter |
| `tables/` | Reference tables transcribed from firmware — see below |
| `figures/` | Artwork (currently empty; placeholders are in the chapters) |

## Keeping it in sync with the firmware

Everything in `tables/` is derived from firmware source. Each file names its
sources in a header comment and carries a `\derivedfrom{...}` stamp in the
rendered PDF. When the firmware changes, re-check these:

| Table | Firmware source |
| --- | --- |
| `parameters.tex` | `configuration_parameter_catalog.cpp`, `configuration.h`, `menu.cpp` |
| `messages.tex` | `robot.cpp`, `user_interface_module.cpp`, `menu.cpp` |
| `keymap.tex` | `menu.cpp`, `user_interface_module.cpp`, `configuration.h` |
| `settings.tex` | `menu.cpp`, `machine_settings.hpp`, `user_interface_module.cpp` |
| `specifications.tex` | `configuration.h` |
| `defaults.tex` | `configuration_parameter_catalog.cpp`, `configuration.h` |

Quick checks:

```sh
# Parameter count must be 31
grep -c '^\\menuitem{' tables/parameters.tex

# Every operator-visible string should appear in messages.tex
grep -ohE '"[^"]{3,}"' ../Firmware/Core/Src/robot.cpp \
                       ../Firmware/Core/Src/user_interface_module.cpp
```

## Unwritten sections

`make todo` lists them. Four remain, and all four are **local policy decisions**
that neither the firmware nor the K&S manual can answer:

| Where | What is needed |
| --- | --- |
| `chapters/00-safety.tex` | Declaration of conformity for the machine **as rebuilt** — the base machine's original K&S markings do not carry over once the control electronics are replaced |
| `chapters/06-configurations.tex` | Recipe control: who may change a qualified configuration, and when re-qualification is required |
| `chapters/11-good-bonds.tex` | House starting parameters, once recipes are proven here |
| `chapters/11-good-bonds.tex` | Pull-test limits, acceptable failure modes and sampling frequency |

Everything else is written. Three `\figureplaceholder` boxes remain in
`chapters/01-overview.tex` and `chapters/02-controls.tex` — machine and panel
photographs.

## Product naming

The firmware carries no product name, model number, manufacturer or version
string anywhere. `product.tex` currently uses "Wirebonder" with placeholders for
the rest; edit that one file to change them everywhere.

## Firmware issues noted while writing

1. **Height parameters accept values the Z axis cannot reach.**
   `CONFIGURATION_EDITOR_HEIGHT_MAX` is 20 mm but
   `BONDER_MODULE_ZAXIS_WORKSPACE_SIZE` is 9 mm, and `setZMotorPosition()` does
   not clamp the setpoint. A height above ~9 mm is commanded, never arrives,
   and the cycle fails with `Z POS. NOT SETTLED` or `TIMEOUT`. The manual warns
   about this prominently, but clamping the editor limit to the actual travel
   would be the real fix.


2. `Core/Inc/Protocol/protocol_force_setup.hpp`'s header comment says the
   operator holds the **left** mouse button. `protocol_force_setup.cpp` waits on
   `EVENT_RIGHT_BUTTON_PRESSED`/`_RELEASED`. The manual documents the right
   button, matching the code; the comment is stale.
3. The panel's **HIGH RESET** button (`m_btnHighReset` in `robot.hpp`,
   `panel_pins.txt`) has no handler bound and does nothing. The manual says so
   explicitly. Worth deciding whether to wire it up or blank the keycap.
