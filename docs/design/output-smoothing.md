# Output smoothing: spatial + temporal filter for all effects

Status: **implemented on branch** (step 1 global smoothing). Verify on hardware still pending.
Branch: `feature/output-smoothing` (local, created from `main` @ `d440ea89`).
Read `AGENTS.md` first and follow it (see section 11).

Implementation notes (2026-03-23):
- Core: `wled00/output_smoothing.{h,cpp}` — binomial spatial (max 12 passes) + 2-stage IIR temporal.
- Hook: `WS2812FX::show()` before overlays/gamma; skipped for realtime passthrough.
- API: state `sm.{x,t}`, cfg `hw.led.sm.{x,t}`, settings form `SMX`/`SMT`.
- UI: live sliders on Segments tab; defaults on LED settings Advanced section.
- `esp32dev` build SUCCESS after change: flash ~85.0% (+~3.6 KB vs baseline ~84.7%), RAM +32 B static.

## 1. Goal (plain language, from the requester)

Most effects have very sharp color/brightness transitions, both between neighbouring LEDs
(lateral) and between consecutive frames (time). A change from one color to another, or
from dark to bright, can happen from one LED to the next or from one frame to the next.
This feels harsh to the eye and sometimes like hard blinking.

Wanted: a general additional setting that applies to all effects (it sits *on top of* the
effect calculation) with two independent Gaussian-like filters:

- **spatial**: each pixel is mixed with its neighbours over a configurable width,
- **temporal**: each pixel is mixed with its previous frames over a configurable time.

Changes in the web UI must take effect **immediately** (live preview while dragging a
slider), not only after a save.

## 2. Decisions already taken

| Topic | Decision |
|---|---|
| Scope step 1 | One **global** setting for the whole strip. |
| Scope step 2 | Optional later: per segment (see section 10). Do not build it now, but do not block it. |
| Filter position | Before gamma correction (`gamma32()` stays after the filter). |
| Targets | ESP32 and all variants (classic, S2, S3, C3, C5, C6, P4). ESP8266 builds compile the feature out. |
| Realtime sources (DDP, E1.31, Art-Net, ...) | Nice to have, not important. Step 1 bypasses the filter in realtime mode (reason in 5.4). |
| Live preview | Required. Controls live in the main UI and use the JSON API (section 6). |

## 3. Background: how pixels flow today (as of `d440ea89`)

1. Each effect writes into its own per-segment buffer `Segment::pixels`
   (`wled00/FX.h`, private member). **Effects read their own previous frame back**
   (`getPixelColor`, `fadeToBlackBy`, `blur`, ... about 109 uses in `FX.cpp`), so the
   segment buffer must **never** be filtered.
2. `WS2812FX::service()` (`FX_fcn.cpp`) runs the effects, then calls `show()`.
3. `WS2812FX::show()` (`FX_fcn.cpp`, near line 1900):
   - clears the global frame buffer `_pixels[]` (`memset`) and composes all segments into
     it via `blendSegment()`, **but only if** `realtimeMode == REALTIME_MODE_INACTIVE ||
     useMainSegmentOnly || realtimeOverride > REALTIME_OVERRIDE_NONE`. Otherwise `_pixels[]`
     holds realtime data written by `setRealtimePixelColor()` and is *not* cleared,
   - calls `handleOverlayDraw()`,
   - loops over `_pixels[]`: `gamma32()` then `BusManager::setPixelColor(getMappedPixelIndex(i), c)`,
   - calls `BusManager::show()`.
   `show()` already computes `diff = showNow - _lastShow` (ms since the last frame).
4. `_pixels[]` is logical order. For matrices it is addressed as `x + y * Segment::maxWidth`
   (see the `XY` lambda in `blendSegment()`). The LED map is applied only in the final
   loop, so "neighbour" means the logical neighbour, which is what effects see.
5. Master brightness and ABL are applied at bus level, after `show()`. The filter history
   is therefore independent of brightness.
6. ESP8266 builds define `WLED_DISABLE_GLOBAL_PIXELBUFFER` (`platformio.ini`): no `_pixels[]`
   exists, pixels go straight into the bus encode buffers. The feature cannot exist there.
7. Memory facts: pixel buffers on classic ESP32 are allocated with `BFRALLOC_NOBYTEACCESS`
   (the extra 32-bit-only memory region, `allocate_buffer()` in `util.cpp`). **16-bit or
   8-bit accesses are not allowed in that region.** `MAX_LED_MEMORY` (`const.h`) is 85 KB on
   classic ESP32. The settings page counts `len * 8` bytes of pixel buffers
   (`getMem()` in `settings_leds.htm`).

## 4. Design

### 4.1 Insert point

In `WS2812FX::show()` after the compose block and **before** `handleOverlayDraw()`
(so overlays stay sharp), operating on `_pixels[]` in place for the effects path.
The gamma loop afterwards is unchanged.

```text
effects -> Segment::pixels -> blendSegment -> _pixels[] -> [ SMOOTHING ] -> overlay -> gamma -> bus
```

### 4.2 Parameters (stable contract, units are physical on purpose)

| Name | JSON key | Range | Unit | Meaning |
|---|---|---|---|---|
| spatial width | `sm.x` | 0..50 (0 = off) | 0.1 LED | Standard deviation sigma of the Gaussian-like kernel, along the logical pixel order (1D) or x and y (2D). |
| temporal width | `sm.t` | 0..250 (0 = off) | 10 ms | Mean latency (time constant tau) of the cascaded filter for a step input. |

Values above the range are clamped. Both 0 means the feature is fully off.

### 4.3 Spatial filter

- Separable: horizontal pass, then vertical pass for 2D.
- 1D (`!strip.isMatrix`): one pass over `totalLen`.
- 2D: passes over the matrix region (`maxWidth * maxHeight`). If `totalLen` is larger than
  the matrix, treat the tail as 1D. Check how `FX_2Dfcn.cpp` sets `_length` and `isMatrix`.
- Kernel candidates (the implementing agent decides after measuring, see 7.2):
  - repeated 3-tap binomial `[1 2 1]/4`. One pass has variance 0.5. A fractional pass
    `[w/4, 1-w/2, w/4]` has variance `w/2`, so any sigma is reachable with n full passes
    plus one fractional pass. Cost grows with sigma squared, so cap the range if needed.
  - three box passes with a running sum (O(1) per pixel regardless of width,
    sigma^2 = 3(b^2-1)/12 for box width b).
  - If a cap is needed, lower the `sm.x` upper bound and document it. Do not change the unit.
- Work on packed 8-bit-per-channel words with SWAR tricks (see `fast_color_scale()` in
  `colors.h`) or per-channel integer math. No floating point per pixel.
- In-place filtering needs the original neighbours: keep a rolling window in registers
  (1D) or one scratch row (2D). Do not allocate a full second frame buffer.
- Segment borders and bus borders are not respected in step 1 (neighbouring segments bleed
  into each other). Document this limitation in the UI tooltip or readme.

### 4.4 Temporal filter

A true Gaussian over many past frames would need about 25 frame copies, which is too much
RAM. Use a **cascade of 2 one-pole IIR filters** (exponential smoothing). The cascade
approximates a Gaussian-ish step response, and two stages remove the instant first-frame
jump that a single stage has (a single stage still jumps by `alpha * delta` at once).

Per frame, per stage:

```text
tauStage = tau / 2                       // tau = sm.t * 10 ms; mean latencies add up
alpha    = 1 - exp(-dt / tauStage)       // dt = ms since last show(); computed once per frame
stage1  += alpha * (input  - stage1)     // per channel
stage2  += alpha * (stage1 - stage2)     // per channel
output   = round(stage2)
```

- `dt` comes from `show()` (`diff`). This makes the smoothing independent of FPS, including
  unlimited FPS mode and irregular frame times. Clamp dt to 1..250 ms. If dt is 0, skip the
  temporal update and output the current state.
- `expf()` once or twice per frame is fine. `alpha = dt / (tauStage + dt)` is an acceptable
  cheaper alternative (document which one is used).
- **State precision: 16 bits per channel (8.8 fixed point), 4 channels (RGBW).** 8-bit state
  stalls short of the target (faint ghost glow, never reaching black).
- Fixed-point math: keep `alpha` as Q15 (0..32768). Check for signed 32-bit overflow:
  `|diff|` up to 65280 times 32768 is about 2.14e9, just below 2^31. Verify with a test.
- **Guarantee convergence:** if `diff != 0` and the computed step is 0, step by +-1 (in 8.8
  units). Otherwise small alpha (long tau, high FPS) leaves a permanent error.
- Rounding: `out = (state + 128) >> 8`.
- State layout: per pixel and stage **two `uint32_t`** (`R | G << 16` and `B | W << 16`), always
  accessed as 32-bit words. This keeps the buffer valid in the classic-ESP32 32-bit-only
  region (`BFRALLOC_NOBYTEACCESS`).
- Memory: 8 bytes per LED per stage, so **16 bytes per LED with 2 stages** (16 KB at 1000 LEDs,
  64 KB at 4096 LEDs). Allocate only while `sm.t > 0`; free when it returns to 0.

### 4.5 Order and fusion

Both filters are linear, so their order does not matter mathematically. Fuse them in one
loop where possible for cache locality: spatial result of pixel `i`, then temporal update
of pixel `i`.

### 4.6 Gamma

The filter runs on gamma-uncorrected values (before `gamma32()`), so fades look
perceptually even. Averaging after gamma would mix real light instead. This is the agreed
choice. Note it in the code comments.

## 5. Behaviour and edge cases (must be handled)

1. **Off by default and free when off.** With both values 0: no allocation, one branch in
   `show()`, output bit-identical to the current firmware.
2. **Reset history** (copy the current frame into the state, no fade-in of stale content) when:
   the feature goes from off to on, `updatePixelBuffer()` changes the length, the matrix
   configuration changes, and the strip is switched on after being off (`bri == 0` before).
3. **Concurrency.** The JSON handler runs in another task than `show()` on ESP32. Do **not**
   allocate or free the history buffer in the setter. The setter only stores the two target
   values (`volatile uint8_t`). `show()` (main loop context) notices the change and does
   allocation, free and reset itself. This avoids a race without `suspend()/waitForIt()`.
4. **Allocation failure** degrades gracefully: disable temporal smoothing, keep rendering,
   optionally set `errorFlag = ERR_NORAM_PX` once. Never stop rendering for this feature.
5. **Realtime mode.** In realtime mode `_pixels[]` is not cleared and may be updated
   partially (for example some E1.31 universes). An in-place filter would feed its own output
   back in the next frame (compounding blur). Step 1: skip smoothing whenever `_pixels[]` is
   not rebuilt from segments, i.e. when the condition of the compose block in `show()` is
   false. A later option (step 1b) is a non-destructive variant that writes into a scratch
   buffer.
6. **Compile-time switch.** `WLED_DISABLE_OUTPUT_SMOOTHING`, automatically defined when
   `WLED_DISABLE_GLOBAL_PIXELBUFFER` is defined (ESP8266). Do not edit `platformio.ini`
   (maintainer approval needed). All new code is wrapped in `#ifndef WLED_DISABLE_OUTPUT_SMOOTHING`.
7. **Flash size.** Measure the increase on the tightest ESP32 envs (S2, C3, 4 MB boards) and
   report it. If an env no longer fits, say so and propose a flag rather than editing
   `platformio.ini`.
8. **Expected visual side effects** (document, not a bug): moving sharp edges get short
   trails, single-LED features lose peak brightness, the temporal filter adds latency of about `tau`.

## 6. API contract (frozen so UI and firmware can be built in parallel)

State JSON (`/json/state`, WebSocket, presets), global, next to `transition` and `bs`:

```json
{"sm": {"x": 12, "t": 40}}
```

- Request: both keys optional. Missing key means "keep the current value". Missing `sm`
  object means "unchanged" (old presets must not reset the setting).
- Response (`serializeState`): written inside the `if (includeBri)` block next to
  `transition` and `bs` (`json.cpp`, near line 650), so presets store it.
- Do **not** set `stateChanged` for `sm` (same as `bs`): this keeps the active preset marker
  and avoids sync broadcasts while dragging a slider.

Persistent defaults (`cfg.json`, section `hw.led`):

```json
{"hw": {"led": {"sm": {"x": 0, "t": 0}}}}
```

Loaded in `deserializeConfig()` (`cfg.cpp`, near the `fps` handling at line 190), written in
`serializeConfig()` (near line 1010). Applied once at boot.

## 7. Work packages

Dependencies: WP1 and WP5 can run in parallel (UI is built against the contract in section 6).
WP2 needs WP1. WP3 needs WP2. WP4 and WP6 need WP3. WP7 needs all.

### WP1 - Filter core (pure functions, no WLED globals)

- New files `wled00/output_smoothing.h/.cpp` (everything under `#ifndef WLED_DISABLE_OUTPUT_SMOOTHING`).
- Suggested interface (adjust if needed):

```cpp
class OutputSmoother {
  public:
    void setTargets(uint8_t spatial, uint8_t temporal);    // cheap, any task
    bool isActive() const;                                 // either value > 0
    // called from WS2812FX::show() only (main loop context)
    void apply(uint32_t *px, unsigned width, unsigned height, unsigned len, unsigned dtMs);
    void reset();                                          // history re-sync on next apply()
    size_t memoryUsed() const;
};
```

- Implements 4.3 to 4.5 and all of section 5 that concerns the filter (lazy allocation,
  reset, graceful failure). No Arduino/WLED dependencies in the math functions so they can
  be tested on the host.
- Mark AI-generated code with `// AI: below section was generated by an AI` / `// AI: end`.
- Acceptance: both values 0 means no allocation and no per-pixel work; a constant input
  frame stays exactly constant at any setting (no drift, no rounding errors); a step input
  converges to the exact target value; no signed overflow (check with min/max test values).

### WP2 - Integration in `show()`

- `FX.h`: add `OutputSmoother _smoother;` to `WS2812FX`, public `setSmoothing(uint8_t x, uint8_t t)`
  and getters.
- `FX_fcn.cpp`: call `_smoother.apply(...)` after the compose block and before
  `handleOverlayDraw()`, only when active and only when `_pixels[]` was rebuilt this frame
  (5.5). Pass `diff` as `dtMs`. Reset history in `updatePixelBuffer()` and when `bri`
  turns on from off.
- Under `WLED_DEBUG`: measure `micros()` around the call and print it with `DEBUG_PRINTF`
  (no direct `Serial` calls).

### WP3 - State JSON

- `json.cpp`: deserialize `sm` next to `bs` (near line 408), serialize inside `includeBri`
  (near line 650). Clamp values. Call `strip.setSmoothing()`.

### WP4 - Persistent defaults (config)

- `wled.h`: `smoothDefaultX`, `smoothDefaultT` (`WLED_GLOBAL ... _INIT(0)`).
- `cfg.cpp`: read and write under `hw.led.sm`; apply at boot.
- `set.cpp` and `xml.cpp`: settings form fields `SMX` and `SMT` (grep first that these names
  are unused), pattern as `FR` (`set.cpp` near 215, `xml.cpp` near 365).

### WP5 - Main web UI with live preview

Files: `wled00/data/index.htm`, `wled00/data/index.js` (tabs, camelCase, reuse `common.js`).

- Add a block to the Segments tab directly below the "Transition" and blending-style rows
  (`index.htm`, near the `bsp` element): two range sliders "Spatial smoothing" (0..50, shown as
  LEDs with one decimal) and "Temporal smoothing" (0..250, shown in ms). Reuse the existing
  slider markup/classes (`noslide`, `sliderbubble`, `updateTrail`).
- **Live:** send on `oninput` (not only `onchange`), throttled to about 15 messages per second
  with a trailing send so the final value always arrives. Command: `requestJson({sm:{x:..., t:...}})`
  (WebSocket when open, HTTP fallback is handled by `requestJson()`).
- `readState(s)` (`index.js`, near the `bs` handling at line 1484): update both sliders and the
  displayed values from `s.sm`, without sending anything back (avoid feedback loops while dragging).
- No page reload, no save button needed. Values are RAM-only until saved in a preset (or set
  as defaults in WP6).
- Optional follow-up: a "Set as default" button that POSTs `{"hw":{"led":{"sm":{...}}}}` to
  `/json/cfg` (PIN-protected if a settings PIN is set).
- Run `npm run build` after editing. Never edit `html_*.h` or `js_*.h`.

### WP6 - Settings page

- `settings_leds.htm`: section "Smoothing (defaults)" with the two numeric fields and a short
  hint that live tuning is done in the main UI.
- Update `getMem()` so the LED memory estimate adds 16 bytes per LED when the temporal default
  is above 0 (and mention that the runtime slider can allocate it later).

### WP7 - Verification and measurement

- Builds (all must compile): `esp32dev` (required), `esp32s3dev_8MB_opi`, `esp32c3dev`,
  `lolin_s2_mini`, `nodemcuv2` (feature compiled out, no size or behaviour change), and
  `esp32dev` with `-D WLED_DISABLE_OUTPUT_SMOOTHING`. Report flash/RAM deltas per env.
- `npm ci && npm run build && npm test`.
- Timing (on hardware if available, otherwise say it was not measured): microseconds per
  `apply()` for 300, 1000 and 4096 LEDs for: spatial only, temporal only, both. Also for a 2D
  matrix (for example 32x32). Compare with the total frame time at 42 FPS.
- Host test of the core math (compile the filter functions with plain `g++`, keep the harness
  in `docs/temp/`, which is gitignored): constant input stays constant, step converges to the
  exact value for tau from 10 ms to 2.5 s at dt from 1 to 50 ms, no overflow with extreme
  inputs, frame-rate independence (same step response at 25 and 100 FPS within tolerance).
- Manual visual check on effects with hard edges or fast change (for example Colorloop, Chase,
  Twinkle, Blink, Scan, a 2D effect) while moving the sliders live. Check that turning the
  sliders to 0 gives exactly the original look.
- State clearly in the PR what was *measured* and what was only *read from code*.

## 8. Memory and speed budget (estimates, not measurements)

- RAM when temporal smoothing is on: 16 bytes per LED (2 stages x 8 bytes). Spatial: no full
  buffer (at most one row in 2D).
- Time: roughly 60 to 80 cycles per LED for spatial + 2 temporal stages (instruction count
  estimate). About 0.3 ms per frame for 1000 LEDs on a 240 MHz ESP32, against 23.8 ms per
  frame at 42 FPS. Roughly twice as slow on a 160 MHz C3. These numbers must be replaced with
  measurements from WP7.
- PSRAM history is acceptable (sequential access), but note the slow raw PSRAM access of
  quad-SPI S2/S3 variants in the measurements.

## 9. Test environment notes (this checkout)

- Use `~/.platformio/penv/bin/pio` (PlatformIO Core 6.1.19). The launcher at
  `~/.local/bin/pio` does not work here (its Python module is missing).
- `~/.platformio/penv/bin/python3.13` was re-linked to a uv-managed CPython 3.13.7 because
  `/usr/bin/python3.13` no longer exists. A clean `pio run -e esp32dev` took about 2m42s and
  succeeded (RAM 26.5%, flash 84.7% of the 1.5 MB app partition) on the baseline.
- `build_output/`, `.pio/` and the generated `wled00/html_*.h` / `js_*.h` are git-ignored.

## 10. Step 2 (not part of this work): per-segment smoothing

Sketch only, so step 1 does not block it:
- Add `smoothX`, `smoothT` to `Segment` (next to `blendMode`), JSON `{"seg":{"sm":{...}}}`
  (copy/move constructors in `FX_fcn.cpp`, `serializeSegment`/`deserializeSegment`).
- Run the filter per segment region inside `show()`, with the global value as default.
  History stays indexed by global pixel position. Neighbour reads are clamped to the segment
  to stop bleeding. Overlapping segments need a rule (top segment wins).
- Add per-segment sliders next to the blend mode in the segment panel.

## 11. Rules for agents working on this (from `AGENTS.md`)

- The human has confirmed the AI-contribution agreement for this task, and local-branch
  changes are allowed. Do not push, open a PR, post replies or create issues without being asked.
- Follow `docs/cpp.instructions.md` and `docs/web.instructions.md`: C++ with 2-space indent,
  web UI with tabs, `DEBUG_PRINTF()` instead of `Serial`, no VLAs, no exceptions, no
  `portENTER_CRITICAL`, `static` for file-local items.
- Mark AI-generated code blocks with `// AI: below section was generated by an AI` / `// AI: end`
  and give non-trivial functions a short comment.
- Never edit or commit `wled00/html_*.h` / `wled00/js_*.h`. Do not edit `platformio.ini`.
- No unrelated whitespace or formatting changes.
- Commits need a `Co-authored-by:` trailer naming the model and platform (the harness may add it).
- A PR needs the repository template (`.github/PULL_REQUEST_TEMPLATE.md`), a clear AI
  disclosure, the requester's plain-language summary from section 1, and honest wording about
  what was verified versus what only reads correct. Last line: `_You are interacting with an AI system_`.
- Avoid overconfident claims ("proves", "fully validated"); use neutral wording for results.

## 12. Open questions for the maintainers/requester

1. Is the compile flag name `WLED_DISABLE_OUTPUT_SMOOTHING` acceptable, and should the feature
   default to enabled in release builds once flash size is known?
2. Should realtime sources be smoothed in step 1b, and with which rule for partial updates?
3. Are the JSON keys `sm.x` / `sm.t` free of conflicts with WLED-Docs and known external clients?
4. Is the spatial range of 5 LEDs sigma needed, or is a lower cap (for example 2 LEDs) enough
   given the cost of the binomial approach?
