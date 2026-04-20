# Line Sensor Scientific Analysis Plan

## 1. Objectives

1. **Characterise the sensor response curve**: Map the relationship between surface reflectance (grayscale value 0-255) and raw ADC reading (0-1023) for each of the 5 sensors
2. **Quantify measurement noise**: Determine the standard deviation and repeatability of readings at each shade level
3. **Assess ambient light sensitivity**: Measure how different lighting conditions shift the absolute readings
4. **Extract the transfer function**: Derive a mathematical model (likely logarithmic/exponential) relating printed shade to sensor reading
5. **Evaluate gradient resolution**: Determine the minimum grayscale step the sensor can reliably distinguish
6. **Inform map design**: Use the response curve to optimise gradient slopes for maximum positional accuracy

---

## 2. Circuit Analysis

### 2.1 Sensor Hardware

The 3Pi+ 32U4 has **5 down-facing reflectance sensors** (DN1-DN5) along the front edge, plus 2 forward-facing bump sensors (BL, BR). All share the Pololu QTR-type reflectance sensor design.

Each sensor channel consists of:
- **IR emitter LED**: Down-facing infrared LED, current-limited
- **Phototransistor**: Detects reflected IR light; collector current proportional to incident IR intensity
- **220 ohm series resistor**: On each sensor output line (short-circuit protection)
- **Shared emitter control**: Pin 11 (EMIT) - drive HIGH to activate line sensor IR LEDs

### 2.2 Pin Mapping

| Sensor | Label | Arduino Pin | AVR Port | ADC Channel |
|--------|-------|-------------|----------|-------------|
| DN1 (leftmost) | Line 1 | 12 | PD6 | ADC9 (via MUX) → actually **A11** in code |
| DN2 | Line 2 | A0 | PF7 | ADC7 |
| DN3 (centre) | Line 3 | A2 | PF5 | ADC5 |
| DN4 | Line 4 | A3 | PF4 | ADC4 |
| DN5 (rightmost) | Line 5 | A4 | PF1 | ADC1 |
| Emitter control | EMIT | 11 | PB7 | - |

**Note on DN1**: The code maps `sensor_pins[0] = A11`, which on the ATmega32U4 corresponds to the ADC channel accessible via pin 12 (PD6). This is a quirk of the Arduino pin mapping for the 32U4.

### 2.3 Two Reading Methods

The 3Pi+ sensors support two fundamentally different reading methods. Understanding both is critical for choosing the best approach for gradient sensing.

#### Method 1: ADC (Analog) Reading — Currently Implemented

```
Circuit model (ADC mode):

    Vcc (5V)
     |
    [Internal Pull-up ~20-50kΩ]   ← enabled by INPUT_PULLUP
     |
     +----[220Ω]----+---- ADC Pin (analogRead)
                     |
                [Phototransistor]
                     |
                    GND
```

**How it works**:
1. `pinMode(pin, INPUT_PULLUP)` enables the ATmega32U4's internal pull-up resistor (~20-50kΩ)
2. This forms a **voltage divider** between the pull-up resistor and the phototransistor
3. The phototransistor acts as a variable resistance: high reflectance (white) → low resistance → more current → voltage drops toward 0V
4. `analogRead()` samples via the 10-bit ADC → value 0-1023
5. **White surface → low ADC value** (phototransistor conducting hard, pulling voltage down)
6. **Black surface → high ADC value** (phototransistor barely conducting, voltage stays near Vcc)

**Response characteristics (ADC mode)**:
- The phototransistor's collector current is approximately proportional to incident light intensity
- The voltage divider output is: `V_out = Vcc × R_photo / (R_pullup + R_photo)`
- Since R_photo varies inversely with light intensity, the ADC reading has a **nonlinear (hyperbolic)** relationship with reflectance
- Near white surfaces (R_photo << R_pullup): small changes in reflectance produce small ADC changes
- Near black surfaces (R_photo >> R_pullup): small changes in reflectance produce small ADC changes
- **Best sensitivity is in the mid-range** where R_photo ≈ R_pullup

**Key limitation**: The internal pull-up resistance is not precisely specified (~20-50kΩ), varies between pins, and varies with temperature. This makes absolute readings inconsistent between sensors and sessions.

#### Method 2: RC Timing (Digital) — Documented but Not Implemented

```
Circuit model (RC mode):

    MCU Pin ----[220Ω]----+---- Capacitor (on-board)
                          |
                     [Phototransistor]
                          |
                         GND
```

**How it works**:
1. Drive the pin HIGH as OUTPUT → charges the on-board capacitor through the 220Ω resistor
2. Wait ≥10μs for capacitor to fully charge
3. Switch pin to INPUT (high-impedance) → capacitor begins discharging through the phototransistor
4. Time how long the pin stays HIGH (using `micros()` or a timer)
5. **White surface → fast discharge** (phototransistor conducting) → short time
6. **Black surface → slow discharge** (phototransistor barely conducting) → long time

**Response characteristics (RC mode)**:
- Discharge time τ = R_photo × C
- Since R_photo ∝ 1/light_intensity: τ ∝ 1/reflectance
- This is inherently **logarithmic in time domain** — taking log(time) gives approximately linear reflectance
- Pololu experimental data (at 1/8" height): white ≈ 120μs, black ≈ 2600μs
- Dynamic range: ~22:1 in timing
- **Better discrimination at the dark end** compared to ADC mode

### 2.4 Sensor Response Model

For the **ADC mode** currently used in the codebase:

The phototransistor can be modelled as a variable resistor whose resistance R_pt is:

```
R_pt = R_dark / (1 + k × I_reflected)
```

Where:
- `R_dark` = phototransistor resistance with no light (~MΩ range)
- `k` = sensitivity constant
- `I_reflected` = reflected IR intensity (proportional to surface reflectance ρ)

The ADC output is proportional to the voltage divider:

```
V_adc = Vcc × R_pt / (R_pullup + R_pt + 220)
ADC_value = V_adc × 1023 / Vcc = 1023 × R_pt / (R_pullup + R_pt + 220)
```

Substituting the reflectance relationship:

```
ADC ≈ 1023 / (1 + (R_pullup / R_dark) × (1 + k × ρ))
```

This is a **rectangular hyperbola** in ρ (reflectance). The key insight is that the response is NOT linear — it compresses at both extremes and is most sensitive in the middle of the range.

### 2.5 Implications for Gradient Map Design

If the sensor response is confirmed to be hyperbolic:
- A **linear grayscale gradient** (equal steps in printed shade) will NOT produce equal steps in ADC reading
- The map gradient should follow the **inverse of the sensor response function** to linearise the reading
- Specifically, if ADC = f(ρ), then the map should print shade values such that ρ(position) = f⁻¹(desired_linear_ADC(position))
- This maximises the bits of positional information extractable per mm of gradient

---

## 3. Test Maps Description

### 3.1 "25 Shades of Gray" (discrete patches)

- **Layout**: 10 rows × 3 columns of uniform rectangular patches
- **Shades**: Progress from black (top-left) through grays to white (bottom-right)
- **Purpose**: Discrete calibration points — each patch is a known, uniform grayscale value
- **Usage**: Place sensor over centre of each patch, take N repeated readings
- **Grayscale values**: Approximately 25 evenly-spaced steps from RGB(0,0,0) to RGB(255,255,255), i.e., steps of ~10.2 per patch

### 3.2 "Gradient 1" (three parallel strips, varying steepness)

- **Layout**: 3 vertical strips side by side, each with black-to-white gradient
- **Left strip**: Full-page gradient (black at top → white at bottom, longest/most gradual)
- **Centre strip**: Shorter gradient (steeper, transitions faster from black to white, with white extending further down)
- **Right strip**: Even shorter gradient region, most of the strip is dark, transitioning to white in the middle section
- **Purpose**: Test sensor response at different gradient steepnesses; determine minimum gradient length for reliable position discrimination

### 3.3 "Gradient 2" (single full-page gradient)

- **Layout**: Single gradient covering the entire page
- **Gradient**: Black at top → white at bottom, linear across full page height
- **Purpose**: Continuous sweep measurement; drive robot slowly across the gradient while logging readings; extract the full response curve in one pass

### 3.4 "Thin Line" (calibration reference)

- **Layout**: Two thin black lines on white background
- **Purpose**: Edge detection reference; measure sensor response at sharp black/white transitions

---

## 4. Experimental Methodology

### Phase 1: Static Discrete Shade Characterisation (25 Shades Map)

**Goal**: Map ADC value vs known grayscale shade for each sensor

#### 4.1.1 Setup
1. Print "25 shades of gray.pdf" on A3 or A4 paper using a laser printer (inkjet pigments may have different IR reflectance than visible appearance suggests)
2. Place on a flat, rigid surface
3. Ensure consistent overhead lighting (or shield from ambient light changes during test)
4. Connect robot via USB serial cable

#### 4.1.2 Arduino Test Sketch

The sketch should:
1. Wait for button press to start each measurement
2. Read all 5 sensors N times (e.g., N=50) in rapid succession
3. Transmit all raw ADC readings over Serial in CSV format
4. Include a patch identifier (manually entered or sequentially numbered)
5. Include timestamp for each reading

**Serial output format** (CSV):
```
patch_id, sensor_0, sensor_1, sensor_2, sensor_3, sensor_4, timestamp_ms
```

**Key design decisions**:
- Read each sensor 50 times per patch to characterise noise distribution
- Small delay (2-5ms) between readings to allow ADC to fully settle
- Keep IR emitters ON throughout (don't toggle between readings)
- Record with and without IR emitters to measure ambient light baseline

#### 4.1.3 Procedure
1. Power on robot, wait for initialisation
2. Position the **centre sensor (DN3)** over the centre of patch 1 (black)
3. Press button to trigger 50 readings → data transmitted over serial
4. Move to patch 2, repeat
5. Continue through all 25 patches (or as many as printed)
6. **Repeat for each of the 5 sensors** if sensor-to-sensor variation is of interest (optional — can position robot so all 5 sensors are over the same patch if patches are wide enough)
7. **Repeat entire procedure with different ambient lighting** (overhead light off, overhead light on, daylight from window, etc.)

#### 4.1.4 Ambient Light Test Protocol

For each lighting condition:
1. Label the condition (e.g., "fluorescent overhead", "dark room", "window daylight")
2. Take a **dark reading** (sensor over black patch, IR emitters OFF) — this measures ambient IR
3. Take a **bright reading** (sensor over white patch, IR emitters OFF) — measures ambient reflected IR
4. Then run full 25-patch measurement with IR emitters ON

### Phase 2: Continuous Gradient Sweep (Gradient Maps)

**Goal**: Capture the full response curve continuously; validate discrete measurements

#### 4.2.1 Setup
1. Print gradient2.pdf (full-page gradient) on A3 paper
2. Place on flat surface with gradient oriented along one axis
3. Mark known positions along the gradient (e.g., every 20mm from the black end)

#### 4.2.2 Arduino Test Sketch (Modified)

The sketch should:
1. Continuously read all 5 sensors at a fixed rate (e.g., every 10ms)
2. Simultaneously read encoder counts for position tracking
3. Transmit: `timestamp_ms, encoder_count, sensor_0, sensor_1, sensor_2, sensor_3, sensor_4`
4. Robot drives forward at very slow constant speed (e.g., 0.1 counts/ms) along the gradient

**Alternative (preferred for accuracy)**: Drive robot by hand, pushing it slowly along the gradient while sensors log continuously. This avoids motor vibration affecting readings and gives better position control.

#### 4.2.3 Procedure
1. Place robot at the black end of the gradient, centre sensor aligned with gradient centreline
2. Start logging
3. Slowly drive/push robot toward the white end
4. Stop logging when robot reaches the white end
5. **Repeat 3-5 times** for statistical averaging
6. Repeat with gradient1.pdf (3 strips) to compare different gradient steepnesses

### Phase 3: Temporal Noise Analysis

**Goal**: Characterise reading stability over time at a fixed position

#### 4.3.1 Procedure
1. Place robot stationary over a mid-gray patch
2. Log readings continuously for 30-60 seconds (3000-6000 samples at 10ms rate)
3. Analyse for:
   - Standard deviation
   - Drift over time
   - Any periodic noise (from motor drivers, PWM, etc.)
   - Autocorrelation (are consecutive readings correlated?)

### Phase 4: RC Timing Method (Optional Advanced)

**Goal**: Compare RC timing approach to ADC approach for gradient sensing

#### 4.4.1 Implementation
The `readSensorsDigital()` function in LineSensors.h is currently empty. Implement it:

```c
void readSensorsDigital() {
    // 1. Charge capacitors: set all sensor pins as OUTPUT HIGH
    for(int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensor_pins[i], OUTPUT);
        digitalWrite(sensor_pins[i], HIGH);
    }
    delayMicroseconds(10); // Allow capacitors to charge

    // 2. Start discharge: switch all pins to INPUT (no pull-up)
    unsigned long start_time = micros();
    for(int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensor_pins[i], INPUT);
        readings[i] = 0; // Will store discharge time
    }

    // 3. Time each pin going LOW
    bool all_done = false;
    int done_count = 0;
    unsigned long timeout = 3000; // 3ms timeout

    while(!all_done && (micros() - start_time) < timeout) {
        all_done = true;
        for(int i = 0; i < NUM_SENSORS; i++) {
            if(readings[i] == 0) { // Not yet measured
                if(digitalRead(sensor_pins[i]) == LOW) {
                    readings[i] = micros() - start_time;
                } else {
                    all_done = false;
                }
            }
        }
    }

    // Set timeout value for any sensors that didn't discharge
    for(int i = 0; i < NUM_SENSORS; i++) {
        if(readings[i] == 0) readings[i] = timeout;
    }
}
```

#### 4.4.2 Procedure
- Repeat Phase 1 and 2 experiments using `readSensorsDigital()` instead of `readSensorsADC()`
- Compare response curves: ADC mode vs RC timing mode
- Evaluate which offers better gradient discrimination

---

## 5. Arduino Test Sketch Specification

### 5.1 Operating Modes

The test sketch should support multiple modes selectable via serial command:

| Mode | Command | Description |
|------|---------|-------------|
| 0 | `M0` | Idle — do nothing |
| 1 | `M1` | Static burst: read N samples on button press, transmit CSV |
| 2 | `M2` | Continuous logging: read at fixed rate, transmit continuously |
| 3 | `M3` | Drive + log: slow forward drive while logging sensors + encoders |
| 4 | `M4` | Emitter test: read with emitters ON then OFF, report both |
| 5 | `M5` | RC timing mode: use digital discharge method |

### 5.2 Serial Protocol

**Baud rate**: 115200 (faster than default 9600 for high-rate logging)

**Header line** (sent once at start of each mode):
```
# mode=<M>, samples_per_burst=<N>, read_interval_ms=<I>, method=<ADC|RC>
```

**Data lines** (CSV):
```
timestamp_ms, [patch_id], [encoder_left], [encoder_right], s0, s1, s2, s3, s4
```

Where:
- `timestamp_ms`: millis() value
- `patch_id`: only in Mode 1 (incremented on each button press)
- `encoder_left/right`: only in Mode 3
- `s0-s4`: raw sensor readings (ADC 0-1023 or RC timing in μs)

**End marker**: `# END` sent when mode exits

### 5.3 Configuration via Serial

Commands sent from PC to robot:
- `M<n>` — set mode
- `N<n>` — set samples per burst (default 50)
- `I<n>` — set read interval in ms (default 10)
- `S` — start current mode
- `X` — stop/abort current mode
- `E0` / `E1` — emitters off/on

---

## 6. Python Data Collection & Visualisation

### 6.1 Serial Logger Script (`log_sensors.py`)

**Purpose**: Capture serial data from robot and save to timestamped CSV files

**Features**:
- Auto-detect serial port or accept as argument
- Open serial at 115200 baud
- Log all received lines to a CSV file with filename: `sensor_data_YYYYMMDD_HHMMSS.csv`
- Display live readings in terminal
- Accept keyboard input to send commands to robot (mode changes, start/stop)
- Handle clean shutdown (Ctrl+C saves and closes file)

**Dependencies**: `pyserial` only (standard library otherwise)

### 6.2 Analysis Script (`analyse_sensors.py`)

**Purpose**: Process logged CSV data and generate plots

#### Plot 1: Response Curve (ADC vs Shade)
- X-axis: Grayscale value (0=black, 255=white) — derived from patch number
- Y-axis: Raw ADC reading (0-1023)
- One line per sensor (5 lines, colour-coded)
- Error bars: ±1 standard deviation from repeated measurements
- Overlay: theoretical hyperbolic model fit

#### Plot 2: Noise Distribution per Shade
- Subplot grid (5×5 or similar)
- Each subplot: histogram of N readings for one shade level
- Title shows mean ± std
- Colour-coded by shade level

#### Plot 3: Sensor-to-Sensor Variation
- X-axis: Shade level
- Y-axis: ADC reading
- Box plot for each sensor at each shade level
- Highlights sensor-to-sensor manufacturing variation

#### Plot 4: Continuous Gradient Response
- X-axis: Position along gradient (from encoder counts → mm)
- Y-axis: ADC reading
- One line per sensor
- Overlay: expected linear gradient
- Highlights the nonlinear sensor response

#### Plot 5: Ambient Light Comparison
- Same as Plot 1 but with multiple curves per lighting condition
- Shows shift in absolute readings
- Evaluates whether relative changes (gradient slope) are preserved

#### Plot 6: Signal-to-Noise Ratio vs Shade
- X-axis: Shade level
- Y-axis: SNR = mean / std_dev (or ΔADCper_shade_step / noise_std)
- Identifies the shade range with best discrimination ability
- Critical for optimal gradient design

#### Plot 7: ADC vs RC Comparison (if Phase 4 completed)
- Two response curves overlaid
- Highlights which method offers better linearity and dynamic range

#### Plot 8: Transfer Function Fit
- Raw data points + fitted model curve
- Report model parameters and R² goodness of fit
- Candidate models:
  - Hyperbolic: `ADC = a / (1 + b × ρ) + c`
  - Power law: `ADC = a × ρ^b + c`
  - Logarithmic: `ADC = a × ln(ρ + d) + c`

**Dependencies**: `matplotlib`, `numpy`, `scipy` (for curve fitting), `pandas`

---

## 7. Step-by-Step Implementation Plan

### Step 1: Prepare Test Materials
- [ ] Print "25 shades of gray.pdf" — ideally on a **laser printer** (inkjet inks may not reflect IR the same as they absorb visible light)
- [ ] Print "gradient1.pdf" and "gradient2.pdf"
- [ ] **Important**: Note the exact printer model and paper used — ink chemistry affects IR reflectance differently than visible reflectance. A patch that looks gray to the eye may appear very different to an IR sensor
- [ ] Measure and mark the gradient maps with position markers (every 10-20mm)
- [ ] Prepare at least 2 lighting conditions (e.g., room lights on, room lights off with only USB power LED)

### Step 2: Create Arduino Test Sketch
- [ ] Create a new sketch file (e.g., `LineSensorTest.ino`) or a dedicated test mode
- [ ] Implement Mode 1 (static burst): button-triggered N-sample burst
- [ ] Implement Mode 2 (continuous logging): timed sensor reads
- [ ] Implement Mode 3 (drive + log): slow drive with position tracking
- [ ] Implement Mode 4 (emitter test): compare emitters on/off
- [ ] Set serial baud to 115200
- [ ] Test compilation

### Step 3: Create Python Logger
- [ ] Write `log_sensors.py` using `pyserial`
- [ ] Test serial connection and data capture
- [ ] Verify CSV output format

### Step 4: Run Phase 1 — Static Discrete Measurements
- [ ] Position robot over each patch of "25 shades of gray"
- [ ] Collect 50 readings per patch per sensor
- [ ] Save data to CSV
- [ ] Repeat with different ambient lighting (at least 2 conditions)
- [ ] Repeat with emitters OFF (ambient-only baseline)

### Step 5: Run Phase 2 — Continuous Gradient Sweep
- [ ] Place robot at black end of gradient2
- [ ] Slowly push/drive robot along gradient while logging
- [ ] Repeat 3-5 times for averaging
- [ ] Repeat on gradient1 (all 3 strips)

### Step 6: Run Phase 3 — Temporal Noise Analysis
- [ ] Place robot on mid-gray patch, stationary
- [ ] Log for 60 seconds continuously
- [ ] Repeat on white and black patches

### Step 7: Analyse Data (Python)
- [ ] Run analysis script to generate all plots
- [ ] Fit transfer function model to data
- [ ] Calculate SNR across shade range
- [ ] Identify optimal gradient slope region
- [ ] Compare ADC vs RC if Phase 4 data available

### Step 8: Optimise Map Gradient Design
- [ ] Using the fitted transfer function, compute the **inverse mapping**: for a desired linear ADC response along the gradient, what grayscale values should be printed at each position?
- [ ] Generate an optimised gradient PDF where the shade at each position is chosen to linearise the sensor response
- [ ] Print and test the optimised gradient

### Step 9: (Optional) Implement RC Timing
- [ ] Implement `readSensorsDigital()` in LineSensors.h
- [ ] Repeat Phase 1 and 2 with RC mode
- [ ] Compare to ADC mode in analysis

### Step 10: Document Results
- [ ] Write up findings with key plots
- [ ] Record calibration constants and transfer function for use in robot code
- [ ] Update Project_log.md

---

## 8. Critical Considerations

### 8.1 IR vs Visible Reflectance
**This is the most important caveat**: The printed grayscale values are specified in visible light. IR reflectance of printer toner/ink is NOT necessarily proportional to visible reflectance. Carbon-based toners (laser printers) tend to absorb IR well, giving a reasonable correlation. Inkjet dyes can be nearly IR-transparent regardless of visible colour. **Always verify with actual measurements.**

### 8.2 Sensor Height
The sensors are mounted at a fixed height on the 3Pi+ chassis (~3mm from ground). The Pololu documentation notes dramatic sensitivity reduction with distance:
- At 1/8" (3.2mm): ~4.5V range between black and white
- At 3/8" (9.5mm): ~1.2V range

The fixed mounting height means this is not a variable, but surface unevenness (paper curl, table flex) will affect readings.

### 8.3 Sensor Spacing
The 5 sensors are spaced along the front edge. For gradient work, the **centre sensor (DN3)** is the most useful for single-point measurements. Multi-sensor readings could be used for differential gradient detection (measuring the gradient slope by comparing adjacent sensor readings).

### 8.4 ADC Reference
The ATmega32U4 ADC uses Vcc (5V from USB) as reference by default. If powering from batteries vs USB, the reference voltage may differ slightly, shifting all readings. For consistent results, always use the same power source.

### 8.5 Reading Speed
`analogRead()` on ATmega32U4 takes approximately 100μs per conversion. Reading all 5 sensors sequentially takes ~500μs. At 10ms intervals, this is well within budget but limits maximum sample rate to ~2kHz per sensor if needed.

### 8.6 Motor Noise
The Pololu documentation explicitly warns about motor-generated electrical noise affecting sensor readings. For best results:
- Take static measurements with motors OFF
- For drive-mode measurements, use averaging (multiple reads per reported value)
- Consider adding software low-pass filtering

---

## 9. Expected Outcomes

1. **Response curve**: Expect a nonlinear (hyperbolic/sigmoidal) curve with maximum sensitivity in the mid-gray range and compression at both extremes
2. **Noise**: Expect ±2-5 ADC counts for static readings; more with motors running
3. **Ambient light**: Expect a DC offset shift but preserved shape — the gradient slope should be ambient-light-independent if using relative (calibrated) readings
4. **Sensor variation**: Expect ~5-10% variation between sensors due to component tolerances
5. **RC vs ADC**: Expect RC mode to show better dynamic range (22:1 time ratio vs ~10:1 voltage ratio) and better dark-end discrimination, but more implementation complexity
6. **Optimal gradient**: The analysis should reveal whether a printed gradient with ~10-15 distinguishable levels across 200-300mm is feasible, which would provide ~15-20mm position resolution from a single sensor reading
