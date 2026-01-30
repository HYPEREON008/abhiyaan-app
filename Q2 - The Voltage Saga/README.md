
# Q2. The Voltage Saga  
## Part A: Power Play

1. Convertors

A switching regulator is a broad category of DC-DC converters that use a switching element (MOSFET) for efficient voltage conversion  
Buck Convertor is a specific type of switching regulator

Buck converters are highly efficient for large voltage drops and high power but introduce switching noise, while LDOs (Linear Regulators) offer cleaner, low-noise output with simplicity and low cost but are less efficient, wasting excess voltage as heat, making them ideal for small voltage differences or sensitive analog circuits

Src: [https://www.circuitbread.com/ee-faq/when-should-you-choose-ldo-or-buck-converter](https://www.circuitbread.com/ee-faq/when-should-you-choose-ldo-or-buck-converter)  
Src: [https://resources.altium.com/p/using-ldo-vs-switching-regulator-your-pcb](https://resources.altium.com/p/using-ldo-vs-switching-regulator-your-pcb)  
Src: [https://www.us.lambda.tdk.com/resources/blogs/20200731.html](https://www.us.lambda.tdk.com/resources/blogs/20200731.html)


**1: Linear regulators (LDOs) (low dropout regulators)**

**linear DC voltage regulators that efficiently step down input voltage to a regulated, lower output voltage, even when the input-output differential is very small (\< 1V)**

**Example**

* 26 V → 24 V linearly

* 26 V → 15 V → 5 V linearly

**but its bad in this case**

* Massive power loss  
* Huge heat sinks required

**2: Individual buck converters from battery (Flat architecture)**

Each rail gets its **own buck converter** directly from the battery

Battery (24–28V)

 ├─ Buck → 24V (Motor controllers) (or buck-boost)

 ├─ Buck → 24V (Encoders) (or buck-boost)

 ├─ Buck → 15V (Jetson Orin)

 └─ Buck → 5V  (MCU)

**Cascaded (multi-stage) power architecture**

**Step 1: Create a main regulated bus**

* Battery (24–28 V) → **Stable 24 V bus**  
* Use a **buck or buck-boost converter**

Buck-boost is safer if the battery can dip below 24 V. Though it has higher cost and lower efficiency than normal buck convertor

**Step 2: Derive secondary rails from the 24 V bus**

Battery (24–28V)  
 |  
Main DC-DC (Buck/Buck-Boost)  
 |  
24V Bus  
 ├─ Direct → Motor Controllers (24V)

 ├─ Direct → Encoders (24V)

 ├─ Buck → 15V → Jetson Orin

 └─ Buck → 5V  → Microcontroller

2. Switching Circuitry

**Benefits of independent switching**

**1\. Fault isolation**

* If one subsystem **shorts or fails**, you can **cut power only to that subsystem**

* Rest of the bot keeps running (or at least shuts down safely)

* Example: motor driver short → MCU still alive → emergency stop logic works

**2\. Power sequencing**

* Subsystems should turn on in a specific order**

  * MCU first → motor controllers later
  * Sensors stabilize → control starts

* Independent switches allow **controlled startup and shutdown**

**3\. Energy efficiency**

* Subsystems can be **turned off when not needed**

  * Motors off while idle
  * Sensors off when unused

**4\. Safety and debugging**

* You can:
  * Kill motors instantly (emergency stop)
  * Restart only one subsystem
  * Diagnose failures subsystem-wise

---

**1\. Mechanical switches and relays**

**Types**
* Toggle switches
* Electromechanical relays

**Cons**
* Bulky
* Slow
* Mechanical wear
* Not controllable by MCU easily

**Not suitable for internal control in a bot**, but okay for:

* Main power switch
* Emergency cutoff relay

**2\. BJTs (Bipolar Junction Transistors)**
**Pros**
* Cheap
* Easy to understand
* Solid-state (no mechanical wear)

**Cons**
* Continuous base current required
* Inefficient at high current
* Generates heat
* Not ideal for battery-powered systems

Rarely used for power switching in modern bots.

**3\. MOSFETs**

**Types**

* N-channel MOSFET

* P-channel MOSFET

**Why MOSFETs are ideal**

* Voltage-controlled → **no steady gate current**
* Very low ON resistance
* Fast switching
* High efficiency → **less heat**
* Compact, solid-state → **no mechanical wear**
* Easily controlled by MCU

**Typical uses**

* Motor power gating
* Sensor rail switching

**Configurations**

* Low-side switching (simpler, common)
  MOSFET between load and ground

>in low-side switching when the MOSFET is off, it is not fully isolating the load from ground due to body diode conduction.

* High-side switching (preferred for safety)
  MOSFET between power and load
  

**4\. DC-DC converters with Enable pin**

**Instead of just switching power:**

* **Use buck converters with enable pins**  
* **You get:**  
  * Voltage regulation
  * Power switching  
  * Efficiency

**Pros**

* Clean power rails  
* MCU-controlled  
* Efficiency

|Subsystem	| Switching Method Used |
| :--- | :---: |
|Battery main	| Fuse + Emergency relay / master switch |
|Motors	| High-current N-channel MOSFET or motor driver with EN pin |
|MCU + logic	| DC-DC buck converter with enable pin |
|Sensors	| Low-current MOSFET (high-side or low-side) |
|Communication modules	| MOSFET-based power switching |

**1 Overcurrent protection**
1. Main battery fuse:
a. Protects entire system from major shorts
b. Rated slightly above max expected current draw
2. Subsystem fuses
a. Motor driver fuse
b. MCU fuse
c. Sensor fuse
>Resettable PTC (Polyfuse) fuses can be used for convenience; they automatically reset after tripping but may have slower response times compared to traditional fuses.

3. Current Sensing
a. Current sense resistors (shunt resistors + op-amp)
b. Hall-effect current sensors
4. Circuit breakers (skip)

**2 Voltage Protection**
1. Under-voltage
a. Under-voltage lockout (UVLO) circuits
b. Brown-out detection (MCU feature)
2. Over-voltage
a. Zener diodes for clamping
b. TVS (transient voltage suppression) diodes for transient spikes

**3 Thermal Protection**
1. Temperature sensors on critical components
2. Thermal shutdown features in DC-DC converters
3. Heat sinks and thermal pads for high-power components

**4 Noise Protection and Signal Integrity**
1. Optocouplers for signal isolation
2. Isolated DC-DC converters for sensitive electronics
3. Separate power domains for noisy components (motors) vs sensitive components (MCU, sensors)
4. LC filters (high power and high frequency) and RC filters (low power and low frequency) for noise reduction
5. Proper grounding techniques (star ground, ground planes)
6. Decoupling/Bypass capacitors near IC power pins
7. Thick traces for high-current paths
## Part B: Death By Circuits

Choosing nrf24l01+PA+LNA module for wireless communication reasons explained in Q3
#### Hardware-Level Safety (to not rely only on software)
Wireless STOP signal triggers:
- Motor driver ENABLE pin = LOW
- Or a relay / MOSFET cutoff

MCU crash should not result in motors spinning
#### 

src: https://docs.cirkitdesigner.com/component/4acab6fe-65db-4dca-af6f-273ea71244f7
for reference of nrf24L01

manual calculations: https://www.ti.com/lit/an/slva477b/slva477b.pdf?ts=1769588984044&ref_url=https%253A%252F%252Fwww.bing.com%252F

WBDesign auto calculations: https://webench.ti.com/power-designer/switching-regulator?base_pn=LM2596&origin=ODS&litsection=application


Motor driver: BTN8982TA : https://www.infineon.com/assets/row/public/documents/10/49/infineon-btn8982ta-ds-en.pdf 

features:
- suitable voltaage and currents
- seems simple for me to work with
- has enable pin for safety

or maybe use:

Motor driver: DRV8412 : https://www.ti.com/product/DRV8412

MCU: MSPM0G3507 : https://www.ti.com/product/MSPM0G3507 : with CAN communication; spent some time in its datasheets previously.


