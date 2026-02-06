// =================================================================================
// ESP32 SOLAR TRACKER FIRMWARE DOCUMENTATION
// =================================================================================
//
// Version:      1.0
// Created:      27-Jan-2025
// Platform:     ESP32 (PlatformIO / Arduino Framework)
// Architecture: FreeRTOS Multi-Tasking (Dual Core)
//
// =================================================================================
// 1. REQUIRED EXTERNAL LIBRARIES
// =================================================================================
// These third-party libraries provide the foundational capabilities for networking,
// timekeeping, and solar mathematics.
//
// 1.1 NTPClient & WiFiUdp
// ---------------------------------------------------------------------------------
// - Purpose: Retrieves the precise Coordinated Universal Time (UTC) from internet
//   time servers (pool.ntp.org).
// - Why it is needed: The ESP32 does not have a battery-backed Real Time Clock (RTC).
//   Accurate time is mathematically required to calculate the sun's position relative
//   to the earth.
// - Referenced by: main.cpp (NetworkTask), SolarWebServer (indirectly via Globals).
//
// 1.2 ESPAsyncWebServer & AsyncJson
// ---------------------------------------------------------------------------------
// - Purpose: Creates a non-blocking (asynchronous) HTTP server.
// - Why it is needed: Unlike the standard WebServer.h, this library does not pause
//   the CPU while waiting for a client request. This ensures that motor control tasks
//   and mathematical calculations are never interrupted by network traffic.
// - Referenced by: SolarWebServer.h, SolarWebServer.cpp.
//
// 1.3 SolarPosition
// ---------------------------------------------------------------------------------
// - Purpose: A mathematical library that inputs a specific time, latitude, and
//   longitude to output the Solar Azimuth (horizontal angle) and Elevation
//   (vertical angle).
// - Referenced by: main.cpp (SolarTask), SolarWebServer.cpp (for object re-initialization).
//
// 1.4 ArduinoJson
// ---------------------------------------------------------------------------------
// - Purpose: Handles parsing and serialization of JSON data.
// - Why it is needed: Used to decode the configuration payload sent from the Android
//   App and to format the status response sent back to the phone.
// - Referenced by: SolarWebServer.cpp.
//
// =================================================================================
// 2. CUSTOM CLASSES
// =================================================================================
// These classes were created to separate hardware drivers and network logic from 
// the main application loop.
//
// 2.1 MotorDriver (MotorDriver.h / .cpp)
// ---------------------------------------------------------------------------------
// - Purpose: Encapsulates all hardware-level control for the tracking mechanism. It
//   abstracts the physical pins away from the main logic.
// - Current Functionality: Manages the visual feedback system (LED patterns) to
//   indicate system state.
// - Public Methods:
//   * signalWaiting(): Blocking pattern (Triple Blink) indicating the system is
//     locked/uninitialized.
//   * signalTracking(): Non-blocking pattern (Pulse) indicating normal operation.
//   * initiateCleaningCycle(): Executes a blocking motor sequence to wipe the panel.
// - Referenced by: main.cpp (MotorTask).
//
// 2.2 SolarWebServer (SolarWebServer.h / .cpp)
// ---------------------------------------------------------------------------------
// - Purpose: Manages the WiFi connection lifecycle, mDNS service discovery, and API
//   route definitions. It acts as the "Controller" in the MVC pattern, receiving data
//   updates and modifying the global state variables.
// - Key Responsibilities:
//   * Hosting the /status endpoint (System Telemetry).
//   * Hosting the /update endpoint (The "Gatekeeper Key" that unlocks the main loop).
// - Referenced by: main.cpp (Setup).
//
// 2.3 Globals.h
// ---------------------------------------------------------------------------------
// - Purpose: Acts as the "Shared Context" for the application. It declares 'extern'
//   variables so that main.cpp, SolarWebServer.cpp, and other modules can read/write
//   the same memory locations without circular dependency errors.
// - Key Variables: isSystemInitialized (The main loop protection flag), currentLat, currentLon.
//
// 2.4 SensorInputManager (SensorInputManager.h / .cpp)
// ---------------------------------------------------------------------------------
// - Purpose: Centralizes the reading of all environmental sensors (Current, IR).
//   It normalizes raw pin data into usable values (Amps, Booleans).
// - Functionality:
//   * Instantiated globally in main.cpp.
//   * Polled continuously inside the high-priority MotorTask.
//   * isCurrentBelowThreshold(): Returns true if solar output < 0.5A (Dirty Panel).
//   * areIRSensorsReflected(): Returns true if both IR sensors detect an object
//     (Debris on panel).
// - Public Methods:
//   * update(): Reads analog/digital pins. Must be called in a loop.
//   * isSafeToMove(): Returns 'false' if Stall Current detected or Limit Switch hit.
// - Referenced by: main.cpp (MotorTask).
//
// =================================================================================
// 3. EXECUTION TIMELINE (THE "THREAD OF EXECUTION")
// =================================================================================
// This section traces the chronological flow of the program from power-on to full
// operation.
//
// PHASE 1: INITIALIZATION & PRE-CONNECTION HOUSEKEEPING
// Context: Static Memory Allocation (Before setup() runs).
// ---------------------------------------------------------------------------------
// 3.1.1 Global Variable Creation (main.cpp)
//   - The compiler allocates memory for the critical state flags, most importantly
//     isSystemInitialized = false.
//   - This default 'false' state is the "Gatekeeper," preventing the system from
//     moving motors until explicit user authorization is received.
//
// 3.1.2 Module Instantiation (main.cpp)
//   - MotorDriver motorSystem(2); -> The constructor saves Pin 2 as the target output.
//   - SolarWebServer webSystem(SSID, PASSWORD); -> The constructor stores the WiFi
//     credentials but does *not* connect yet.
//
// PHASE 2: THE SETUP SEQUENCE (SINGLE CORE)
// Context: void setup() running on Core 1.
// ---------------------------------------------------------------------------------
// 3.2.1 Serial Debugging Start
//   - Initializes UART communication at 115200 baud for debugging.
//
// 3.2.2 Subsystem Boot (webSystem.begin())
//   - Control transfers to SolarWebServer.cpp.
//   - The ESP32 attempts to connect to the WiFi Hotspot (Blocking loop until connected).
//   - Once connected, it starts the mDNS responder (esp32-solar.local).
//   - It defines the API routes (/status, /update) and starts listening for HTTP requests.
//   - *Control returns to main.cpp.*
//
// 3.2.3 RTOS Task Creation
//   - The system splits execution into three parallel threads (Tasks) using
//     xTaskCreatePinnedToCore.
//   - NetworkTask is pinned to Core 0 (leaving Core 1 free for math/motors).
//   - SolarTask and MotorTask are pinned to Core 1.
//
// PHASE 3: THE "GATEKEEPER" STATE (WAITING FOR USER)
// Context: loop() has been deleted. The three tasks are now running in parallel.
// ---------------------------------------------------------------------------------
// 3.3.1 NetworkTask (Core 0)
//   - Continuously polls NTP servers to keep the internal system time accurate.
//     It runs independently of the other tasks.
//
// 3.3.2 SolarTask (Core 1)
//   - Status: BLOCKED.
//   - It sits in a while (!isSystemInitialized) loop. It consumes minimal CPU cycles
//     while checking this flag once per second. It performs no calculations yet because
//     it lacks GPS coordinates.
//
// 3.3.3 MotorTask (Core 1)
//   - Status: BLOCKED (Visually Active).
//   - It sits in its own while (!isSystemInitialized) loop.
//   - It calls motorSystem.signalWaiting(), causing the LED to blink a "Triple Blip"
//     pattern. This visually informs the user: "I am online, but I need GPS data."
//
// PHASE 4: THE HANDSHAKE (USER INTERACTION)
// Context: Asynchronous Event triggered by Android App.
// ---------------------------------------------------------------------------------
// 3.4.1 The Trigger
//   - User presses "Sync" on the Android App.
//   - App sends a POST request to http://esp32-solar.local/update with JSON payload:
//     {"lat": 45.50, "lon": -73.56}.
//
// 3.4.2 The Unlock (SolarWebServer.cpp)
//   - The Web Server parses the JSON.
//   - It updates the global currentLat and currentLon.
//   - CRITICAL STEP: It sets isSystemInitialized = true.
//   - It sends a 200 OK JSON response back to the phone.
//
// PHASE 5: NORMAL OPERATION (TRACKING MODE)
// Context: The isSystemInitialized flag is now 'true'.
// ---------------------------------------------------------------------------------
// 3.5.1 SolarTask Unlocks
//   - The while loop condition fails. The task enters its main operations loop.
//   - Every 5 seconds, it fetches the current Epoch Time, calculates the
//     Azimuth/Elevation, and updates the global variables.
//
// 3.5.2 MotorTask Unlocks
//   - The while loop condition fails. The task enters its main operations loop.
//   - It switches the LED pattern from "Triple Blip" to "Slow Pulse" (Heartbeat),
//     visually confirming to the user that tracking has begun.
//   - The task continuously polls sensorSystem.update().
//   - Automated Maintenance:
//     If (Low Current OR Debris Detected) AND (Cooldown Passed):
//       -> Execute initiateCleaningCycle()
//   - If no cleaning needed, it proceeds to standard tracking (Pulse LED).
//   - Future Implementation: This loop will read the calculated Azimuth from
//     SolarTask and step the motors to match that angle.
// =================================================================================