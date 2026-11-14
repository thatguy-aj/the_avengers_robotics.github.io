# INAV SITL — Square Waypoint Planner (experimental)

This is the **INAV 7 + Windows Python/Tkinter** companion to our search-and-rescue drone project. It preserves the working RealFlight-protocol motor-driven simulation and **adds an on-screen square mission plan and waypoint arrival tracker**.

> **This is a simulation-only visualization and manual waypoint-tracking prototype, not autonomous INAV NAV WP, not GPS navigation, and not a tested real-aircraft flight controller.** It does not write commands to any physical drone. Do not represent the drawn route as a route that the aircraft automatically flies.

## What's new

- Home → WP1 → WP2 → WP3 → Home **square mission overlay** on a top-down local virtual map.
- Marker colors for upcoming, current, and reached waypoints.
- Current virtual position, altitude, and distance to the next waypoint.
- **Start route tracking / Pause tracking / Reset route progress** buttons.
- Arrival tracking uses the simulated aircraft's **actual computed X/Y position** (derived from INAV motor outputs) and checks that altitude is at least 3 m; points are **not** marked complete by a scripted animation.
- Existing keyboard controls, ARM CH5 input, and motor-driven physics remain.

## Route (local virtual meters)

| Stop | East X (m) | North Y (m) | Planned altitude (m) |
| --- | ---: | ---: | ---: |
| HOME (launch) | 0 | 0 | Ground |
| WP1 | 0 | 15 | 5 |
| WP2 | 20 | 15 | 5 |
| WP3 | 20 | 0 | 5 |
| HOME (return above launch) | 0 | 0 | 5 |

**Waypoint acceptance:** within 2 m horizontally and at least 3 m above the virtual ground. Planned 5 m cruise altitude is a **goal shown on the route**, not enforced by altitude control. Reaching HOME does not automatically land.

## Windows setup

1. Start the working **INAV 7 SITL / RealFlight** configuration used in the previous flight demo. **Use exactly one `inav_SITL.exe` process.** Multiple instances previously produced alternating zero/nonzero motor messages.
2. Close the old green-grid simulator before starting this file, because only one Python simulator should listen on `127.0.0.1:18083`.
3. Save `inav_square_waypoints.py` to your Windows **Downloads** folder.
4. In PowerShell, run:

   ```powershell
   & "$env:LOCALAPPDATA\Programs\Python\Python313-arm64\python.exe" "$env:USERPROFILE\Downloads\inav_square_waypoints.py"
   ```

   If Python is installed elsewhere, use the correct full path to your `python.exe` instead.

5. Look for **CONNECTED: INAV SITL is exchanging motor signals** and increasing exchange-packet numbers.
6. Click **Start route tracking (manual flight)**. ARM using **A**, then use **W/S** for throttle and the **arrow keys** to test manual lateral control. Keep the virtual altitude and displayed mission progress in view. **Tracking is manual, not automatic guidance.**
7. Press **A** to disarm; this also resets throttle to zero. Use **Reset route progress** to retry the planned route.

### Keyboard shortcuts

| Key | Function |
| --- | --- |
| A | Toggle CH5 arming switch; throttle resets to zero |
| W / S | Increase / decrease throttle |
| Arrow keys | Manual pitch/roll commands while held |
| R | Reset position **only when disarmed** |

## Offline self-test

No SITL or GUI connection is needed for this test:

```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python313-arm64\python.exe" "$env:USERPROFILE\Downloads\inav_square_waypoints.py" --self-test
```

The self-test covers the square route, waypoint arrival safeguards, reset behavior, and SOAP/RC response. It does **not** prove live waypoint navigation.

## Team repository placement

Put this folder under:

`rescue_drone_flight_controller/inav_sitl/`

The existing `rescue_drone_flight_controller/flight_controller.py` uses **PX4/Gazebo** and should stay intact. These are separate experiments with different flight stacks. The new INAV simulator should **not** be used as evidence that PX4 and INAV are already integrated.

## Limitations / next milestone

- The route is visible and progress is measured, but the drone **does not follow waypoints autonomously**.
- The simulator physics is intentionally approximate; position, altitude, and direction are not real-world GPS or validated flight dynamics.
- Before claiming a full route flight, verify in the Windows/SITL setup that lateral RC controls move the virtual vehicle in the intended direction and that each waypoint is genuinely reached.
- Next milestone: add and validate a simulation-only guidance controller, or configure **true INAV NAV WP** with suitable simulated GPS/navigation signals, while retaining manual override and safe disarm.
