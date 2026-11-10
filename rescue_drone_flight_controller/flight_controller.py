"""Search-and-rescue mission controller for a *simulated* PX4 quadcopter.

This module handles high-level commands; PX4 handles low-level stabilization.
It intentionally does NOT control a real aircraft or replace flight firmware.
"""

import argparse
import asyncio
import json
import logging
from dataclasses import dataclass
from enum import Enum
from math import asin, cos, radians, sin, sqrt
from pathlib import Path
from typing import Optional, Protocol

LOG = logging.getLogger("rescue_drone")


class MissionState(str, Enum):
    IDLE = "IDLE"
    READY = "READY"
    TAKING_OFF = "TAKING_OFF"
    SEARCHING = "SEARCHING"
    TARGET_FOUND = "TARGET_FOUND"
    RETURNING = "RETURNING"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class Waypoint:
    north_m: float
    east_m: float
    altitude_m: float = 5.0  # Relative to the simulated home location


@dataclass(frozen=True)
class Detection:
    person_detected: bool
    confidence: float
    source: str = "unknown"


class Vehicle(Protocol):
    async def connect(self) -> None: ...
    async def takeoff(self, altitude_m: float) -> None: ...
    async def go_to(self, waypoint: Waypoint) -> None: ...
    async def return_to_launch(self) -> None: ...


class DryRunVehicle:
    """Prints simulated commands; no physical aircraft and no 3D engine."""

    def __init__(self):
        self.commands = []

    async def connect(self):
        self.commands.append("connect")
        LOG.info("[DRY RUN] Connected to virtual vehicle")

    async def takeoff(self, altitude_m):
        self.commands.append("takeoff")
        LOG.info("[DRY RUN] Arm and take off to %.1f m", altitude_m)

    async def go_to(self, waypoint):
        self.commands.append("waypoint")
        LOG.info(
            "[DRY RUN] Fly to north=%.1f m, east=%.1f m, altitude=%.1f m",
            waypoint.north_m, waypoint.east_m, waypoint.altitude_m,
        )

    async def return_to_launch(self):
        self.commands.append("rtl")
        LOG.info("[DRY RUN] Request return to launch (no actual landing simulated)")


class PX4SITLVehicle:
    """Uses MAVSDK gRPC to command PX4 Software-in-the-Loop over local UDP.

    ONLY use while a PX4/Gazebo simulation is running. Its actions would also
    affect a real MAVLink-enabled vehicle if you point it at one.
    """

    def __init__(self):
        self.drone = None
        self.home_lat = None
        self.home_lon = None
        self.home_alt_msl = None

    async def connect(self):
        try:
            from mavsdk_grpc import System
        except ImportError as exc:
            raise RuntimeError(
                "Install SITL dependency first: pip install -r requirements-sitl.txt"
            ) from exc
        self.drone = System()
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

        async def wait_for_connection():
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    return

        async def wait_for_position():
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    return

        LOG.info("Connecting to PX4 SITL at UDP port 14540...")
        await asyncio.wait_for(wait_for_connection(), timeout=30)
        LOG.info("Connected; waiting for simulated GPS/home position...")
        await asyncio.wait_for(wait_for_position(), timeout=90)
        async for home in self.drone.telemetry.home():
            self.home_alt_msl = home.absolute_altitude_m
            break
        async for position in self.drone.telemetry.position():
            self.home_lat = position.latitude_deg
            self.home_lon = position.longitude_deg
            break
        LOG.info("Simulated home recorded")

    async def takeoff(self, altitude_m):
        await self.drone.action.set_takeoff_altitude(altitude_m)
        LOG.info("Arming simulated drone")
        await self.drone.action.arm()
        LOG.info("Taking off to %.1f m", altitude_m)
        await self.drone.action.takeoff()

        async def wait_for_altitude():
            async for pos in self.drone.telemetry.position():
                if pos.relative_altitude_m >= altitude_m - 1.2:
                    return

        await asyncio.wait_for(wait_for_altitude(), timeout=40)

    async def go_to(self, waypoint):
        # Convert small local north/east offsets to a nearby GPS coordinate.
        # Approximate conversion is adequate for this short *virtual* mission.
        lat = self.home_lat + waypoint.north_m / 111_111.0
        lon = self.home_lon + waypoint.east_m / (
            111_111.0 * cos(radians(self.home_lat))
        )
        altitude_msl = self.home_alt_msl + waypoint.altitude_m
        LOG.info("Sending simulated goto north=%.1f east=%.1f", waypoint.north_m, waypoint.east_m)
        await self.drone.action.goto_location(lat, lon, altitude_msl, 0)

        async def wait_for_arrival():
            async for position in self.drone.telemetry.position():
                distance_m = haversine_m(
                    lat, lon, position.latitude_deg, position.longitude_deg
                )
                if distance_m <= 3.0 and abs(position.absolute_altitude_m - altitude_msl) <= 3.0:
                    return

        await asyncio.wait_for(wait_for_arrival(), timeout=75)
        LOG.info("Reached simulated waypoint")

    async def return_to_launch(self):
        LOG.info("Requesting return-to-launch in PX4 SITL")
        await self.drone.action.return_to_launch()


def haversine_m(lat1, lon1, lat2, lon2):
    radius_m = 6_371_000
    delta_lat = radians(lat2 - lat1)
    delta_lon = radians(lon2 - lon1)
    a = (sin(delta_lat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(delta_lon / 2) ** 2)
    return 2 * radius_m * asin(min(1.0, sqrt(a)))


class DetectionFeed:
    """Integration point for the other teammate's detection software.

    Option A: teammate writes a JSON event to detection_event.json.
    Option B: use --demo-detection-at for a clearly labeled mock event.
    File is checked only on arrival at each waypoint in this initial version.
    """

    def __init__(self, file_path: Path, demo_detection_at: Optional[int] = None):
        self.file_path = file_path
        self.demo_detection_at = demo_detection_at

    async def check(self, waypoint_number: int) -> Optional[Detection]:
        if self.demo_detection_at == waypoint_number:
            LOG.info("[MOCK DETECTOR] Test person-detection event at waypoint %d", waypoint_number)
            return Detection(person_detected=True, confidence=0.96, source="mock")

        if self.file_path.exists():
            content = json.loads(self.file_path.read_text(encoding="utf-8"))
            # Only a true detection counts. No false claims of vision inference.
            return Detection(
                person_detected=content.get("person_detected") is True,
                confidence=float(content.get("confidence", 0)),
                source=str(content.get("source", "external")),
            )
        return None


DEFAULT_ROUTE = [
    Waypoint(north_m=8, east_m=0),
    Waypoint(north_m=8, east_m=16),
    Waypoint(north_m=0, east_m=16),
    Waypoint(north_m=0, east_m=0),
]


class MissionController:
    def __init__(self, vehicle: Vehicle, detector: DetectionFeed, confidence_threshold: float = 0.8):
        self.vehicle = vehicle
        self.detector = detector
        self.confidence_threshold = confidence_threshold
        self.state = MissionState.IDLE
        self.visited_waypoints = 0
        self.airborne = False

    def set_state(self, state: MissionState):
        self.state = state
        LOG.info("MISSION STATE: %s", state.value)

    async def run(self):
        await self.vehicle.connect()
        self.set_state(MissionState.READY)
        try:
            self.set_state(MissionState.TAKING_OFF)
            # Assume takeoff may have started even if its confirmation times out.
            # An RTL attempt is then safer than silently exiting the demo.
            self.airborne = True
            await self.vehicle.takeoff(altitude_m=5)
            self.set_state(MissionState.SEARCHING)
            for index, waypoint in enumerate(DEFAULT_ROUTE, start=1):
                await self.vehicle.go_to(waypoint)
                self.visited_waypoints += 1
                event = await self.detector.check(index)
                if event and event.person_detected and event.confidence >= self.confidence_threshold:
                    LOG.info("DETECTION ALERT: source=%s confidence=%.2f", event.source, event.confidence)
                    self.set_state(MissionState.TARGET_FOUND)
                    break
            self.set_state(MissionState.RETURNING)
            await self.vehicle.return_to_launch()
            # RTL accepted; PX4 completes its landing asynchronously. In dry-run
            # it is only a printed command. Do not claim 'landed' here.
            self.set_state(MissionState.COMPLETED)
        except Exception:
            self.set_state(MissionState.ABORTED)
            LOG.exception("Mission error")
            if self.airborne:
                try:
                    await self.vehicle.return_to_launch()
                except Exception:
                    LOG.exception("Return-to-launch was not accepted; operator intervention required")
            raise


def parse_args():
    parser = argparse.ArgumentParser(description="Search-and-rescue drone mission-control prototype")
    parser.add_argument("--mode", choices=["dry-run", "sitl"], default="dry-run")
    parser.add_argument("--demo-detection-at", type=int, choices=[1, 2, 3, 4])
    parser.add_argument("--detection-file", type=Path, default=Path("detection_event.json"))
    return parser.parse_args()


async def main():
    args = parse_args()
    vehicle = DryRunVehicle() if args.mode == "dry-run" else PX4SITLVehicle()
    detector = DetectionFeed(args.detection_file, args.demo_detection_at)
    controller = MissionController(vehicle, detector)
    await controller.run()
    LOG.info("Waypoints visited: %d of %d", controller.visited_waypoints, len(DEFAULT_ROUTE))
    LOG.info("Completed means RTL command accepted; landing must be monitored separately")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, asyncio.CancelledError):
        LOG.warning("Interrupted. If SITL is airborne, use QGroundControl/PX4 to land safely.")
