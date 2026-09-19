from dataclasses import dataclass, field
from typing import List


WP_ACTIONS = {
    0: "Via",
    1: "Orbit",
    2: "Perch",
    3: "POI",
    4: "Land",
    5: "GeoVertex",
}


WP_ACTION_VALUES = {v: k for k, v in WP_ACTIONS.items()}


@dataclass
class Waypoint:
    index: int = 0
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    velocity: float = 0.0
    loiter: float = 0.0
    orbit_radius: float = 0.0
    orbit_alt: float = 0.0
    orbit_velocity: float = 0.0
    pulse_width: int = 0
    pulse_period: int = 0
    action: str = "Via"

    @classmethod
    def from_packet(cls, data: dict) -> "Waypoint":
        return cls(
            index=data.get("wp_index", 0),
            lat=data.get("wp_lat", 0.0),
            lon=data.get("wp_lon", 0.0),
            alt=data.get("wp_alt", 0),
            velocity=data.get("wp_velocity", 0.0),
            loiter=data.get("wp_loiter", 0),
            orbit_radius=data.get("wp_orbit_radius", 0),
            orbit_alt=data.get("wp_orbit_alt", 0),
            orbit_velocity=data.get("wp_orbit_velocity", 0.0),
            pulse_width=data.get("wp_pulse_width", 0),
            pulse_period=data.get("wp_pulse_period", 0),
            action=WP_ACTIONS.get(data.get("wp_action", 0), "Via"),
        )

    def to_dict(self) -> dict:
        return {
            "index": self.index,
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "velocity": self.velocity,
            "loiter": self.loiter,
            "orbit_radius": self.orbit_radius,
            "orbit_alt": self.orbit_alt,
            "orbit_velocity": self.orbit_velocity,
            "pulse_width": self.pulse_width,
            "pulse_period": self.pulse_period,
            "action": self.action,
        }
