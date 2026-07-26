# logging/logger.py
"""
CSV and KML logging
"""

import csv
import os
from datetime import datetime
from typing import Optional


class Logger:
    """Simple data logger"""
    
    def __init__(self, base_dir: str = None):
        if base_dir is None:
            base_dir = os.path.expanduser("~/UAVXLogs")
        self.base_dir = base_dir
        os.makedirs(base_dir, exist_ok=True)
        self.csv_file: Optional[object] = None
        self.csv_writer: Optional[object] = None
    
    def start_log(self, prefix: str = "flight", path: Optional[str] = None):
        """Start a new log file"""
        if path:
            filename = path
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.base_dir}/{timestamp}_{prefix}.csv"
        os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        headers = [
            "Time", "Alt", "Roc", "Heading", "BattV", "BattA",
            "Lat", "Lon", "GPS_Alt", "Sats", "Fix",
            "Roll", "Pitch", "Yaw",
            "Thr", "Nav", "WP"
        ]
        self.csv_writer.writerow(headers)
        self.csv_file.flush()  # make the header visible immediately on start

        return filename
    
    def log_flight_data(self, data):
        """Log flight data"""
        if not self.csv_writer:
            return
        
        row = [
            getattr(data, 'mission_time', 0),
            getattr(data, 'altitude', 0),
            getattr(data, 'roc', 0),
            getattr(data, 'heading', 0),
            getattr(data, 'battery_volts', 0),
            getattr(data, 'battery_current', 0),
            getattr(data, 'gps_lat', 0),
            getattr(data, 'gps_lon', 0),
            getattr(data, 'gps_altitude', 0),
            getattr(data, 'gps_sats', 0),
            getattr(data, 'gps_fix', 0),
            getattr(data, 'angle_roll', 0),
            getattr(data, 'angle_pitch', 0),
            getattr(data, 'angle_yaw', 0),
            getattr(data, 'desired_throttle', 0),
            getattr(data, 'nav_state', 0),
            getattr(data, 'curr_wp', 0),
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()
    
    def close(self):
        """Close log file"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None


class GpsKmlLogger:
    """GPS track logger writing KML (Google Earth)"""

    def __init__(self):
        self.points = []
        self.active = False
        self.filepath = None

    def start(self, filepath=None):
        self.points = []
        if not filepath:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(os.path.expanduser("~/UAVXLogs"), f"{timestamp}_gps_track.kml")
        self.filepath = filepath
        os.makedirs(os.path.dirname(self.filepath) or ".", exist_ok=True)
        self.active = True

    def add_point(self, lat, lon, alt, heading, speed, timestamp):
        if self.active and lat != 0.0 and lon != 0.0:
            self.points.append((lon, lat, alt, heading, speed, timestamp))

    def stop(self):
        if not self.active:
            return None
        self.active = False
        if not self.points:
            return None
        coords = "\n".join(
            f"          {p[0]:.7f},{p[1]:.7f},{p[2]:.1f}"
            for p in self.points
        )
        kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>UAVX Flight Track</name>
  <Placemark>
    <name>Track</name>
    <LineString>
      <extrude>1</extrude>
      <altitudeMode>absolute</altitudeMode>
      <coordinates>
{coords}
      </coordinates>
    </LineString>
  </Placemark>
</Document>
</kml>
"""
        with open(self.filepath, 'w') as f:
            f.write(kml)
        return self.filepath
