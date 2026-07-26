# core/data_manager.py
"""
Shared data manager for all windows
Singleton pattern to share telemetry data
"""

from typing import Optional
from packet_parser import FlightData


class DataManager:
    """Singleton data manager for sharing telemetry data across windows"""
    
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DataManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self.flight_data: Optional[FlightData] = None
        self.nav_data: Optional[FlightData] = None
        self.calibration_data: Optional[dict] = None
        self._observers = []
    
    def update_flight_data(self, data: FlightData):
        """Update flight data and notify observers"""
        self.flight_data = data
        self._notify_observers()
    
    def update_nav_data(self, data: FlightData):
        """Update navigation data"""
        self.nav_data = data
    
    def get_flight_data(self) -> Optional[FlightData]:
        """Get current flight data"""
        return self.flight_data
    
    def get_nav_data(self) -> Optional[FlightData]:
        """Get current navigation data"""
        return self.nav_data

    def update_calibration_data(self, data: dict):
        """Update calibration packet data"""
        self.calibration_data = data

    def get_calibration_data(self) -> Optional[dict]:
        """Get current calibration packet data"""
        return self.calibration_data

    def register_observer(self, callback):
        """Register a callback to be called when data updates"""
        if callback not in self._observers:
            self._observers.append(callback)
    
    def unregister_observer(self, callback):
        """Unregister a callback"""
        if callback in self._observers:
            self._observers.remove(callback)
    
    def _notify_observers(self):
        """Notify all observers of data update"""
        for callback in self._observers:
            try:
                callback(self.flight_data)
            except Exception as e:
                print(f"Error notifying observer: {e}")

# Global instance
data_manager = DataManager()
