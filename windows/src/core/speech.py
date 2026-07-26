# core/speech.py

import threading
from enum import IntEnum
from typing import Optional


class SpeechLevel(IntEnum):
    OFF = 0
    ALERTS = 1
    STATUS = 2
    ALL = 3


LEVEL_LABELS = {
    SpeechLevel.OFF: "Off",
    SpeechLevel.ALERTS: "Alerts Only",
    SpeechLevel.STATUS: "Alerts + Status",
    SpeechLevel.ALL: "All",
}

try:
    import pyttsx3
    _has_speech = True
except (ImportError, OSError, RuntimeError):
    _has_speech = False

import platform


class SpeechController:
    # Preferred voices in order of preference
    _WINDOWS_VOICES = [
        "Microsoft Catherine",      # Win11 Natural Voice — calm, educated British female
        "Microsoft Hazel",          # SAPI5 British female
        "Microsoft Zira",           # SAPI5 US female
        "Microsoft Susan",          # SAPI5 British female (legacy)
    ]
    _LINUX_VOICES = [
        "en-gb+f3",                 # espeak-ng British English female 3 — gentle, clear
        "en-gb+f2",                 # espeak-ng British English female 2
        "mb-us1",                   # MBROLA US English female (natural, apt install mbrola mbrola-us1)
        "us-mbrola-1",              # MBROLA US English female (alt name)
        "en+f3",                    # espeak-ng US English female 3
        "en-gb",                    # espeak-ng British English default
    ]

    def __init__(self):
        self._engine = None
        self._level = SpeechLevel.ALL
        self._last_level = SpeechLevel.ALL
        if _has_speech:
            try:
                self._engine = pyttsx3.init()
                self._select_preferred_voice()
            except (OSError, RuntimeError):
                self._engine = None
        if self._engine is None:
            self._level = SpeechLevel.OFF

    @property
    def level(self) -> SpeechLevel:
        return self._level

    @level.setter
    def level(self, value: SpeechLevel):
        self._level = value
        if value != SpeechLevel.OFF and self._engine is None and _has_speech:
            try:
                self._engine = pyttsx3.init()
                self._select_preferred_voice()
            except (OSError, RuntimeError):
                self._engine = None
        if self._engine is None:
            self._level = SpeechLevel.OFF

    def _select_preferred_voice(self):
        voices = self._engine.getProperty('voices')
        system = platform.system()

        if system == 'Windows':
            for want in self._WINDOWS_VOICES:
                for v in voices:
                    if want.lower() in v.name.lower():
                        self._engine.setProperty('voice', v.id)
                        break
                else:
                    continue
                break
        else:
            # Linux: espeak voice variants (+f3 etc) don't appear in the voice list.
            # Try each candidate as a raw voice ID; espeak-ng handles the +f suffix.
            for want in self._LINUX_VOICES:
                try:
                    self._engine.setProperty('voice', want)
                    # Verify it stuck
                    if want.lower() in self._engine.getProperty('voice').lower():
                        break
                except Exception:
                    continue
            else:
                # Ultimate fallback — first available English voice
                for v in voices:
                    if v.name.lower().startswith('en') or v.name.lower().startswith('english'):
                        self._engine.setProperty('voice', v.id)
                        break
        self._engine.setProperty('rate', 150)
        self._engine.setProperty('volume', 0.8)

    @property
    def level(self) -> SpeechLevel:
        return self._level

    @level.setter
    def level(self, value: SpeechLevel):
        self._level = value
        if value != SpeechLevel.OFF and self._engine is None and _has_speech:
            try:
                self._engine = pyttsx3.init()
                self._engine.setProperty('rate', 150)
                self._engine.setProperty('volume', 0.8)
            except (OSError, RuntimeError):
                self._engine = None
        if self._engine is None:
            self._level = SpeechLevel.OFF

    @property
    def available(self) -> bool:
        return self._engine is not None

    def speak(self, text: str, level: SpeechLevel = SpeechLevel.ALL, async_mode: bool = True):
        if self._engine is None or self._level == SpeechLevel.OFF:
            return
        if self._level < level:
            return
        if async_mode:
            threading.Thread(target=self._speak_sync, args=(text,), daemon=True).start()
        else:
            self._speak_sync(text)

    def _speak_sync(self, text: str):
        try:
            self._engine.say(text)
            self._engine.runAndWait()
        except Exception:
            pass

    def speak_altitude(self, altitude: float):
        self.speak(f"Altitude {int(altitude)} meters", SpeechLevel.ALL)

    def speak_battery(self, volts: float):
        self.speak(f"Battery {volts:.1f} volts", SpeechLevel.STATUS)

    def speak_battery_warning(self, volts: float):
        self.speak(f"Warning, battery low: {volts:.1f} volts", SpeechLevel.ALERTS)

    def speak_nav_status(self, state: str, wp: int):
        self.speak(f"{state} waypoint {wp}", SpeechLevel.STATUS)

    def speak_waypoint_reached(self, wp: int):
        self.speak(f"Reached waypoint {wp}", SpeechLevel.STATUS)

    def speak_gps_lost(self):
        self.speak("GPS fix lost", SpeechLevel.ALERTS)

    def speak_gps_acquired(self, sats: int):
        self.speak(f"GPS fix acquired, {sats} satellites", SpeechLevel.STATUS)

    def speak_armed(self):
        self.speak("Armed", SpeechLevel.STATUS)

    def speak_disarmed(self):
        self.speak("Disarmed", SpeechLevel.STATUS)

    def speak_alarm(self, alarm: str):
        self.speak(f"Alarm: {alarm}", SpeechLevel.ALERTS)

    def speak_mode(self, mode: str):
        self.speak(f"Mode: {mode}", SpeechLevel.STATUS)
