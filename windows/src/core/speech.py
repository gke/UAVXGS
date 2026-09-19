# core/speech.py

import platform
import queue
import shutil
import subprocess
import threading
from enum import IntEnum


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
    _has_pyttsx3 = True
except (ImportError, OSError, RuntimeError):
    _has_pyttsx3 = False

# Fallback command-line TTS engines, tried in order when pyttsx3 is unusable.
_COMMAND_TTS = [
    ("espeak-ng", []),                # standard on most Linux distros
    ("espeak", []),                   # legacy name
    ("spd-say", ["-w"]),              # speech-dispatcher
    ("flite", ["-t"]),                # small embedded TTS
]

# Female English voice per command-line backend. The FC/GCS speaks flight
# states and alerts, so a calm British-English female voice reads most clearly.
# en-gb+f1 (deep/warm female) is the primary pick over the brighter f3.
_VOICE_ARG = {
    "espeak-ng": ["-v", "en-gb+f1", "-s", "135"],
    "espeak": ["-v", "en-gb+f1", "-s", "135"],
    "spd-say": ["-v", "female"],
    "flite": ["-voice", "slt"],       # US English female (slt = default female)
}


class _SubprocessTts:
    # Minimal sh: out goes through a TTS binary run on the worker thread.
    def __init__(self):
        self.cmd = None
        self.voice = []
        for binary, prefix in _COMMAND_TTS:
            if shutil.which(binary):
                self.cmd = [binary] + prefix
                self.voice = _VOICE_ARG.get(binary, [])
                break
        self.error = None if self.cmd else "no TTS backend found"

    def say(self, text, volume=None):
        if self.cmd is None:
            return
        cmd = list(self.cmd)
        if self.voice:
            cmd += self.voice
        if volume is not None and self.cmd[0] in ("espeak-ng", "espeak"):
            cmd.append("-a")
            cmd.append(str(int(min(max(volume * 200, 0), 200))))
        try:
            subprocess.run(cmd + [text], capture_output=True, timeout=20)
        except Exception:
            pass


class SpeechController:
    # Preferred voices in order of preference
    _DEFAULT_VOLUME = 0.4
    _WINDOWS_VOICES = [
        "Microsoft Catherine",      # Win11 Natural Voice — calm, educated British female
        "Microsoft Hazel",          # SAPI5 British female
        "Microsoft Zira",           # SAPI5 US female
        "Microsoft Susan",          # SAPI5 British female (legacy)
    ]
    _LINUX_VOICES = [
        "en-gb+f1",                 # espeak-ng British English female 1 — deep, warm,
                                    # calm; closest pleasant-female espeak gets to the
                                    # EdgeTX "Kate" tone (a Piper voice espeak can't load)
        "en-gb+f2",                 # espeak-ng British English female 2
        "en-gb+f3",                 # espeak-ng British English female 3 — brighter (old default)
        "mb-us1",                   # MBROLA US English female (natural, apt install mbrola mbrola-us1)
        "us-mbrola-1",              # MBROLA US English female (alt name)
        "en-us+f3",                 # espeak-ng US English female
        "en-gb",                    # espeak-ng British English default (male)
    ]

    def __init__(self):
        self._engine = None
        self._fallback = None
        self._init_error = None
        self._init_tried = False
        self._level = SpeechLevel.OFF
        self._speech_queue = queue.Queue()
        threading.Thread(target=self._speech_worker, daemon=True).start()
        self._ensure_engine()

    @property
    def level(self) -> SpeechLevel:
        # Pure user preference. Deliberately NOT downgraded to OFF when the
        # engine is unavailable: the UI combo must reflect what the operator
        # selected, and actual capability is exposed via `available`.
        return self._level

    @level.setter
    def level(self, value):
        self._level = SpeechLevel(value)

    @property
    def available(self) -> bool:
        return self._engine is not None or self._fallback is not None

    @property
    def backend(self) -> str:
        if self._engine is not None:
            return "pyttsx3"
        if self._fallback is not None:
            return self._fallback.cmd[0]
        return "none"

    @property
    def init_error(self) -> str:
        return self._init_error

    def _ensure_engine(self):
        if self._init_tried:
            return
        self._init_tried = True

        if _has_pyttsx3:
            try:
                self._engine = pyttsx3.init()
            except Exception as e:  # broken driver/backend must not break the GCS
                self._engine = None
                self._init_error = f"pyttsx3 init failed: {e}"
            if self._engine is not None:
                # Best-effort voice selection: any failure keeps a working
                # engine (default voice) rather than throwing it away.
                try:
                    self._select_preferred_voice()
                except Exception as e:
                    self._init_error = f"voice selection failed ({e}); using default"

        if self._engine is None:
            self._fallback = _SubprocessTts()
            reason = self._init_error or "pyttsx3 not installed"
            if self._fallback.cmd is None:
                self._init_error = (f"{reason}; no command-line TTS "
                                    "(install espeak-ng or spd-say/flite)")
            else:
                self._init_error = f"{reason}; falling back to {self._fallback.cmd[0]}"

    def _select_preferred_voice(self):
        try:
            voices = self._engine.getProperty('voices') or []
        except Exception:
            voices = []
        system = platform.system()

        if system == 'Windows':
            for want in self._WINDOWS_VOICES:
                for v in voices:
                    if want.lower() in v.name.lower():
                        self._engine.setProperty('voice', v.id)
                        return
        else:
            # Linux: espeak voice variants (+f3 / mbrola etc) don't appear in the
            # voice list, so set them as raw voice IDs. The engine normally echoes
            # back the base name ("en-gb") rather than the variant, so verify via
            # the base name to avoid wrongly rejecting an applied variant.
            for want in self._LINUX_VOICES:
                try:
                    self._engine.setProperty('voice', want)
                    base = want.split('+')[0].lower()
                    got = str(self._engine.getProperty('voice') or '').lower()
                    if base in got or want.lower() in got:
                        return
                except Exception:
                    continue
            # Ultimate fallback — first available English voice
            for v in voices:
                if v.name.lower().startswith('en') or v.name.lower().startswith('english'):
                    self._engine.setProperty('voice', v.id)
                    return
        self._engine.setProperty('rate', 135)
        self._engine.setProperty('volume', self._DEFAULT_VOLUME)
        return

    def _speech_worker(self):
        # Single consumer serializes utterances: a shared pyttsx3 engine is not
        # thread-safe, and overlapping runs silently drop audio.
        while True:
            item = self._speech_queue.get()
            if item is None:
                break
            text, volume = item
            try:
                if self._engine is not None:
                    # Set per utterance so a quieter greeting doesn't leak into
                    # subsequent announcements (volume is sticky on the engine).
                    self._engine.setProperty(
                        'volume', volume if volume is not None else self._DEFAULT_VOLUME)
                    self._engine.say(text)
                    self._engine.runAndWait()
                elif self._fallback is not None:
                    self._fallback.say(text, volume)
            except Exception:
                pass

    def speak(self, text: str, level: SpeechLevel = SpeechLevel.ALL,
              async_mode: bool = True, volume: float = None):
        if text is None or self._level == SpeechLevel.OFF:
            return
        if self._level < level:
            return
        self._ensure_engine()
        if not self.available:
            return
        self._speech_queue.put((text, volume))
        if not async_mode:
            self._speech_queue.join()

    def speak_altitude(self, altitude: float):
        self.speak(f"Altitude {int(altitude)}", SpeechLevel.ALL)

    def speak_battery(self, volts: float):
        self.speak(f"Battery {volts:.1f}", SpeechLevel.STATUS)

    def speak_battery_warning(self, volts: float):
        self.speak(f"Low battery {volts:.1f}", SpeechLevel.ALERTS)

    def speak_nav_status(self, state: str, wp: int):
        self.speak(f"{state} waypoint {wp}", SpeechLevel.STATUS)

    def speak_waypoint_reached(self, wp: int):
        self.speak(f"Reached {wp}", SpeechLevel.STATUS)

    def speak_gps_lost(self):
        self.speak("GPS fix lost", SpeechLevel.ALERTS)

    def speak_gps_acquired(self, sats: int):
        self.speak(f"GPS acquired, {sats}", SpeechLevel.STATUS)

    def speak_armed(self):
        self.speak("Armed", SpeechLevel.STATUS)

    def speak_disarmed(self):
        self.speak("Disarmed", SpeechLevel.STATUS)

    def speak_alarm(self, alarm: str):
        self.speak(f"{alarm}", SpeechLevel.ALERTS)

    def speak_mode(self, mode: str):
        self.speak(mode, SpeechLevel.STATUS)

    def speak_direction(self, distance: float, point: str):
        # "620, South West" — distance in front so the pilot can read it first.
        self.speak(f"{int(distance)}, {point}", SpeechLevel.STATUS)