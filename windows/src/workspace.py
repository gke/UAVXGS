"""User workspace location — the one user-chosen folder for saved airframes
and logs.

Only the user's own files live here; the read-only airframe library (generic,
contributed, proposed) ships inside the application package. The default is
``<Documents>/UAVX`` (Qt's per-OS Documents location, so a redirected Windows
Documents folder is honoured), overridable per user and remembered in
QSettings. The older ``log_dir`` setting is adopted as the workspace root on
first run so an existing install keeps its chosen location.
"""

import os

from PyQt5.QtCore import QSettings, QStandardPaths

_SETTINGS_ORG = "UAVX"
_SETTINGS_APP = "Groundstation"
_WORKSPACE_KEY = "workspace_dir"


def _settings():
    return QSettings(_SETTINGS_ORG, _SETTINGS_APP)


def default_workspace_dir():
    """Documents/UAVX, cross-platform, with a home-dir fallback."""
    base = QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation)
    if not base:
        base = os.path.expanduser("~")
    return os.path.join(base, "UAVX")


def has_workspace_dir():
    """True once a workspace location has been chosen or defaulted."""
    return bool(_settings().value(_WORKSPACE_KEY, ""))


def workspace_dir():
    """The workspace root, defaulting and persisting on first use."""
    settings = _settings()
    path = settings.value(_WORKSPACE_KEY, "")
    if not path:
        legacy = settings.value("log_dir", "")
        path = legacy or default_workspace_dir()
        settings.setValue(_WORKSPACE_KEY, path)
    return path


def set_workspace_dir(path):
    _settings().setValue(_WORKSPACE_KEY, path)


def airframes_dir():
    """Writable home for the user's saved .af files."""
    return os.path.join(workspace_dir(), "airframes")


def logs_dir():
    """Home for raw telemetry logs and later KML exports."""
    return os.path.join(workspace_dir(), "logs")


def ensure_dirs():
    """Create the workspace subfolders if needed; return (airframes, logs)."""
    air = airframes_dir()
    log = logs_dir()
    os.makedirs(air, exist_ok=True)
    os.makedirs(log, exist_ok=True)
    return air, log
