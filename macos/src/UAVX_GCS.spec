# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import collect_submodules
from pathlib import Path

hiddenimports = [
    'PyQt5.QtWebEngineWidgets',
    'PyQt5.QtWebChannel',
    'serial',
    'serial.tools.list_ports',
    'folium',
    'pyttsx3',
    'pyttsx3.drivers',
    'pyttsx3.drivers.nsss'
]

hiddenimports += collect_submodules('core')
hiddenimports += collect_submodules('ui')
hiddenimports += collect_submodules('widgets')
hiddenimports += collect_submodules('logger')

a = Analysis(
    ['main.py'],
    pathex=[str(Path.cwd())],
    binaries=[],
    datas=[('airframes', 'airframes')],
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='UAVX_GCS',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

app = BUNDLE(
    exe,
    name='UAVX_GCS.app',
    icon=None,
    bundle_identifier=None,
)
