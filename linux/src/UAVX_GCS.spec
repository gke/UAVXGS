# -*- mode: python ; coding: utf-8 -*-
# UAVX Groundstation — generic PyInstaller spec (Linux EXE / macOS app bundle)
#
# Invoked by linux/build.sh and macos/build.sh / build_macos.sh as:
#     pyinstaller --clean UAVX_GCS.spec
# (run from the kit src/ directory, same layout as windows/build.bat).
#
# The hidden-imports / collect-submodules set mirrors the canonical
# windows/build.bat inline arguments so all three platform builds ship the
# same dependency closure (PyQt5 WebEngine, pyserial, folium, pyttsx3).

import os
import sys
from PyInstaller.utils.hooks import collect_submodules

block_cipher = None

# Ship only the runtime airframe support: the loader modules are collected as
# Python modules, and these are the shipped .af folders. The canonical src/
# tree also holds dev tooling, CSV mirrors and original/proposed airframes;
# scripts/helper/kit_manifest.txt keeps the kit tree clean, and these explicit
# datas keep the frozen bundle clean even when built from a full tree.
datas = [(d, d) for d in ('airframes/generic', 'airframes/contributed')
         if os.path.isdir(d)]

hidden_imports = (
    ['PyQt5.QtWebEngineWidgets',
     'PyQt5.QtWebChannel',
     'serial',
     'serial.tools.list_ports',
     'folium',
     'pyttsx3',
     'pyttsx3.drivers',
     'pyttsx3.drivers.sapi5']
    + collect_submodules('core')
    + collect_submodules('ui')
    + collect_submodules('widgets')
    + collect_submodules('logger')
)

a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hidden_imports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

if sys.platform == 'darwin':
    exe = EXE(
        pyz,
        a.scripts,
        [],
        exclude_binaries=True,
        name='UAVX_GCS',
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=True,
        console=False,
    )
    coll = COLLECT(
        exe,
        a.binaries,
        a.zipfiles,
        a.datas,
        strip=False,
        upx=True,
        upx_exclude=[],
        name='UAVX_GCS',
    )
    app = BUNDLE(
        coll,
        name='UAVX_GCS.app',
        icon=None,
        bundle_identifier='org.uavx.gcs',
    )
else:
    exe = EXE(
        pyz,
        a.scripts,
        a.binaries,
        a.zipfiles,
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
    )