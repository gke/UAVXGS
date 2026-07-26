; UAVX Groundstation Inno Setup Installer
; 1. Run build.bat first to produce UAVX_GCS.exe
; 2. Right-click this file → Compile (or run ISCC from command line)
;
; NOTE: This file assumes the porting-kit layout (src/ inside windows/).
; If you moved files, adjust the #define paths below.

#define MyAppName "UAVX Groundstation"
#define MyAppVersion "0.1.0"
#define MyAppPublisher "UAVX"
#define MyAppExeName "UAVX_GCS.exe"

#define SrcDir "src"
#define DistDir SrcDir + "\dist"

[Setup]
AppId={{B8A7C3D1-5E4F-4A3B-9C2D-1E6F7A8B9C0D}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
AllowNoIcons=yes
OutputDir=installer_output
OutputBaseFilename=UAVX_GCS_Setup_{#MyAppVersion}
Compression=lzma
SolidCompression=yes
WizardStyle=modern
UninstallDisplayIcon={app}\{#MyAppExeName}
PrivilegesRequired=admin

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional icons:"

[Files]
; Main executable (build with build.bat first!)
Source: "{#DistDir}\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion

; Airframe definitions
Source: "{#SrcDir}\airframes\*.af"; DestDir: "{app}\airframes"; Flags: ignoreversion
Source: "{#SrcDir}\airframes\__init__.py"; DestDir: "{app}\airframes"; Flags: ignoreversion
Source: "{#SrcDir}\airframes\airframes.py"; DestDir: "{app}\airframes"; Flags: ignoreversion

; FC firmware (for reference/flashing)
Source: "UAVXF4V3Q_r16.bin"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{group}\Uninstall {#MyAppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent
