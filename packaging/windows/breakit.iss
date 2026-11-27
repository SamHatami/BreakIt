; BreakIt Windows installer (Inno Setup 6). Built by .github/workflows/release.yml:
;   ISCC /DAppVersion=... /DSourceDir=<folder with BreakIt.exe> /DOutputDir=... /DOutputName=... breakit.iss
;
; Installs for the current user only (no admin prompt) into
; %LOCALAPPDATA%\Programs\BreakIt. The AppId never changes, so running a newer
; setup upgrades the installed copy in place: it closes a running BreakIt,
; replaces the files and keeps logs\ and playground\ (the game writes those
; next to the exe). Uninstalling also leaves those two folders.

#ifndef AppVersion
  #define AppVersion "dev"
#endif
#ifndef SourceDir
  #define SourceDir "..\..\target\dist"
#endif
#ifndef OutputDir
  #define OutputDir "..\..\dist"
#endif
#ifndef OutputName
  #define OutputName "BreakIt-setup"
#endif

[Setup]
; Do not change: this is how Windows knows a new setup is the same program.
AppId={{6F1D2C3B-8A4E-4B7F-9E21-B5A0C7D3E914}
AppName=BreakIt
AppVersion={#AppVersion}
AppVerName=BreakIt {#AppVersion}
AppPublisher=BreakIt
AppPublisherURL=https://github.com/SamHatami/BreakIt
AppSupportURL=https://github.com/SamHatami/BreakIt/issues
AppUpdatesURL=https://github.com/SamHatami/BreakIt/releases
DefaultDirName={localappdata}\Programs\BreakIt
DisableDirPage=yes
DefaultGroupName=BreakIt
DisableProgramGroupPage=yes
DisableReadyPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
CloseApplications=force
RestartApplications=no
UninstallDisplayIcon={app}\BreakIt.exe
UninstallDisplayName=BreakIt
OutputDir={#OutputDir}
OutputBaseFilename={#OutputName}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; Flags: unchecked

[Files]
Source: "{#SourceDir}\BreakIt.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\PLAYTEST.md"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\VERSION.txt"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{autoprograms}\BreakIt"; Filename: "{app}\BreakIt.exe"
Name: "{autoprograms}\BreakIt playtest guide"; Filename: "{app}\PLAYTEST.md"
Name: "{autoprograms}\BreakIt logs"; Filename: "{app}\logs"
Name: "{autodesktop}\BreakIt"; Filename: "{app}\BreakIt.exe"; Tasks: desktopicon

[Dirs]
; So the "BreakIt logs" shortcut works before the first run.
Name: "{app}\logs"; Flags: uninsneveruninstall

[Run]
Filename: "{app}\BreakIt.exe"; Description: "Start BreakIt"; Flags: nowait postinstall skipifsilent
