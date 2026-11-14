; Inno Setup script for the Navigator Windows installer (unsigned; code
; signing is a later phase). Built by .github/workflows/release.yml:
;
;   ISCC /DAppVersion=0.3.0 /DAppChannel=stable /DDistDir=<dist> ^
;        /DNavDir=<nav bin+share> /DLicenseDir=<licences> /DOutDir=<out> ^
;        tools\windows\navigator.iss
;
; DistDir   output of tools/windows/deploy-navigator-msys.sh (Navigator.exe + DLLs)
; NavDir    unpacked nav win64.zip (bin\nav.exe, share\nav\...)
; LicenseDir  LICENSE.md, THIRD-PARTY-NOTICES.md, licenses\ (see tools/release/make-notices.sh)
; No flashing tools ship on Windows yet: which ones and from where is open.
#ifndef AppVersion
  #error Pass /DAppVersion=<version>
#endif
#ifndef AppChannel
  #define AppChannel "stable"
#endif
#ifndef OutDir
  #define OutDir "."
#endif

[Setup]
; Fixed GUID: it is how a later installer recognises and upgrades this app.
AppId={{6F0B8C57-3B7E-4B1A-9A52-4E5A1F6C2D10}
AppName=Navigator
AppVersion={#AppVersion}
AppVerName=Navigator {#AppVersion}
AppPublisher=NAVRobotec Pvt Ltd
DefaultDirName={autopf}\Navigator
DefaultGroupName=Navigator
DisableProgramGroupPage=yes
UninstallDisplayIcon={app}\Navigator.exe
LicenseFile={#LicenseDir}\LICENSE.md
OutputDir={#OutDir}
OutputBaseFilename=navigator-{#AppVersion}-{#AppChannel}-windows-x64-setup
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
Compression=lzma2
SolidCompression=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
WizardStyle=modern

[Files]
Source: "{#DistDir}\*"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs ignoreversion
Source: "{#NavDir}\bin\nav.exe"; DestDir: "{app}\tools"; Flags: ignoreversion
Source: "{#NavDir}\share\nav\*"; DestDir: "{app}\share\nav"; Flags: recursesubdirs createallsubdirs ignoreversion
Source: "{#LicenseDir}\LICENSE.md"; DestDir: "{app}\licenses"; Flags: ignoreversion
Source: "{#LicenseDir}\THIRD-PARTY-NOTICES.md"; DestDir: "{app}\licenses"; Flags: ignoreversion
Source: "{#LicenseDir}\licenses\*"; DestDir: "{app}\licenses\texts"; Flags: recursesubdirs createallsubdirs ignoreversion

[Icons]
Name: "{group}\Navigator"; Filename: "{app}\Navigator.exe"
Name: "{group}\Licences and notices"; Filename: "{app}\licenses"
Name: "{group}\Uninstall Navigator"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\Navigator.exe"; Description: "Launch Navigator"; Flags: nowait postinstall skipifsilent
