---
title: "Configuring NixOS for GPD MicroPC 2"
short: "Handling screen rotation configuration."
---

# Configuring NixOS for GPD MicroPC 2

So, there is a GPD MicroPC handheld laptop.
Pretty compact and nice device.

Unfortunately, default screen orientation is in portrait mode.

## Fixing screen rotation

In `configuration.nix` the following options are to be set:

### Console framebuffer

```nix
boot.kernelParams = [
  "fbcon=rotate:1"
];
```

### SDDM

Provide custom configuration file for `weston`

```nix
services.displayManager.sddm.settings.Wayland.CompositorCommand =
  "${lib.getExe pkgs.weston} --shell=kiosk -c /etc/xdg/weston/weston.ini";
services.displayManager.sddm.wayland.compositor = "weston";
```

And generate that file:

```nix
environment.etc."xdg/weston/weston.ini".text = ''
 [output]
 name=DSI-1
 transform=rotate-270
''
```

### Touchscreen configuration

```nix
environment.etc."X11/xorg.conf.d/99-touchscreen-matrix.conf".text = ''
  Section "InputClass"
      Identifier    "rotate ILTP7807 touchscreen"
      Driver        "libinput"
      MatchProduct  "ILTP7807:00 222A:FFF1"
      Option        "TransformationMatrix" "0 1 0 -1 0 1 0 0 1"
  EndSection
'';
```

### Touchpad configuration

```nix
environment.etc."X11/xorg.conf.d/99-alps-touchpad.conf".text = ''
  Section "InputClass"
      Identifier    "ALPS touchpad"
      MatchProduct  "ALPS0001:00 36B6:C001 Touchpad"
      Driver        "synaptics"
      Option        "ClickPad" "false"
      Option        "TapButton1" "1"
      Option        "TapButton2" "3"
      Option        "TapButton3" "2"
  EndSection
'';
environment.etc."libinput/local-overrides.quirks".text = ''
  [ALPS touchpad with external buttons]
  MatchName=ALPS0001:00 36B6:C001 Touchpad
  MatchUdevType=touchpad
  MatchDMIModalias=dmi:*svnGPD:pnG1688-08:*
  # Drop the "buttonpad" property so libinput treats it as a normal touchpad
  AttrInputProp=-INPUT_PROP_BUTTONPAD
'';
```
