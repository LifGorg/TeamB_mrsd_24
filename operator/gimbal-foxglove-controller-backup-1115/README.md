# gimbal-foxglove-controller

This Foxglove extension provides two gimbal control panels:

## Panels

### 1. Gimbal Virtual Controller (Legacy)
**Panel name:** `gimbal-virtual-controller`

Full teleoperation controller with on-screen buttons acting like a joystick:

- Pan: Up / Down / Left / Right
- Zoom: In / Out
- Trigger button (capture)
- Mode switch button (record)
- Recenter camera button

Hold the directional and zoom buttons to continuously send commands; the others send one-shot actions.

### 2. Gimbal Capture Controller (New/Recommended)
**Panel name:** `gimbal-capture-controller`

Simplified controller with only essential capture/record functions:

- 📷 Capture button → publishes to `/trigger_capture` (std_msgs/Bool)
- 🔴 Record Toggle button → publishes to `/trigger_record` (std_msgs/Bool)

All pan/tilt/zoom teleoperation controls have been removed. Use a physical joystick controller for manual gimbal control.

## Build and install locally

From the extension folder:

```
npm run build
npm run local-install
```

Then in Foxglove Studio, add either panel from the Add panel menu:
- `gimbal-virtual-controller` (legacy, full controls)
- `gimbal-capture-controller` (new, capture/record only)

You can change the publish topic and other settings in the panel settings.

[Foxglove](https://foxglove.dev) allows developers to create [extensions](https://docs.foxglove.dev/docs/visualization/extensions/introduction), or custom code that is loaded and executed inside the Foxglove application. This can be used to add custom panels. Extensions are authored in TypeScript using the `@foxglove/extension` SDK.

## Develop

Extension development uses the `npm` package manager to install development dependencies and run build scripts.

To install extension dependencies, run `npm` from the root of the extension package.

```sh
npm install
```

To build and install the extension into your local Foxglove desktop app, run:

```sh
npm run local-install
```

Open the Foxglove desktop (or `ctrl-R` to refresh if it is already open). Your extension is installed and available within the app.

## Package

Extensions are packaged into `.foxe` files. These files contain the metadata (package.json) and the build code for the extension.

Before packaging, make sure to set `name`, `publisher`, `version`, and `description` fields in _package.json_. When ready to distribute the extension, run:

```sh
npm run package
```

This command will package the extension into a `.foxe` file in the local directory.

## Publish

You can publish the extension to the public registry or privately for your organization.

See documentation here: https://docs.foxglove.dev/docs/visualization/extensions/publish/#packaging-your-extension
