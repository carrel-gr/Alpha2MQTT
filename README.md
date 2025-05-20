# Alpha2MQTT (A2M)
## A Home Assistant controller for AlphaESS solar and battery inverters.

_**Credit!** This project is entirely derived from https://github.com/dxoverdy/Alpha2MQTT (original README is [here](README-orig.md)) which is a really great bit of work.  My hardware includes some tweaks and modernizations on that project's hardware.  My software has changed a lot from that project, but at its core, this project is derived from that project and would not exist without that project. So HUGE credit and thanks._

**What is this?** This project is a controller that connects Home Assistant to an AlphaESS system via RS485 and uses MQTT discovery to set up a ready to go integration.  This is local control with NO cloud dependancy.  Once you plug-in and turn on this A2M hardware, your ESS will appear as a unique HA/MQTT device with HA entities ready to monitor and [control](#controlling-your-ess) your AlphaESS system.  No HA configuration is needed.  The device and entities are ready to be used with all units, classes, icons, unique IDs, availability and more.   Entities are ready to be used by the Energy dashboard and can be included in other [dashboards](#dashboard-example) and [automations](#automation-examples).  This controller also provides [Operating Modes](#operating-modes) that enhance the AlphaESS functionality while making it simpler to [control](#controlling-your-ess).

**What's Different?** The big change is that rather than being a general purpose interface between MQTT and the raw AlphaESS system data, this project is a controller specifically designed to work as a Home Assistant integration for your AlphaESS system.  It uses MQTT discovery to be plug-and-play so that no HA configuration is needed.  And it adds some "smart", real-time capabilities in the controller so that HA automation control is simplified.  This also incorporates several newer versions of the [AlphaESS specs](#alphaess-specs), and has enhancements/tweaks/fixes to WiFi and RS485 functionality.

![Alpha2MQTT](Pics/Dave_HW2.jpg)

### Supported AlphaESS devices are:
- I have only tested this on a SMILE-SPB system.

- The [original project](README-orig.md) was tested on these systems which _should_ work, but I really can't say until someone tests them.
  - SMILE5,
  - SMILE-B3,
  - SMILE-T10,
  - Storion T30,
  - SMILE-B3-PLUS
  - *Others are likely to work too.

## Steps to get running.
- Build the hardware.  (See [HARDWARE](#hardware) below)
- Configure, build and load the software.  Follow the instructions in the [original README](README-orig.md#flashing) for setting up the software environment and compiling/loading.  Changes are needed in `Definitions.h` and you should follow the instructions in the file itself, as this file has changed a fair bit.
- Enable MQTT discovery in Home Assistant (if this isn't on already).
- Plug in RS485 and power.  At this point your device/entities will appear under the MQTT integration as "A2M-ALXXXXXXXXXXXXX".)  You can now see and monitor your ESS in HA.
- (optional) Configure the HA Energy dashboard to use these new entities.  All the necessary entities are provided for grid, solar, and battery.
- (optional) Create an ESS dashboard.  (See [Dashboard Example](#dashboard-example) below.)
- (optional) Create ESS automations to control the ESS.  (See [Automations Example](#automation-examples) below.)

## More Details
### What you will see
- Once your Alpha2MQTT device is working, in Home Assistant go to Settings->Devices & Services->Integrations->MQTT->devices
- In this list is your new Alpha2MQTT device which starts with "A2M" and ends with your serial number.  In this image it is the top entry. (The 2nd entry is my dummy testing device which you won't see.)
![MQTT Devices](Pics/Dave_MQTT_Devices.png)
- Now click on your device and you'll see this, which is every entity that is provided for your device:
![MQTT A2M](Pics/Dave_MQTT_A2M.png)

### Controlling Your ESS
There are 5 control entities for controlling your ESS.  There is "**Op Mode**", "**SOC Target**", "**Charge Power**", "**Discharge Power**", and "**Push Power**".  Control is simple.  You set an "**Op Mode**" and the other control entities that are appropriate for that mode.  Operating Modes are a bit different from AlphaESS internal modes.  They are similar, but add more fuctionality.  For example, most Alpha modes don't honor a target SOC, but most Operating Modes do.  Here are all the modes and which other controls each uses:
- "**Load Follow**": This is the same as the AlphaESS "Load Follow" except this also honors the "**SOC Target**" setting.  (Alpha says their "Load Follow" mode and their "Normal" modes are the same.)  This mode uses "**Charge Power**" and "**Discharge Power**" settings.  I don't personally use this mode in my automations.
- "**Target SOC**": This will charge or discharge the ESS, as appropriate, to reach the "**SOC Target**".  This will use the "**Charge Power**" and "**Discharge Power**" settings.
- "**Push To Grid**": This will push power to the grid until the "**SOC Target**" is reached.  It only uses the "**Push Power**" setting to determine the discharge power.  When PV is producing more than the house load, this pushes the excess PV power **plus** the "**Push Power**" amount to the grid.  When PV is producing less than the house load, then this pushes the "**Push Power**" amount to the grid.  The Alpha2MQTT controller dynamically adjusts the AlphaESS setting (without needing HA involvement) to achieve this.
- "**PV Charge**": This will charge the ESS only from PV power that exceeds the house load.  This uses the "**SOC Target**" and "**Charge Power**" settings.  When PV power is less than the house load, the grid is used to supply the remainder of the house load.  The ESS is charged only when PV power exceeds the house load.  If PV power exceeds the house load plus "**Charge Power**", or if the "**SOC Target**" is reached, then excess PV is pushed to the grid.
- "**Max Charge**": This will charge the ESS from any source (PV or grid) as fast as it can until it is full.  Grid supplies the house load.  Excess PV is pushed to the grid.
- "**No Charge**": This will not charge the ESS from any source, and will use the ESS for house loads when PV is less than the house load.  Excess PV is pushed to the grid.

### AlphaESS Specs
Alpha2MQTT honours 1.28 AlphaESS Modbus documentation.  The latest register list I found came from October 2024.

- [AlphaESS Modbus Documentation](Pics/AlphaESS_Register_parameter_list_Oct_2024.pdf)

- [AlphaESS Register List](Pics/AlphaESS_Modbus_Protocol_V1.28.pdf)

### Hardware
To build this hardware you should start with the [original project instructions](README-orig.md#how-to-build).  I originally started with an ESP8266, then I switched to an ESP32 for better WiFi and I have only tested with the ESP32 for a while.  (However, the ESP8266 _should_ still work.)  I also went to a larger display.  This isn't necessary, but did help with debugging.  And I used a different MAX3485 part because I liked the form factor better.  If you use this, be sure to connect the EN pin.  Finally I switched from a "generic" ESP32 to the Seeed XIAO ESP32C6 which is insanely small, has FAR better WiFi (including WiFi 6), and is quite cheap.  The XAIO ESP32C6 supports both an internal and external antennas (selectable via software).  I found the XAIO ESP32C6 internal antenna gave me a stronger signal than my older ESP32s with an external antenna.  (Select which antenna you are using in `Definitions.h`)

Here are links to the parts I used.
- XIAO ESP32C6 - [Seeed](https://www.seeedstudio.com/XIAO-Main-ESP32C6-2354.html), [amazon](https://www.amazon.com/gp/product/B0D2NKVB34)
  - or generic ESP 32 - [amazon](https://www.amazon.com/gp/product/B0CL5VGC8J)
- Display - [amazon](https://www.amazon.com/gp/product/B09C5K91H7)
- MAX3485 - [amazon](https://www.amazon.com/gp/product/B09SYZ98KF)

Device wiring:
- Display GND -> XIAO ESP32C6 GND (Pin 13)
- Display VCC -> XIAO ESP32C6 3.3V (Pin 12)
- Display SCL -> XIAO ESP32C6 SCL (Pin 6)
- Display SDA -> XIAO ESP32C6 SDA (Pin 5)
- MAX3485 EN  -> XIAO ESP32C6 D8/GPIO19 (Pin 9)
- MAX3485 VCC -> XIAO ESP32C6 3.3V (Pin 12)
- MAX3485 RXD -> XIAO ESP32C6 TX/D6/GPIO16 (Pin 7)
- MAX3485 TXD -> XIAO ESP32C6 RX/D7/GPIO17 (Pin 8)
- MAX3485 GND -> XIAO ESP32C6 GND (Pin 13)
- MAX3485 A   -> RJ45 (Pin 5)
- MAX3485 B   -> RJ45 (Pin 4)

### Dashboard Example
- I tied into the Home Assistant builtin Energy dashboard by simply editing its configuration and adding grid to/from, solar, battery to/from, and battery SOC.  Voila!
- Here is my ESS dashboard.  This was simple to make using the HA builtin visual editor. ([Here](Dave_Examples/Dave_ESS_Dashboard.yaml.txt) is the yaml.)  I used the builtin"energy-distribution" card and "power-flow-card-plus" which is available through HACS.  In the middle section there are several entities that appear and disappear depending on the current Op Mode.
![ESS Dashboard](Pics/Dave_ESS_Dashboard.png)
- Here is the same dashboard when the ESS is in a different Op Mode.  Note: The middle column has more entities showing in this Op Mode.
![ESS Dashboard](Pics/Dave_ESS_Dashboard_2.png)
- The "Electricity Tariff" and "NWS Alerts" are the two entities on this page that don't come from Alpha2MQTT.  I include them here because my automations use these to help control the ESS.
- You will also note that this dashboard also has views for an "Energy" (almost identical to HA's builtin Energy Dashboard) and "Power".  I won't bore you with pics, but the yaml has all the details.
### Automation Examples
- ESS control - First off, I **highly** recommend that you have only ONE system controlling your ESS.  Alpha2MQTT can be used to simply monitor your ESS.  However, if you are using Alpha2MQTT to control your ESS, then be sure no other system is controlling it.  I allow the Alpha cloud to monitor my system, but it does NO control.
  - This control is a little complex, but not too bad.  Let me try to explain all the parts.
    - My peak electricity price hours are 4pm to 9pm.  My mid-peak is 3pm to 4pm and 9pm to midnight.  Off-peak is midnight to 3pm.  My tarrif allows me to sell power that I push to the grid for the same price that I would pay for it.
    - I try to ONLY use battery or solar during peak and mid-peak.
    - I try to push some saved battery power back to the grid during peak, but only if the SOC is high enough and only when there are no storm alerts. I check, and adjust, each hour during peak.
    - Even though it costs the same, I prefer to charge using solar rather than using the grid, except when there are storms, and then I charge as soon as the rates are cheap.  A bit before rates get expensive, I fill the battery from the grid in case it isn't full. (It is _almost_ always already full.)
  - [Here](Dave_Examples/Dave_ESS_Automation.yaml.txt) is my yaml that accomplishes this.  This is all in a single automation named "ESS".
- Other Device control
  - You can also use the ESS state to control other devices in Home Assistant.  For example, if Alpha2MQTT detects that the grid has become unavailable, then it turns off my EVSE (car "charger").
  - I'd love to hear what you are controlling...

### Tips/Hints/Suggestions
- Only the "**Max Charge**" "Op Mode" will do cell balancing.  Other modes are fine to use, but every so often you should run a "**Max Charge**" to balance things out.
- There are several Faults and Warnings entities for each of the system components, and each of them monitors multiple AlphaESS registers.  These entities are simple numbers, but they also have an attribute which provides more detail. Click on the entity, then click Attributes, and you will see which registers are monitored and which bits are set.
- There is one entity ("**Register Number**") that allows you to view the actual AlphaESS register values.  Enter a register number (in decimal, not hex) in this entity, and you will then see the (formatted) register value displayed in "**Register Value**".  This is only intended for debugging.

### Other Changes and Enhancements
- Quite a few new registers have been added.  (See [Specs](#alphaess-specs) above.)
- This uses an MQTT Last Will and Testament (LWT) to set availability for all entities.  In addition some entities also use the RS485 status to set their availability.  And for others, even the grid status is used.
- This uses the MQTT retain flag along with other MQTT options to ensure as-graceful-as-possible handling of situations where this device, HA, or the MQTT broker might reboot or go offline.  By default, HA is assumed to be the authority for knowing what state the ESS should be in.  If Alpha2MQTT reboots, HA will set the state upon reconnect.  However, this can be changed in `Definitions.h` so that the ESS is the authority and Alpha2MQTT will then tell HA what the state is.
- The WiFi code has been tweaked to better handle Multi-AP environments and low signal situations.
- The RS485 code has been tweaked to better handle more than two peers on the RS485 bus.  While this situation is rare, I had to deal with it during development because my vendor provided control via RS485 as well.  So until I trusted my device to take over, I had to make both controllers coexist.  RS485 fully supports more than two peers.  However, the MAX3485 doesn't provide enough control to make this perfect.  But I was able to make it robust enough.  Also, MODBUS says there can be only one "master" and Alpha2MQTT and my vendor's device were both masters.  But again, it still works well enough.  And now that I trust this controller, I have disconnected the vendor's device.
- The larger display support allows for easier debugging.  This really isn't needed for anything other than debugging.
- There's a lot of debugging information available via MQTT and the large display.  Enabling the different DEBUG options in `Definitions.h` will add more rotating debug values on the screen and add new Diagnostic entities under the MQTT device.
- The AlphaSniffer project is a quick and dirty RS485 sniffer that lets this hardware sniff the RS485 bus while some other master talks to your AlphaESS.  It streams the (somewhat parsed) output over TCP to a host.
