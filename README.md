# MavLink_M — MAVLink → AdHoc converter demo

Translates [MAVLink](https://mavlink.io/) message-definition XML files into modern [AdHoc](https://github.com/AdHoc-Protocol)
protocol-description `.cs` files. The intent is to demonstrate the AdHoc data-modelling capabilities
on a non-trivial real-world protocol (the standard drone telemetry / command format used by
ArduPilot, PX4, QGroundControl, and many other open-source flight-control stacks).

The converter is a single Java SAX walker. It reads each XML dialect, maps every `<message>`,
`<enum>`, `<entry>`, `<param>` to its AdHoc equivalent, captures the canonical MAVLink message
IDs into an AdHoc Dashboard, and emits a self-contained `.cs` per dialect — ready to be fed to
AdHocAgent for code generation in Java / C# / C++ / TypeScript / Go / Rust.

## Layout

| Path                                 | Contents                                              |
|:-------------------------------------|:------------------------------------------------------|
| `msgs/*.xml`                         | 21 MAVLink dialect XMLs synced from upstream `master` |
| `src/org/unirail/MavLink2AdHoc.java` | The converter (~1k lines, single class, SAX-based)    |
| `out/`                               | IntelliJ build output (gitignored / ephemeral)        |

## What gets generated per dialect

For each `<dialect>.xml`, the converter writes `<dialect>.cs` containing, in order:

1. **Dashboard block** — alphabetised `<see cref='Pack' id='N'/>` entries that lock every pack
   to its canonical MAVLink message ID. AdHocAgent honours these IDs across regenerations, so
   wire compatibility with stock MAVLink endpoints is preserved.
2. **Pack classes** — every `<message>` becomes `class <Name> { … }` with field types translated
   (table below). `<deprecated/>` and `<wip/>` notes are inlined into the doc-comment block as
   `**DEPRECATED** since=… replaced_by=…: <body>` markers.
3. **Enums** — every `<enum>` becomes `enum <Name> { … }`. Bit-flag enums (every value is 0 or
   a power of two) get the canonical AdHoc `[Flags]` attribute. Enums whose values overflow `int`
   are emitted as `enum <Name>:long { … }`.
4. **`SI_Unit` Constants Container** — a `struct` of nested constant strings (time / distance /
   temperature / angle / electricity / magnetism / energy / power / force / mass / pressure /
   ratio / digital / flow / volume), mirrored from the [MAVLink XSD schema](https://github.com/ArduPilot/pymavlink/blob/master/generator/mavschema.xsd).
5. **`MAV_CMD` enum** — the unified command enum (one entry per `<entry>` inside the XML
   `<enum name="MAV_CMD">`).
6. **`struct MAV_CMD_PARAMS`** — a non-transmittable Constants Container that documents each
   command's parameter conventions. Nested `struct param_N` blocks hold `label`, `units`,
   `Enum`, `decimalPlaces`, `increment`, `minValue`, `maxValue`, `reserved`, `Default` as
   `const string` constants.
7. **Hosts** — `struct GroundControl : Host { }` and `struct MicroAirVehicle : Host { }`.
8. **Channel** — `interface CommunicationChannel : Connects<GroundControl, MicroAirVehicle>`
   with a single `[_____lr_____<@<Dialect>>] struct Start { }` non-transitional state. Either
   side may send any pack; the FSM never transitions.

## Demo topology

MAVLink itself defines no network topology — packets carry only `system_id` and `component_id`
addresses. The demo invents a two-host system to exercise AdHoc's `Connects<>` model:

| Host              | Role                                                    |
|:------------------|:--------------------------------------------------------|
| `GroundControl`   | Operator-side station (laptop / phone with QGC-like UI) |
| `MicroAirVehicle` | The drone / vehicle running the autopilot               |

In the generated descriptor, every dialect's packs are registered in a single bidirectional
`_____lr_____` branch. If you need finer-grained routing (e.g. only the vehicle sends
`HEARTBEAT`-style telemetry; only the operator sends commands), edit the generated file and
replace the single `Start` state with explicit `l____________` / `____________r` branches.

## Type mapping

| MAVLink                   | AdHoc / C#         | Notes                                                       |
|:--------------------------|:-------------------|:------------------------------------------------------------|
| `uint8_t`                 | `byte`             |                                                             |
| `int8_t`                  | `sbyte`            |                                                             |
| `uint8_t_mavlink_version` | `byte`             | The auto-injected protocol-version magic byte               |
| `uint16_t`                | `ushort`           |                                                             |
| `int16_t`                 | `short`            |                                                             |
| `uint32_t`                | `uint`             |                                                             |
| `int32_t`                 | `int`              |                                                             |
| `uint64_t`                | `ulong`            |                                                             |
| `int64_t`                 | `long`             |                                                             |
| `float`                   | `float`            |                                                             |
| `double`                  | `double`           |                                                             |
| `char`                    | `char`             |                                                             |
| `<T>[N]`                  | `[D(N)] <T>[]`     | Fixed-size array; `<T>` is the translated scalar            |
| `char[N]`                 | `[D(+N)] string`   | Null-padded fixed-length string (variable up to N chars)    |
| `<enum>` field            | `<enum>` reference | `<field type="..." enum="EnumName">` resolves to `EnumName` |

## Tag handling

| XML tag         | Behaviour                                                                                |
|:----------------|:-----------------------------------------------------------------------------------------|
| `<include>`     | Recursively parsed; included packs / enums / dashboard entries merge into the parent     |
| `<message>`     | Emit `class <name> { … }` + Dashboard `<see cref id/>` entry                             |
| `<enum>`        | Emit `enum <name> { … }`; auto-detects bit flags → `[Flags]`; long-value → `enum X:long` |
| `<entry>`       | Emit `<NAME> = <value>,` inside the enum body                                            |
| `<param>`       | Emit `public struct param_<index> { … }` inside MAV_CMD_PARAMS; deduped by index         |
| `<description>` | Inlined as a `/** … */` doc-comment block above the parent class / enum / field          |
| `<deprecated>`  | Appended to the parent's doc as `**DEPRECATED** since=… replaced_by=…: <body>`           |
| `<wip>`         | Appended to the parent's doc as `**WIP**: <body>`                                        |
| `<extensions/>` | Silently ignored (MAVLink-2 extension marker has no AdHoc-level analogue)                |
| `<version>`     | Ignored (not part of the protocol description)                                           |
| `<dialect>`     | Ignored                                                                                  |

## Build & run

Java 17+ JDK required.

```bash
# 1. Compile
javac -d out src/org/unirail/MavLink2AdHoc.java

# 2. Run — argument is the directory containing MAVLink XML files.
#    Output is written to <cwd>/AdHoc/<dialect>.cs.
mkdir -p AdHoc
java -cp out org.unirail.MavLink2AdHoc msgs

# 3. Feed any generated descriptor to AdHocAgent for cross-language code generation:
AdHocAgent.exe AdHoc/common.cs
```

To target a different output location, run from a directory whose `AdHoc/` subdirectory is
where you want the files. For example:

```bash
mkdir -p D:/AdHocStuff/mavlink2adhoc/AdHoc
cd D:/AdHocStuff/mavlink2adhoc
java -cp /path/to/out org.unirail.MavLink2AdHoc D:/AdHoc/MavLink_M/msgs
```

## Refreshing the XML dialects

The `msgs/` directory mirrors `https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0`.
To pull the latest:

```bash
cd msgs
for f in ASLUAV AVSSUAS all ardupilotmega common csAirLink cubepilot development \
         icarous loweheiser marsh matrixpilot minimal paparazzi python_array_test \
         standard stemstudios storm32 test uAvionix ualberta; do
    curl -sSf -o "${f}.xml" \
        "https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/${f}.xml"
done
```

If upstream adds new dialects, append their bare names to the list above.


