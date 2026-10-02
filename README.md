# navigator

The Vayu ground control station: a Qt6 desktop GCS, the headless SDK that
drives the flight stack from Python, and the autotuner.

## Layout

    navigator/            the Qt6 application
    navigator/headless-sdk/  the installable vayu_headless package + its tests
    tools/autotune/       PID autotuning against the simulator
    navlink/              the wire protocol, as a submodule

## Building

    git submodule update --init --recursive
    cmake -S navigator -B build
    cmake --build build -j

Qt6 (Core, Widgets, SerialPort, Network), assimp and libpulse are required;
`-DNAVIGATOR_SITL=OFF` drops the simulator and with it assimp.

## The simulator needs a firmware release

Nothing here builds firmware. The in-app simulator is the firmware compiled
for the host, shipped by the
[vayu](https://github.com/ragnar-vallhala/vayu/releases) repository as a
release asset and loaded at runtime. CI builds against **v0.1.0**; grab that
release, or the rolling `sitl-sdk-latest` if you want whatever is on firmware
main. Unpack it at the repo root:

    tar xzf vayu-sitl-sdk-v0.1.0-linux-x86_64.tar.gz   # -> ./vayu-sitl-sdk/

CMake finds `vayu-sitl-sdk/include` from there with no flags. At runtime:

  - the app loads `lib/libvayu_sitl.so`, found beside the executable or via
    `$VAYU_SITL_MODULE`
  - the headless SDK and the autotuner spawn `bin/vayu_sitl_rtos`, found via
    `$VAYU_SITL_RTOS_BIN`

Without the SDK the GCS still builds and runs; the simulator reports itself
unavailable and says where it looked.

The GCS and the firmware are versioned independently. The module states which
firmware it is (`SitlModule::buildId()`, logged when the sim starts), and a
module built against a different ABI is refused at load rather than trusted.

## Tests

    ctest --test-dir build                       # 20 QtTest cases
    cd navigator/headless-sdk && python3 -m pytest tests

The pytest suite needs `bin/vayu_sitl_rtos` from the SDK, and the generated
Python codec: run `python3 navlink/generate.py --lang both` once (the CMake
build does this for C).

## License

Apache License 2.0 — see [LICENSE.md](LICENSE.md). Copyright (C) 2026
NAVRobotec Pvt Ltd.

The flight controller firmware this talks to is a separate, private
repository. What is here is the ground station: the protocol it speaks
(`navlink`, also Apache-2.0) is public, so a third party can build against it.
