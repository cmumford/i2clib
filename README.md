This is a simple library to simplify and abstract using the I²C bus.

This library exists primarily to provide a platform independent
interface to the I²C functionality on a given device.

At the present the only supported framework is
[ESP-IDF](https://github.com/espressif/esp-idf), and the only build system
is [PlatformIO](https://platformio.org/). The intention is that this will
change in a future release.

This library is written in C++ (v11).

## Developer Notes

### Code formatting

All code should be formatted as so:

```shell
clang-format -i <source_file>
```

### Running Tests

All tests run **on hardware** and expect to have the appropriate real-time
clocks attached. See platformio.ini to learn the necessary GPIO pins for
your hardware. Run the tests as follows:

```sh
make test
```
