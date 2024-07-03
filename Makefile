
# PlatformIO install location.
PLATFORMIO=${HOME}/.platformio/penv/bin/platformio

# Port used for running tests.
PORT=/dev/cu.SLAB_USBtoUART
PORT=/dev/cu.usbserial-0001

.PHONY: format
format:
	clang-format -i include/i2clib/*.h src/*.cc test/test_embedded/*.cc

docs: doxygen.conf Makefile include/i2clib/*.h src/*.cc
	doxygen doxygen.conf

.PHONY: clean
clean:
	${PLATFORMIO} --target clean
	rm -rf docs

.PHONY: tags
tags:
	ctags --extra=+f --languages=+C,+C++ --recurse=yes --links=no

.PHONY: test
test:
	${PLATFORMIO} test -vvv --test-port=${PORT} --upload-port=${PORT}
