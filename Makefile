
# PlatformIO install location.
PLATFORMIO=${HOME}/.platformio/penv/bin/platformio
PLATFORMIO=platformio
CLANG_FORMAT=/opt/homebrew/bin/clang-format

LIB_DIR=$(shell pwd)

.PHONY: format
format:
	${CLANG_FORMAT} -i include/i2clib/*.h src/*.cc test/test_embedded/*.cc

docs: doxygen.conf Makefile include/i2clib/*.h src/*.cc
	doxygen doxygen.conf

.PHONY: clean
clean:
	${PLATFORMIO} run --target clean
	rm -rf docs

.PHONY: tags
tags:
	ctags --extra=+f --languages=+C,+C++ --recurse=yes --links=no

.PHONY: test
test:
	${PLATFORMIO} test --project-dir=${LIB_DIR} --environment=esp32
