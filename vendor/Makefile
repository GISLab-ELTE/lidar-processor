CMAKE = cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
ROOT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
MKDIR = mkdir -p
RMDIR = rm -rf

.PHONY: all
all: LASlib libpointmatcher

.ONESHELL:
.PHONY: LASlib
LASlib:
	cd LAStools
	$(MKDIR) build && cd build
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=../install ..
	$(MAKE)
	$(MAKE) install

.ONESHELL:
.PHONY: libpointmatcher
libpointmatcher: libnabo
	cd libpointmatcher
	$(MKDIR) build && cd build
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=../install ..
	$(MAKE)
	$(MAKE) install

.ONESHELL:
.PHONY: libnabo
libnabo:
	cd libnabo
	$(MKDIR) build && cd build
	$(CMAKE) -DCMAKE_INSTALL_PREFIX=../install ..
	$(MAKE)
	$(MAKE) install

.PHONY: clean
clean:
	$(RMDIR) LAStools/build LAStools/install
	$(RMDIR) libnabo/build libnabo/install
	$(RMDIR) libpointmatcher/build libpointmatcher/install
