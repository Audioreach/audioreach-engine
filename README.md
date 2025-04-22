# AudioReach Engine

## Introduction
This repository hosts implementation of generic signal processing framework, modules which can be used as part of audio graph, and platform & OS abstraction layer for different architecture & SoCs

## Documentation
Refer AudioReach docs [here](https://audioreach.github.io/design/arspf_design.html)

## Build and Usage
This page provides instructions to build, Adding new Modules, describes build artifacts and provides information on code layout.
- [Setup the environment](#setup-the-environment)
- [Build and Install](#build-and-install)
- [Adding New Module](#adding-new-module)
- [Directory Layout](#directory-layout)
- [Build Artifacts](#build-artifacts)

Please note that AudioReach Engine support OpenEmbedded system and Qualcomm Hexagon DSP at the time of writing.

### Setup the environment

#### Build Management Software
Build infrastructure is developed using CMake and requires CMake>=3.24.
Follow download instruction from [CMake Download Page](https://cmake.org/download/)

#### Qualcomm Hexagon Toolchain
Hexagon toolchain is bundled with Hexagon SDK which can be downloaded from [Qualcomm Developer Page](https://developer.qualcomm.com/software/hexagon-dsp-sdk/tools).

## Build and Install
### Get AudioReach Engine
```
git clone https://github.com/Audioreach/audioreach-engine.git <WORKSPACE>
```

### Kick-off build process

#### Open Embedded System
Refer meta-audioreach [README](https://github.com/Audioreach/meta-audioreach?tab=readme-ov-file#openembedded-build--development-process)
for instructions to use AudioReach Engine on OpenEmbedded system.

#### Hexagon DSP

```
$ cd $WORKSPACE
$ mkdir build && cd build
$ cmake -DARCH=hexagon -DV_ARCH=v65 -DCONFIG=defconfig ..
$ make menuconfig
```
Enter Hexagon Architecture to update the absolute path to Hexagon toolchain and Hexagon SDK which would update Kconfig Symbol HEXAGON_TOOLS_ROOT and HEXAGON_SDK_ROOT.

In addition, use make menuconfig to choose the modules and feature of SPF to build

```
$ make -j20
```
Once build process is done, all libraries both static and dynamic are generated in top
build directory

### Build Artifacts
- Libraries: All libraries both static and dynamic are generated in top build directory
	- libspf.so:  This is SPF in binary form. It is expected to be loaded onto audio DSP dynamically.
	- Modules which were selected through menuconfig

- AMDB files:
	- All amdb files are generated at location fwk/spf/amdb/autogen/\<target directory e.g. hexagon or linux\>

- H2XML Files:
	- H2XML files are generated at location: build/h2xml_autogen/

## Adding new Module

Adding new modules to AudioReach Engine is very simple by following steps below:

- Step 1:
	- Add module specific Kconfig symbol in modules/Kconfig
Feel free to add module customization configs here.
Ex:
```
config ENCODER
        tristate "Enable Example Encoder Library"
        default y
```
- Step 2:
	- Create your own module directory in modules directory
with source files & headers, finally add the directory
name to modules/CMakeLists.txt
ex:
```
if(CONFIG_ENCODER)
	add_subdirectory(examples/encoder)
endif()
```

- Step 3:
	- Define module CMakeLists, to make things easy you could use
spf_module_sources function
This function takes various obviously named arguments

Ex:
```
spf_module_sources(
	KCONFIG		CONFIG_ENCODER
	NAME		example_encoder
	MAJOR_VER	1
	MINOR_VER	0
	AMDB_ITYPE	"capi"
	AMDB_MTYPE	"encoder"
	AMDB_MID	"0x0700109B"
	AMDB_TAG	"capi_example_enc"
	AMDB_MOD_NAME	"MODULE_ID_EXAMPLE_ENC"
	AMDB_FMT_ID1	"MEDIA_FMT_ID_EXAMPLE"
	SRCS		${example_encoder_sources}
	INCLUDES	${example_encoder_includes}
	H2XML_HEADERS	"api/example_encoder_module_api.h"
	CFLAGS		""
)
```
Thats it! depending on the config this module will be either built
as static or dynamic module. You can also add module specific customizations using Kconfig.

### AMDB
AMDB stands for Audio Module DataBase. When a graph is created, framework looks for entry point functions of the modules.
This information is retrieved from the AMDB. AMDB handles both static and dynamic modules.
- NAME: Name of the module
- MAJOR_VER: Major version of the module
- MINOR_VER: Minor version of the module
- AMDB_ITYPE: Interface type of the module. Only supported value: "capi"
- AMDB_MTYPE: Module type: Supported values "generic", "decoder", "encoder", "packetizer", "depacketizer", "converter", "detector", "generator", "pp", "end_point", "framework"
- AMDB_MID: GUID of the module E.g. "0x0700109B"
- AMDB_TAG: Tag is the prefix used for entry point function <tag>+"_get_static_properties" and <tag>+"_init" give the entry-point function names for CAPI.
- AMDB_MOD_NAME: Name of the module.
- AMDB_FMT_ID1:	Format ID for decoders and encoders. E.g. MEDIA_FMT_ID_MP3 or NOT_APPLICABLE
- SRCS:	source file list
- INCLUDES:	inc path list.
- H2XML_HEADERS: H2XML header files.


## Directory Layout
- arch
	- This directory is expected to host architecture, e.g Hexagon DSP, specific file and configs ex: hexagon/configs/defconfig has default build kconfig

- fwk
	- Contains the signal processing framework including APM, containers, APIs, and few modules essential for framework functionality.
- modules
	- Contains modules aka signal processing algorithms wrapped in CAPI.

## License
AudioReach Engine is licensed under the BSD-3-Clause-Clear. Check out the [LICENSE](LICENSE) for more details
