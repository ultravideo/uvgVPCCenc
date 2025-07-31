uvgVPCCenc
=======

uvgVPCCenc is an academic open-source volumetric video encoder for the state of the art Video-based Point-Compression (V-PCC) standard. uvgVPCCenc is being developed in C++ under the BSD-3-Clause license. 
              
The uvgVPCC encoder accepts point cloud frames as input (with geometry being positive integers), supports all voxel sizes, and runs on Linux. In version 1.0, the focus is on building a functional encoding pipeline using only essential tools and algorithms inspired by the TMC2 reference software. Prioritizing practical encoding, we’ve omitted lossless tools and those adding significant computational complexity such as geometry reconstruction. In the future we plan on developing innovative methods to reduce the massive complexity of the V-PCC encoding process. 
  
uvgVPCCenc serves as a research platform for new coding tool development and other encoder research activities as well as provides a high-quality and practical V-PCC encoder for the public to use.

uvgVPCCenc is still under development. Speed and RD-quality will continue to improve.

Join our Discord channel to contact us [![Discord](https://img.shields.io/discord/973260924288901140?style=plastic)](https://discord.gg/fZpub7BPUA).

https://ultravideo.fi/uvgvpccenc.html for more information.


## Table of Contents

- [Compiling and testing uvgVPCCenc](#compilation-and-testing)
  - [Dependencies](#dependencies)
  - [Compilation](#compile-uvgvpccenc)
  - [Test](#test-uvgvpccenc)
- [Using uvgVPCCenc](#using-uvgvpccenc)
  - [Example](#example)
  - [Application parameters](#application-parameters)
  - [uvgVPCCenc parameters](#uvgvpccenc-parameters)
  - [Presets](#presets)
  - [Version 1.1](#uvgvpccenc-version-11)
- [Academic research](#academic-research)

## Compilation and testing
### Dependencies
To compile the library and the application, please install ```CMake``` and ```g++```.
On Debian/Ubuntu:
```
sudo apt install cmake g++
```

To generate the documentation, ```doxygen``` and ```graphviz``` are required.
On Debian/Ubuntu:
```
sudo apt install doxygen graphviz
```

### Compile uvgVPCCenc
To compile the encoder, please use following commands:
```
cmake --preset=Release
cmake --build --preset=Release
```

By default, the build application is located in ```_build/Release/src/app/```.
Refer to "Using uvgVPCCenc" section to learn how to use the library, and attached application.

### Test uvgVPCCenc
To test the encoder, please use following commands:
```
ctest --preset=Release                   
```

This runs small tests and checks whether the resulting encoding behaves as expected by verifying the bitstream MD5.

### Compile documentation
To generate the doxygen documentation, please use the following:
```
cmake --preset=docs
cmake --build --preset=docs
```
The doxygen is then accessible under docs/html/index.html.

## Using uvgVPCCenc

To demonstrate how to use the uvgVPCCenc library, a straightforward example application is provided in src/app, with its main() function located in [uvgVPCCencAppExample.cpp](src/app/uvgVPCCencAppExample.cpp).

### Example:
```
    uvgVPCCenc -i <path_to_ply> -n 10 -o out.vpcc
```

The mandatory parameters are input, output and the number of frames. The input path should use ```%0xd``` for frame numbering. Example sequences are available in the [UVG-VPC](https://ultravideo.fi/UVG-VPC) dataset. Example of an input sequence name: ```ReadyForWinter_UVG_vox10_25_0_250_%04d.ply```.

If the voxel size (aka voxel resolution or geometry precision) is not in the filename, the voxel size must be given: ```--geo-precision=10```.

If the start frame (the index of the first frame of the sequence) is not in the filename, the start frame must be given: ```--start-frame=0```.

uvgVPCCenc accepts .ply files, using positive integer for geometry.

The application option ```--uvgvpcc``` accept a string containing uvgVPCCenc parameters, separated by commas. Here is an example command:
```
    uvgVPCCenc -i <path_to_ply> -n 10 -o out.vpcc -t 20 --uvgvpcc rate=16-22-2,presetName=slow,mode=AI
```

The list of uvgVPCCenc parameters that can be modified with the ```--uvgvpcc``` option is defined in the ```parameterMap``` initialized in [parameters.cpp](src/lib/utils/parameters.cpp).


### Application parameters
```
    -i, --input <file>           Input filename (using %0Xd)
    -o, --output <file>          Output filename
    -n, --frames <number>        Number of frames to encode
    -s, --start-frame <number>   Frame number to start the encoding
    -g, --geo-precision <number> Geometry precision for encoding
    -t, --threads <number>       Maximum number of threads to be used (0 auto detection)
    -l, --loop-input <number>    Number of input loop (0 for inifinite loop)
        --uvgvpcc <params>       Encoder configuration parameters (see next section)
        --help                   Show this help message
        --version                Show version information
```

### uvgVPCCenc parameters

The uvgVPCCenc library includes several options that can only be configured using the API function uvgvpcc_enc::API::setParameter(...). The example application provides the --uvgvpcc <string> option, which passes the specified parameter string to the setParameter API function.

All user-accessible parameters for uvgVPCCenc are defined in the parameterMap variable, located in the file [parameters.cpp](src/lib/utils/parameters.cpp).

Here are a selection of common uvgVPCCenc parameters:
```
    presetName          Preset name, i.g. slow or fast
    rate                V-PCC rate, i.g. 16-22-2
    mode                Encoding mode, i.g. AI or RA
    logLevel            Level of logging, i.g. TRACE
```

### Presets

Six presets for Voxel 9, 10, and 11, labeled Fast and Slow, have been created to clarify the encoding pipeline parametrization of uvgVPCCenc. These presets were manually crafted starting from Kvazaar's own presets and refined through extensive tool space exploration using stochastic methods. While they are not exhaustive and do not address certain features, such as single- and double-layer options, they provide a practical reference for parameter adjustments. Notably, such presets are exclusive to uvgVPCCenc and do not exist for TMC2.

## uvgVPCCenc Version 1.1

Version 1.1 shows better performance with no quality degradation or change in compression ratio.

Compared to 1.0, uvgVPCCenc 1.1 features a better implementation of the point cloud and map processing parts (with no impact on 2D encoding speed), resulting in a speed-up of up to 1.7× in Fast voxel 9. Memory management is also significantly improved: uvgVPCCenc 1.1 runs smoothly with a small and stable RAM footprint over time. New developer features and bug fixes are also included in this version. As shown in the results below, compression efficiency (in terms of quality and bitrate) is not affected when switching from 1.0 to 1.1.

### Encoding speed comparison: Major performance boosts in non-video encoding tasks

| Voxel Size | Preset | Mode | Version | Speedup | FPS  |
|------------|--------|------|---------|---------|------|
| Voxel 10   | slow   | RA   | V1.0    |    -    | 0.8  |
|            |        |      | V1.1    | x1.1    | 0.9  |
|            |        | AI   | V1.0    |    -    | 1.0  |
|            |        |      | V1.1    | x1.14   | 1.2  |
|            | fast   | RA   | V1.0    |    -    | 4.7  |
|            |        |      | V1.1    | x1.70   | 7.9  |
|            |        | AI   | V1.0    |    -    | 5.0  |
|            |        |      | V1.1    | x1.81   | 9.1  |
| Voxel 9    | slow   | RA   | V1.0    |    -    | 3.7  |
|            |        |      | V1.1    | x1.09   | 4.0  |
|            |        | AI   | V1.0    |    -    | 4.4  |
|            |        |      | V1.1    | x1.13   | 4.9  |
|            | fast   | RA   | V1.0    |    -    | 22.3 |
|            |        |      | V1.1    | x1.63   | 36.3 |
|            |        | AI   | V1.0    |    -    | 24.8 |
|            |        |      | V1.1    | x1.75   | 43.4 |


### Compression efficiency comparison: Negligible impact on quality and bitrate

BD-BR PCQM for V1.1 using V1.0 as anchor (negative BD-BR shows better compression efficiency).

| Voxel Size | Preset | Mode | BD-BR PCQM |
|------------|--------|------|------------|
| Voxel 10   | slow   | RA   | -1.09%     |
|            |        | AI   |  0.06%     |
|            | fast   | RA   | -0.89%     |
|            |        | AI   |  0.01%     |
| Voxel 9    | slow   | RA   | -3.33%     |
|            |        | AI   | -0.27%     |
|            | fast   | RA   | -2.30%     |
|            |        | AI   |  0.12%     |
| | | **Total average**      | **-0.96%** |


## Academic research

If you use uvgVPCCenc in your research, please cite the most appropriate papers in the following papers:

```
[1] L. Fréneau, G. Gautier, A. Mercat, and J. Vanne, “uvgVPCCenc: Practical open-source encoder for fast V-PCC compression,” Accepted to ACM Multimedia System, Stellenbosch, South Africa, Mar.—Apr. 2025.
[2] L. Fréneau, G. Gautier, H. Tampio, A. Mercat, and J. Vanne, “Real-Time Video-based Point Cloud Compression,” in Proc. IEEE Visual Comm. Image Proc., Tokyo, Japan, Dec. 2024.
```
