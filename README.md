uvgVPCCenc
=======

uvgVPCCenc is an academic open-source volumetric video encoder for the state of the art Video-based Point-Compression (V-PCC) standard. uvgVPCCenc is being developed in C++ under the BSD-3-Clause license. 
              
The uvgVPCC encoder accepts point cloud frames as input (with geometry being positive integers), supports all voxel sizes, and runs on Linux, ~~Windows~~*(coming soon), and macOS. In version 1.0, the focus is on building a functional encoding pipeline using only essential tools and algorithms inspired by the TMC2 reference software. Prioritizing practical encoding, we’ve omitted lossless tools and those adding significant computational complexity such as geometry reconstruction. In the future we plan on developing innovative methods to reduce the massive complexity of the V-PCC encoding process. 
  
uvgVPCCenc serves as a research platform for new coding tool development and other encoder research activities as well as provides a high-quality and practical V-PCC encoder for the public to use.

uvgVPCCenc is still under development. Speed and RD-quality will continue to improve.

https://ultravideo.fi/uvgvpccenc.html for more information.

## Table of Contents

- [Compiling and testing uvgVPCCenc](#compilation-and-testing)
- [Using uvgVPCCenc](#using-uvgVPCCenc)
  - [Example](#example)
  - [Parameters](#parameters)
- [Presets](#presets)

## Compilation and testing

To compile and test the encoder, please use following commands:

```
cmake --preset=CI
cmake --build --preset=CI
ctest --preset=CI
```

## Using uvgVPCCenc

To demonstrate how to use the uvgVPCCenc library, a straightforward example application is provided in src/app, with its main() function located in [uvgVPCCencAppExample.cpp](src/app/uvgVPCCencAppExample.cpp).

### Example:
    uvpVPCCenc -i ReadyForWinter_UVG_vox10_25_0_250_%04d.ply -n 10 -o out.vpcc

The mandatory parameters are input, output and the number of frames. The input path should use ```%0xd``` for frame numbering.

If the voxel size (aka voxel resolution or geometry precision) is not in the filename, the voxel size must be given: ```--geo-precision=10```.

If the start frame (the index of the first frame of the sequence) is not in the filename, the start frame must be given: ```--start-frame=0```.

uvgVPCCenc accepts .ply files, using positive integer for geometry.

The application option ```--uvgvpcc``` accept a string containing uvgVPCCenc parameters, separated by commas. Here is an example command:
    uvgVPCCenc -i ReadyForWinter_UVG_vox9_25_0_250_%04d.ply -n 10 -o out.vpcc -t 20 --uvgvpcc rate=16-22-2,presetName=slow,mode=AI,

Speed and compression quality can be selected with ```--uvgvpcc presetName```, or by setting the uvgVPCCenc parameters manually.

## Parameters

### Application parameters

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

### uvgVPCCenc parameters

The uvgVPCCenc library includes several options that can only be configured using the API function uvgvpcc_enc::API::setParameter(...). The example application provides the --uvgvpcc <string> option, which passes the specified parameter string to the setParameter API function.

All user-accessible parameters for uvgVPCCenc are defined in the parameterMap variable, located in the file [parameters.cpp](src/lib/utils/parameters.cpp).

Here are a selection of common uvgVPCCenc parameters:

    presetName          Preset name, i.e. slow or fast
    rate                V-PCC rate, i.e. 16-22-2
    mode                Encoding mode, i.e. AI or RA
    logLevel            Level of logging, i.e. TRACE

## Presets

Six presets for Voxel 9, 10, and 11, labeled Fast and Slow, have been created to clarify the encoding pipeline parametrization of uvgVPCCenc. These presets were manually crafted starting from Kvazaar's own presets and refined through extensive tool space exploration using stochastic methods. While they are not exhaustive and do not address certain features, such as single- and double-layer options, they provide a practical reference for parameter adjustments. Notably, such presets are exclusive to uvgVPCCenc and do not 
exist for TMC2.

