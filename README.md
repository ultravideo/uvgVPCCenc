# uvgVPCCenc

uvgVPCCenc is an academic open-source volumetric video encoder for the state of the art Video-based Point-Compression (V-PCC) standard. uvgVPCCenc is being developed in C++ under the BSD-3-Clause license. 
              
The uvgVPCC encoder accepts point cloud frames as input, supports all voxel sizes, and runs on Linux, ~~Windows~~*(coming soon), and macOS. In version 1.0, the focus is on building a functional encoding pipeline using only essential tools and algorithms inspired by the TMC2 reference software. Prioritizing practical encoding, we’ve omitted lossless tools and those adding significant computational complexity. In the future we plan on developing innovative methods to reduce the massive complexity of the V-PCC encoding process. 
  
uvgVPCCenc serves as a research platform for new coding tool development and other encoder research activities as well as provides a high-quality and practical V-PCC encoder for the public to use.

## Compilation and testing

To compile and test the encoder, please use following commands:

```
cmake --preset=CI
cmake --build --preset=CI
ctest --preset=CI
```


