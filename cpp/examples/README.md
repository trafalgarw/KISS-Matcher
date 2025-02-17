# KISS-Matcher 예제


## How to Build

1. 필요 Library:
- https://github.com/LimHyungTae/TEASER-plusplus (시간 측정 모듈 추가해 둚)
- https://github.com/MIT-SPARK/ROBIN (kiss-matcher의 매칭 부분 필요)


2. Then, run the following script to install this repository.

```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j 48
```


## How to Run Quatro

### Case A. Bunny dataset

```
OMP_NUM_THREADS=8 ./quatro_cpp_fpfh
```


![](materials/quatro_teaser_bunny.png)

(Red: source, green: target, blue: estimate from Quatro, magenta: estimate from TEASER++. The blue and magenta clouds are overlapped)

```bash
=====================================
           Quatro Results
=====================================
Error (deg): 0.990611
Estimated translation (m): 0.00152444
Time taken (s): 0.010767
=====================================
          TEASER++ Results
=====================================
Error (deg): 2.2537
Estimated translation (m): 0.00269122
Time taken (s): 0.012313
```

In general, if the yaw rotation is dominant in SO(3), Quatro showed a promising performance. This is because TEASER++ is a non-minimal solver, so some undesirable roll and pitch errors happen.

### Case B. KITTI dataset

The original FPFH for a 3D point cloud captured by a 64-channel LiDAR sensor takes **tens of seconds**, which is too slow. For this reason, we employ voxel-sampled FPFH, which is preceded by voxel-sampling. This is followed by the correspondence test.

```
OMP_NUM_THREADS=48 ./run_kiss_matcher_in_kitti KISS-Matcher
```

```
OMP_NUM_THREADS=48 ./run_kiss_matcher_in_kitti FPFH
```


---

![](materials/quatro_teaser_kitti.png)

(Red: source, green: target, blue: estimate from Quatro, magenta: estimate from TEASER++. The blue and magenta clouds are overlapped)

Both methods succeeded!

Note that, `optimizedMatching` is way more faster than `advancedMatching` (0.10 < 0.42).
It may output the command lines as follows:

```commandline
         [Build KdTree]: 0.019237 sec
   [Search using FLANN]: 0.060132 sec
       [Cross checking]: 0.014097 sec
           [Tuple test]: 0.005469 sec
```

In contrast, `advancedMatching` takes almost 0.45 sec, which is **four time slower** than my implementation.

```commandline
   [Build KdTree & matching]: 0.368105 sec
            [Cross checking]: 0.075848 sec
                [Tuple test]: 0.010516 sec
[Set unique correspondences]: 6e-06 sec
```

Interestingly, `parallel_reduce` of TBB is 7 times faster than single threading cross checking, which was implemented as:

```cpp
///////////////////////////////////////////
// Original, single-threaded implementation
///////////////////////////////////////////
std::vector<std::pair<int, int>> corres;
corres.reserve(std::max(nPti, nPtj) / 4); // reserve 1/4 of the maximum number of points, which is heuristic

///////////////////////////
/// INITIAL MATCHING
///////////////////////////
for (int j = 0; j < nPtj; j++) {
  // `dis` is a squared distance
  if (dis[j] > thr_dist * thr_dist) { continue; }
  const int &i = corres_K[j];

  if (i_to_j[i] == -1) {
    searchKDTree(&feature_tree_j, features_[fi][i], corres_K, dis, 1);
    i_to_j[i] = corres_K[0];
    if (corres_K[0] == j) {
      corres.emplace_back(i, j);
    }
  }
}
```

## Citation

If our research has been helpful, please cite the below papers:

```
@article{lim2022quatro,
    title={A Single Correspondence Is Enough: Robust Global Registration to Avoid Degeneracy in Urban Environments},
    author={Lim, Hyungtae and Yeon, Suyong and Ryu, Suyong and Lee, Yonghan and Kim, Youngji and Yun, Jaeseong and Jung, Euigon and Lee, Donghwan and Myung, Hyun},
    booktitle={Proc. IEEE Int. Conf. Robot. Autom.},
    year={2022},
    pages={Accepted. To appear}
    }
```

```
@article{yang2020teaser,
  title={{TEASER: Fast and certifiable point cloud registration}},
  author={Yang, Heng and Shi, Jingnan and Carlone, Luca},
  journal={IEEE Trans. on Robot.},
  volume={37},
  number={2},
  pages={314--333},
  year={2020}
}
```

### Copyright
- All codes on this page are copyrighted by KAIST published under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 License. You must attribute the work in the manner specified by the author. You may not use the work for commercial purposes, and you may only distribute the resulting work under the same license if you alter, transform, or create the work.
