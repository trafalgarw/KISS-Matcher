<div align="center">
    <h1>KISS-Matcher</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="80%"/></p>
  <p><strong><em>Keep it simple, make it scalable.</em></strong></p>
</div>

---

# KISS-Matcher Example codes


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

This study is the culmination of my continuous research since my graduate school years.
If you use this library for any academic work, please cite our original [paper](https://arxiv.org/abs/2409.15615), and refer to other papers that are highly relevant to KISS-Matcher.

<details>
  <summary><strong>See bibtex lists</a></strong></summary>

   ```bibtex
    @inproceedings{lim2025icra-KISSMatcher,
      title={{KISS-Matcher: Fast and Robust Point Cloud Registration Revisited}},
      author={Lim, Hyungtae and Kim, Daebeom and Shin, Gunhee and Shi, Jingnan and Vizzo, Ignacio and Myung, Hyun and Park, Jaesik and Carlone, Luca},
      booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
      year={2025},
      codeurl   = {https://github.com/MIT-SPARK/KISS-Matcher},
      note   = {Accepted. To appear}
    }
   ```

   ```bibtex
    @article{Lim24ijrr-Quatropp,
      title={{Quatro++: R}obust global registration exploiting ground segmentation for loop closing in {LiDAR SLAM}},
      author={Lim, Hyungtae and Kim, Beomsoo and Kim, Daebeom and Mason Lee, Eungchang and Myung, Hyun},
      journal={Int. J. Robot. Res.},
      pages={685--715},
      year={2024},
      doi={10.1177/02783649231207654}
    }
   ```

   ```bibtex
   @inproceedings{Lim22icra-Quatro,
       title={A single correspondence is enough: Robust global registration to avoid degeneracy in urban environments},
       author={Lim, Hyungtae and Yeon, Suyong and Ryu, Soohyun and Lee, Yonghan and Kim, Youngji and Yun, Jaeseong and Jung, Euigon and Lee, Donghwan and Myung, Hyun},
       booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
       pages={8010--8017},
       year={2022}
   }
   ```

   ```bibtex
   @InProceedings{Shi21icra-robin,
       title={{ROBIN:} a Graph-Theoretic Approach to Reject Outliers in Robust Estimation using Invariants},
       author={J. Shi and H. Yang and L. Carlone},
       booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
       note = {arXiv preprint: 2011.03659, \linkToPdf{https://arxiv.org/pdf/2011.03659.pdf}},
       pdf="https://arxiv.org/pdf/2011.03659.pdf",
       year={2021}
   }
   ```

   ```bibtex
   @article{Yang20tro-teaser,
       title={{TEASER: Fast and Certifiable Point Cloud Registration}},
       author={H. Yang and J. Shi and L. Carlone},
       journal={IEEE Trans. Robot.},
       volume = 37,
       number = 2,
       pages = {314--333},
       note = {extended arXiv version 2001.07715 \linkToPdf{https://arxiv.org/pdf/2001.07715.pdf}},
       pdf={https://arxiv.org/pdf/2001.07715.pdf},
       Year = {2020}
   }
   ```

   ```bibtex
   @inproceedings{Zhou16eccv-FGR,
       title={Fast global registration},
       fullauthor={Zhou, Qian-Yi and Park, Jaesik and Koltun, Vladlen},
       author={Q.Y. Zhou and J. Park and V. Koltun},
       booktitle={Proc. Eur. Conf. Comput. Vis.},
       pages={766--782},
       year={2016}
   }
   ```

</details>
