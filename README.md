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


## :package: Installation

> All installations are set up automatically in an out-of-the-box manner.

Run the command below:

```commandline
make deps
```

### C++

Run the command below. That's it.

```commandline
make cppinstall
```

<details>
  <summary><strong>Q. How doest it work?</a></strong></summary>

  The `cppinstall` command is encapsulated in the [Makefile](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/Makefile). and `cppinstall` calls `deps` to automatically install the dependencies.
  In addition, KISS-Matcher requires [ROBIN](https://github.com/MIT-SPARK/ROBIN).
  But it's also automatically linked via [robin.cmake](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/cpp/kiss_matcher/3rdparty/robin/robin.cmake)

  After installation, `kiss_matcher` and `robin` are place in the installation directory using the following commands in the `Makefile`, respectively:
    ```
	@$(SUDO) cmake --install cpp/kiss_matcher/build
	@$(SUDO) cmake --install cpp/kiss_matcher/build/_deps/robin-build
    ```
</details>


#### How To Use in Other Package?

See [CMakeLists.txt](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/cpp/examples/CMakeLists.txt) in `cpp/examples`.

### Python

It should work. TBU.

---

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


## Contributing

Like [KISS-ICP](https://github.com/PRBonn/kiss-icp),
we envision KISS-Matcher as a community-driven project, we love to see how the project is growing thanks to the contributions from the community. We would love to see your face in the list below, just open a Pull Request!

<a href="https://github.com/PRBonn/kiss-icp/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=MIT-SPARK/KISS-Matcher" />
</a>
