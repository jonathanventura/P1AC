# P1AC

This repository provides code for our ICCV 2023 paper "P1AC: Revisiting Absolute Pose From a Single Affine Correspondence."  

### Abstract ###

Affine correspondences have traditionally been used to improve feature matching over wide baselines. While recent work has successfully used affine correspondences to solve various relative camera pose estimation problems, less attention has been given to their use in absolute pose estimation. We introduce the first general solution to the problem of estimating the pose of a calibrated camera given a single observation of an oriented point and an affine correspondence. The advantage of our approach (P1AC) is that it requires only a single correspondence, in comparison to the traditional point-based approach (P3P), significantly reducing the combinatorics in robust estimation. P1AC provides a general solution that removes restrictive assumptions made in prior work and is applicable to large-scale image-based localization. We propose a minimal solution to the P1AC problem and evaluate our novel solver on synthetic data, showing its numerical stability and performance under various types of noise. On standard image-based localization benchmarks we show that P1AC achieves more accurate results than the widely used P3P algorithm.

### Dependencies ###

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PoseLib](https://github.com/PoseLib/PoseLib)

### Code ###

Minimal solver code is provided in `p1ac/p1ac.h` and `p1ac/p1ac.cpp`.  The function `p1ac::solve()` accepts the following inputs:

* `x` 2D point observation in reference image
* `d` depth of point in reference image
* `n` surface normal in reference camera's coordinate system
* `y` 2D point observation in query image
* `A` 2x2 affine transformation between reference image and query image

and outputs at most eight rotation and translation solutions for the query image, in world-to-camera format and in the reference images' coordinate system.

An example program is provided in `examples/test_random_problems.cpp` which tests the solver on random synthetic data problems.

### Reference ###

Ventura, Jonathan, Kueklova, Zuzana, Sattler, Torsten and Baráth, Dániel.  [P1AC: Revisiting Absolute Pose From a Single Affine Correspondence.](https://arxiv.org/abs/2011.08790)  Proceedings of the International Conference on Computer Vision, 2023.

    @InProceedings{Ventura_2023_ICCV,
        author    = {Ventura, Jonathan and Kukelova, Zuzana and Sattler, Torsten and Bar\'ath, D\'aniel},
        title     = {P1AC: Revisiting Absolute Pose From a Single Affine Correspondence},
        booktitle = {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
        month     = {October},
        year      = {2023},
        pages     = {19751-19761}
    }
