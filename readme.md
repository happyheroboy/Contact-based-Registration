# Efficient contact-based registration for minimally invasive anterior hip arthroplasty

## Introduction

We present an efficient patient-image registration method termed the *contact-based registration* method, which only relies on one internal contact within the cup-shaped acetabulum and two external collections of the anterior superior iliac spine (ASIS). Our method decouples translation from the overall transformation by computing the stable point and estimates rotation in a center-aware manner.

## Requirements

1) Opencv 4.5

2) PCL 1.13

3) HIKROBOT's SDK

## Usage

We provide Matlab code of the visual stable point computation method.  Run "CR.m" to see demo examples.
In addition, we upload the C++ code for the contact-based registration method. The main function is "test_Main.cpp".

## Citation

Please cite our papers if the code is useful for your research:

```
@article{xie2025efficient,
  title={Efficient contact-based registration for minimally invasive anterior hip arthroplasty},
  author={Xie, Xianzhong and Zhu, Mingzhu and Chen, Weijian and Xu, Jie and He, Bingwei},
  journal={Biomedical Signal Processing and Control},
  volume={102},
  pages={107216},
  year={2025},
  publisher={Elsevier}
}
```

> 
