# Week 1 blog
**Date:** 31 Jan - 6 Feb

During the group discussion and work in week 1, we downloaded the software to be used and configured the development environment of the project.
The purpose of this project is to use software to simulate the movement of a quadruped robot, so a suitable development environment is very important. In addition, the team members learned how to use GitHub for development collaboration, especially the git function, which will be very helpful for future work.
This is the beginning of the project, no major code parts are involved. So the first week was not very difficult.


## Brief introduction to the project

Use the knowledge of kinematic modeling to simulate the quadruped robot on the pybullet platform with python, so that it can complete simple actions.

## Divide tasks for our project

Basically, the task of our project can be divided into two parts, programming and writing parts. The programming part consists of configure the simulation environment, build a simulation platform and coding for the simulation. The writing part consists of blogs, supervisor weekly meeting log, poster and report.

## Dispatch tasks

The initial plan is each person complete the simulation of one action. Thus, we can complete the simulation of four actions in total. The writing part will be done by the whole group. Minghong and Kai are responsible for blog writing, Zepeng is responsible for the writing of supervisor weekly meeting log, and Zehao is responsible for the preparation of the report in advance. After the programming part is complete, all of us write the rest of the report together.

## Goals for week 1

The main goal of the first week is to complete the construction of the simulation environment. Since Minghong has some experience in this field, he is responsible for guiding us to achieve our goal. The secondary task is to prepare software, plug-ins, etc. required for poster and other work in advance. All in all, it's about getting ready.

## Implementation of the task

First of all, in order to achieve better communication and file interaction among group members, we decided to use Github as our communication platform and created a repository called year2-proj.

![Github repository](https://lh3.googleusercontent.com/7QdU1M6eamvrinmSt-rcjyeHrok4XLPLHfoOs0-HHd9yVU1lroYVQgu9i18McI2l3Z9MBEcxvRBRktnRXVVLoGkwGZHuLdqwCAV9RliIsR1ZOVc0JSu4atDbqAq2w_SVM6tziuZwzFA8MjRtLYYOPmhR5N4ebjxm8uoVaSx1Ja_6JsjGBdP9nMV1-vRH5TqgD3wYGGkA2VWBa6LMYzZ-rYfjM8IgNnshdM2kmPQvVCkPHx4yS6mD5e-AJbfHC2uTLJwTAEdaA7RIwNXBl6jtfaXYrngAUVZghx7ip6lraxUb7c3BCNyQcunwMgwISTqdTFc2kAAb9g7OTQ4MxWctAaloFdIRdO68E4pPWpcMBzqBEHUQbed15SDe2yRXS0ycKJPQv9mGBXoN_9GF0Vl91JtvX7fLJAFPfNbttpftWK_ptC91TkEa3_Z6ujiSb3yi01rQT3yVLOmIUvm33f4S2zbpMvXb0EJft1b8nxGIbE-vfZbH_A7NgZuojkTT3EUeaf19HNFXY0MxW4A_QLmSoJNvTwCNBTxudA-aa9sD-LeKjic1KcRmEbTNzesAS7YhvxzYM9IUHafcBiFkAOtwpeQvhZK73xJcqja8uTSs6Nbv7BDGQBwx2niPp4Th37UUAOBEQKCzFEsG6ANaM0F2g_aPlFqgp0awR5sBzPzp8cTPDpY09iGtOjg5u-ljKuJVeqLFq3fguMFtPmB0znfevw=w1920-h937-no?authuser=0)

After that we chose Pybullet as our simulation platform. The reason for choosing Pybullet is that the installation of Pybullet is very simple compared to other simulation software. Secondly, Pybullet is free.

![Pybullet](https://lh3.googleusercontent.com/_fe-210zHo53QZB-_bbKHMRfbmrURvLIbqK56c0xt_w5tE7hpK13uOkoRbd6H0yqbK6ICRyWrUeh3ltUbCdI7NjoARH7-_pty61d05Ki5jNPXepovxwz-TFEvNq3pGoE8j8Hk5s_A0JsI-YsfCtqEZEebDXFaZC1eh-N9Pz8uUcpvg9nxP6Jd5WGPZ1_E2wo6JJzyh4bEeh_h6DefJhqar2S0L8xPEDDEd6FR06A3DTiuB2wYhMGc4tVFDpAw1Vk1deafmFb9de441zSGs8q8i7MOF5xGlauhoxTo5FJjNIvkmAz1zNfOjgUoZg3MbOgWr5eWF9egrG5xKAO0HUvehZHT6YhNIa8QVzLVI2nZrq9SljmKuMlkaM-_cj3sYrkeKzvgV_4_1bqbnOvvY-6o-W1atb9oXG96zR2z8qxuhMS4_8pio6Z1H02B67oU4o4A5zuWVXG5li4VvTEE7nmIEDhUixOraeUOBPKG5mfkooF4CxxPAQV8yH6pbxy9wUOp4IqdCNJ0L2Bv5vXfP_ITwsePt_m1vmHB9CEnIyvdVn7tFyqIkkzhMbP86kCjvjJAHfQFj6biw_EHqF70wt5bocDdCcT2oMheLLbMmCm5OxnmaU9DmPdOODMCl1QOg9ARAzTxHgjGEOLOlnJMmp3w8zWVNoGkME_ms_JrGdl4vWK65rBeteAmBaSPK9PhcO74ZkNpZK1v3Z-SsbIsXlrLQ=w1023-h799-no?authuser=0)

Additionally, we installed VSCode as the tool for the programming part. And installed a series of plug-ins in the software. For example, latex, used to make posters; Markdown, used to write blogs and meeting logs; jupyter, used to help us write programs

![VSCode](https://lh3.googleusercontent.com/Nn_97XCzKE7Dd6z2hiXH66hrijxcr1KK_qWDEmQaaCZIEGOWWmv5Kzrr2c1NoXYhzN4rZVVNVIWajnW9jrvYeF-3NdSBKHWehIHwfShWDq8JnK0_nFQyJy9Mg8FZBNIyRkiI4s6BjIF58LDqzRwkBvXXMsU7jp5fsZuy9ysWPzl1ywOg58drkM-ZlMf-iDnEyUwcwx6PBaKv3XMU6pixq_BVVaXCE4BHkI74cLJORUo5N1nMzHVFM1LsLGX2gHANnthzmmjd9jySEMmfi3gYSkSwF0I4DJyajaVHCQNri1nq7b2N_UBrdi2N6Khkq4bcF4MzWgR_rg7gqoWzChvaIqqQJ0JM_atOjVi2_q91LlLp6ByKZahHqaR7RdIkor5keQstJcxHD7kz4a0H-ImtlcZwaCxewsQ3daaQdmFeh8Y9JoqTX8IQv_H3CXw8KTVxrFx3_nQTAiKMXapxw1eDrB5NiLEihfxJhKbly0LanM_eRhXQafszU6mvARC0VuihcJ_3tLIxm0wy3ewrHfSSJAtnMIhCpQ68flOczPwUQOayB2mL6tMzMDpGAfa3VBp0eaGFCgVysXIVKu7DgCDZmuv5jAYjBhXBPC22LnKwoL5KfppgwC7Epoo53W7T5RhhASTnf07dEiHaBSdf_Kgz61BABPecEp7icTEOvXmLKmfLZKwLsG39Z-fFG6IYZumf5N17ipcA3M6qlkGw6EJFjg=w1730-h937-no?authuser=0)


## Week 1 summary

This week we successfully completed the preset tasks, building the simulation environment and preparing for the future work.
