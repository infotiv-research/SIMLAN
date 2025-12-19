# RTF tests for paper

The following tests were performed on the PC Research2 with the following specs:

- Intel(R) Core(TM) i7-10700K CPU @ 3.80GHz
- NVIDIA GeForce RTX 2070 SUPER
- Corsair Vengeance LPX 32GB kit (2 × 16GB), 3000 MHz, CL16. running at 2133 MT/s
- MSI MEG Z490 UNIFY (MS-7C71)
- KINGSTON SA2000M8 1TB NVMe SSD
- Ubuntu 24.04.3 LTS

every instance in the tables below are referenced in %
with camera feed update rates as

- image 20 Hz
- depth 6 Hz
- semantic 10 Hz

number_of_agents=0 world=empty:

| number of cameras | image | image depth semantic |
| ----------------- | ----- | -------------------- |
| n=1               | 99    | 44                   |
| n=2               | 56    | 22                   |
| n=3               | 46    | 14                   |
| n=4               | 34    | 9                    |
| n=5               | 32    | 9                    |
| n=6               | 23    | 7                    |
| n=7               | 21    | 5                    |
| n=8               | 19    | 4                    |
| n=9               | 17    | 5                    |
| n=10              | 15    | 3                    |
| n=11              | 8     | 3                    |
| n=12              | 7     | 2                    |

camera_feed=image, worth noting the default world took ~30s to render in gazebo GUI no robots:

| world   | 0 cameras | 6 cameras | 12 cameras |
| ------- | --------- | --------- | ---------- |
| empty   | 99        | 24        | 7          |
| light   | 99        | 22        | 7          |
| medium  | 85        | 17        | 7          |
| default | 84        | 17        | 7          |

world=empty camera_feed=image:

| robots                               | 0 cameras | 6 cameras | 12 cameras |
| ------------------------------------ | --------- | --------- | ---------- |
| robot_agentx1                        | 99        | 11        | 5          |
| robot_agentx2                        | 99        | 11        | 4          |
| robot_agentx3                        | 99        | 9         | 4          |
| robot_agentx4                        | 99        | 9         | 4          |
| humanoidx1                           | 99        | -         | -          |
| humanoidx2                           | 99        | -         | -          |
| panda_armx1                          | 99        | -         | -          |
| robot_agentx4+humanoidx1             | 99        | 9         | 4          |
| robot_agentx4+humanoidx2             | 99        | 8         | 4          |
| robot_agentx4+humanoidx2+panda_armx1 | 99        | 9         | 3          |

navigation world=light camera_feed=image:

| robots         | 0 cams | 9 cams |
| -------------- | ------ | ------ |
| humanoidsx2    | 99     | 13     |
| humanoidsx4    | 90     | 10     |
| robot_agentsx4 | -      | 5      |
