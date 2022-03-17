# Stereo-LIME
## Stereo Camera assisted Localization in Mapped Environment

# Prerequisites
### 1.1 **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html)

Install [OpenCV](https://gist.github.com/Mahedi-61/804a663b449e4cdb31b5fea96bb9d561) and [PCL](https://pcl.readthedocs.io/projects/tutorials/en/master/) from source.

# Depth Map Generation

The left and right image from the stereo camera is used to compute the disparity map. From disparity map, we obtain the depth using the formula:

`disparity = x - x' = (B*f)/Z`

Here, B is baseline, i.e, distance between the left and right camera & f is the focal length of the camera. Z is the depth of that pixel value.
