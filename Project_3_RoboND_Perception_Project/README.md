[//]: # (Image References)
[pr2_robot]: ./images/pr2_robot.png
[pipeline_0_raw_cloud]: ./images/pipeline_0_raw_cloud.png
[pipeline_1_outlier_removal_filter]: ./images/pipeline_1_outlier_removal_filter.png
[pipeline_2_voxel_grid_filter]: ./images/pipeline_2_voxel_grid_filter.png
[pipeline_3_passthrough_filter]: ./images/pipeline_3_passthrough_filter.png
[pipeline_4_extracted_inliers]: ./images/pipeline_4_extracted_inliers.png
[pipeline_5_extracted_outliers]: ./images/pipeline_5_extracted_outliers.png
[dbscan_object_cluster]: ./images/dbscan_object_cluster.png
[confusion_matrices]: ./images/normalized_confusion_matrices.JPG
[world_1_object_recognition]: ./images/world_1_object_recognition.JPG
[world_2_object_recognition]: ./images/world_2_object_recognition.JPG
[world_3_object_recognition]: ./images/world_3_object_recognition.JPG

# 3D Perception Project
---

![pr2 robot][pr2_robot]

This project focuses on 3D perception using a PR2 robot simulation utilizing an RGB-D camera. The goal of perception is to convert sensor input into a point cloud image where specific objects can be identified and isolated.

The three main parts of perception include filtering the point cloud, clustering relevant objects, and recognizing objects.

To learn more about sensor types and camera calibration, read [this](Sensor_Types.md) page.

## Contents

- [Filtering](#filtering)
- [Clustering for Segmentation](#clustering-for-segmentation)
- [Object Recognition](#object-recognition)
- [PR2 Robot Simulation](#pr2-robot-simulation)
- [Code Sources](#code-sources)
- [Run the Simulation](#run-the-simulation)

## Filtering

Processing the complete pointcloud including the excessive data is inefficient and leads to wastage of compute cycles. To alleviate this problem, we can use some simple techniques called point cloud filtering to remove additional data points. Filtering not only removes useless and excessive data, but also the data that is deemed adversarial for a given application. Adversarial data, in this case might correspond to an object on the ground that resembles the target object on the table.


The raw point cloud object from the PR2 simulation looks like this:

![raw cloud object][pipeline_0_raw_cloud]

Some of the most commonly used filters are:

1. Outlier removal filter

2. Voxel grid Downsampling filter

3. Passthrough filter

4. RANSAC Plane Segmentation

### Outlier Removal Filter

While calibration takes care of distortion, noise due to extra factors like dust in the environment, humidity in the air, or presence or various light sources lead to sparse outliers which corrupt the results even more. Such outliers lead to complications in the estimation of point cloud characteristics like curvature, gradients etc leading to erroneous values, which in turn might cause failures at various stages in our perception pipeline. One of the filtering techniques used to remove such outliers is to perform a statistical analysis in the neighbourhood of each point and remove those points which do not meet a certain criteria. The outlier removal filter is one of such techniques. For each point in the point cloud, it computes the distance to all of its neighbours and then calculates a mean distance. By assuming a gaussian distribution, all points whose mean distances are outside of an interval defined by the global distances mean + std-deviation are considered to be outliers and are removed from the point cloud.

For the PR2 simulation, I found that a mean k value of *20* and a standard deviation threshold of *0.1* provided the optimal outlier filtering. Here is the cloud after performing the outlier removal filter.

![outlier removal filtering][pipeline_1_outlier_removal_filter]

### Voxel Grid Filter

![voxel grid filter][pipeline_2_voxel_grid_filter]

The raw point cloud will often have more details than required, causing excess processing power use when analyzing it.

So in many cases, it is advantageous to downsample the data. In particular, we are going to use a Voxelgrid Downsampling Filter to derive a point cloud that has fewer points but should still do a good job of representing the input point cloud as a whole. A voxel grid filter allows us to downsample the data by taking the spatial average of the points in the cloud confined by each voxel. We can adjust the sampling size by setting the voxel size along each dimension. The set of points which lie within the bounds of a voxel are assigned to that voxel and statistically combined into one output point.

I used an X, Y, and Z voxel grid filter leaf size of *0.01*. This was a good compromise of leaving enough detail while minimizing processing time.

### Passthrough Filter

![passthrough filter][pipeline_3_passthrough_filter]

If we have some prior information about the location of our target in the scene, we can apply a passthrough filter to remove useless data from our point cloud. The passthrough filter works much like a cropping tool, which allows us to crop any given 3d point cloud by specifying an axis with cutoff values along that axis. The regions we allow to passthrough, are often referred to as "regions of interest".

The PR2 robot simulation required passthrough filters for both the Y and Z axis (global). This prevented processing values outside the area immediately in front of the robot. For the Y axis, I used a range of *-0.4* to *0.4*, and for the Z axis, I used a range of *0.61* to *0.9*.

### RANSAC Plane Segmentation

Next, in our perception pipeline, we need to remove the table itself from the scene. To do this, we will use a popular technique known as Random Sample Concensus or RANSAC. RANSAC is an algorithm that we can use to identify points in our dataset that belong to a particular model. The RANSAC algorithm assumes that all of the data in a dataset is composed of both inliers and outliers, where inliers are defined by a particular model with a specific set of parameters, while outliers do not fit that model and hence can be discarded. 

If we have a prior knowledge of a certain shape being present in a given dataset, we can use RANSAC to estimate what pieces of the point cloud set belong to that shape by assuming a particular model. By modelling the table as a plane we can remove it from the point cloud. 

On the other hand, a disadvantage of RANSAC is that there is no upper limit on the time it can take to compute the model parameters. This is alleviated by choosing a fixed number of iterations but that also has its own demerits. If we choose a lower number of iterations, the solution obtained may not be optimal. In this way, RANSAC offers a trade-off between computation time and model detection accuracy.

I used a RANSAC max distance value of *0.01*.

The extracted inliers includes the table. It looks like this:

![RANSAC plane segmentation - extracted inliers][pipeline_4_extracted_inliers]

The extracted outliers contains the objects on the table, and looks like this:

![RANSAC plane segmentation - extracted outliers][pipeline_5_extracted_outliers]

## Clustering for Segmentation

With a cleaned point cloud, it is possible to identify individual objects within it. One straightforward approach is to use Euclidean clustering to perform this calculation.

The two main algorithms possible include:

- K-means

- DBSCAN

### K-means

K-means clustering algorithm is able to group data points into n groups based on their distance to randomly chosen centroids. However, K-means clustering requires that you know the number of groups to be clustered, which may not always be the case.

### DBSCAN

Density-based spatial cluster of applications with noise (DBSCAN) (sometimes called *Euclidean clustering*) is a clustering algorithm that creates clusters by grouping data points that are within some threshold distance from their nearest neighbor.

DBSCAN is unique over k-means because you don't need to know how many clusters to expect in the data. However, you do need to know something about the density of the data points being clustered.

Performing a DBSCAN within the PR2 simulation required converting the XYZRGB point cloud to a XYZ point cloud, making a k-d tree (decreases the computation required), preparing the Euclidean clustering function, and extracting the clusters from the cloud. This process generates a list of points for each detected object.

By assigning random colors to the isolated objects within the scene, I was able to generate this cloud of objects:

![DBSCAN object cluster][dbscan_object_cluster]

## Object Recognition

The object recognition code allows each object within the object cluster to be identified. In order to do this, the system first needs to train a model to learn what each object looks like. Once it has this model, the system will be able to make predictions as to which object it sees.

### Capture Object Features

Color histograms are used to measure how each object looks when captured as an image. Each object is positioned in random orientations to give the model a more complete understanding. For better feature extraction, the RGB image can be converted to HSV before being analyzed as a histogram. The number of bins used for the histogram changes how detailed each object is mapped, however too many bins will over-fit the object.

The code for building the histograms can be found in [features.py](PR2-Project/sensor_stick/src/sensor_stick/features.py).

The [capture_features_pr2.py](PR2-Project/sensor_stick/scripts/capture_features_pr2.py) script saved the object features to a file named `training_set_pr2.sav`. It captured each object in *50* random orientations, using the *HSV* color space and *128* bins when creating the image histograms.

### Train SVM Model

A support vector machine (SVM) is used to train the model (specifically a SVC). The SVM loads the training set generated from the `capture_features_pr2.py` script, and prepares the raw data for classification. I found that a *linear kernel* using a C value of *0.1* builds a model with good accuracy.

I experimented with cross validation of the model and found that a *50* fold cross validation worked best. A *leave one out* cross validation strategy provided marginally better accuracy results, however, it required much longer to process.

In the end, I was able to generate an accuracy score of 92.17%. The [train_svm.py](PR2-Project/RoboND-Perception-Project/pr2_robot/scripts/train_svm.py) script trained the SVM, saving the final model as `model.sav`.

The confusion matrices below shows the non-normalized and normalized results for a test case using the trained model generated above.

![Confusion matrices for the non-normalized and normalized test case][confusion_matrices]

## PR2 Robot Simulation

The PR2 robot simulation has three test scenarios to evaluate the object recognition performance. The following sections demonstrate each scenario.

### Test 1

![Test 1 object recognition][world_1_object_recognition]

View the goal [pick list](PR2-Project/RoboND-Perception-Project/pr2_robot/config/pick_list_1.yaml), and the calculated [output values](PR2-Project/RoboND-Perception-Project/pr2_robot/scripts/output_1.yaml).

### Test 2

![Test 2 object recognition][world_2_object_recognition]

View the goal [pick list](PR2-Project/RoboND-Perception-Project/pr2_robot/config/pick_list_2.yaml), and the calculated [output values](PR2-Project/RoboND-Perception-Project/pr2_robot/scripts/output_2.yaml).

### Test 3

![Test 3 object recognition][world_3_object_recognition]

View the goal [pick list](PR2-Project/RoboND-Perception-Project/pr2_robot/config/pick_list_3.yaml), and the calculated [output values](PR2-Project/RoboND-Perception-Project/pr2_robot/scripts/output_3.yaml).

## Code Sources

The original Udacity perception exercise code can be found [here](https://github.com/udacity/RoboND-Perception-Exercises). To install the exercise data on your own computer, view the installation guide [here](Install_Guide.md).

The PR2 3D perception project code source can be found [here](https://github.com/udacity/RoboND-Perception-Project). Follow the instructions in the README.md to get the demo code working.

## Run the Simulation

View instructions for running the code mentioned above [here](PR2-Project/RoboND-Perception-Project/pr2_robot/scripts/README.md).