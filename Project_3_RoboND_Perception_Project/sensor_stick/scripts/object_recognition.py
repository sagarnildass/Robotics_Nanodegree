#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    message = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = message.make_voxel_grid_filter()
    
    #choose leaf size
    LEAF_SIZE = 0.01

    #set the voxel size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    #call the filter function to obtain the resulting downsampled point cloud
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    #create a passthrough filter object
    passthrough = cloud_filtered.make_passthrough_filter()
    
    #assign axis and range to the passthrough filter object
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.75
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    
    #finally use the filter function to obtain resultant point cloud
    cloud_filtered = passthrough.filter()


    # TODO: RANSAC Plane Segmentation
    #create segmentation object
    seg = cloud_filtered.make_segmenter()
    
    #set the model we wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    
    #call the segment function to obtain a set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    #extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    
    #extract outliers(objects)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    
    # Assign a color corresponding to each segmented object in the scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    

    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table) 
    

    # TODO: Publish ROS messages
    pcl_objects_cloud_pub.publish(ros_cluster_cloud)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)


# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
   
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cloud = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        histogram_bins = 64
        chists = compute_color_histograms(ros_cloud,
                                          nbins=histogram_bins,
                                          using_hsv=True)
        normals = get_normals(ros_cloud)
        nhists = compute_normal_histograms(normals,
                                           nbins=histogram_bins)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # Create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_object_cluster = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages of the point clouds and detected objects
    pcl_objects_cloud_pub.publish(ros_cloud_object_cluster) # solid color objects
    pcl_objects_pub.publish(ros_cloud_objects)      # original color objects
    pcl_table_pub.publish(ros_cloud_table)          # table cloud
    detected_objects_pub.publish(detected_objects)  # detected object labels
        
if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscriber to receive the published data coming from the
    #   pcl_callback() function that will be processing the point clouds
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray,
                                           queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_objects_cloud_pub = rospy.Publisher('/pcl_objects_cloud', PointCloud2,
                                            queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # Load model from disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()


