import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized

def create_histogram(ch1, ch2, ch3, nbins=32, hist_range=(0, 256)):

    ch1_hist = np.histogram(ch1, bins=nbins, range=hist_range)
    ch2_hist = np.histogram(ch2, bins=nbins, range=hist_range)
    ch3_hist = np.histogram(ch3, bins=nbins, range=hist_range)
 
    #concatenate the histograms into a single feature vector
    hist_features = np.concatenate((ch1_hist[0], ch2_hist[0], ch3_hist[0])).astype(np.float64)

    #normalize
    norm_features = hist_features/np.sum(hist_features)

    return norm_features


def compute_color_histograms(cloud, nbins=32, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    normed_features = create_histogram(ch1=channel_1_vals, ch2=channel_2_vals, ch3=channel_3_vals, nbins=nbins)
    
    return normed_features 


def compute_normal_histograms(normal_cloud, nbins=32):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    normed_features = create_histogram(ch1=norm_x_vals, ch2=norm_y_vals, ch3=norm_z_vals, nbins=nbins)


    return normed_features
