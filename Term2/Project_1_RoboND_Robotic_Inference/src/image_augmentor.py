import sys
import json
import Augmentor

AUGMENT_CONFIG = {
     "Image_path" : "/home/sagarnil/Robotics_nanodegree/Term_2/project_1_robotic_inference/Data/train/200_rupee",
     "rotate90_probability" : "0.4",
     "rotate270_probability" : "0.4", 
     "flip_left_right_probability" : "0.6",
     "flip_top_bottom_probability" : "0.1",	 
     "crop_random_probability" : "0.8",	 
     "crop_random_percentage_area" : "0.6",	 
     "resize_probability" : "1.0",	 
     "resize_width" : "256",	 
     "resize_height" : "256",		
     "sample_number" : "20"	 	 
}

def displayconfig():
    print("Image_path = ", AUGMENT_CONFIG["Image_path"])
    print("rotate90_probability = ", AUGMENT_CONFIG["rotate90_probability"])
    print("rotate270_probability = ", AUGMENT_CONFIG["rotate270_probability"])
    print("flip_left_right_probability = ", AUGMENT_CONFIG["flip_left_right_probability"])	
    print("flip_top_bottom_probability = ", AUGMENT_CONFIG["flip_top_bottom_probability"])
    print("crop_random_probability = ", AUGMENT_CONFIG["crop_random_probability"])
    print("crop_random_percentage_area = ", AUGMENT_CONFIG["crop_random_percentage_area"])
    print("resize_probability = ", AUGMENT_CONFIG["resize_probability"])	
    print("resize_width = ", AUGMENT_CONFIG["resize_width"])
    print("resize_height = ", AUGMENT_CONFIG["resize_height"])
    print("sample_number = ", AUGMENT_CONFIG["sample_number"])	

def loadConfig(config_file_name):
    with open(config_file_name, 'r') as f:
        config = json.load(f)
        AUGMENT_CONFIG["Image_path"] = config['Path']['Image_path']
        AUGMENT_CONFIG["rotate90_probability"] = config["Parameters"]["rotate90_probability"]
        AUGMENT_CONFIG["rotate270_probability"] = config["Parameters"]["rotate270_probability"]	
        AUGMENT_CONFIG["flip_left_right_probability"] = config["Parameters"]["flip_left_right_probability"]
        AUGMENT_CONFIG["flip_top_bottom_probability"] = config["Parameters"]["flip_top_bottom_probability"]
        AUGMENT_CONFIG["rotate270_probability"] = config["Parameters"]["rotate270_probability"]	
        AUGMENT_CONFIG["crop_random_probability"] = config["Parameters"]["crop_random_probability"]
        AUGMENT_CONFIG["crop_random_percentage_area"] = config["Parameters"]["crop_random_percentage_area"]
        AUGMENT_CONFIG["resize_probability"] = config["Parameters"]["resize_probability"]	
        AUGMENT_CONFIG["resize_width"] = config["Parameters"]["resize_width"]
        AUGMENT_CONFIG["resize_height"] = config["Parameters"]["resize_height"]
        AUGMENT_CONFIG["sample_number"] = config["Parameters"]["sample_number"]	
 
		
	
def augment():	
    p = Augmentor.Pipeline(AUGMENT_CONFIG["Image_path"])
    p.rotate90(float(AUGMENT_CONFIG["rotate90_probability"]))
    p.rotate270(float(AUGMENT_CONFIG["rotate270_probability"]))
    p.flip_left_right(float(AUGMENT_CONFIG["flip_left_right_probability"]))
    p.flip_top_bottom(float(AUGMENT_CONFIG["flip_top_bottom_probability"]))
    p.crop_random(float(AUGMENT_CONFIG["crop_random_probability"]), float(AUGMENT_CONFIG["crop_random_percentage_area"]))
    p.resize(float(AUGMENT_CONFIG["resize_probability"]), int(AUGMENT_CONFIG["resize_width"]), int(AUGMENT_CONFIG["resize_height"]))
    p.sample(int(AUGMENT_CONFIG["sample_number"]))

if __name__ == "__main__":
    config_file_name = sys.argv[1]
    print("config file name = ", config_file_name)
    try:
        loadConfig(config_file_name)
        displayconfig()
        augment()
    except:
        print('message: ', sys.exc_info())   	

