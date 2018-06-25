# RoboND Project 4 - Follow-Me
## Author - Sagarnil Das

[//]: # (Image References)
[title_image]: ./images/hero_far.png
[image0]: ./images/lr_0.01_30.png
[image1]: ./images/lr_0.01_60.png
[image2]: ./images/lr_0.01_90.png
[image3]: ./images/lr_0.01_115.png

[image5]: ./images/hero_sim.png
[image6]: ./images/hero_close.png
[image7]: ./images/hero_absent.png
[image8]: ./images/hero_far_2.png

![Model Performance][title_image]


The image shows the final segmentation result done by fully convolutional networks. The left one is raw feed, middle one is ground truth and the right one is the segmented image done by the model.

### Fully Convolutional Networks

The basic idea behind a fully convolutional network is that it doesn't have any dense fully connected layers i.e. all of its layers are convolutional layers. FCNs, instead of using fully connected layers at the end, typically used for classification, use convolutional layers to classify each pixel in an image.

So the final output layer will be the same height and width as the input image, but the number of channels will be equal to the number of classes. If we’re classifying each pixel as one of fifteen different classes, then the final output layer will be height x width x 15 classes. FCNs comprises of mainly 3 parts: an encoder, a 1x1 convolution and a decoder. The 1x1 convolution has small height and width but much bigger depth due to the use of filters in the preceeding encoder part. The 1x1 convolution's height and width will be the same as the layer preceeding it. The reason for using 1x1 convolution is that it acts as a fully connected layer, but the major advantage of it is that it retains spatial information of all the pixels.

The 2nd portion of the network is called the decoder.t is built up of layers of transposed convolutions whose outputs increase the height and width of the input while decreasing it's depth. Skip connections are also used at all points in the decoder which in this case concatenates the input to the encoder layer of the same size with the decoded layer.

For this project I've chosen a model with 3 encoder layers, the 1x1 convolution, and then 3 decoder layers. The output shape of each layer is as following:

	Features Layer (?, 160, 160, 3) - - |
	Encoder 1 (?, 80, 80, 32) - - - -|  |
	Encoder 2 (?, 40, 40, 64)  - -|  |  |
	Encoder 3 (?, 20, 20, 128)    |  |  |  Skip
	1x1 Conv (?, 20, 20, 128)     |  |  |  Connections
	Decoder 1 (?, 40, 40, 128) - -|  |  |
	Decoder 2 (?, 80, 80, 64) - - - -|  |
	Decoder 3 (?, 80, 80, 64) - - - - - |
	Output Layer (?, 160, 160, 3)


The skip connections are indicated by the connecting lines.

### Hyper parameters

The steps per epoch was always set to the total number of images in the training set divided by the batch size so that every epoch was approximately one run through all the training images. I have used a batch size of 32, which is quite large. The main motivation behind using a large batch size was achieving stability in the training process. Then I used two learning rates of 0.0001 and 0.01 respectively. I initially ran with 30 epochs with 0.0001 LR and achieved a total IOU score of 0.29. After the training was completed, unfortunately my internet crashed and I had to do it all over again. But this time, I didn't want to re-run the first training again due to high charge of AWS. But from the loss function plots, I could deduce that the loss has not settled or plateaued and it's still changing. So for my final model, I decided to ramp up my learning rate to 0.01 and see if it speeds up the convergence. If at some point, I would have seen that convergence has been deteriorating, my plan was to use a decaying learning rate.

Based on these observations, my final run was for 115 epochs with a batch size of 32 and a leanring rate of 0.01. I used Adam as the optimizer. The following curves show the different stages of the training process.

At epoch 30:

![LR 0.01][image0]

At epoch 60:

![LR 0.01][image1]

At epoch 90:

![LR 0.01][image2]

At epoch 115:

![LR 0.01][image3]

A considerable performance increase was gained by dropping the learning rate. This fine tuning with a small learning rate helped go through the plateau the 0.01 learning rate was stuck on.

I then trained for another 30 epochs at a further reduced learning rate of 0.0001, but no signifcant improvemennt in val loss or score emerged. However, slight drop in loss was noticed. At this point the model may have begun to overfit so I reverted back the the epoch 115 model for final selection. Even though epoch 114 has a higher score, it's val loss is also higher which indicates that this must just be noise because of the datasets and not indicating a better model. Thus I stuck with epoch 115.

### Results

The following model parameters were used and results were obtained after the final training:

* Epoch = 30

* Batch Size = 32

* Learning rate = 0.01 (first 100 epochs), 0.001 (last 15)

* Loss = 0.0097

* Validation Loss = 0.0280

#### IOU scores 

a) Scores for while the quad is following behind the target:

number of validation samples intersection over the union evaulated on 542
average intersection over union for background is 0.9965918930294348
average intersection over union for other people is 0.4156018979835764
average intersection over union for the hero is 0.918219058838719
number true positives: 539, number false positives: 0, number false negatives: 0

b) Scores for images while the quad is on patrol and the target is not visable:

number of validation samples intersection over the union evaulated on 270
average intersection over union for background is 0.9896461582931216
average intersection over union for other people is 0.7972887652579831
average intersection over union for the hero is 0.0
number true positives: 0, number false positives: 30, number false negatives: 0

c) Score for images when target from far away:

number of validation samples intersection over the union evaulated on 322
average intersection over union for background is 0.9970258887276762
average intersection over union for other people is 0.48810254263717745
average intersection over union for the hero is 0.21821227960944378
number true positives: 118, number false positives: 0, number false negatives: 183

d) Weight = 0.7551724137931034

e) Final IOU = 0.57

f) Final grade score = 0.43


### Performance on simulator 

While testing the model in the simulator, it performed remarkably well to detect the hero from a reasonably large distance and once it detects and zeros in on the hero, it never lost sight of it. It has a 100% success ratio when it is close to the hero. This is evident from the 539 true positives we obtained from the model evaluation.

The models performance could be improved by using more training data, particularly when the hero is far away. Other than that the model performs really well.

I believe this model could be used to also idenify and track multiple objects one at a time where each object zeros in to the other object and once it reaches the other object, the drone leaves the first object and follows the 2nd one. This can have a huge military application.

![Hero Tracking][image5]


![Hero Close][image6]

![Hero absent][image7]

![Hero far][image8]




















