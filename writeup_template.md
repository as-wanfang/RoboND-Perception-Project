## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify).
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

[//]: # (Image References)

[image1]: ./Figure_1.png
[image2]: ./Figure_2.png
[image3]: ./FIgure_3.png
[image4]: ./FIgure_4.png
[image5]: ./FIgure_5.png

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
This point is addressed in the pcl_callback function in project_template.py. I used voxel grid downnsampling first. The parameter LEAF_SIZE is set to be 0.003 at a high resolution, which helps to increase the precision of the object recognition step. Then I used pass through filter in both z and y axis. Y-axis is chosen to be within -0.5 and 0.5 so that the pipeline doesn't detect the two side tables. After this, a statistical outlier filter is used followed by the ransac plane segmentation.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
This point is addressed in the pcl_callback function in project_template.py. The Euclidean clustering method is used to detect and extract point cloud for each object.


![Cloud points before filtering][image1]
![Cloud points after filtering and clustering][image2]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Feature extraction for training is addressed by features.py and capture_features.py. The histogram feature is defined in features.py where 64 bins and 32 bins are used for color and normal respectively. The features of the eight objects to be detedcted in the project are collected. Each object is presented to the camera 500 times with random pose. More random samples help to decrease the overfitting of the training process.

SVM model training is addressed by train_svm.py. Linear kernal is used for the training and the penalty parameter is set to the default value of 1, which can be further tuned to decrease overfitting. The training result is shown in the figure below.

![Normalized confusion matrix][image3]


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.
The parameters for PickPlace request are save to output_1.yaml, output_2.yaml and output_3.yaml for pick list 1, 2 and 3 respectively. The position of pick pose is specified by the centroid of the object cloud point. The position of place pose is specified by the position of the drop box with modified x-axis value so that the object will not stack in the drop box. The z-axis value is also set to 0.9 which is above the table.

![Pict list 1][image2]

![Pict list 2][image4]

![Pict list 3][image5]
