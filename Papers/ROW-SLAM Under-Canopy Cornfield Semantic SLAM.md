ROW-SLAM: Under-Canopy Cornﬁeld Semantic SLAM

Jiacheng Yuan1, Jungseok Hong2, Junaed Sattar2, and Volkan Isler2

1
2
0
2

p
e
S
5
1

]

O
R
.
s
c
[

1
v
4
3
1
7
0
.
9
0
1
2
:
v
i
X
r
a

Abstract— We study a semantic SLAM problem faced by a
robot tasked with autonomous weeding under the corn canopy.
The goal is to detect corn stalks and localize them in a global
coordinate frame. This is a challenging setup for existing
algorithms because there is very little space between the camera
and the plants, and the camera motion is primarily restricted
to be along the row. To overcome these challenges, we present
a multi-camera system where a side camera (facing the plants)
is used for detection whereas front and back cameras are used
for motion estimation. Next, we show how semantic features in
the environment (corn stalks, ground, and crop planes) can be
used to develop a robust semantic SLAM solution and present
results from ﬁeld trials performed throughout the growing
season across various cornﬁelds.

I. INTRODUCTION

Cornﬁeld weed control conventionally has relied heavily
on herbicides which are undesirable due to environmental
and health-related concerns and can not be used in organic
ﬁelds. The alternative, manual weeding, is labor-intensive
and costly. Therefore, there has been signiﬁcant interest in
robotic weeding [1]–[4]. However, most existing techniques
focus on early season weeding. During this time, since
the canopy is not closed, usually a reliable GPS signal is
available. Further, the robots can obtain top-down views,
which are convenient for detection and localization. In this
work, we focus on mid-season weeding where the robot must
operate under the canopy.

This setup introduces unique challenges for autonomous
weeding: (1) top-down views are no longer available, and
therefore a “planar world” assumption does not hold (2) mid-
season corn and weed canopies are usually in close distance
and overlap with each other (3) there is frequent occlusion
from corn leaves and weed even if we lower the height of the
cameras closer to the ground in between two rows. (Fig. 1)
We observe that established SLAM algorithms [5], [6]
struggle in the setup because: (1) dynamic features such as
shaking leaves and weeds due to wind or robot motion cause
the SLAM module to fail (2) the corn row lacks regular
structures such as cylinders or walls, making it challenging
to observe the corn plane from the map directly.

In Fig. 2, we present image samples from conventional and
organic ﬁelds with heavy weed infestation as a comparison.
We highlight the stronger interference from the weed stems
and lower corn stalk visibility due to the existence of the
weeds.

Fig. 1: Our robot in a mid-season corn ﬁeld. Dense canopy and
narrow row spacing makes it challenging for semantic SLAM.

In this paper, we introduce a multi-view vision system
for the detection and 3D localization of corn stalks in mid-
season corn rows, called ROW SLAM. We present
this
system (Fig. 1, 3) as a prototype of the vision module for an
autonomous weeding robot currently under development. In
our approach, we model the corn stalks in the same row as
a plane that is perpendicular to the ground. A ground-view
camera (back camera) performs SLAM using ground plane
features to obtain 3D odometry. We implement a Structure-
from-Motion (SfM) strategy that accommodates the multi-
view inputs to estimate the 3D pose of the corn stalk plane.
Combining this motion estimation module with the object
detection and tracking modules, we build a map that has both
metric (location and orientation) and semantic information
of the corn stalks in it. We present ﬁeld results which
demonstrate the accuracy and robustness of our approach
toward weeding mid-season cornﬁelds.

Our contributions can be summarized as follows:
• We present ROW SLAM: a multi-camera vision system
with non-overlapping views for semantic SLAM in
mid-season corn rows. Speciﬁcally, we use the system
to build maps with both 3D location and semantic
information of corn stalks.

• We present a multi-view Structure-from-Motion (SfM)

strategy for robust corn stalk plane estimation.

• We test our system in 8 different corn rows with heavy
weed infestation across the growing season. Our method
outperforms all baseline approaches.

II. RELATED WORK

1 is with Department of Electrical and Computer Engineering, University

of Minnesota, Minneapolis, MN, 55455, USA yuanx320@umn.edu

2 are with the Department of Computer Science and Engineering,
University of Minnesota, Minneapolis, MN, 55455, USA {jungseok,
junaed, isler}@umn.edu

The topic of robotic weed control [7] has been extensively
studied over the last decade from various viewpoints: weed
detection, ﬁeld coverage, and vehicle development. Most
recent weed control robots have been developed in the

(a) Front view (conventional)

(b) Side view (conventional)

(c) Front view (organic)

(d) Side view (organic)

Fig. 2: Sampled images from both the conventional ﬁeld and the
organic ﬁeld. The conventional ﬁeld has almost no weeds due to
herbicides, while the organic ﬁeld shows heavy weed infestation.

form of drones and ground vehicles. While drone-based
they only provide
approaches [8] can cover large areas,
weed information to manage weeds via ground vehicles
(e.g., applying herbicide) [9]. Recently, Wu et al.
[10]
proposed a method using ground vehicles with downward-
looking cameras to recognize weeds and remove them by
end effectors. However, such an approach could fail as the
crop canopies grow and block the top view.

Sivakumar et al.

[11] use a front view camera to predict
a vanishing point and corn lines for navigation under-canopy
tasks. However, using the front view loses a lot of the details
of the objects in the narrow corn row due to the viewing
angle. In Sec. V-C, we compare against their method for corn
plane estimation. Zhang et al.
[12] use side view for corn
stalk counting, but we show that side view also experiences
frequent occlusion. To get accurate semantic features (i.e.,
corn stalks, ground plane, corn plane) from the map, we
propose to combine the input from multiple views.

SfM [13] is one of the classical methods available to obtain
the 3D location of the features. In our application, since
there is very little space and frequent occlusion between the
camera and the plants, the camera motion is also restricted.
Classical SfM strategies perform poorly for estimating the
motion when observing only the plants. Hasheminasab et al.
[14] proposes to assist the SfM pipeline with GPS signals.
Since GPS is unreliable under the canopy, we use SLAM
odometry from the back camera and the front view corridor
estimation to assist the SfM process using the side view.

For localization in outdoor environments, SLAM with
RGB input [15]–[17] has been widely used. However, the
localization accuracy of SLAM is sensitive to dynamic
features. Recent works [5], [18] adopt neural networks to
detect moving objects and remove their features. The re-
maining static features are then used to build the map. In our
application scenario, most of the features above ground are
susceptible to unconstrained motion due to the interference
from wind or robot motion, making it challenging to detect
dynamic features. So we select the ground view as input for
the SLAM module to ensure the static map assumption. In

Sec. V-D we compare the performance using different views
to support this choice.

Another aspect of obtaining semantic features is object
detection. Some recent works [6], [19], [20] build ofﬂine
approximation models for the objects of interest and detect
them by model matching while building the map. However,
such a method is unsuitable for corn stalk detection due to
the lack of structured shapes in the non-uniform canopy. The
dense occlusion from the leaves and the weeds makes it even
more challenging to detect corn stalks by matching ofﬂine
models.

Recent advances in deep learning allow accurate real-time
object detection [21] and detect a wide range of objects from
a single model with a large amount of data. Such advances
have enabled adopting deep learning-based object detection
in agricultural applications [22], and many studies [23]
apply the object detection to detect weed. However, these
approaches are limited because they (1) require a large
amount of weed data to train the models which is challenging
to obtain, (2) can only be used for the ﬁeld that existing weed
information is previously known, and (3) may need a survey
of a target ﬁeld to collect ﬁeld-speciﬁc weed information
and its visual data before the deployment in a new ﬁeld. In
contrast, we propose to focus on detecting corns and consider
the remaining as weeds. Our approach can be applied to a
broader range of ﬁelds and growing stages since corns have
fewer variants compared to weeds.

To associate object detection across frames, object tracking
algorithms [24] with visual sensors have been proposed
to track a wide range of objects (e.g., pedestrians [25],
vehicles [26], sports players [27], crops [28], [29]). The
tracking algorithms generally use an estimation model and
data association method such as optical ﬂow [30], Kalman
ﬁlter, and Hungarian algorithm to associate detection results
from the previous frame to the next frame. In our pipeline, we
use Simple online and realtime tracking (SORT) [31] due to
its speed and accuracy, as demonstrated on the MOT15 [32]
dataset.

In this work, we show that robotic weed control in mid-
season corn row poses unique challenges to existing SLAM
algorithms. With our system design and combining multiple
existing algorithms,
the results demonstrate that we can
overcome the challenges.

III. PROBLEM FORMULATION

We are given a cornﬁeld planted in a row, and have a
vehicle that can traverse the ﬁeld. A camera frame is rigidly
attached to the vehicle and moves under the corn canopy
with three cameras (one in the front-facing along the row,
one on the side facing the corn plane, and one on the back
facing the ground as shown in Fig. 3).

Our goal is to detect the corn stalks and localize them
in the global coordinate frame. Therefore, we formulate this
problem as a semantic SLAM problem where we aim to build
a map with semantic features (corn stalks, corn plane, and the
ground plane). We choose corn stalks as the target semantic
feature instead of weeds because, unlike the corn stalks,

Side Camera

ROW SLAM

Back Camera

Front Camera

Corn Detection

Tracking Module

Bbox Feature Extraction
& Matching

Corn IDs
2D Pos. of Corns

Corn IDs
3D Pos. of Corn stalks

ca

cb

cc

cd

ce

SLAM

T

Multiview
SfM

dp

Downsampled
PCD

Plane ﬁtting

PCA

ng

vl

np

Corn
plane

Back Camera

Front Camera

Corn Plane

Side Camera

Ground Plane

Fig. 3: (Top) Proposed pipeline of ROW SLAM: Our pipeline takes images from {front, back, side} cameras and yields 1)
IDs for each corn (denoted with C{a,..,e} in the scene, and 2) 3D positions of the corn stalks (denoted by yellow cylinders).
(Bottom-Left) Proposed robot design with multi-view cameras. (Bottom-Right) A 3D reconstruction sample of the corn
ﬁeld. Beige and green planes represent ground and corn planes, respectively. Each camera pose is displayed separately.

weeds in the ﬁeld have various species and appearances.
They also tend to vary across ﬁelds and regions. As a result,
corn detection can be more consistent and robust than weed
detection. Once corn stalks are identiﬁed, we can treat the
remaining plants as weeds.

IV. SYSTEM OVERVIEW

In this section, we introduce the necessary mathematical
notations in Table I and describe our system’s hardware.
We discuss the details of our approach for 3D corn stalk
detection and localization next.

A. System Description

Our hardware system consists of three cameras (Fig. 3
Bottom-Left). The front and side cameras (Intel Realsense
D435) publish RGB images and depth images at 30 Hz. The
back camera (Intel Realsense D435i) has a built-in IMU and
is mounted at an angle facing the ground. The back camera
publishes RGB and depth images at 30 Hz, gyro at 400 Hz,
and accelerometer at 250 Hz. We use ROS for robot control
and routing sensor data. For our experiments, we mount the
system on a small 4-wheel rover from Rover Robotics.

TABLE I: Summary of Notations

Notation

Description

np, dp

Normal vector and distance to the origin for the corn
plane (shown in Fig. 3)

ng

vl

T

Normal vector for the ground plane (shown in Fig. 3)
Intersection of the ground plane and the corn plane, also
the orientation of the corn row

Relative transformation between sequential side camera
poses

Notes: bold uppercase letters for matrix, bold lowercase for column
vector, normal ones are scalar

B. Method Overview

As shown in Fig. 3, we use RGB images from the side
view camera for corn stalk detection. The detections only
localize the stalks in 2D. Thus, to ﬁnd the 3D position of
the detected corn stalks, we also need to ﬁnd the 3D pose of
the corn plane. We address the corn plane estimation problem
in two parts: (1) ﬁnding the plane normal direction, np, and
(2) ﬁnding the distance from the camera center to the plane,
dp, shown in Fig. 4.

We use the front and back camera to assist with the
estimation of the corn plane np and dp. With the RGB-D

input from the front view camera, we estimate the ground
plane normal direction ng and the direction of the corn line
vl. The corn plane orientation is found by the cross product
of these two vectors.

np = ng × vl
(1)
Then, we use the corn plane normal np, combined with
side view object detection results and SLAM odometry T
as the input for the Multiview SfM module to estimate dp.
Before going through the details of this module, we ﬁrst
introduce the detection and tracking.

C. Corn Stalk Detection

We implement our corn stalk detection model using Faster
R-CNN [33] with MobileNet V3 [34] as the backbone
and Fully Connected Network (FCN) as a head. We select
MobileNet to provide fast inference, thus preventing the deep
detection model from being a bottleneck in our pipeline.
The backbone can be replaced with a larger network such
as Resnet-18, -34, -50 [35] to improve the accuracy of the
model. We train our model with pre-trained weights using
the COCO dataset [36], and reﬁne further using our corn
stalk dataset.

D. Corn Stalk Tracking

With the outputs from the detection model, we implement
the corn stalk tracking pipeline using (1) optical ﬂow with
centroids [37], and (2) simple online and realtime tracking
(SORT) [31].

1) Optical Flow: We build the corn stalk tracking al-
gorithm by applying optical ﬂow algorithm and the corn
detection results. The model detects bounding boxes for
each corn stalk every 200 frames, and the centroids of the
bounding boxes are calculated. After we obtain the centroids
from the output of the detection model at frame X, the
iterative Lucas-Kanade method with pyramids [38] is used
to track each centroid using optical ﬂow from frame {X +1}
to {X + 199}.

2) SORT: SORT performs four actions for each input
frame: detection, estimation, data association, and update
tracking identities. The detections from Faster R-CNN are
propagated to the next frame using a linear constant velocity
model. The results are also utilized to update the target state
with the Kalman ﬁlter [39]. When a new detection is obtained
from the next frame, the Hungarian algorithm [40] is used to
associate detections from the previous frame to the current
frame using the intersection-over-union (IoU) metric. When
the IoU metric is below a predeﬁned threshold, then new
identiﬁcation of the detection is created.

E. Corn Plane Estimation

Plane Normal To get the ground plane pose, we ﬁrst use
the front view RGB-D input to compute the point cloud of
the scene, followed by color thresholding and RANSAC [41]
plane ﬁtting. The points are usually denser when closer to
the camera center, so we downsample the ground plane inlier
points before applying principal component analysis (PCA).
Since the corn row is narrow (≈ 70cm), the two eigenvectors

Fig. 4: Illustration for the multi-view SfM strategy. The relative
camera pose (mint) is given by the SLAM module, and feature
matching is masked by detected bounding boxes. np is the plane
normal of the corn plane, and dp is the distance between the corn
plane and the left camera center.

with the largest and smallest eigenvalues from PCA indicate
the directions of the corn line vl and the ground plane normal
ng, respectively. The corn plane normal is then computed by
the cross product of these two vectors.

Plane Distance Across different frames, the motion of
the features on the corn stalks can be modeled by planar
homography. However, instead of doing the full SfM purely
on the side view input, we ﬁnd it much more stable if we
use the ground features to estimate the camera trajectory.
We thus use the images from the back camera, providing
a predominantly ground-plane view to avoid the unstable
features from the corn leaves and weeds. With these images,
we use the off-the-shelf RTAB-Map SLAM [17] package
to perform visual SLAM and use the Sigma Point Kalman
Filter [42] to fuse the SLAM odometry with IMU input. The
resulting odometry runs at 200 Hz. After calibrating the back
view and side view cameras, we can use the camera trajectory
as is. Combining this with the estimated corn plane normal,
we formulate plane distance estimation as the following least
square problem over re-projection error (shown in Fig. 3 as
the Multiview SfM block):

Given two side view RGB frames I1, I2,

the relative
transformation T2
1 between their camera poses (where R
and c is the rotation and translation component) and the corn
plane normal np, we want to estimate the plane distance dp,
as shown in Fig. 4.

We ﬁrst run Faster R-CNN on I1, I2 to get bounding boxes
for detected corn stalks. Next, we compute SIFT features
f1, f2 for I1, I2 only within the regions deﬁned by the
bounding boxes, and then use cross-matching and ratio test to
ﬁnd good matches between f1, f2. For each matched pair, let
(u1, v1) and (u2, v2) be the corresponding pixel coordinates,
K and λ1 respectively be the camera intrinsic matrix and the
depth for the ﬁrst feature.

Letting x1 = (cid:2)u1
v1
expressed as: λ1K−1x1.

1(cid:3)T

, the 3D position of f1 can be

Since we assume the feature lies in the corn plane,

according to the plane equation

λ1nT

p K−1x1 + dp = 0

We can rewrite λ1 in terms of dp:

λ1 = −dp/nT

p K−1x1

(2)

(3)

Fig. 5: Corridor projection of the corn row. Two corn lines (blue)
are parallel and intersects at the vanishing point (red). Due to
occlusion, the corn lines are usually not clearly visible.

After applying the rotation R and translation c,
the
projection of the 3D position of f1 in the second camera
frame can be expressed as:

λ1KRK−1x1 + Kc
If we substitute Eq. 3 in Eq. 4 we can rewrite Eq. 4 as

(4)

follows:

where,

− dp

(cid:2)l0

l1

(cid:3) + (cid:2)s0

l2

s1

(cid:3)

s2

(cid:2)l0

l1

(cid:3)T

l2

=

−1
p K−1x1

nT

KRK−1x1

(5)

(6)

(cid:3)T

s2

s1

(7)
The projected pixel position of f1 in the second camera
2, v(cid:48)

2) is:

= Kc

(u(cid:48)

(cid:2)s0

(cid:40)

u(cid:48)
2 = (−dpl0 + s0)/(−dpl2 + s2)
v(cid:48)
2 = (−dpl1 + s1)/(−dpl2 + s2)

(8)

We can linearize the re-projection error (u2 − u(cid:48)

2, v2 − v(cid:48)
2)
with respect to dp so it can be estimated by least squares and
RANSAC. However, notice that the least square robustness is
sensitive to the second term Kc in Eq. 4. So when applying
this algorithm, we need to make sure the translation scale is
above a threshold to ensure robust triangulation.

F. 3D Localization

The 3D position of a corn stalk is obtained by the
projection of the bounding box centroid onto the corn stalk
plane. To show that it is more robust than directly using
RGB-D input, we compare against RANSAC Plane Fitting
in Sec. V-D. We also use the tracking module to link the
3D corn stalk positions with corn IDs. Such association
across multiple frames allows us to reject outliers and false
positives, making the localization more robust.

V. EXPERIMENTS AND RESULTS

We present our data collection procedures, followed by
the evaluation metrics for our method. Then we introduce
the baseline methods we compare against.

A. Data Collection

The rover is controlled by a human operator to drive at
around 0.3m/s. To provide additional control of imaging
conditions, we used a wheeled arch platform to go over the
rows and provide cover for the rover. In the future, we are
planning to combine the rover and the cover into a single
robotic platform. Our collected dataset includes 8 different
corn rows where 4 of them are from conventional ﬁelds,

and the other 4 are from organic ﬁelds. We collected data
across different growing stages throughout the mid-season
(between V5 stage and V12 stage, late June to early August
in Minnesota, USA).

B. Evaluation Metric

We perform ofﬂine evaluations using the dataset collected
in two different aspects, detection accuracy and localization.
For evaluating the corn detection module, we label 763
images and separate them into 563 training images (235
images from conventional ﬁeld and 328 images from organic
ﬁeld) and 200 test images (100 images from each ﬁeld).
Training and test dataset images for each ﬁeld are picked
from different rows to reduce the correlation between the
datasets and provide an accurate measure of the general-
ization capability of our network. For localization under
the canopy, it is difﬁcult to get high-accuracy GPS signals
reliably. So we manually measure the distance between
neighboring corn stalks and use the corn stalks as landmarks
along a straight line to evaluate the localization accuracy. We
ﬁrst line ﬁt on the camera trajectory and project predicted
target positions to the ﬁtted line to reduce their dimension
to one. To remove the duplicated predictions for a single
target, we compute its nearest neighbor from the world
measurements for each predicted corn stalk position. The
distance error (cid:15)1 is computed as the mean absolute error of
the neighboring corn stalks distance between prediction and
real-world measurements. Finally, we measure the tracking
variance through re-projection error (cid:15)2. After accumulating
the 3D position measurements for each corn stalk, we com-
pute the mean position of the centroids and re-project them
back to each frame. The re-projection error is computed by
the mean pixel error between the centroids and the centroids
of the bounding boxes proposed by the tracking module
across all the frames.

C. Baselines

SLAM Input As the SLAM pipeline is sensitive to
moving objects in the scene, we compare the robustness
and accuracy of SLAM while using different input views.
Speciﬁcally, we replace the SLAM input in Fig. 3 with
the front camera view or side camera view and re-run the
Semantic SLAM pipeline.

Corridor Prediction The corridor prediction method re-
places the corn plane prediction module in Fig. 4. It is also
based on the planar approximation of the corn row. Since the
corn stalk planes on both sides are parallel, their intersection
line with the ground plane must also be parallel. Therefore
by projective geometry, they intersect in the image plane at
the vanishing point (Fig. 5). After obtaining the 3D pose
of the ground plane, the vanishing point can be used to get
vl. The corn stalk plane normal np can be computed by the
cross product of vl and ng. Then, dp can be obtained by the
3D position of any point along the corn line since it is also
on the ground plane.

It is known that color thresholding yields unreliable re-
sults [11] for vanishing line detection. Thus, we combine

Fig. 6: The reconstructed point cloud of a corn row and the estimated corn stalk position (red cylinders) from ROW SLAM.

TABLE II: Faster RCNN Corn Detection Accuracy (%)

Conventional ﬁeld Organic ﬁeld

AP
AP50
AP75

26.2
83.1
9.3

47.8
89.1
46.6

TABLE III: Performance of Corn Stalk Localization
((cid:15)1: Metric Error, (cid:15)2: Centroid Re-Projection Error)

Conventional ﬁeld
(cid:15)1 (cm)

(cid:15)2

Organic ﬁeld
(cid:15)2
(cid:15)1 (cm)

our approach
corridor prediction
front-view SLAM
side-view SLAM
RANSAC plane ﬁtting
optical ﬂow tracking

1.8
8.5
2.9
4.5
3.7
3.3

5.6
19.3
8.2
16.8
9.5
11.7

3.5
9.7
6.2
7.1
9.3
4.6

10.4
27.8
13.3
15.5
24.2
17.9

the ResNet34 head with a 3-layer multi-layer perceptron to
predict the vanishing point pixel location and two slopes for
the corn line. We do not predict the full parameters for the
two lines so that we can enforce the projective geometry.

RANSAC Plane Fitting The RANSAC plane ﬁtting
method replaces the multi-view SfM module in Fig. 4 for
the estimation of dp. After obtaining the side plane normal
np, one way to estimate dp is by direct observation of the 3D
scene in the side view. Therefore to support our claim on the
low observability of the corn plane, we present plane ﬁtting
in the side view as another baseline method. The detected
object bounding boxes are used to ﬁnd candidate points in
the corn plane. We only need the 3D position of one selected
point with the plane normal to compute dp. Then RANSAC
is applied to make the estimation robust.

Optical Flow Tracking To ensure high localization ac-
curacy, we use SORT as our tracking module which runs
detection at every frame and uses the Kalman Filter to make
the tracking more robust. In this baseline, we replace the
tracking module with optical ﬂow tracking. Unlike SORT,
the optical ﬂow tracking method will re-initialize target id
every k frame. In our case, we choose k = 200.

D. Results

Faster R-CNN is trained for 100 epochs, and Table II
shows evaluation results for each ﬁeld. It tracks objects at
≈ 11 FPS on a laptop (Intel i7-8850H, Quadro P3200, and
32GB RAM). The organic ﬁeld case has higher Average
Precision (AP) values since about 60% of the training dataset
is from organic ﬁelds. AP50 (AP at IoU=.50) has the highest

accuracy for both cases, and it is because irregular shapes of
the corns tend to have relatively low IoU values for many
detections while they are still detected as corns.

We compute the metric localization accuracy by compar-
ing predicted target positions against the manually measured
distance between corn stalks. We also evaluate the tracking
variance through the re-projection error of the centroid. In
Table III we summarize the metric error (cid:15)1 and centroid re-
projection error (cid:15)2 using the dataset from both conventional
ﬁeld and the more challenging organic ﬁeld.

Our method outperforms all the baseline methods, which
demonstrates its accuracy and robustness. As a comparison,
the corridor prediction method shows fragile estimations of
#»
dp. Due to the challenge of accurate estimation for
v l and
projective geometry, the error of dp is signiﬁcantly impacted
by the angular error of the vanishing line in the front view.
As for the view selection in SLAM, we notice that the
front-view SLAM achieves comparable accuracy with our
method during a windless day in the conventional ﬁeld.
the comparison between our ROW
However,
the static
SLAM and front/side view SLAM shows that
features are critical for accurate under-canopy localization
in the corn rows. For the RANSAC plane ﬁtting method,
the performance drop indicates the challenge in directly
observing the corn stalk plane from a single view. Lastly,
we observe greater centroid position drift in image plane
with optical ﬂow tracking than SORT, which affects the
localization accuracy and variance, proving the importance
of robust visual tracking.

in general,

VI. CONCLUSIONS

This paper presents our effort to address the under-canopy
corn stalk detection and localization problem within narrow
corn rows by proposing ROW SLAM and a multi-view
camera system mounted on a ground vehicle. ROW SLAM
combines existing algorithms (i.e., object tracking, SLAM,
SfM) and applies several geometric methods to accurately
estimate the three planes that bound the row as well as in-
dividual plants. Our results demonstrate ROW SLAM yields
an accurate map containing corn stalk positions and their
IDs while existing SLAM algorithms fail. Future work will
add an end-effector to our existing system to remove weed
physically. Furthermore, we are developing our algorithm to
minimize the drift from the SLAM module by using corn
stalks as landmarks together with an ofﬂine map.

REFERENCES

[1] A. Michaels, S. Haug, and A. Albert, “Vision-based High-speed Ma-
nipulation for Robotic Ultra-precise Weed Control,” in 2015 IEEE/RSJ
International Conference on Intelligent Robots and Systems (IROS).
IEEE, 2015, pp. 5498–5505.

[2] X. Wu, S. Aravecchia, P. Lottes, C. Stachniss, and C. Pradalier,
“Robotic Weed Control Using Automated Weed and Crop Classiﬁ-
cation,” Journal of Field Robotics, vol. 37, no. 2, pp. 322–340, 2020.
[3] R. Raja, T. T. Nguyen, D. C. Slaughter, and S. A. Fennimore, “Real-
time Weed-crop Classiﬁcation and Localisation Technique for Robotic
Weed Control in Lettuce,” Biosystems Engineering, vol. 192, pp. 257–
274, 2020.

[4] R. Bogue, “Robots Poised to Transform Agriculture,” Industrial
Robot: the international journal of robotics research and application,
2021.

[5] F. Zhong, S. Wang, Z. Zhang, and Y. Wang, “Detect-SLAM: Making
Object Detection and SLAM Mutually Beneﬁcial,” in 2018 IEEE
Winter Conference on Applications of Computer Vision (WACV).
IEEE, 2018, pp. 1001–1010.

[6] K. Ok, K. Liu, K. Frey, J. P. How, and N. Roy, “Robust Object-
based SLAM for High-speed Autonomous Navigation,” in 2019 In-
ternational Conference on Robotics and Automation (ICRA).
IEEE,
2019, pp. 669–675.

[7] P. Pandey, H. N. Dakshinamurthy, and S. Young, “A Literature Review

of Non-Herbicide, Robotic Weeding: A Decade of Progress,” 2020.

[8] P. Lottes, R. Khanna, J. Pfeifer, R. Siegwart, and C. Stachniss, “UAV-
based Crop and Weed Classiﬁcation for Smart Farming,” in 2017 IEEE
International Conference on Robotics and Automation (ICRA), 2017,
pp. 3024–3031.

[9] F. Castaldi, F. Pelosi, S. Pascucci, and R. Casa, “Assessing the
Potential of Images from Unmanned Aerial Vehicles (UAV) to Support
Herbicide Patch Spraying in Maize,” Precision Agriculture, vol. 18,
no. 1, pp. 76–94, 2017.

[10] X. Wu, S. Aravecchia, P. Lottes, C. Stachniss, and C. Pradalier,
“Robotic Weed Control Using Automated Weed and Crop Classiﬁ-
cation,” Journal of Field Robotics, vol. 37, no. 2, pp. 322–340, 2020.
[11] A. N. Sivakumar, S. Modi, M. V. Gasparino, C. Ellis, A. E. B.
Velasquez, G. Chowdhary, and S. Gupta, “Learned Visual Nav-
igation for Under-Canopy Agricultural Robots,” arXiv preprint
arXiv:2107.02792, 2021.

[12] Z. Zhang, E. Kayacan, B. Thompson, and G. Chowdhary, “High
Precision Control and Deep Learning-based Corn Stand Counting
Algorithms for Agricultural Robot,” Autonomous Robots, vol. 44,
no. 7, pp. 1289–1302, 2020.

[13] P. Moulon, P. Monasse, and R. Marlet, “Global Fusion of Relative
Motions for Robust, Accurate and Scalable Structure from Motion,”
in Proceedings of the IEEE International Conference on Computer
Vision, 2013, pp. 3248–3255.

[14] S. M. Hasheminasab, T. Zhou, and A. Habib, “GNSS/INS-Assisted
Structure from Motion Strategies for UAV-Based Imagery over Mech-
anized Agricultural Fields,” Remote Sensing, vol. 12, no. 3, p. 351,
2020.

[15] B. Czupry´nski and A. Strupczewski, “Real-time RGBD SLAM Sys-
tem,” in Photonics Applications in Astronomy, Communications, In-
dustry, and High-Energy Physics Experiments 2015, vol. 9662.
In-
ternational Society for Optics and Photonics, 2015, p. 96622B.
[16] R. Mur-Artal, J. M. M. Montiel, and J. D. Tardos, “ORB-SLAM: a
Versatile and Accurate Monocular SLAM System,” IEEE transactions
on robotics, vol. 31, no. 5, pp. 1147–1163, 2015.

[17] M. Labb´e and F. Michaud, “RTAB-Map as an Open-source Lidar and
Visual Simultaneous Localization and Mapping Library for Large-
scale and Long-term Online Operation,” Journal of Field Robotics,
vol. 36, no. 2, pp. 416–446, 2019.

[18] Z. Wang, Q. Zhang, J. Li, S. Zhang, and J. Liu, “A Computationally
Efﬁcient Semantic SLAM Solution for Dynamic Scenes,” Remote
Sensing, vol. 11, no. 11, p. 1363, 2019.

[19] C. Rubino, M. Crocco, and A. Del Bue, “3d Object Localisation from
Multi-view Image Detections,” IEEE transactions on pattern analysis
and machine intelligence, vol. 40, no. 6, pp. 1281–1294, 2017.
[20] L. Nicholson, M. Milford, and N. S¨underhauf, “QuadricSLAM: Dual
Quadrics from Object Detections as Landmarks in Object-oriented
SLAM,” IEEE Robotics and Automation Letters, vol. 4, no. 1, pp.
1–8, 2018.

[21] Z. Zou, Z. Shi, Y. Guo, and J. Ye, “Object Detection in 20 Years: A

Survey,” 2019.

[22] A. Kamilaris and F. X. Prenafeta-Bold´u, “Deep Learning in
Agriculture: A survey,” Computers and Electronics in Agriculture,
vol. 147, pp. 70–90, 2018.
[Online]. Available: https://www.
sciencedirect.com/science/article/pii/S0168169917308803

[23] A. M. Hasan, F. Sohel, D. Diepeveen, H. Laga, and M. G. Jones,
“A Survey of Deep Learning Techniques for Weed Detection from
Images,” Computers and Electronics in Agriculture, vol. 184, p.
106067, 2021.

[24] G. Ciaparrone, F. L. S´anchez, S. Tabik, L. Troiano, R. Tagliaferri,
and F. Herrera, “Deep Learning in Video Multi-object Tracking: A
Survey,” Neurocomputing, vol. 381, pp. 61–88, 2020.

[25] A. Brunetti, D. Buongiorno, G. F. Trotta, and V. Bevilacqua, “Com-
puter Vision and Deep Learning Techniques for Pedestrian Detection
and Tracking: A Survey,” Neurocomputing, vol. 300, pp. 17–33, 2018.
[26] A. Osep, W. Mehner, M. Mathias, and B. Leibe, “Combined Image and
World-space Tracking in Trafﬁc Scenes,” in 2017 IEEE International
Conference on Robotics and Automation (ICRA).
IEEE, 2017, pp.
1988–1995.

[27] L. Bridgeman, M. Volino, J.-Y. Guillemaut, and A. Hilton, “Multi-
person 3d Pose Estimation and Tracking in Sports,” in Proceedings of
the IEEE/CVF Conference on Computer Vision and Pattern Recogni-
tion Workshops, 2019, pp. 0–0.

[28] X. Liu, S. W. Chen, S. Aditya, N. Sivakumar, S. Dcunha, C. Qu,
C. J. Taylor, J. Das, and V. Kumar, “Robust Fruit Counting: Combin-
ing Deep Learning, Tracking, and Structure from Motion,” in 2018
IEEE/RSJ International Conference on Intelligent Robots and Systems
(IROS).

IEEE, 2018, pp. 1045–1052.

[29] P. Roy and V. Isler, “Surveying Apple Orchards with a Monocular Vi-
sion System,” in 2016 IEEE International Conference on Automation
Science and Engineering (CASE), 2016, pp. 916–921.

[30] B. K. Horn and B. G. Schunck, “Determining Optical Flow,” Artiﬁcial

intelligence, vol. 17, no. 1-3, pp. 185–203, 1981.

[31] A. Bewley, Z. Ge, L. Ott, F. Ramos, and B. Upcroft, “Simple Online
and Realtime Tracking,” in 2016 IEEE international conference on
image processing (ICIP).

IEEE, 2016, pp. 3464–3468.

[32] L. Leal-Taix´e, A. Milan, I. Reid, S. Roth, and K. Schindler, “Motchal-
lenge 2015: Towards a Benchmark for Multi-target Tracking,” arXiv
preprint arXiv:1504.01942, 2015.

[33] S. Ren, K. He, R. Girshick, and J. Sun, “Faster R-CNN: Towards Real-
time Object Detection with Region Proposal Networks,” Advances in
neural information processing systems, vol. 28, pp. 91–99, 2015.
[34] A. Howard, M. Sandler, G. Chu, L.-C. Chen, B. Chen, M. Tan,
W. Wang, Y. Zhu, R. Pang, V. Vasudevan, et al., “Searching for Mo-
bilenetv3,” in Proceedings of the IEEE/CVF International Conference
on Computer Vision, 2019, pp. 1314–1324.

[35] K. He, X. Zhang, S. Ren, and J. Sun, “Deep Residual Learning
for Image Recognition,” in Proceedings of the IEEE conference on
computer vision and pattern recognition, 2016, pp. 770–778.

[36] T.-Y. Lin, M. Maire, S. Belongie, J. Hays, P. Perona, D. Ramanan,
P. Doll´ar, and C. L. Zitnick, “Microsoft CoCo: Common objects in
Context,” in European conference on computer vision. Springer, 2014,
pp. 740–755.

[37] R. Revathi and M. Hemalatha, “Certain Approach of Object Tracking
using Optical Flow Techniques,” International Journal of Computer
Applications, vol. 53, no. 8, 2012.

[38] J.-Y. Bouguet et al., “Pyramidal Implementation of the Afﬁne Lucas
Kanade Feature Tracker Description of the Algorithm,” Intel corpora-
tion, vol. 5, no. 1-10, p. 4, 2001.

[39] G. Welch, G. Bishop, et al., “An Introduction to the Kalman Filter,”

1995.

[40] H. W. Kuhn and B. Yaw, “The Hungarian Method for the Assignment

Problem,” Naval Res. Logist. Quart, pp. 83–97, 1955.

[41] M. A. Fischler and R. C. Bolles, “Random Sample Consensus: a
Paradigm for Model Fitting with Applications to Image Analysis and
Automated Cartography,” Communications of the ACM, vol. 24, no. 6,
pp. 381–395, 1981.

[42] J. L. Crassidis, “Sigma-point Kalman Filtering for Integrated GPS and
Inertial Navigation,” IEEE Transactions on Aerospace and Electronic
Systems, vol. 42, no. 2, pp. 750–756, 2006.

