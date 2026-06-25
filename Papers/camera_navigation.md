Learned Visual Navigation for Under-Canopy
Agricultural Robots

Arun Narenthiran Sivakumar1

Sahil Modi2 Mateus Valverde Gasparino1

Che Ellis3

Andres Eduardo Baquero Velasquez1 Girish Chowdhary∗,1,2

Saurabh Gupta∗,4

1Department of Agricultural and Biological Engineering, University of Illinois at Urbana-Champaign (UIUC)
2Department of Computer Science, UIUC, 4Department of Electrical and Computer Engineering, UIUC,
3EarthSense Inc.

1
2
0
2

l
u
J

6

]

O
R
.
s
c
[

1
v
2
9
7
2
0
.
7
0
1
2
:
v
i
X
r
a

Abstract—This paper describes a system for visually guided
autonomous navigation of under-canopy farm robots. Low-cost
under-canopy robots can drive between crop rows under the
plant canopy and accomplish tasks that are infeasible for over-
the-canopy drones or larger agricultural equipment. However,
autonomously navigating them under the canopy presents a
number of challenges: unreliable GPS and LiDAR, high cost
of sensing, challenging farm terrain, clutter due to leaves and
weeds, and large variability in appearance over the season and
across crop types. We address these challenges by building a
modular system that leverages machine learning for robust and
generalizable perception from monocular RGB images from
low-cost cameras, and model predictive control for accurate
control in challenging terrain. Our system, CropFollow, is able
to autonomously drive 485 meters per intervention on average,
outperforming a state-of-the-art LiDAR based system (286 meters
per intervention) in extensive ﬁeld testing spanning over 25 km.

I. INTRODUCTION

This paper describes the design of a visually-guided naviga-
tion system for compact, low-cost, under-canopy agricultural
robots for commodity row-crops (corn, soybean, sugarcane
etc), such as that shown in Figure 1. Our system, called
CropFollow, uses monocular RGB images from an on-board
front-facing camera to steer the robot to autonomously traverse
in between crop rows in harsh, visually cluttered, uneven,
and variable real-world agricultural ﬁelds. Robust and reli-
able autonomous navigation of such under-canopy robots has
the potential to enable a number of practical and scientiﬁc
applications: High-throughput plant phenotyping [43, 37, 68,
66, 58, 25], ultra-precise pesticide treatments, mechanical
weeding [41], plant manipulation [17, 61], and cover crop
planting [64, 62]. Such applications are not possible with over-
canopy larger tractors and UAVs, and are crucial for increasing
agricultural sustainability [55, 22].

Autonomous row-following is a foundational capability for
robots that need to navigate between crop rows in agricul-
tural ﬁelds. Such robots cannot rely on RTK (Real-Time
Kinematic)-GPS [21] based methods which are used for over-
the-canopy autonomy (e.g. for drones, tractors, and combine

and videos: https://ansivakumar.github.io/

Project website with data
learned-visual-navigation/.
Correspondence to {av7,girishc}@illinois.edu.
∗Girish Chowdhary and Saurabh Gupta contributed equally and are listed
alphabetically.

Fig. 1: CropFollow is an autonomous navigation system for under-
canopy agriculture robots. It uses RGB images from a front-facing
camera to output steering commands to drive the robot in crop rows.

harvesters) because of GPS signal attenuation and multi-
path errors. The under-canopy row-following task consists of
detecting and following the rows of crop, by determining the
distance from the rows and the angle relative to the row, and
using this to track speciﬁed row-relative pose. In a typical
80 acre land-parcel in row-crops, the rows are about 400
meter long and full of visual clutter. The crop rapidly grows
during the growing season, rendering a constantly chang-
ing visual environment. Therefore, autonomous navigation of
under-canopy robots has remained a challenging and open
problem. LiDAR is known to work under the canopy and
can return geometric information [32]. However, LiDAR is
costly, and it does not capture semantic information. For
example, LiDAR cannot directly distinguish whether observed
occupancy corresponds to untraversable obstacles (actual crop
plant stalk), or traversable obstacles (hanging leaves, weeds,
uneven terrain). This fundamentally limits LiDAR based meth-
ods from estimating distance and angle from the row, leading
to low robustness of autonomy, as reported by low distance-
between-interventions [32]. This motivates our use of richer
sensing and lower-cost modalities in the form of RGB images.
Using RGB images for under-canopy navigation however
has proven to be non-trivial and has become a primary
bottleneck for under-canopy robotics. Importance of semantics
precludes the use of traditional methods that infer geometry

CornrowChallenging terrainSensor occlusionFront RGB camera image

from monocular RGB image streams [45, 23]. Visual vari-
ability during the day and the season limits heuristic based
crop-lane detection algorithms, and visual similarity results
in positional drift with SLAM algorithms[67]. It
is clear
therefore, that the high variability and clutter in the agricultural
environment necessitates the use of learning. However, the
lack of large-scale datasets, the difﬁculty of collecting ﬁeld
data, lack of a clear reward signal, and the infeasibility of
building a simulator for this task, makes it challenging to
employ machine learning.

Our contribution in this paper is a ﬁeld-validated modular
vision based crop-row following system to overcome the above
challenges. We term this system CropFollow, as it provides
the foundational row-following capability to small, low-cost
robots. Our system decouples perception and control. The
perception system uses monocular RGB image from the on-
board camera to estimate row-relative robot pose. It does
so by directly estimating the robot’s relative heading to the
row (measured as the angle the robot makes with the row
direction), and robot’s placement in row (measured as the ratio
of distance from the left row to inter-row separation). These
data are fused with inertial measurements using a Bayesian
sensor fusion system (Extended Kalman ﬁlter (EKF)), and
utilized to generate row-following control in terms of desired
angle and speed for staying in the center of the row using a
nonlinear robust controller (Model Predictive Control (MPC)).
The ability to directly predict relative heading and distance
from monocular RGB images is one key novelty of our
approach, and has key efﬁciency and robustness beneﬁts: the
approach avoids having to ﬁrst detect the plants (which can be
many) [27], or explicitly segmenting the ground from plants
(which is highly challenging with more clutter in the envi-
ronment) [67]. Our presented system is able to successfully
traverse crop rows regardless of the crop’s growth stage. In
ﬁeld trials of about 25 kilometers, our system required fewer
interventions than a LiDAR based system [63](485 meters
per intervention vs. 286 m), while at the same time cutting
down sensing cost by 50×. In ofﬂine experiments, we ﬁnd
that the proposed perception models generalize well to new
crops. These results clearly establish that our modular visual
navigation system enables vision based autonomy for under-
canopy ﬁeld robots.

II. RELATED WORK

Autonomous Navigation in Agricultural Fields. GPS, alone
and in combination with IMU and RTK corrections,
is
commonly used for outdoor navigation for
tractors and
over-canopy agricultural robots [52, 2, 3, 70, 37, 18, 36].
Under-canopy navigation is concerned with autonomous row-
following between the rows of crops. In such under-canopy
environments, GPS suffers from signiﬁcant multipath errors
and signal attenuation under the canopy [32], furthermore
RTK correction signals aren’t always available. As an alterna-
tive, LiDAR data along with heuristics based algorithms for
row-following have been used for under-canopy and orchard
navigation [33, 6, 62, 32, 63]. However, LiDAR is costly,

sensitivity to noise, and cannot sense semantic or contextual
information.

in

This has motivated vision-based navigation systems. Past
work in vision-based agricultural navigation can be classiﬁed
into over the canopy [72, 26, 69, 34, 4], under-canopy in
orchards [59, 51, 7, 1] and under-canopy in row crops and
horticultural crops [67, 27]. Vanishing lines based heuristics
was commonly used in these works. In orchards and over-
canopy visual navigation setting crop rows are clearly visible,
which makes heuristic based line ﬁtting possible. However,
these algorithms do not directly apply to under-canopy naviga-
tion in commodity crops such as corn and soybean (the focus
of this paper) where the row-spacing is much tighter (10×
smaller than orchards), there is a high degree of visual clutter,
complete and frequent occlusion of the camera by leaves,
presence of weeds, crop residue on the ground, and changing
visual appearance as the crop grows (see Figure 4 and Figure 7
for examples). Incidentally, corn and soybean acerage is atleast
10× larger than orchards. Recent visual servoing with RGB-D
has been used for orchar navigation [1], however this approach
will not work in corn-soybean canopies due to visual clutter
and small-size of crops earlier in the growing season.
Classical Navigation. Navigation
classical mobile
robotics [60, 56] follows a modular approach with perception
(simultaneous
localization and mapping (SLAM)), path
planning relative to generated map, and trajectory tracking
control. There are various successful SLAM techniques
for this in structured and static environments such as in
urban self-driving and indoor navigation. However, geometric
reconstruction and localization in deformable and dynamic
under-canopy
challenging.
Furthermore, geometric approaches equate traversability with
free space. While generally true, in off-road ﬁeld settings this
is not true (short weeds are ﬁne to run over, hanging plant
leaves can be run into), and necessitates the use of learning.
Visual-inertial odometry (VIO) based approaches (e.g. [50])
that are common in other outdoor navigation tasks are not
useful here without a pre-built map, or GPS waypoints to
close the loop and prevent drift (see Figure 8), or navigation
in non-straight rows.
Learned Navigation. Researchers have used machine learning
for navigation and locomotion in situations where heuristics
have failed. Learning has been used in different ways: [28, 73,
65] learn high-level semantic cues and statistical regularities
for navigation, [39, 19] use learning to provide robustness to
actuation noise for path following, while [24, 54, 47, 53, 8]
rely on learning to reduce or eliminate the dependence on
expensive sensors for collision-free local navigation. Our work
falls into this last category. Our use of learning not only
eliminates dependence on LiDAR, but surpasses its perfor-
mance through better discrimination between traversible and
intraversible areas by use of learning on camera images.
Research in this last category can be further distinguished
based on the policy design and supervision used for training.
Given the infeasibility of simulation, challenging terrain, lack
of a reliable unsupervised self-supervision signal (as used

environments

agriculture

is

Fig. 2: CropFollow Overview. We use a convolutional network to output robot heading and placement in row. This is used to compute the
row center which is used as a reference trajectory. A model predictive controller converts reference trajectories to angular velocity commands.

learning,

in BADGR [35]), and difﬁculty of large-scale ﬁeld exper-
iments, renders reinforcement
imitation learning,
and self-supervision based methods infeasible for our task
[54, 47, 47, 24, 35, 49, 30]. Also, lack of large-scale datasets
for training has prevented the use of machine learning (over-
canopy datasets e.g. [15, 48], and urban self-driving datasets
e.g. [10] exist, but aren’t useful for under-canopy training).
Therefore, we employ a modular approach [5, 44, 11, 42] and
use supervised learning for training the perception module.
Eliminating trial-and-error from learning improves sample efﬁ-
ciency, and the use of an analytical low-level controller allows
easy generalization over varying terrains. Our contribution is in
the design and experimental validation of a modular autonomy
system in unique, challenging agricultural settings.
Learned Lane Following. Crop row following is similar to
lane following in context of self-driving cars, however is much
more challenging given no clear lane markings and extreme
amounts of clutter. Past lane following works use reactive
control based on traditional vanishing line estimation [71].
However, vanishing line estimation is brittle. Consequently,
recent works employ learning. [9] trained a vanishing point
estimation network from an urban driving dataset with clearly
visible lanes. Such lanes are not directly visible in our cluttered
under-canopy environments (See Figure 7 and Figure 9). Thus,
those models won’t work, as is, in our setting. Second, they
only output the vanishing point which only tells us about the
heading and not the distance ratio (see Appendix Section IV-
E). Our method bypasses having to estimate the vanishing
point and directly outputs all the necessary information re-
quired for the robot to navigate in under-canopy. Therefore,
our method is the most direct and efﬁcient way to achieve
under-canopy row following. [40, 46, 74] predict semantic
segmentation of the scene to estimate lane boundaries, while
[13, 57, 14] employ end-to-end learning to directly output
control commands via classiﬁcation or regression). The former

techniques require ﬁne-grained pixel
level annotations for
training, and real-time inference is computationally expensive.
End-to-end control is impractical in our setting as mentioned
above. [29] learns to predict the location of lane in the image
to estimate distance but does not predict heading and distance
directly. [12] show that CNNs can be trained to predict driving
affordances in uncluttered simulation environments where lane
markings are clearly visible. In contrast, our work provides
substantial experimental results that demonstrate that CNN
based state estimators can lead to high-performing autonomous
navigation systems capable of operating in the wild cluttered
under-canopy ﬁelds, surpassing the current default practice of
using a LiDAR.

Closest to our work, Gu et al. [27] use learning to detect
corn stalks and ﬁt lines. This approach suffers when corn stalks
are not visible, and has not been validated in real corn ﬁelds.
We follow an implicit approach to directly estimate the states
(row-relative heading and offset). This allows us to train a
machine learning system that is robust to these challenges, as
shown by our extensive in-ﬁeld validation.

III. SYSTEM DESIGN

Figure 2 shows an overview of our presented system. Images
from on-board RGB camera on the robot are processed through
a convolutional network to predict robot heading φ, and rela-
tive placement d between crop rows. This relative placement is
converted into the robot’s distance from the left and the right
crop rows by multiplying with the lane width. These heading
and distance predictions are ﬁltered using a Bayesian ﬁlter
(we use the Extended Kalman Filter) that optionally also fuses
them with high-frequency input from an inertial measurement
unit. The ﬁltered heading and distances are used to generate a
course correcting reference path in the robot coordinate frame.
A model predictive controller is used to compute angular
velocity commands to achieve this reference path. A lower-

Applied turn rate commandConstrained cost optimizationMPC output pathReference path to followEKFReference pathKinematic model: Heading angle𝑑𝜙𝑊: Row width: Distance ratioCNN state estimation𝑊(1−𝑑)𝑊𝑑𝜙MPC output pathωConv2D(𝐾,𝑆,𝑃,𝐶𝑜𝑢𝑡) = (3, 2, 1, 64)Linear1280 (flat)Linear64, dropoutTruncated ResNet-18512240320Model 1: HeadingModel 2: Distance RatioFig. 3: Our method uses the robot’s heading, φ and ratio of distance
from the left and the right crop row, d = dL/(dL + dR), as the
intermediate representation between perception and planning.

level proportional–integral–derivative (PID) controller is used
to track the commanded angular velocity.

In this section, we describe the robot platform, the CNN
architecture, the Extended Kalman Filter, and the model pre-
dictive controller. We describe the data collection and ground
truth generation procedure in Section IV.
Robot Platform. TerraSentia is an ultra-compact 4-wheeled
skid-steering mobile robot designed to drive through ﬁelds and
collect data. It has a Raspberry Pi 3 on-board for lower-level
motor control and an Intel i7 NUC for data processing and
navigation. Note that our unit had no discrete GPU, so the
integrated Intel GPU is used for model inference. This robotic
system is equipped with various sensors but only 4 are relevant
to this paper. There is a dedicated GPS module that determines
baseline autonomous driving performance (when GPS signal is
reliable). The current LiDAR-based autonomy is fueled by the
2D horizontal-scanning LiDAR (Hokuyu UST-10LX) and a 6
DOF Inertial Measurement Unit (IMU). Finally, our approach
utilizes only the forward facing, 720p at 30 fps monocular
camera sensor (OV2710) and an IMU. We note speciﬁcally
that since LiDAR is not utilized in our presented visual system,
no explicit real-time depth signal is available to the model.
Perception Model. We choose a learning approach due to
its superior generalizability compared to color-based seg-
mentation navigation proposed by previous works. Figure 7
shows the classical system’s failure to segment the lane in
common late stage data. CropFollow’s perception model takes
in 320 × 240 RGB images and outputs the robot heading (in
degrees) and its relative placement in the crop row. Figure 3
shows how the heading and the relative placement is deﬁned.
Heading φ is the angle of the robot relative to crop rows. The
relative distance d is the ratio of the distance to the left of the
row to the lane width, i.e. d = dL
, where dL and dR are
the distances to left and right crop rows.

dL+dR

The perception model uses a ResNet-18 [31] backbone that
has been pretrained on ImageNet [20]. We truncate ResNet-18
right before the average-pooling layer, and add in an additional
convolutional
layer, a fully connected layer, dropout, and
ﬁnal prediction layer. The ﬁnal prediction layer outputs the
heading φ, and the distance ratio d. We found that independent
networks to predict heading and distance ratio worked better
than a single joint network.

Fig. 4: Sample images from the collected dataset.

IMU Fusion with Extended Kalman Filter. An Extended
Kalman Filter [16, 21, 56] was used to reduce the effect of
uncertainties in distance and heading estimations by fusing the
inertial data with the vision data. We used s = ( dL dR φ )T
as the state. State sk evolves over time as per the prediction
function f (sk−1, uk−1) (derived using the robot’s kinematics,
see supplementary). Here sk−1 is the state at the previous
time step, and uk−1 is the linear and angular velocity at the
previous time step. Robot’s linear speed v and angular speed ω
are calculated from wheel encoders, and IMU respectively. We
assume additive zero-mean Gaussian process and measurement
noise. As we directly observe s, the measurement function
is an identity function. Output from the CNN is used in the
update step. More details about the form of the prediction
function, and co-variances of the Gaussian noise are provided
in the supplementary material.
Model Predictive Controller. We used a non-linear Model
Predictive Controller (MPC) to generate angular speed com-
mands to the robot given the reference path to be followed,
as shown in Figure 2 [37, 36]. MPC uses the fused output
states s = ( dL dR φ )T from the EKF, the Unicycle kinematic
model (see supplementary) of the robot and reference path,
which is a straight line through the center of the lane, to solve
a constrained optimization problem with the minimum and
maximum curvature radius as the constraints. The output is a
path deﬁned in terms of the curvature ρ, which determines the
angular velocity ω = ρ v where v is the linear velocity. The
angular speed for the ﬁrst point in the output path is applied
and the optimization process is repeated. A PID controller
is used to maintain the commanded angular speed, based on
feedback from IMU’s yaw angular speed.

IV. DATA COLLECTION AND GROUND TRUTHING

Given lack of any under-canopy agriculture datasets, we
collected a large dataset by driving the TerraSentia robot under
the canopy. We manually operated the robot in 19 corn and
4 soybean ﬁelds across Illinois and Indiana, and collected
time-series data from the front-facing RGB camera, LiDAR,
and IMU. We collected 2.7 hours of corn data and 1.2 hours
of soybean data, and made sure to collect data for different
growth stages. We also included data where the robot was
driven in a zigzag manner. This was done to expose the

𝑑𝐿𝜙𝑑𝑅perception models to a broader distribution of data that may be
experienced during autonomous runs. Figures 4 and 12 shows
sample corn and soybean images from the dataset. We note the
variability in appearance, occlusion, challenging illumination
(shadows, low-light under the canopy), challenging terrain, and
leafy plants. This raw data and a subset of annotations will be
made available upon acceptance.

Ground Truthing. To train our perception model from
Section III, we need labels for robot heading and the ratio of
the distance from the left and the right crop rows. Preliminary
investigation of using LiDAR for extracting this information
for training wasn’t fruitful. Hence, we gathered human labels.
However, asking humans for such geometric labels is not
easy. Unlike semantic labels, such metric geometric quantities
are non-trivial for humans to label. As an example, consider
images in Figure 4, and consider speculating the robot heading
and placement
this issue, we
in the row. To circumvent
designed an indirect annotation procedure. We asked humans
to label the horizon and the vanishing lines corresponding
to the crop row (Figure 6 (left)). This together with the
camera calibration information allows us to recover the robot
heading and placement
in row using projective geometry.
Figure 5 provides an overview of the different steps involved in
computing these quantities from the annotated images. For the
case where the horizon is not visible, we instead ask humans to
mark out vertical crop stalks (Figure 6 (right)). This allows us
to estimate the vanishing point for the vertical direction which
readily provides the slope of the horizon. Precise formulae and
derivations are provided in the supplementary material.

We annotated a total of 25, 296 corn images. 28% of these
are from early growth stage, while 72% are from late growth
stage. We split the dataset into a training and a validation set
(83% training, 17% validation). We made sure that data from
the same video is either entirely in the training set, or entirely
in the validation set. Our main experiments use this corn data.
We also labeled 10, 685 soybean images (54% early, 46% mid)
to study transfer across crops.

V. EXPERIMENTAL RESULTS

Our experiments are designed to test the autonomous crop
row traversal capability of our proposed system, effectiveness
of the proposed modular policy, and data efﬁciency and gen-
eralization of our learned models. We evaluate these aspects
through a combination of ofﬂine and online (ﬁeld) experi-
ments. Ofﬂine experiments are conducted on our collected
dataset. They allow us to systematically study data efﬁciency
and model generalization, and help us chose models for online
experiments. Online experiments are conducted in the ﬁeld,
and allow us to study the interplay between perception and
control systems. We also conduct end-to-end evaluation for
the task of crop row traversal, and compare against an existing
system based on LiDAR [63].

A. Ofﬂine Evaluation of Perception Model

Ofﬂine evaluation of the perception module is conducted
on the collected dataset. All experiments except ones for

Model

Baseline
Combined
Separate

Mean

Median

95%ile

φerr

11.41
2.24
1.99

derr

0.48
0.08
0.04

φerr

8.81
1.39
1.21

derr

0.48
0.06
0.03

φerr

30.33
5.37
4.71

derr

0.65
0.20
0.10

TABLE I: Perception Module Performance: We report L1 error in
heading (in ◦) and distance ratio prediction. The trivial baseline model
always predicts median φ, d from the training set. The combined
model learns heading and distance simultaneously, but ultimately
performs worse than individually trained models.

generalization across crops, use the corn to train and test.
Metrics. We measure prediction performance using L1 error
in heading and distance ratio predictions, φ and d.
Training. We used ResNet-18 [31] pretrained on Ima-
geNet [20] to initialize our models. Models were trained to
minimize the L2 loss with the Adam optimizer [38] for 50
epochs. We started with an initial learning rate of 10−4 and
dropped it by a factor of 10 at 40th and 45th epochs. All layers
of the network were optimized.
Results. Table I presents the performance of our CNN models.
We experimented with 2 variants: predicting heading and
distance ratio separately using two models, and a single multi-
task network. For reference, we also report the performance for
a trivial predictor that always predicts the median heading and
distance ratio from the training set. This measures the hardness
of the task, puts performance of our model in context.

Both models worked well, with the separate model variant
working better. Our best model achieves an average L1 error of
1.99◦ for heading, and 0.04 for distance ratio. Inference speed
for this model on the robot was around 20 FPS, which is fast
enough for accurate control (more on this in Section V-D).
Our main ﬁeld experiments are conducted with this model.

B. Comparison with Classical Baselines

Color-based segmentation is a common ﬁrst step in classical
vanishing lines based row following literature. Figure 7 shows
the results of automatic color-based segmentation on common
late stage data. We see that the segmented lane is not clear.
This validates CropFollow’s learning-based approach as a gen-
eral navigation system for all growth stages across the season.
To compare with a feature matching based VIO algorithm,
Vins-fusion was used as the baseline [50]. To compare with
stereo based Vins-fusion as well, data collected from Intel
Realsense D435i camera was used only for this experiment
and recommended intrinsic values from Realsense library was
used as Vins-fusion parameters. Figure 8 demonstrates the
heading and cross track error of CropFollow, Vins-fusion with
monocular RGB camera and with stereo IR camera. Note that
in case of distance the plot shows cross track error (offset
distance from the middle of the lane) and not the relative
error with respect to the ground truth. The ground truth was
calculated by annotating vanishing lines and horizon (same
approach as training labels for CropFollow). Ground truth
heading and distance ratio at ﬁrst frame was used to initialize
Vins-fusion localization (both monocular RGB and stereo
IR). CropFollow is vastly superior to Vins-fusion in distance

Fig. 5: Ground truthing procedure. Using the horizon annotations, we correct for the camera roll, and pitch. After this, heading, φ can
be calculated by looking at the crop row vanishing point, and distance ratio can be computed from the intercepts of the crop row lines in
the heading corrected image, as dL/(dL + dR).

Fig. 6: Annotations. We annotate the horizon and crop rows for
early season images (left). For late season images when the horizon
is not visible, we annotate the vertical corn stalks (right).

Fig. 7: Classical color-based vanishing line segmentation on late stage
data according to related works [7, 67]. We see that crop segmentation
does not produce a clear visual of the lane, so automatic vanishing
line based lane-following is not possible. In particular, extraneous
leaves artiﬁcially alter the boundaries of the lane.

prediction as seen from very similar cross track error as ground
truth and is comparable in heading. Although Vins-fusion
shows comparable heading tracking, it suffers signiﬁcantly
from position drift which is orders of magnitude greater than
the lane width between crop rows (about 0.75m) making it
impractical to use for row following. This is because there
is no opportunity for loop closure in long crop rows. This
validates reactive navigation as pursued in CropFollow is a
valid approach for row following.

C. In Field End-to-End System Evaluation

We conducted end-to-end system evaluation with the model
described above. We compared the performance of the follow-
ing 2 systems, along with 2 variants each:
• CropFollow (w/ IMU). This is our proposed system that
uses the above CNN model for heading and distance ratio
prediction, EKF for fusing IMU information, and MPC
for executing control commands. We also compare with
a variant that does not use IMU information (denoted by
CropFollow (w/o IMU)).

• LiDAR System [63] (w/ IMU). This system uses readings
from the LiDAR mounted on top of the robot to estimate the
robot heading and distance from the crop rows using line
ﬁtting. Other parts of the system are same as our system:
Use of an EKF to fuse information from the IMU, and use
of MPC for generating control commands. We also compare

Fig. 8: CropFollow vs. Vins-fusion mono vs. Vins-fusion stereo.
We compare the cross track error (CTE) (offset distance of the robot
from the middle of the lane) and heading of CropFollow, Vins-fusion
with mono and Vision-fusion with stereo IR at different frames in a
trajectory. CropFollow shows better CTE than Vins-fusion.

Fig. 9: Sample images from ﬁeld trials. Bottom row consists of
traditionally adverse conditions for vision-based navigation.

to a variant that does not use IMU information (denoted by
LiDAR System [63] (w/o IMU)).

Evaluation Methodology. All 4 systems are tested on the
same unique 4.85 km. These 4.85 km come from 15 different
experiments that were done in different parts of the ﬁeld,
over different growth stages, different days, different time
of the day, and weather conditions. While there is a lot of
variability in these 4.85 km, we attempted to minimized the
variability in conditions for the 4 systems to ensure result
comparability. Runs for the different systems for each of the
15 experiments were done one after another over the same

𝜃𝑟Rotate image about yaw axis by−𝜙, where 𝜙=arctan(𝑥/𝑓)Rotate image about roll axis by −𝜃𝑟𝑦midmid𝑥Rotate image about pitch axis by −arctan𝑦/𝑓𝜆𝑑𝑅𝜆𝑑𝐿Front Camera ImageRecovered Lane MaskSegmented LaneSegmentating Corn Rows for Vanishing Lines0102030405060Timestamp (s)0246Cross-Track Error (m)Distance ComparisonStereoMonocularCropFollowGround Truth0102030405060Timestamp (s)15105051015Heading (degrees)Heading ComparisonGrowth Length LiDAR LiDAR CropFollow CropFollow
Stage

(in m) w/ IMU w/o IMU

w/o IMU

w/ IMU

Early
Late

1120
3726

-
13

-
72

3
7

4
8

TABLE II: Field Experiments: We report the number of interven-
tions for the different methods. LiDAR can’t operate in early season
as crops are too short. Our system can work under both conditions
and requires interventions.

Method

Trial 1

Trial 2

Trial 3

Trial 4

Trial 5

LiDAR
CropFollow

9
0

17
0

8
0

7
0

19
0

Fig. 10: Performance as a function of amount of training data. We
sub-sample training data by either removing entire data collection
runs, or by removing frames. Our perception model starts doing well
even with small amounts of data.

TABLE III: We report the number of interventions of LiDAR w/o
IMU and CropFollow w/o IMU by repeating the test on the same row
5 times in the ﬁeld. CropFollow outperformed LiDAR in all trials.
Variation in LiDAR counts shows its sensitivity to noise.

routes, and with the same constant linear robot velocity of
0.6 m/s. Run order for the different systems was randomized
to prevent environmental bias. This experiment thus presents
results pooled over ﬁeld trials of 19.4 km. For each method,
we measure the number of human interventions needed to
complete the experiment. Human interventions were required
when the robot crashed into the corn stalks. This metric
measures autonomy effectiveness.
Results. Table II reports the number of interventions for the
4 systems that we evaluated. We separately report results for
early and late season experiments. Note that LiDAR system
from [63] can’t operate in early season data since early season
corn stalks are shorter than the robot, and not detected by
the 2-D LiDAR. Our vision based systems works reasonably
well. In late season when the LiDAR based system does
work, we note that it had more interventions than our system,
72 vs. 8 without IMU, and 13 vs. 7 with IMU. Thus, our
presented vision-based system outperforms the LiDAR based
system, while also reducing sensing cost by 50× ($30 for
RGB camera, while $1500 for LiDAR). Note that these are
paired experiments done over long run lengths (4.85 km), and
the performance gap is statistically signiﬁcant (with p-value
< 10−3). To further compare the without IMU versions of the
LiDAR system and CropFollow, we did an experiment where
LiDAR failed and CropFollow succeeded, and did 4 additional
runs for each method (Table III). We found CropFollow to
work better than the LiDAR system in all 5 trials. The quality
of our output is further shown by the fact that our system is
closing the loop only at about 20Hz, vs. 40Hz for the LiDAR
system, but still achieves a better end performance.

D. Training Data Efﬁciency and Generalization

The above experiments demonstrate that our proposed sys-
tem works. We next conduct experiments to measure data
efﬁciency and generalization ability of our trained models. We
investigate three questions: How much labeled data did we
actually need to get good prediction and ﬁeld performance?
How much data do we need for the next crop? And what is
the best use of annotation budget? We answer these questions

Fig. 11: Performance when training and testing on early vs. late vs.
combined data. Models trained on only early or only late data don’t
generalize well, and training on combined data works best.

through a combination of ﬁeld and ofﬂine experiments.
Data Efﬁciency for Corn. We ﬁrst measure the data efﬁ-
ciency of learning through ofﬂine experiments. We report the
validation performance as a function of the amount of training
data. We consider 2 versions obtained by sub-sampling a) at
the level of data collection runs; b) at the level of frames.
Figure 10 plots performance as a function of training dataset
size. We make two observations. First, models start performing
well at around 10K labeled images. Second,
is more
beneﬁcial to label images from many different runs, than many
images from a few runs.

it

We also study if we need data from all growth stages to
learn a good model. Figure 11 reports validation performance
on each growth stage for models trained on 6000 images of

Number of Training Images

100

1000

20986

Validation Metrics (Mean L1 Error)

Heading Error
Distance Error

6.28
0.09

4.19
0.08

1.99
0.04

In ﬁeld Metrics (Number of Interventions)

CropFollow (w/ IMU) @ 22 FPS
CropFollow (w/ IMU) @ 10 FPS
CropFollow (w/ IMU) @ 5 FPS
CropFollow (w/ IMU) @ 2.3 FPS

CropFollow (w/o IMU) @ 22 FPS
CropFollow (w/o IMU) @ 10 FPS
CropFollow (w/o IMU) @ 5 FPS
CropFollow (w/o IMU) @ 2.3 FPS

0
0
4
failed

0
1
2
failed

0
0
0
0

1
2
0
8

0
0
0
0

0
0
0
9

TABLE IV: Field and ofﬂine validation of models trained with 100,
1000 and 20986 images to study training data efﬁciency.

1024204840968192163842.02.53.03.54.0Mean L1 ErrorHeadingRemove FramesRemove Runs1024204840968192163840.050.060.070.08Distance RatioRemove FramesRemove RunsTraining Data Size (# images, log scale) ( D U O \ / D W H % R W K ( D U O \ / D W H % R W K                                     + H D G L Q J  ( U U R U ( D U O \ / D W H % R W K ( D U O \ / D W H % R W K                                     ' L V W D Q F H  5 D W L R  ( U U R U 9 D O L G D W L R Q  6 H W 7 U D L Q L Q J  6 H WFig. 12: Sample early and mid stage soybean. Note the stark
difference to corn (right). Soybean is stouter with broader leaves.

Fig. 13: Generalization from corn to soybean. Model trained on
corn (Transfer) generalizes well to soybean in comparison to training
from ImageNet initialization (Scratch).

either early stage, late stage, or an equal combination of both.
We note that both models trained on a single growth stage
have poor performance on the other growth stage. Our model
that is trained on a blend of early and late stage data is most
accurate throughout the entire season with an average error of
1.28◦ and 0.03 for heading and distance ratio respectively.
Field Experiments for Data Efﬁciency for Corn. However,
note this is only performance of the perception module in
isolation. It will be more instructive to look at
the ﬁeld
performance of the whole system as a function of the training
set size. Table IV reports ﬁeld performance of 3 models trained
with 100, 1000 and 21K images (we took the models that
sub-sampled data at the level of runs as they had a sharper
drop in performance), in the same crop row of length 428m.
Interestingly, we note that at the base control frequency of
20Hz, systems trained with as little as 100 images worked
without interventions! It should be noted that this does not
mean that 100 images are sufﬁcient for robust and repeatable
performance, but shows that the system learns quite a bit with
little data, and the modular approach which leverages the IMU
and a robust controller is capable of tolerating a less perfect
perception system. Indeed, difference in performance is more
evident at lower update rates. Perception models trained on
larger datasets are likely more robust to extreme viewpoints
and hence can recover better from off-center locations that may
arise at lower update rates. These results provide information
on allowable heading and distance ratio prediction error at dif-
ferent speeds and update rates with which amount of training
data needed for training in new crops can be determined. It
can be seen that with higher prediction errors, using IMU and
higher model update rate makes the system robust.
Generalization to Another Crop. We also study the data
efﬁciency for enabling autonomous navigation for a new
crop. We do this via ofﬂine experiments and measure how
much additional training data is needed to adapt a model
trained on corn to achieve good performance. Figure 13 plots

the validation metrics as a function of number of Soybean
training images (Glycine max, Figure 12), for our transferred
model, and for a baseline model that starts from ImageNet
initialization. We note strong transfer of the model trained on
Corn. Even without any training on Soybean, our two models
achieve good performance with a average error of just 2.20◦
and 0.07 for heading and distance. Although only from Corn
to Soybean, this is a very desirable result. It suggests that our
Corn model might already work in Soybean rows with minimal
additional labeling. We leave ﬁeld trials to future work.

E. Error Modes and Stress Testing

To understand the common error modes in our CropFollow
and LiDAR system, we visualized the front camera video
stream before failures in ﬁeld experiments. Also, to stress
test the proposed CropFollow, ﬁeld tests were conducted in
a challenging ﬁeld with a sharp curve, occlusions and gaps
and experiments were also conducted to test the performance
of CropFollow at increased speeds.
Visualization of different error modes. Figure 14 shows the
different error modes in CropFollow and LiDAR navigation
system. Large gaps in crop rows was the common cause of
failure in CropFollow (our training data did not include such
cases). Sensor occlusion and bumpy terrain were the other rare
causes of failures. In contrast, failure due to gaps was rarely
observed in LiDAR since it was speciﬁcally engineered to be
robust to it. But because of its high sensitivity to noise, even
minor sensor occlusion by leaves affects LiDAR performance
and leads to interventions. CropFollow’s performance in gaps
could be improved with adding training data whereas LiDAR’s
occlusion problem is a sensor limitation.
Stress testing. To test the performance in challenging con-
ditions, CropFollow (w/ and w/o IMU) was tested in a ﬁeld
with sharp curves, gaps and occlusion from weeds. 3 and 6
interventions w/ and w/o IMU respectively was observed in a
test of 600m. Last row in Figure 9 shows the challenging
condition in this ﬁeld. Also, CropFollow’s performance at
higher speeds was tested. CropFollow showed same stable
behavior at 1m/s but oscillations in trajectory due to latency
was observed at 1.4m/s or more.

VI. CONCLUSION

We presented a vision based autonomous under-canopy
navigation system. Through a modular architecture and a
learning-based approach we showed that machine vision can
be applied for reliable and robust navigation in cluttered,
changing, and harsh under-canopy environments. 25 km of
real-world validation on an under-canopy robot demonstrated
that our visual navigation approach is not only 50× more cost-
effective than LiDAR but also leads to fewer interventions.
Our system forms a new benchmark for visual navigation
under the canopy, and our openly accessible dataset (1030
labeled images and 24266 unlabeled images of our corn
data) will enable further research. We hope our results and
dataset pave the way for wider adoption of learned visual

1001000100002.55.07.510.012.5Mean L1 ErrorHeadingScratchTransfer1001000100000.060.080.10Distance RatioScratchTransferAdditional Soy Training Data Size (# images, log scale)Fig. 14: Failure scenarios for the different navigation systems. We group them into modes: a) bumpy terrain causing noisy, blurry images,
b) sensor occlusion from leaves, and c) gaps in crop rows.

navigation systems in challenging application domains, such
as agriculture and off-road driving.

Learning for Visual Navigation in Novel Environments.
In Conference on Robot Learning, 2019.

VII. ACKNOWLEDGMENTS

This paper was

supported in part by NSF STTR
#1820332, USDA/NSF CPS project #2018-67007-28379,
USDA/NSF AIFARMS National AI Institute USDA #020-
accession no. 1024178, NSF IIS
67021-32799/project
#2007035, and DARPA Machine Common Sense. We thank
Earthsense Inc. for the robots used in this work and we thank
the Department of Agricultural and Biological Engineering
and Center for Digital Agriculture (CDA) at UIUC for the Illi-
nois Autonomous Farm (IAF) facility used for data collection
and ﬁeld validation of CropFollow. We thank Vitor Akihiro H.
Higuti and Sri Theja Vuppala for their help in integration of
CropFollow on the robot and ﬁeld validation.

REFERENCES

[1] Diego Aghi, Vittorio Mazzia, and Marcello Chiaberge.
Local motion planner for autonomous navigation in vine-
yards with a rgb-d camera-based algorithm and deep
learning synergy. Machines, 8(2):27, 2020.

[2] Thomas Bak and Hans Jakobsen. Agricultural robotic
platform with four wheel steering for weed detection.
Biosystems Engineering, 87(2):125–136, 2004.

[3] Tijmen Bakker, Kees van Asselt, Jan Bontsema, Joachim
M¨uller, and Gerrit van Straten. Autonomous navigation
using a robot platform in a sugar beet ﬁeld. Biosystems
Engineering, 109(4):357–368, 2011.

[4] David Ball, Ben Upcroft, Gordon Wyeth, Peter Corke,
Andrew English, Patrick Ross, Tim Patten, Robert Fitch,
Salah Sukkarieh, and Andrew Bate. Vision-based ob-
stacle detection and navigation for an agricultural robot.
Journal of ﬁeld robotics, 33(8):1107–1130, 2016.

[5] Somil Bansal, Varun Tolani, Saurabh Gupta, Jitendra Ma-
lik, and Claire Tomlin. Combining Optimal Control and

[6] Oscar C Barawid Jr, Akira Mizushima, Kazunobu Ishii,
and Noboru Noguchi. Development of an autonomous
navigation system using a two-dimensional laser scanner
in an orchard application. Biosystems Engineering, 96
(2):139–149, 2007.

[7] Marcel Bergerman, Silvio M Maeta, Ji Zhang, Gustavo M
Freitas, Bradley Hamner, Sanjiv Singh, and George Kan-
tor. Robot farmers: Autonomous orchard vehicles help
IEEE Robotics & Automation
tree fruit production.
Magazine, 22(1):54–63, 2015.

[8] Mariusz

Bojarski, Davide Del

Testa, Daniel
Dworakowski, Bernhard Firner, Beat Flepp, Prasoon
Goyal, Lawrence D Jackel, Mathew Monfort, Urs
Muller, Jiakai Zhang, et al. End to end learning for
arXiv preprint arXiv:1604.07316,
self-driving cars.
2016.

[9] Chin-Kai Chang,

Jiaping Zhao, and Laurent

Itti.
DeepVP: Deep learning for vanishing point detection on
1 million street view images. In 2018 IEEE International
Conference on Robotics and Automation (ICRA), pages
4496–4503. IEEE, 2018.

[10] Ming-Fang Chang, John Lambert, Patsorn Sangkloy, Jag-
jeet Singh, Slawomir Bak, Andrew Hartnett, De Wang,
Peter Carr, Simon Lucey, Deva Ramanan, et al. Argo-
verse: 3d tracking and forecasting with rich maps.
In
Computer Vision and Pattern Recognition, 2019.
[11] Devendra Singh Chaplot, Saurabh Gupta, Dhiraj Gandhi,
Abhinav Gupta, and Ruslan Salakhutdinov. Learning To
Explore Using Active Neural Mapping. In International
Conference on Learning Representations, 2020.

[12] Chenyi Chen, Ari Seff, Alain Kornhauser, and Jianxiong
Xiao. Deepdriving: Learning affordance for direct per-
In Proceedings of the
ception in autonomous driving.
IEEE international conference on computer vision, pages

a) Bumpy terrainb) Occlusionc) Gaps in the row CropFollow w/ IMUCropFolloww/o IMULiDARw/IMU2722–2730, 2015.

[13] Z. Chen and X. Huang. End-to-end learning for lane
In 2017 IEEE Intelligent
keeping of self-driving cars.
Vehicles Symposium (IV), pages 1856–1860, 2017. doi:
10.1109/IVS.2017.7995975.

[14] Lu Chi and Yadong Mu. Deep steering: Learning end-to-
end driving model from spatial and temporal visual cues.
arXiv preprint arXiv:1708.03798, 2017.

[15] Mang Tik Chiu, Xingqian Xu, Yunchao Wei, Zilong
Huang, Alexander G Schwing, Robert Brunner, Hrant
Khachatrian, Hovnatan Karapetyan, Ivan Dozier, Greg
Rose, et al. Agriculture-vision: A large aerial image
database for agricultural pattern analysis. In Computer
Vision and Pattern Recognition, 2020.

[16] Girish Chowdhary, Eric N. Johnson, Daniel Magree,
Allen Wu, and Andy Shein. GPS-denied Indoor and Out-
door Monocular Vision Aided Navigation and Control of
Unmanned Aircraft. Journal of Field Robotics, 30(3):
415–438, 2013.

[17] Girish Chowdhary, Mattia Gazzola, Girish Krishnan,
Chinmay Soman, and Sarah Lovell. Soft Robotics as
an Enabling Technology for Agroforestry Practice and
Research. Sustainability, 11(23):6751, Nov 2019. ISSN
2071-1050. doi: 10.3390/su11236751.

[18] L Cordesses, Christian Cariou, and M Berducat. Com-
bine harvester control using real time kinematic GPS.
Precision Agriculture, 2(2):147–161, 2000.

[19] Samyak Datta, Oleksandr Maksymets, Judy Hoffman,
Stefan Lee, Dhruv Batra, and Devi Parikh.
Integrating
Egocentric Localization for More Realistic Point-Goal
Navigation Agents. volume abs/2009.03231, 2020.
[20] Jia Deng, Wei Dong, Richard Socher, Li-Jia Li, Kai Li,
and Li Fei-Fei.
Imagenet: A large-scale hierarchical
image database. In 2009 IEEE conference on computer
vision and pattern recognition, pages 248–255. IEEE,
2009.
[21] Jay Farrell.

Aided navigation: GPS with high rate

sensors. McGraw-Hill, Inc., 2008.

[22] Jonathan A Foley, Navin Ramankutty, Kate A Brau-
man, Emily S Cassidy, James S Gerber, Matt Johnston,
Nathaniel D Mueller, Christine OaConnell, Deepak K
Ray, Paul C West, Christian Balzer, Elena M. Ben-
nett, Stephen R. Carpenter, Jason Hill, Chad Monfreda,
Stephen Polasky, Johan Rockstram, John Sheehan, Stefan
Siebert, David Tilman, and David P. M. Zaks. Solutions
for a cultivated planet. Nature, 478(7369):337–342,
2011.

[23] Yasutaka Furukawa and Jean Ponce. Accurate, dense,
IEEE transactions on
and robust multiview stereopsis.
pattern analysis and machine intelligence, 32(8):1362–
1376, 2009.

[24] Dhiraj Gandhi, Lerrel Pinto, and Abhinav Gupta. Learn-

ing to ﬂy by crashing. In IROS, 2017.

[25] Tianshuang Gao, Hamid Emadi, Homagni Saha, Jiaoping
Zhang, Alec Lofquist, Arti Singh, Baskar Ganapathysub-
ramanian, Soumik Sarkar, Asheesh K Singh, and Sourabh

Bhattacharya. A novel multirobot system for plant
phenotyping. Robotics, 7(4):61, 2018.

[26] Iv´an D Garc´ıa-Santill´an, Mart´ın Montalvo, Jos´e M Guer-
rero, and Gonzalo Pajares. Automatic detection of curved
and straight crop rows from images in maize ﬁelds.
Biosystems Engineering, 156:61–79, 2017.

[27] Yili Gu, Zhiqiang Li, Zhen Zhang, Jun Li, and Liqing
Chen.
Path Tracking Control of Field Information-
Collecting Robot Based on Improved Convolutional Neu-
ral Network Algorithm. Sensors, 20(3):797, 2020.
[28] Saurabh Gupta, James Davidson, Sergey Levine, Rahul
Sukthankar, and Jitendra Malik. Cognitive mapping
In Proceedings of
and planning for visual navigation.
the IEEE Conference on Computer Vision and Pattern
Recognition, pages 2616–2625, 2017.

[29] Alexandru Gurghian, Tejaswi Koduri, Smita V Bailur,
Kyle J Carey, and Vidya N Murali. Deeplanes: End-to-
end lane position estimation using deep neural networks.
In Proceedings of the IEEE Conference on Computer
Vision and Pattern Recognition Workshops, pages 38–45,
2016.

[30] Raia Hadsell, Pierre Sermanet, Jan Ben, Ayse Erkan,
Marco Scofﬁer, Koray Kavukcuoglu, Urs Muller, and
Yann LeCun. Learning long-range vision for autonomous
off-road driving. Journal of Field Robotics, 26(2):120–
144, 2009.

[31] Kaiming He, Xiangyu Zhang, Shaoqing Ren, and Jian
In
Sun. Deep residual learning for image recognition.
Proceedings of the IEEE Conference on Computer Vision
and Pattern Recognition, pages 770–778, 2016.

[32] Vitor AH Higuti, Andres EB Velasquez, Daniel Varela
Magalhaes, Marcelo Becker, and Girish Chowdhary. Un-
der canopy light detection and ranging-based autonomous
navigation. Journal of Field Robotics, 36(3):547–567,
2019.

[33] Santosh A Hiremath, Gerie WAM Van Der Heijden,
Frits K Van Evert, Alfred Stein, and Cajo JF Ter Braak.
Laser range ﬁnder model for autonomous navigation of
a robot in a maize ﬁeld using a particle ﬁlter. Computers
and Electronics in Agriculture, 100:41–50, 2014.
[34] Guoquan Jiang, Zhiheng Wang, and Hongmin Liu. Auto-
matic detection of crop rows based on multi-ROIs. Expert
systems with applications, 42(5):2429–2441, 2015.
[35] Gregory Kahn, Pieter Abbeel, and Sergey Levine. Badgr:
An autonomous self-supervised learning-based naviga-
tion system. arXiv preprint arXiv:2002.05700, 2020.
[36] Erkan Kayacan, Sierra N Young, Joshua M Peschel, and
Girish Chowdhary. High-precision control of tracked
ﬁeld robots in the presence of unknown traction coef-
Journal of Field Robotics, 35(7):1050–1062,
ﬁcients.
2018.

[37] Erkan Kayacan, Zhongzhong Zhang, and Girish Chowd-
hary. Embedded High Precision Control and Corn Stand
Counting Algorithms for an Ultra-Compact 3D Printed
Proceedings of Robotics: Science and
Field Robot.
Systems. Pittsburgh, Pennsylvania, 2018.

[38] Diederik P Kingma and Jimmy Ba. Adam: A method for
stochastic optimization. arXiv preprint arXiv:1412.6980,
2014.

[39] Ashish Kumar, Saurabh Gupta, David Fouhey, Sergey
Levine, and Jitendra Malik. Visual Memory for Robust
Path Following. In NeurIPS, 2018.

[40] Yecheng Lyu, Lin Bai, and Xinming Huang. Road
In 2019
segmentation using cnn and distributed lstm.
IEEE International Symposium on Circuits and Systems
(ISCAS), pages 1–5. IEEE, 2019.

[41] Wyatt McAllister, Joshua Whitman, Allan Axelrod,
Joshua Varghese, Girish Chowdhary, and Adam Davis.
Agbots 2.0: Weeding Denser Fields with Fewer Robots.
In Proceedings of Robotics: Science and Systems, Cor-
valis, Oregon, USA, July 2020. doi: 10.15607/RSS.2020.
XVI.062.

[42] Xiangyun Meng, Nathan Ratliff, Yu Xiang, and Dieter
Fox. Scaling local control to large-scale topological
navigation. In 2020 IEEE International Conference on
Robotics and Automation (ICRA), pages 672–678. IEEE,
2020.

[43] Tim Mueller-Sim, Merritt Jenkins, Justin Abel, and
George Kantor. The Robotanist: a ground-based agri-
cultural robot for high-throughput crop phenotyping. In
2017 IEEE International Conference on Robotics and
Automation (ICRA), pages 3634–3639. IEEE, 2017.
[44] Matthias M¨uller, Alexey Dosovitskiy, Bernard Ghanem,
and Vladlen Koltun. Driving policy transfer via modu-
larity and abstraction. In Conference on Robot Learning,
2018.

[45] Raul Mur-Artal, Jose Maria Martinez Montiel, and
Juan D Tardos. ORB-SLAM: a versatile and accu-
IEEE transactions on
rate monocular SLAM system.
robotics, 31(5):1147–1163, 2015.

[46] Davy Neven, Bert De Brabandere, Stamatios Georgoulis,
Marc Proesmans, and Luc Van Gool. Towards end-to-
end lane detection: an instance segmentation approach.
In 2018 IEEE intelligent vehicles symposium (IV), pages
286–291. IEEE, 2018.

[47] Yunpeng Pan, Ching-An Cheng, Kamil Saigol, Keuntaek
Lee, Xinyan Yan, Evangelos Theodorou, and Byron
Boots. Agile autonomous driving using end-to-end deep
imitation learning. In Proceedings of Robotics: Science
and Systems, Pittsburgh, Pennsylvania, June 2018. doi:
10.15607/RSS.2018.XIV.056.

[48] Taih´u Pire, Mart´ın Mujica, Javier Civera, and Ernesto
Kofman.
The Rosario dataset: Multisensor data for
localization and mapping in agricultural environments.
The International Journal of Robotics Research, 38(6):
633–641, 2019. doi: 10.1177/0278364919841437.
[49] William Qi, Ravi Teja Mullapudi, Saurabh Gupta, and
Deva Ramanan. Learning to Move with Affordance
In International Conference on Learning Rep-
Maps.
resentations, 2020.

[50] Tong Qin, Peiliang Li, and Shaojie Shen. Vins-mono:
A robust and versatile monocular visual-inertial state

estimator. IEEE Transactions on Robotics, 34(4):1004–
1020, 2018.

[51] Josiah Radcliffe, Julie Cox, and Duke M Bulanon.
Machine vision for orchard navigation. Computers in
Industry, 98:165–171, 2018.

[52] John F Reid, Qin Zhang, Noboru Noguchi, and Monte
Dickson. Agricultural automatic guidance research in
North America. Computers and electronics in agricul-
ture, 25(1-2):155–167, 2000.

[53] Stephane Ross, Narek Melik-Barkhudarov, Kumar Shau-
rya Shankar, Andreas Wendel, Debadeepta Dey, J. An-
drew (Drew) Bagnell, and Martial Hebert. Learning
Monocular Reactive UAV Control in Cluttered Natural
In Proceedings of (ICRA) International
Environments.
Conference on Robotics and Automation, pages 1765 –
1772. IEEE, May 2013.

[54] Fereshteh Sadeghi and Sergey Levine. CAD2RL: Real
single-image ﬂight without a single real image. In RSS,
2017.

[55] Redmond Ramin

Shamshiri, Cornelia Weltzien,
Ibrahim A Hameed, Ian J Yule, Tony E Grift, Siva K
Balasundram, Lenka Pitonakova, Desa Ahmad, and
in
Girish Chowdhary.
agricultural robotics: A perspective of digital farming.
International Journal of Agricultural and Biological
Engineering, 11(4):1–14, 2018.

Research and development

[56] Roland Siegwart, Illah Reza Nourbakhsh, and Davide
Scaramuzza. Introduction to autonomous mobile robots.
MIT press, 2011.

[57] Bryce Simmons, Pasham Adwani, Huong Pham, Yazeed
Alhuthaiﬁ, and Artur Wolek. Training a remote-control
car to autonomously lane-follow using end-to-end neural
In 2019 53rd Annual Conference on Infor-
networks.
mation Sciences and Systems (CISS), pages 1–6. IEEE,
2019.

[58] Adam Stager, Herbert G. Tanner, and Erin E. Sparks.
Design and Construction of Unmanned Ground Vehicles
for Sub-Canopy Plant Phenotyping, 2019.

[59] Vijay Subramanian, Thomas F Burks, and AA Arroyo.
Development of machine vision and laser radar based
autonomous vehicle guidance systems for citrus grove
navigation. Computers and electronics in agriculture, 53
(2):130–143, 2006.

[60] Sebastian Thrun, Wolfram Burgard, and Dieter Fox.

Probabilistic robotics. MIT press, 2005.

[61] Naveen Kumar Uppalapati, Benjamin Walt, Aaron
Havens, Armeen Mahdian, Girish Chowdhary, and Girish
Krishnan. A Berry Picking Robot With A Hybrid Soft-
Rigid Arm: Design and Task Space Control. In Proceed-
ings of Robotics: Science and Systems, Corvalis, Oregon,
USA, July 2020. doi: 10.15607/RSS.2020.XVI.027.
[62] AEB Velasquez, VAH Higuti, HB Guerrero, MV Gas-
parino, DV Magalh˜aes, RV Aroca, and M Becker. Reac-
tive navigation system based on H∞ control system and
LiDAR readings on corn crops. Precision Agriculture,
21(2):349–368, 2020.

[63] Andres

Eduardo

Velasquez,

Vitor
Baquero
Akihiro Hisano Higuti, Mateus Valverde Gasparino,
Arun Narenthiran Sivakumar, Marcelo Becker, and
Girish Chowdhary. Multi-Sensor Fusion based Robust
Row Following for Compact Agricultural Robots.
In
arXiv, 2021.

[64] Stavros G Vougioukas. Agricultural robotics. Annual
Review of Control, Robotics, and Autonomous Systems,
2:365–392, 2019.

[65] Erik Wijmans, Abhishek Kadian, Ari Morcos, Stefan
Lee, Irfan Essa, Devi Parikh, Manolis Savva, and Dhruv
Batra. Dd-ppo: Learning near-perfect pointgoal naviga-
tors from 2.5 billion frames. arXiv, pages arXiv–1911,
2019.

[66] Rui Xu, Changying Li, and Javad Mohammadpour Velni.
Development of an Autonomous Ground Robot for Field
High Throughput Phenotyping. IFAC-PapersOnLine, 51
(17):70–74, 2018.

[67] Jinlin Xue, Lei Zhang, and Tony E Grift. Variable ﬁeld-
of-view machine vision based row guidance of an agri-
cultural robot. Computers and Electronics in Agriculture,
84:85–91, 2012.

[68] Sierra N Young, Erkan Kayacan, and Joshua M Peschel.
Design and ﬁeld evaluation of a ground robot for high-
throughput phenotyping of energy sorghum. Precision
Agriculture, 20(4):697–722, 2019.

[69] Zhiqiang Zhai, Zhongxiang Zhu, Yuefeng Du, Zhenghe
Song, and Enrong Mao. Multi-crop-row detection algo-
rithm based on binocular vision. Biosystems engineering,
150:89–103, 2016.

[70] Chi Zhang and Noboru Noguchi. Development of a
tractor system for agriculture ﬁeld work.
multi-robot
Computers and Electronics in Agriculture, 142:79–90,
2017.

[71] Ji Zhang, George Kantor, Marcel Bergerman, and Sanjiv
Singh. Monocular visual navigation of an autonomous
vehicle in natural scene corridor-like environments.
In
2012 IEEE/RSJ International Conference on Intelligent
Robots and Systems, pages 3659–3666. IEEE, 2012.
[72] Qin Zhang, John F Reid, and Noboru Noguchi. Agricul-
tural vehicle navigation using multiple guidance sensors.
In Proceedings of the international conference on ﬁeld
and service robotics, pages 293–298. August, 1999.
[73] Yuke Zhu, Roozbeh Mottaghi, Eric Kolve, Joseph J Lim,
Abhinav Gupta, Li Fei-Fei, and Ali Farhadi. Target-
driven visual navigation in indoor scenes using deep
In 2017 IEEE international
reinforcement
conference on robotics and automation (ICRA), pages
3357–3364. IEEE, 2017.

learning.

[74] Qin Zou, Hanwen Jiang, Qiyu Dai, Yuanhao Yue, Long
Chen, and Qian Wang. Robust
lane detection from
continuous driving scenes using deep neural networks.
IEEE transactions on vehicular technology, 69(1):41–54,
2019.

Supplementary Material

VIII. VIDEO

Please see the accompanying video that provides an
overview of our paper, and shows video executions of our
robots. Video is encoded via H-264 MPEG4 and was tested to
play well on Windows and MacOS through all regular media
players such as Movies & TV (Windows), QuickTime, VLC,
and Google Chrome.

IX. IMU FUSION WITH EXTENDED KALMAN FILTER

An Extended Kalman Filter was used to reduce the effect
of uncertainties in distance and heading estimations. The state
vector was deﬁned as s = ( dL dR φ ω )T where φ is the robot’s
heading, ω is the angular velocity of the robot and dR and
dL are the distances to right and left rows respectively. The
process was modeled using Eq. 1 where actual state s[k] was
deﬁned as a function of f (·) (shown in Eq. 2), the control
inputs uk and the previous state s[k − 1],

s[k] = f (s[k − 1], u[k]) + ωk
z[k] = s[k] + νk

(1)

here, f (s[k − 1], u[k]) is deﬁned as,







=

dL[k]
dR[k]
φ[k]
ω[k]







=







dL[k − 1] − v sin(φ[k − 1])∆t
dR[k − 1] + v sin(φ[k − 1])∆t
φ[k − 1] + ω∆t
ω[k]







.

(2)

Both process noise ωk and measurement noise νk were
deﬁned as zero mean Gaussian noises and their covariances
are [ 0.001 0.001 0.01 0.01 ] and [ 0.05 0.05 0.05 0.5 ] respectively,
corresponding to the states and measurements dL, dR, φ, and
ω. Those values are in the covariance matrices Q (for ωk) and
R (for νk).

The robot’s linear v and angular ω velocities are used to
estimate the states (Eq. 2) in the prediction step. v is calculated
from encoders and ω is obtained from IMU. In the update step,
innovation occurs by considering the calculated values of dL,
dR and φ from 2 CNN networks. As the output of the distance
CNN network is a distance ratio, it is necessary to convert it
to a metric value by multiplying it with average lane width.

X. MODEL PREDICTIVE CONTROL

The kinematic differential model is formulated for a skid-

steering mobile robot as presented in Eq. 3.

˙x = v cos(φ)

˙y = v sin(φ)
˙φ = ω

(3)

The robot’s states (x, y, φ) denote its bi-dimensional posi-
tion and yaw angle, while inputs v and ω denote its linear

and angular speeds. Then, it’s possible to transform the dif-
ferential model to a discrete model and solve it using Euler’s
integration, as given by Eq. 4.

x[k] = x[k − 1] + v cos(φ) ∆t

y[k] = y[k − 1] + v sin(φ) ∆t

(4)

φ[k] = φ[k − 1] + ω ∆t

A transformation of v ∆t = ∆s is adopted, and therefore
ω ∆t = ρ ∆s, where ρ is the robot’s instantaneous curvature.
The new non-linear system is then given by Eq. 5.

x[k] = x[k − 1] + cos(φ) ∆s

y[k] = y[k − 1] + sin(φ) ∆s

(5)

φ[k] = φ[k − 1] + ρ ∆s

The non-linear model is used as dynamic model of the
process to predict future states, and a cost function over the
receding horizon is the optimization cost function using as
control input the curvature ρ.

(cid:40) N
(cid:88)

wde,i d2

e,i +

min
ρi

N
(cid:88)

i=1

wφi φ2

error,i+

(cid:41)

(6)

w∆ρi(ρi − ρi−1)2

i=1
N −2
(cid:88)

i=1

Where de,i is the cross-track error, φerror,i is the heading
error, ρi is the curvature command, wde,i
is the weighting
coefﬁcient reﬂecting the relative importance of de,i, wφi
is
the weighting coefﬁcient reﬂecting the relative importance of
ρi, and w∆ρi is the weighting coefﬁcient penalizing relative
big changes in curvature.

The variables used in the Eq. 6 are calculated as shown
in Eq. 7, which shows the calculation of the cross-track
error using geometry and the heading error as the difference
between current robot’s heading (equal to zero in the local
robot’s frame) and the desired path’s heading. The optimiza-
tion problem is subject to a single constraint inequality acting
in the curvature command, which must lay in −1/Rmax ≤
ρi ≤ 1/Rmax. The Rmax is the maximum permissible curve
radius the robot can follow, which is a tunable parameter for
compromise between aggressiveness and to avoid robot to get
stuck on difﬁcult terrains.

φwpi = arctan 2(wpyi − wpyi−1, wpxi − wpxi−1)

(cid:113)

RU =

(wpxi−1 − xi)2 + (wpyi−1 − yi)2
φU = arctan 2(yi − wpyi−1 , xi − wpxi−1)
de,i = RU sin(φwp − φU )

(7)

φerror,i = arctan 2(wpyi − wpyi−1, wpxi − wpxi−1 )
In Equation 7, wpyi and wpxi are the coordinates of the ith
waypoint used as input in the MPC horizon. These waypoints

w∆ρi = 1000 i = 1, 2, . . . , 20

=

f

are generated as a straight line that represents the middle of the
crop row, whre this line is calculated from the distance ratio
d, lane width W , and the estimated heading φ estimated from
the vision algorithm and EKF. For each iteration, the cross-
track error de,i and the heading error φerror,i are calculated
as shown in Equation 7, and they are used as functions for the
minimization in Equation 6.

During the experiments, the parameters were used as de-

scribed in Equation 8.

Rmin = 0.7
∆s = 0.2

wde,i =

wφi =

(cid:26) 120
1200
(cid:26) 100
1000

i = 1, 2, . . . , 19
i = 20

i = 1, 2, . . . , 19
i = 20

(8)

A PID controller is used as low-level controller to guarantee
the predicted control effort is followed by the robot. The
low-level controller uses the IMU’s yaw angular speed as
feedback to follow the angular speed command that comes
from the MPC controller. As input to the MPC controller, a
waypoints generator algorithm is used. The generated path is
always straight and built in relation to the robot’s local frame.
The distance and angle of the straight line depends on the
measured distance error and heading error from desired path.
Figure 15 shows the overall control diagram, where ωM P C is
the yaw angular speed calculated from MPC algorithm, ωgyro
is the robot’s yaw angular speed measured using the IMU’s
gyroscope, ωerror is the difference between the ωM P C and
ωgyro, and ωcmd is the commanded yaw angular speed that is
sent to the motors.

XI. GROUND TRUTHING

We use projective geometry to obtain the heading φ and
distance ratio d from our obtained annotations. We show the
derivations for the different steps in this section. We assume
a pinhole camera model, and assume camera’s focal length
to be f . We denote world coordinates with a capital letters
(X, Y, Z), and denote their projection in the image as (x, y).
Z and y = f Y
Note under the pin hole camera model, x = f X
Z .
The X-axis goes right from the image center, Y -axis goes
down from the image center, and the Z axis goes into the
scene from the camera center.

As noted our ground truthing process has 4 steps: camera
roll correction (Section XI-B), camera pitch correction (Sec-
tion XI-C), heading estimation (Section XI-D) and distance
ratio estimation (Section XI-E). We do these on top of anno-
tated horizon and crop row vanishing lines. For early season
data, we can directly annotate these. For late season data, the
horizon is not directly visible, and we mark the corn stalks to
estimate the horizon from the vanishing point of the vertical
lines (Section XI-A). Our annotation procedure assumes: a)
ground is ﬂat, b) corn has been planted in parallel rows,

(9)

(10)

(11)

c) corn stalks are vertical. We found these to be reasonable
assumptions for the data that we were working with.

Images are rotated about the camera center by the obtained
roll, pitch and heading angles incrementally between steps,
using homography H = KRK −1, where K is the camera
matrix, and R is the desired rotation about the camera center.

A. Horizon Estimation

Let’s assume the vertical stem lines in the world are in the
direction (DX , DY , DZ). Vanishing point of these lines is can
be obtained by considering a line in this direction through a
point (AX , AY , AZ), projecting points on this line onto the
image plane, and then taking the limit as the points tend to
inﬁnity,

lim
λ→∞
(cid:18)

(cid:18)

fX

DX
DZ

, f

AX + λDX
AZ + λDZ
(cid:19)
DY
DZ

= (vx, vy)

, fY

AY + λDY
AZ + λDZ

(cid:19)

where (VX , VY ) is the vanishing point.
Horizontal plane is given by the following equation,

DX

f X
Z

x + DY

DX X + DY Y + DZZ = c
f c
Z
f c
Z

+ f DZ =

DX · x + DY · y + f DZ =

f Y
Z

The horizon occurs when we go to ∞ on this plane, i.e.
limZ→∞,

DX
DZ

DX · x + DY · y + f DZ = 0
DY
DZ
and DY
DZ

· y + f = 0

· x +

We can substitute for DX
DZ
equation of the horizon as,

from Eq. 9, to obtain the

vx · x + vy · y + f 2 = 0

(12)

Vanishing points for the vertical lines can be found by
ﬁnding the point of intersection of the lines using least squares.

B. Roll Estimation

Suppose the camera is pitched down by pitch θ and has roll
α, then the surface normal of the ground plane is given as
(sin α cos θ, cos α cos θ, − sin θ). The equation of the ground
plane is given by:

X cos θ sin α + Y cos α cos θ − Z sin θ = c

⇒

f X
Z

cos θ sin α +

f Y
Z

cos α cos θ − f sin θ =

⇒ x cos θ sin α + y cos α cos θ − f sin θ =

(13)

f c
Z
f c
Z

Z and y for f Y

where in the last step, we have substituted image coordinates,
x for f X
Z , using the pinhole camera model. We
get the equation of the horizon in the image plane, by seeing
what happens as Z → ∞:

x cos θ sin α + y cos α cos θ − f sin θ = 0

(14)

Fig. 15: Overall control diagram.

We can compute the camera roll α from the slope of the

horizon h(cid:48) in the image,

h(cid:48) = −

cos α cos θ
sin α cos θ

⇒ α = − arctan(1/h(cid:48))

= −1/ tan α

(15)

C. Pitch Estimation

Assuming the image and all annotation lines have been
corrected for roll, we notice that the ground plane will have
a normal vector (0, cos θ, − sin θ). Following a similar proce-
dure as in Section XI-B, we obtain the equation of horizon as:

0 · X + Y cos θ − Z sin θ = c
f X
Z

cos θ − f sin θ =

f Y
Z

+

⇒ 0 ·

⇒ 0 · x + y cos θ − f sin θ =

As we tend Z → ∞,

0 · x + y cos θ − f sin θ = 0

⇒ y = f tan θ

cf
Z
cf
Z

(16)

(17)

Thus, θ can be found using the y-intercept of the horizon in the
image, as arctan(yhorizon/f ), where yhorizon is the y-coordinate
of the horizontal horizon line.

D. Heading Estimation

Now that the image and annotation lines have been corrected
for pitch and roll, heading φ can be obtained from the
vanishing point of the crop row lines. The left crop row is
in the direction of (sin φ, 0, cos φ), and let us assume that it
passes through the point (Al
X , Al
Z). A point on the the
left crop row is given by:

Y , Al

(Al

X , Al

Y , Al

Z) + λ(sin φ, 0, cos φ)

(18)

for different values of λ. These points project to the image
place at locations,

(cid:18)
f

Al
Al

X + λ sin φ
Z + λ cos φ

, f

0
Z + λ cos φ

Al

(cid:19)

Vanishing point can be obtained by taking limλ→∞,

(cid:18)

f

lim
λ→∞

Al
Al

X + λ sin φ
Z + λ cos φ

, f

0
Z + λ cos φ

Al

(cid:19)

= (f tan φ, 0)

(19)

(20)

Fig. 16: Shows (vx, vy) once the image and annotation lines have
been rectiﬁed for roll and pitch.

Thus, the heading φ can be obtained from the x-coordinate
of the vanishing point, vx as, arctan(vx/f ). vx can be
obtained from the intersection of the image of the left and
the right crop row.

E. Distance Ratio Estimation

Assume that we have rotated the image to correct for
the heading. Crop rows are now in the direction (0, 0, 1).
Let’s assume that the left crop row goes through the point
(Xl, H, 0), and the right crop row passes through the point
(Xr, H, 0). Then the image of the left crop rows in the image
plane is given by:

(cid:18)
f

Xl + λl · 0
0 + λl · 1

, f

H + λl · 0
0 + λl · 1

(cid:19)

(cid:18)
f

=

(cid:19)

Xl
λl

, f

H
λl

(21)

The X-intercept of this line in the image plane is given by
the value of λl, such that f H
is equal to h/2 (where h is the
λl
image height), i.e., λl = f H
h/2 . Thus, the X-intercept of the
left crop row, lx is Xl·h
2·H .

Similarly, the X-intercept for the right crop row, rx is Xr·h
2·H .
can be obtained via

Thus, the distance ratio, d = Xl

Xl+Xr

lx
lx+rx

.

(𝑣𝑥,𝑣𝑦)Left crop lineRight crop lineHorizon line𝑌𝑋