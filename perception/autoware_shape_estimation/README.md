# autoware_shape_estimation

## Purpose

This node estimates refined 3D object shapes from point cloud clusters using object labels. It supports both rule-based algorithms (L-shape fitting, cylinder, convex hull with filtering and correction) and ML-based estimation (PointNet) for vehicles, incorporating reference information from prior detections to improve shape accuracy and orientation estimation.

## Inputs / Outputs

### Input

| Name    | Type                                                     | Description                           |
| ------- | -------------------------------------------------------- | ------------------------------------- |
| `input` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | detected objects with labeled cluster |

### Output

| Name             | Type                                             | Description                         |
| ---------------- | ------------------------------------------------ | ----------------------------------- |
| `output/objects` | `autoware_perception_msgs::msg::DetectedObjects` | detected objects with refined shape |

## Parameters

{{ json_to_markdown("perception/autoware_shape_estimation/schema/shape_estimation.schema.json") }}

## Inner-workings / Algorithms

### Rule-based algorithms

This rule-based geometric algorithms applies object-type-specific shape fitting (L-shape for vehicles, cylinder for pedestrians, convex hull for unknown objects), followed by filtering and correction stages that incorporate reference information from prior detections to ensure geometric consistency and improve orientation accuracy.

The shape fitting algorithm pipeline consists of following three stages.

1. Shape Estimation
   - Vehicle Objects (CAR, TRUCK, BUS, TRAILER, MOTORCYCLE, BICYCLE):
     - **L-shape Fitting Algorithm (`fitLShape` function)**:
       - Implements search-based rectangle fitting from IV2017 paper by Zhang et al.

       - **Angle Optimization**:
         - Default search range: 0 to 90 degrees for full angular sweep
         - Reference yaw constraint: +/-search_angle_range around reference when available
         - Two optimization methods: Standard iterative search or Boost-based Brent optimization

       - **Closeness Criterion**: Evaluates fitting quality using Algorithm 4 from referenced paper
         - Distance thresholds: d_min (0.01m squared), d_max (0.16m squared)
         - Point-to-boundary distance calculation for quality assessment

       - **3D Bounding Box Construction**:
         - Projects points onto orthogonal axes e1 and e2
         - Calculates intersection points to determine center and dimensions
         - Height derived from point cloud Z-range with minimum epsilon (0.001m)

       - **Output Validation**: Ensures minimum dimensions to prevent degenerate boxes

   - Pedestrian (PEDESTRIAN):
     - Cylinder shape estimation using cv::minEnclosingCircle

   - Other/Unknown Objects:
     - Convex hull shape estimation using cv::convexHull

2. Filtering
   - Vehicle Type-specific Filtering:
     - Car Filter: Vehicle size validity verification
     - Truck Filter: Truck-specific shape constraints
     - Bus Filter: Bus-specific dimension checks
     - Trailer Filter: Trailer shape validation

   - Physical validity checks of estimated shapes

   - Exclusion of invalid estimation results

3. Corrector
   - **Reference Information-based Correction**:
     - Orientation correction using reference yaw information
     - Dimension correction using reference shape size (minimum/fixed value modes)

   - **Shape Correction Algorithm (`correctWithDefaultValue` function)**:
     - **Purpose**: Rule-based bounding box correction using default vehicle dimensions when estimated shapes violate physical constraints

     - **Correction Vector Application**:
       - Computes correction vector based on conditions by correctWithDefaultValue Function
         - ![correctWithDefaultValueFunction](resource/correctWithDefaultValue.svg)

       - Updates shape dimensions: `shape.dimensions += correction_vector * 2.0`
       - Adjusts pose position: `pose.position += rotation_matrix * correction_vector`

     - **Orientation Normalization**: Ensures longest dimension aligns with x-axis (90 degree rotation if needed)

   - **Vehicle Type-specific Correctors**:
     - Vehicle Corrector: General vehicle correction
     - Dedicated correction logic for each vehicle type

   - **Geometric consistency assurance**

4. Fallback Mechanism
   - Automatic fallback to UNKNOWN label with convex hull estimation when any stage fails

### ML Based Shape Implementation

The model takes a point cloud and object label(provided by camera detections/Apollo instance segmentation) as an input and outputs the 3D bounding box of the object.

ML based shape estimation algorithm uses a PointNet model as a backbone to estimate the 3D bounding box of the object. The model is trained on the NuScenes dataset with vehicle labels (Car, Truck, Bus, Trailer).

The implemented model is concatenated with STN (Spatial Transformer Network) to learn the transformation of the input point cloud to the canonical space and PointNet to predict the 3D bounding box of the object.
Bounding box estimation part of _Frustum PointNets for 3D Object Detection from RGB-D Data_ paper used as a reference.

The model predicts the following outputs for each object:

- x,y,z coordinates of the object center
- object heading angle classification result(Uses 12 bins for angle classification - 30 degrees each)
- object heading angle residuals
- object size classification result
- object size residuals

### Training ML Based Shape Estimation Model

To train the model, you need ground truth 3D bounding box annotations. When using the mmdetection3d repository for training a 3D object detection algorithm, these ground truth annotations are
saved and utilized for data augmentation. These annotations are used as an essential dataset for training the shape estimation model effectively.

### Preparing the Dataset

#### Install MMDetection3D prerequisites

**Step 1.** Download and install Miniconda from the [official website](https://mmpretrain.readthedocs.io/en/latest/get_started.html).

**Step 2.** Create a conda virtual environment and activate it

```bash
conda create --name train-shape-estimation python=3.8 -y
conda activate train-shape-estimation
```

**Step 3.** Install PyTorch

```bash
conda install pytorch torchvision -c pytorch
```

#### Install mmdetection3d

**Step 1.** Install MMEngine, MMCV, and MMDetection using MIM

```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**Step 2.** Install Autoware's MMDetection3D fork

```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### Preparing NuScenes dataset for training

**Step 1.** Download the NuScenes dataset from the [official website](https://www.nuscenes.org/download) and extract the dataset to a folder of your choice.

**Note:** The NuScenes dataset is large and requires significant disk space. Ensure you have enough storage available before proceeding.

**Step 2.** Create a symbolic link to the dataset folder

```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**Step 3.** Prepare the NuScenes data by running:

```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes --only-gt-database True
```

#### Clone Bounding Box Estimator model

```bash
git clone https://github.com/autowarefoundation/bbox_estimator.git
```

#### Split the dataset into training and validation sets

```bash

cd bbox_estimator
python3 utils/split_dbinfos.py --dataset_path /path/to/mmdetection3d/data/nuscenes --classes 'car' 'truck' 'bus' 'trailer'  --train_ratio 0.8
```

### Training and Deploying the model

#### Training the model

```bash
# Detailed training options can be found in the training script
# For more details, run `python3 train.py --help`
python3 train.py --dataset_path /path/to/mmdetection3d/data/nuscenes
```

#### Deploying the model

```bash
# Convert the trained model to ONNX format
python3 onnx_converter.py --weight_path /path/to/best_checkpoint.pth --output_path /path/to/output.onnx
```

Give the output path of the ONNX model to the `model_path` parameter in the `shape_estimation` node launch file.

## Assumptions / Known limits

TBD

## References/External links

L-shape fitting implementation of the paper:

```bibtex
@conference{Zhang-2017-26536,
author = {Xiao Zhang and Wenda Xu and Chiyu Dong and John M. Dolan},
title = {Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners},
booktitle = {2017 IEEE Intelligent Vehicles Symposium},
year = {2017},
month = {June},
keywords = {autonomous driving, laser scanner, perception, segmentation},
}
```

Frustum PointNets for 3D Object Detection from RGB-D Data:

````bibtex
@inproceedings{qi2018frustum,
title={Frustum pointnets for 3d object detection from rgb-d data},
author={Qi, Charles R and Liu, Wei and Wu, Chenxia and Su, Hao and Guibas, Leonidas J},
booktitle={Proceedings of the IEEE conference on computer vision and pattern recognition},
pages={918--927},
year={2018}
}```
````
