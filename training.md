# PointPillars: Model Training and Testing

For training, we use `OpenPCDet`, which defines pytorch model. Trained pytorch model can be converted to ONNX and further TRT models. These models can be run real-time.

### System requirements
If you are planning to train a model yourself, make sure you have access to good enough system as with too low specs training simply is not possible. You need:
- ~450GB storage
- 12GB RAM
- GPU

For what? GPU makes training and inference bearable, training with CPU takes eternity. NuScenes dataset
takes roughly ~430GB space and you also need space for dependencies,
docker images and training checkpoints. Training data generation
requires quite a lot of RAM and this process gets killed
by system if you don't provide enough RAM memory.


## Dataset generation

Download the nuScenes [dataset](https://www.nuscenes.org/) and mount the directory
including data to docker by adjusting makefile instruction.

We follow the OpenPCDet [guide](https://github.com/open-mmlab/OpenPCDet/blob/master/docs/GETTING_STARTED.md). It commands you to run:
```
python3 -m pcdet.datasets.nuscenes.nuscenes_dataset --func create_nuscenes_infos --cfg_file tools/cfgs/dataset_configs/nuscenes_dataset.yaml --version v1.0-trainval
```

Training data generation may take several hours. If generation succeed, then following files should have appeared to your data directory:
```
nuscenes_dbinfos_10sweeps_withvelo.pkl
nuscenes_infos_10sweeps_train.pkl
nuscenes_infos_10sweeps_val.pkl
```

## Training

(You may need to create a symlink: `ln -s /app/OpenPCDet/data/ /app/data`)  

Train:
```
python3 tools/train.py --cfg_file /app/config/cbgs_pp_multihead.yaml
```
Somethig like this should appear to terminal:
```
2021-10-30 20:04:17,060   INFO  CUDA_VISIBLE_DEVICES=ALL                                                                        [553/1816]2021-10-30 20:04:17,060   INFO  cfg_file         /app/config/cbgs_pp_multihead.yaml                                                       2021-10-30 20:04:17,060   INFO  batch_size       4                                                                                        2021-10-30 20:04:17,060   INFO  epochs           20                                                                                       2021-10-30 20:04:17,060   INFO  workers          8                                                                                        
...
cfg.CLASS_NAMES: ['car', 'truck', 'construction_vehicle', 'bus', 'trailer', 'barrier', 'motorcycle', 'bicycle', 'pedestrian', 'traffic_cone']
...
```
and then model starts training.

## Testing

When model has finished first training iteration, a checkpoint file appears to `ckpt` directory. You can evaluate model performance by running tests with the checkpoint:

```
python3 tools/test.py --cfg_file /app/config/cbgs_pp_multihead.yaml --ckpt output/app/config/cbgs_pp_multihead/default/ckpt/checkpoint_epoch_1.pth
```

Testing takes some time, but eventually you should obtain something similar to this:
```
2021-11-11 21:35:42,039   INFO  *************** EPOCH 3 EVALUATION *****************
2021-11-11 21:52:49,977   INFO  *************** Performance of EPOCH 3 *****************
2021-11-11 21:52:49,992   INFO  Generate label finished(sec_per_example: 0.1708 second).
2021-11-11 21:52:49,992   INFO  recall_roi_0.3: 0.000000
2021-11-11 21:52:49,992   INFO  recall_rcnn_0.3: 0.612078
2021-11-11 21:52:49,992   INFO  recall_roi_0.5: 0.000000
2021-11-11 21:52:49,992   INFO  recall_rcnn_0.5: 0.387628
2021-11-11 21:52:49,992   INFO  recall_roi_0.7: 0.000000
2021-11-11 21:52:49,992   INFO  recall_rcnn_0.7: 0.130422
2021-11-11 21:52:50,043   INFO  Average predicted number of objects(6019 samples): 112.272
2021-11-11 21:57:45,559   INFO  The predictions of NuScenes have been saved to /app/OpenPCDet/output/app/config/cbgs_pp_multihead/default/eval/epoch_3/val/default/final_result/data/results_nusc.json

...

--------------average performance-------------
trans_err:	 0.4086
scale_err:	 0.2790
orient_err:	 1.2551
vel_err:	 0.4639
attr_err:	 0.2133
mAP:	 0.3035
NDS:	 0.4153

2021-11-11 22:01:43,850   INFO  Result is save to /app/OpenPCDet/output/app/config/cbgs_pp_multihead/default/eval/epoch_3/val/default
2021-11-11 22:01:43,851   INFO  ****************Evaluation done.*****************
```
