set -x -e

export NUM_GPUS=4
export CUDA_VISIBLE_DEVICES=0,1,2,3

### examples:

# CUDA_VISIBLE_DEVICES=0 python \
torchrun --nproc_per_node=$NUM_GPUS \
    gr00t/experiment/launch_finetune.py \
    --base_model_path nvidia/GR00T-N1.6-3B \
    --dataset_path  examples/Booster/Jan29 \
    --modality_config_path examples/Booster/booster_config.py \
    --embodiment_tag NEW_EMBODIMENT \
    --num_gpus $NUM_GPUS \
    --output_dir /root/Isaac-GR00T/examples/Booster/models/booster_finetune/Jan29 \
    --save_steps 100 \
    --save_total_limit 5 \
    --max_steps 1000 \
    --warmup_ratio 0.05 \
    --weight_decay 1e-5 \
    --learning_rate 1e-4 \
    --use_wandb \
    --global_batch_size 32 \
    --color_jitter_params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
    --dataloader_num_workers 4


torchrun scripts/deployment/standalone_inference_script.py \
    --model-path nvidia/GR00T-N1.6-3B \
    --dataset-path demo_data/gr1.PickNPlace \
    --embodiment-tag GR1 \
    --traj-ids 0 1 2 \
    --inference-mode pytorch \
    --action-horizon 8

##### NOTE THAT DOING uv run will attempt to create a virtualenv before running,
##### IF USING SYSTEM ENV, use torchrun as above
export NUM_GPUS=1

CUDA_VISIBLE_DEVICES=0 uv run python \
    gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.6-3B \
    --dataset-path <DATASET_PATH> \
    --embodiment-tag NEW_EMBODIMENT \
    --modality-config-path <MODALITY_CONFIG_PATH> \
    --num-gpus $NUM_GPUS \
    --output-dir <OUTPUT_PATH> \
    --save-total-limit 5 \
    --save-steps 2000 \
    --max-steps 2000 \
    --use-wandb \
    --global-batch-size 32 \
    --color-jitter-params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
    --dataloader-num-workers 4


### my attempt:
CUDA_VISIBLE_DEVICES=0 uv run python \
    gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.6-3B \
    --dataset-path examples/Booster/Jan29 \
    --embodiment-tag booster_t1 \
    --modality-config-path examples/Booster/booster_config.py \
    --num-gpus $NUM_GPUS \
    --output-dir /root/Isaac-GR00T/examples/Booster/models/booster_finetune/Jan29 \
    --save-total-limit 5 \
    --save-steps 2000 \
    --max-steps 2000 \
    --warmup_ratio 0.05 \
    --weight_decay 1e-5 \
    --learning_rate 1e-4 \
    --use-wandb \
    --global-batch-size 32 \
    --color-jitter-params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
    --dataloader-num-workers 4
    