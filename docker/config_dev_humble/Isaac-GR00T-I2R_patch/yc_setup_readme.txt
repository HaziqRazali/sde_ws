This instruction assumes you are using Docker container made by I2R: u22_gpu_humble_inference_sde:latest

1. Ensure env variables are set by checking: 
echo $FLASH_ATTN_CUDA_ARCHS
echo $MAX_JOBS  # PLEASE SET THIS CAREFULLY!
echo $FLASH_ATTN_CUDA_ARCHS

2. Run the following: 
uv pip install -e . --system

In some cases when facing the error:
Using Python 3.10.12 environment at: /usr
  × No solution found when resolving dependencies:
  ╰─▶ Because datasets==3.6.0 depends on requests>=2.32.2 and only requests==2.28.1 is available, we can conclude that datasets==3.6.0 depends on requests>=3.
      And because wandb==0.23.0 depends on requests==2.28.1, we can conclude that datasets==3.6.0 and wandb==0.23.0 are incompatible.
      And because gr00t==0.1.0 depends on datasets==3.6.0 and wandb==0.23.0, we can conclude that gr00t==0.1.0 cannot be used.
      And because only gr00t==0.1.0 is available and you require gr00t, we can conclude that your requirements are unsatisfiable.

      hint: `requests` was found on https://download.pytorch.org/whl/cu128, but not at the requested version (requests>=2.32.2,<3). A compatible version may be
      available on a subsequent index (e.g., https://pypi.nvidia.com/). By default, uv will only consider versions that are published on the first index that
      contains a given package, to avoid dependency confusion attacks. If all indexes are equally trusted, use `--index-strategy unsafe-best-match` to consider
      all versions from all indexes, regardless of the order in which they were defined.

Do instead:
pip install datasets==3.6.0
pip install wandb==0.23.0
uv pip install -e . --system --index-strategy unsafe-best-match

3. You are done with the setup! You may run the training and inference, etc scripts. 

Things to work on: 
1. How to add embodiment tags for Booster T1 instead of using "NEW_EMBODIMENT"?
2. How to train one model for multiple humanoid robot embodiments using GR00T? 

