
# steps to follow
docker start u22_humble_sde
docker exec -it u22_humble_sde bash
bash tmux/booster_t1/test_sim_control.bash

# then kill the top right window before running the pose estimator node
ros2 run mhr_pose_estimation pose_estimator \
-w src/motion_retargeting/mhr_pose_estimation/models/all_epoch_0514_best_0514_state_dict.pt \
-c src/motion_retargeting/mhr_pose_estimation/models/all_epoch_0514_best_0514_config.yaml \
--smoother oneeuro \
--oneeuro_min_cutoff 5.0 --oneeuro_beta 1.0 \
--retarget gmr --gmr_robot booster_t1_29dof \
--mesh

# smoother options: none | ema | oneeuro (default)
# ema tuning:     --ema_alpha 0.3
# oneeuro tuning: --oneeuro_min_cutoff 1.0 --oneeuro_beta 0.007 --oneeuro_dcutoff 1.0
#
# 1-Euro filter guide:
#   --oneeuro_min_cutoff  (default 1.0 Hz)
#       Base cutoff frequency — controls smoothing when the signal is slow/still.
#       HIGHER = less smoothing, faster response, more jitter at rest.
#       LOWER  = more smoothing, slower response, less jitter.
#       E.g. 2.0–5.0 for less aggressive smoothing; 0.3–0.5 for heavy smoothing.
#
#   --oneeuro_beta  (default 0.007)
#       Speed coefficient — raises the cutoff as motion speed increases.
#       HIGHER = less lag during fast movements (cutoff rises quickly with speed).
#       LOWER  = lag/smoothing is maintained even during fast movements.
#       E.g. 0.1–1.0 to reduce motion lag; keep near 0.0 for speed-independent smoothing.
#
#   Suggested presets:
#     Responsive (less smooth): --oneeuro_min_cutoff 3.0 --oneeuro_beta 0.5
#     Balanced   (default):     --oneeuro_min_cutoff 1.0 --oneeuro_beta 0.007
#     Smooth     (more lag):    --oneeuro_min_cutoff 0.5 --oneeuro_beta 0.001

# if any updates to the code
colcon build --packages-select mhr_pose_estimation
source /root/test/sde_ws/install/setup.bash

# RViz2 topics published by the node
#   smplx_mesh  (visualization_msgs/Marker) — full shaded mesh (TRIANGLE_LIST)
#
# To visualize in RViz2:
#   1. Add → By topic → /smplx_mesh → Marker
#   2. Set Fixed Frame to "world"

: <<'COMMENT'
─────────────────────────────────────────────────────────────────────────────
Git Workflow: Editing on Computer B (no VPN) via GitHub as middleman

Branches:
  haziq-dev         → GitLab  (has large model files)
  haziq-dev-github  → GitHub  (large files removed from history)

── FIRST TIME on Computer B ─────────────────────────────────────────────────

Computer A (with VPN):
  git checkout haziq-dev-github        # switch to github branch
  git merge haziq-dev                  # bring in latest changes from gitlab branch
  git push github haziq-dev-github --force  # upload to GitHub

Computer B:
  git clone https://github.com/HaziqRazali/sde_ws.git
  cd sde_ws
  git checkout haziq-dev-github        # switch to correct branch
  # ... do edits ...
  git add .
  git commit -m "your changes"
  git push origin haziq-dev-github     # upload edits to GitHub

Computer A (back with VPN):
  git checkout haziq-dev-github        # switch to github branch
  git pull github haziq-dev-github     # download edits from Computer B
  git checkout haziq-dev               # switch to gitlab branch
  git merge haziq-dev-github           # bring in edits
  git push origin haziq-dev            # push to GitLab

── SUBSEQUENT TIMES on Computer B ───────────────────────────────────────────

Computer A (with VPN):
  git checkout haziq-dev-github
  git merge haziq-dev
  git push github haziq-dev-github --force

Computer B:
  cd sde_ws
  git pull origin haziq-dev-github     # no clone needed, just pull
  # ... do edits ...
  git add .
  git commit -m "your changes"
  git push origin haziq-dev-github

Computer A (back with VPN):
  git checkout haziq-dev-github
  git pull github haziq-dev-github
  git checkout haziq-dev
  git merge haziq-dev-github
  git push origin haziq-dev
COMMENT
