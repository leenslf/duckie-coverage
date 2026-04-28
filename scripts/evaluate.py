import sys
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from evo.tools import file_interface, plot
from evo.core import sync, metrics, trajectory
from evo.core.metrics import PoseRelation, Unit

if len(sys.argv) < 2:
    print("Usage: evaluate.py <bag_path>")
    sys.exit(1)

bag_path = sys.argv[1]
bag_name = os.path.basename(os.path.normpath(bag_path))
out_dir = os.path.join("experiments", bag_name)
os.makedirs(out_dir, exist_ok=True)

traj_est = file_interface.read_bag_trajectory(bag_path, "/odom")
traj_ref = file_interface.read_bag_trajectory(bag_path, "/ground_truth/odometry")

traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.05)

traj_est_aligned = trajectory.align_trajectory(traj_est, traj_ref, correct_scale=False)

ate_metric = metrics.APE(PoseRelation.translation_part)
ate_metric.process_data((traj_ref, traj_est_aligned))
ate_stats = ate_metric.get_all_statistics()

rpe_metric = metrics.RPE(
    PoseRelation.translation_part,
    delta=1.0,
    delta_unit=Unit.seconds,
    rel_delta_tol=0.1,
)
rpe_metric.process_data((traj_ref, traj_est_aligned))
rpe_stats = rpe_metric.get_all_statistics()

summary = (
    f"=== VIO Evaluation: {bag_path} ===\n"
    f"ATE  RMSE : {ate_stats['rmse']:.3f} m\n"
    f"ATE  mean : {ate_stats['mean']:.3f} m\n"
    f"ATE  max  : {ate_stats['max']:.3f} m\n"
    f"RPE  RMSE : {rpe_stats['rmse']:.3f} m\n"
    f"RPE  mean : {rpe_stats['mean']:.3f} m\n"
    f"RPE  max  : {rpe_stats['max']:.3f} m"
)
print(summary)

summary_path = os.path.join(out_dir, "summary.txt")
with open(summary_path, "w") as f:
    f.write(summary + "\n")

fig, ax = plt.subplots()
plot_collection = plot.PlotCollection("ATE")
plot.trajectories(fig, {
    "reference": traj_ref,
    "estimated (aligned)": traj_est_aligned,
}, plot.PlotMode.xy)
plot.error_array(fig.axes[0], ate_metric.error, name="ATE")
fig.savefig(os.path.join(out_dir, "ate_plot.png"), dpi=150)
plt.close(fig)

fig2, ax2 = plt.subplots()
plot.error_array(ax2, rpe_metric.error, name="RPE")
fig2.savefig(os.path.join(out_dir, "rpe_plot.png"), dpi=150)
plt.close(fig2)
