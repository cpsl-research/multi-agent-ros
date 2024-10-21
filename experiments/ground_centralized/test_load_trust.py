from avstack_rosbag import DatasetPostprocessor


postproc = DatasetPostprocessor(
    bag_folder="/data/shared/CARLA/rosbags/baseline_intersection/",
    bag_name="baseline_intersection_0",
)
print(f"Loaded postprocessor of length {len(postproc)}")

postproc._get_messages_by_time("/trust/trust_agents")
