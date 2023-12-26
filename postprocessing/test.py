import os


def main():
    path_to_results = "../../../outputs/"
    path_to_agent_detections = os.path.join(path_to_results, "agent_detections")
    path_to_agent_tracks = os.path.join(path_to_results, "agent_tracks")
    assert os.path.exists(path_to_agent_detections)
    assert os.path.exists(path_to_agent_tracks)
    print("done")


if __name__ == "__main__":
    main()
