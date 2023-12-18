from typing import Any, Dict, List

from avstack.config import ALGORITHMS, HOOKS, PIPELINE, ConfigDict
from avstack.geometry import ReferenceFrame


@PIPELINE.register_module()
class PassiveAgentPipeline:
    """Fusion pipeline for an agent"""

    def __init__(
        self,
        perception: List[ConfigDict],
        tracking: List[ConfigDict],
        hooks_pre_percep: List[ConfigDict]=[],
        hooks_post_percep: List[ConfigDict]=[],
        hooks_pre_track: List[ConfigDict]=[],
        hooks_post_track: List[ConfigDict]=[],
    ) -> None:
        self.perception = {percep.ID: ALGORITHMS.build(percep) for percep in perception}
        self.tracking = {tracker.ID: ALGORITHMS.build(tracker) for tracker in tracking}

        self.hooks = {
            "pre_perception": [HOOKS.build(hook) for hook in hooks_pre_percep],
            "post_perception": [HOOKS.build(hook) for hook in hooks_post_percep],
            "pre_tracking": [HOOKS.build(hook) for hook in hooks_pre_track],
            "post_tracking": [HOOKS.build(hook) for hook in hooks_post_track],
        }

    def __call__(
        self, sensing, platform, frame, timestamp, *args: Any, **kwds: Any
    ) -> Dict:
        # -- perception
        p_out = {
            k: v(*[sensing[ks] for ks in v.ID_input])
            for k, v in self.perception.items()
        }

        # -- tracking
        t_out = {
            k: v(
                frame=frame,  # TODO: make this the perception data output time and frame
                t=timestamp,
                detections=[p_out[kp] for kp in v.ID_input][
                    0
                ],  # HACK for only 1 percep input...
                platform=platform,
            )
            for k, v in self.tracking.items()
        }

        return t_out


@PIPELINE.register_module()
class CommandCenterPipeline:
    """Fusion pipeline for the command center"""

    def __init__(
        self,
        clustering: ConfigDict,
        group_tracking: ConfigDict,  # trust: ConfigDict
    ) -> None:
        self.clustering = ALGORITHMS.build(clustering)
        self.group_tracking = ALGORITHMS.build(group_tracking)
        # self.trust = PIPELINE.build(trust)

    def __call__(
        self,
        tracks_in: dict,
        platform: ReferenceFrame,
        frame: int,
        timestamp: float,
        *args: Any,
        **kwds: Any
    ) -> list:
        clusters = self.clustering(objects=tracks_in, frame=frame, timestamp=timestamp)
        group_tracks = self.group_tracking(
            clusters=clusters, platform=platform, frame=frame, timestamp=timestamp
        )
        group_tracks = [track for track in group_tracks if len(track.members) > 0]
        # cluster_trusts, agent_trusts = self.trust(
        #     group_tracks=group_tracks, agents=agents, timestamp=timestamp
        # )
        return group_tracks  # , cluster_trusts, agent_trusts
