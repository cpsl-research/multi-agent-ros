import os
import pickle
from glob import glob
from typing import Dict, Tuple

import pandas as pd


def load_metrics_from_subdir(
    metrics_dir: str, hook_file_template: str
) -> Tuple[Dict, Dict]:
    metrics_trials = {}
    hooks_trials = {}

    subdirs = [f.path for f in os.scandir(metrics_dir) if f.is_dir()]
    for trial_dir in subdirs:
        i_trial = int(trial_dir.split("_")[-1])

        # load in hook configs
        with open(os.path.join(trial_dir, hook_file_template), "rb") as f:
            hooks_config = pickle.load(f)
        hooks_trials[i_trial] = hooks_config

        # load in metrics for each frame for this trial
        # reorient so that it is hooks --> frames
        metrics_by_hooks = {i_hook: [] for i_hook in range(len(hooks_config))}
        for file in glob(os.path.join(trial_dir, "metrics*.p")):
            with open(file, "rb") as f:
                metrics_this_frame = pickle.load(f)
                for i_hook in range(len(metrics_this_frame)):
                    metrics_by_hooks[i_hook].append(metrics_this_frame[i_hook])

        # store in the trial slot
        metrics_trials[i_trial] = metrics_by_hooks

    return metrics_trials, hooks_trials


def wrap_metrics_hooks_to_df(
    metrics_trials: Dict, hooks_trials: Dict
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    metrics_dicts = []
    hook_dicts = []

    # wrap the metrics to a dictionary
    for i_trial, m_trial in metrics_trials.items():
        for i_hook, m_hook in m_trial.items():
            # build the config of important parameters for this hook
            this_cfg_est = hooks_trials[i_trial][i_hook]["hook"]["estimation_hook"][
                "model"
            ]
            this_cfg_fus = hooks_trials[i_trial][i_hook]["hook"]["fusion"]
            hook_config = {
                "hook": i_hook,
                "assign_radius": this_cfg_est["measurement"]["assign_radius"],
                "agent_negativity_bias": this_cfg_est["updater"][
                    "agent_negativity_bias"
                ],
                "track_negativity_bias": this_cfg_est["updater"][
                    "track_negativity_bias"
                ],
                "agent_negativity_threshold": this_cfg_est["updater"][
                    "agent_negativity_threshold"
                ],
                "track_negativity_threshold": this_cfg_est["updater"][
                    "track_negativity_threshold"
                ],
                "agent_propagator_prior_alpha": this_cfg_est["updater"][
                    "agent_propagator"
                ]["prior"]["alpha"],
                "agent_propagator_prior_beta": this_cfg_est["updater"][
                    "agent_propagator"
                ]["prior"]["beta"],
                "agent_propagator_prior_dt_return": this_cfg_est["updater"][
                    "agent_propagator"
                ]["dt_return"],
                "track_propagator_prior_alpha": this_cfg_est["updater"][
                    "track_propagator"
                ]["prior"]["alpha"],
                "track_propagator_prior_beta": this_cfg_est["updater"][
                    "track_propagator"
                ]["prior"]["beta"],
                "track_propagator_prior_dt_return": this_cfg_est["updater"][
                    "track_propagator"
                ]["dt_return"],
                "threshold_track_ignore": this_cfg_fus["threshold_track_ignore"],
            }
            hook_dicts.append(hook_config)

            # loop over all frames
            for i_frame, m_frame in enumerate(m_hook):
                if m_frame["trust_agents"] is not None:
                    # build the dictionary of results
                    metrics_frame = {
                        "trial": i_trial,
                        "frame": i_frame,
                        "hook": i_hook,
                        "metric_agent": m_frame["trust_agents"].mean_metric,
                        "metric_track": m_frame["trust_tracks"].mean_metric,
                        "all_precision": m_frame["tracks_all"].precision,
                        "all_recall": m_frame["tracks_all"].recall,
                        "all_f1": m_frame["tracks_all"].f1_score,
                        "all_ospa": m_frame["tracks_all"].ospa,
                        "trusted_precision": m_frame["tracks_trusted"].precision,
                        "trusted_recall": m_frame["tracks_trusted"].recall,
                        "trusted_f1": m_frame["tracks_trusted"].f1_score,
                        "trusted_ospa": m_frame["tracks_trusted"].ospa,
                    }
                    metrics_dicts.append(metrics_frame)

    # convert to dataframe
    hooks_df = pd.DataFrame.from_dict(hook_dicts)
    metrics_df = pd.DataFrame.from_dict(metrics_dicts)

    return metrics_df, hooks_df
