import json
from copy import deepcopy
from datetime import datetime
from typing import Dict, List, Union

import numpy as np
from avstack.config import Config


def parse_mc_parameter(mc_param, rng: np.random.RandomState):
    # parse the before and after of the mcparameter
    if not isinstance(mc_param, str):
        return mc_param
    splits = mc_param.split("__")
    mc_param_data = splits[2]

    # parse the mcparam data
    mc_param_splits = mc_param_data.split("_")
    func = mc_param_splits[0]

    # parse the mc functions
    if func == "RANDINT":
        low = int(mc_param_splits[1])
        high = int(mc_param_splits[2])
        param = rng.randint(low=low, high=high)
    elif func == "RANDFLOAT":
        low = float(mc_param_splits[1])
        high = float(mc_param_splits[2])
        param = (high - low) * rng.rand() + low
    elif func == "RANDCUT":
        thresh = float(mc_param_splits[1])
        param = rng.rand() >= thresh
    elif func == "DATETIME":
        param = datetime.now().strftime("%Y-%m%d_%H:%M:%S:%f")
    else:
        breakpoint()
        raise NotImplementedError(f"MC function {func} not implemented!")

    # rejoin the param
    if splits[0]:
        param = splits[0] + str(param)
    if splits[3]:
        param = str(param) + splits[3]
    return param


def parse_mc_cfgs(cfg: Union[Config, List], rng: np.random.RandomState):
    """Parse all the possible MC configurations"""
    # every time we see an MC parameter, we substitute
    if isinstance(cfg, dict):
        list_keys = list(cfg.keys())
        for key in list_keys:
            # if key == "enabled":
            #     breakpoint()

            # substitute on the key
            if (isinstance(key, str)) and ("__MCPARAMETER__" in key):
                old_key = key
                key = parse_mc_parameter(key, rng=rng)
                cfg[key] = cfg.pop(old_key)
            elif isinstance(key, tuple):
                old_key = key
                key = tuple(
                    parse_mc_parameter(key_item, rng=rng)
                    if "__MCPARAMETER__" in key_item
                    else key_item
                    for key_item in key
                )
                cfg[key] = cfg.pop(old_key)

            # substitute on the value if not a dictionary
            if (isinstance(cfg[key], str)) and ("__MCPARAMETER__" in cfg[key]):
                cfg[key] = parse_mc_parameter(cfg[key], rng=rng)

            # recurse if the value is a dictionary
            if isinstance(cfg[key], dict):
                cfg[key] = parse_mc_cfgs(cfg[key], rng=rng)
            elif isinstance(cfg[key], list):
                cfg[key] = [
                    parse_mc_cfgs(cfg_item, rng=rng)
                    if isinstance(cfg_item, dict)
                    else parse_mc_parameter(cfg_item, rng=rng)
                    for cfg_item in cfg[key]
                ]
    elif isinstance(cfg, list):
        cfg = [parse_mc_cfgs(cfg_item) for cfg_item in cfg]

    return cfg


def remap_dict_for_print(d: Union[Dict, List]) -> dict:
    """Remap keys that are tuples for printing via json"""

    if isinstance(d, dict):
        keys_list = list(d.keys())
        for k in keys_list:
            # replace the tuple with a string
            if isinstance(k, tuple):
                k_new = "-".join(k)
                d[k_new] = d.pop(k)
            else:
                k_new = k

            # recurse on the entires
            if isinstance(d[k_new], dict):
                d[k_new] = remap_dict_for_print(d[k_new])
            elif isinstance(d[k_new], list):
                d[k_new] = [remap_dict_for_print(d_item) for d_item in d[k_new]]
    elif isinstance(d, list):
        d = [remap_dict_for_print(d_item) for d_item in d]

    return d


def print_cfg(cfg: dict):
    cfg_remap = remap_dict_for_print(deepcopy(cfg))
    print(
        json.dumps(
            cfg_remap,
            sort_keys=False,
            indent=4,
        )
    )
