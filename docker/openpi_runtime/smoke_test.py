#!/usr/bin/env python3
import argparse

from openpi.policies import aloha_policy
from openpi.policies import droid_policy
from openpi.policies import libero_policy
from openpi.policies import policy_config
from openpi.training import config as training_config


DEFAULT_CHECKPOINTS = {
    "pi0_aloha_sim": "gs://openpi-assets/checkpoints/pi0_aloha_sim",
    "pi05_droid": "gs://openpi-assets/checkpoints/pi05_droid",
    "pi05_libero": "gs://openpi-assets/checkpoints/pi05_libero",
}


def make_example(config_name: str):
    if "aloha" in config_name:
        return aloha_policy.make_aloha_example()
    if "droid" in config_name:
        return droid_policy.make_droid_example()
    if "libero" in config_name:
        return libero_policy.make_libero_example()
    raise ValueError(f"no example generator for config '{config_name}'")


def expected_action_dim(config_name: str) -> int:
    if "aloha" in config_name:
        return 14
    if "droid" in config_name:
        return 8
    if "libero" in config_name:
        return 7
    raise ValueError(f"no action dim mapping for config '{config_name}'")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="pi0_aloha_sim")
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--pytorch-device", default=None)
    args = parser.parse_args()

    cfg = training_config.get_config(args.config)
    checkpoint = args.checkpoint or DEFAULT_CHECKPOINTS.get(args.config, "")
    if not checkpoint:
        raise SystemExit(f"no default checkpoint known for {args.config}")

    policy = policy_config.create_trained_policy(
        cfg,
        checkpoint,
        pytorch_device=args.pytorch_device,
    )
    result = policy.infer(make_example(args.config))
    actions = result["actions"]

    if actions.shape[0] != cfg.model.action_horizon:
        raise SystemExit(
            f"unexpected action horizon: got {actions.shape[0]}, expected {cfg.model.action_horizon}"
        )

    action_dim = expected_action_dim(args.config)
    if actions.shape[1] != action_dim:
        raise SystemExit(f"unexpected action dim: got {actions.shape[1]}, expected {action_dim}")

    print(
        f"OK config={args.config} checkpoint={checkpoint} "
        f"shape={tuple(actions.shape)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
