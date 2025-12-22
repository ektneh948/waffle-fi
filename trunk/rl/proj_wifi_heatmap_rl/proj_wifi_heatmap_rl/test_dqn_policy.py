#!/usr/bin/env python3
import time
import argparse
import numpy as np
import torch
import torch.nn as nn

# 네 환경 import 경로에 맞게 수정
from proj_wifi_heatmap_rl.env.coverage_ros_env import RosCoverageEnv


# -----------------------------
# Q Network (train과 동일해야 함)
# -----------------------------
class QNet(nn.Module):
    def __init__(self, obs_dim: int, n_actions: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, n_actions),
        )

    def forward(self, x):
        return self.net(x)


@torch.no_grad()
def select_action_greedy(qnet, obs, device):
    x = torch.from_numpy(obs).float().to(device).unsqueeze(0)
    q = qnet(x)
    return int(torch.argmax(q, dim=1).item())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=str, required=True, help="path to trained .pt")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--sleep", type=float, default=0.05, help="step delay (sec)")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"

    # -----------------------------
    # Env 생성
    # -----------------------------
    env = RosCoverageEnv()

    # obs / action dim 추론
    obs = env.reset()
    if isinstance(obs, tuple):
        obs = obs[0]
    obs = np.asarray(obs, dtype=np.float32)

    obs_dim = obs.shape[0]
    n_actions = 4  # 네 action 정의 기준

    # -----------------------------
    # Model 로드
    # -----------------------------
    qnet = QNet(obs_dim, n_actions).to(device)

    ckpt = torch.load(args.ckpt, map_location=device)
    qnet.load_state_dict(ckpt["qnet"])
    qnet.eval()

    print(f"[LOAD] {args.ckpt}")
    print(f"[INFO] obs_dim={obs_dim}, n_actions={n_actions}, device={device}")

    # -----------------------------
    # Run episodes
    # -----------------------------
    for ep in range(1, args.episodes + 1):
        obs = env.reset()
        if isinstance(obs, tuple):
            obs = obs[0]
        obs = np.asarray(obs, dtype=np.float32)

        ep_reward = 0.0
        step = 0

        while True:
            action = select_action_greedy(qnet, obs, device)

            out = env.step(action)
            if len(out) == 5:
                next_obs, reward, terminated, truncated, info = out
                done = terminated or truncated
            else:
                next_obs, reward, done, info = out

            obs = np.asarray(next_obs, dtype=np.float32)
            ep_reward += float(reward)
            step += 1

            if args.sleep > 0:
                time.sleep(args.sleep)

            if done:
                break

        print(f"[EP {ep}] steps={step} reward={ep_reward:.2f}")

    env.close()
    print("[DONE]")


if __name__ == "__main__":
    main()
