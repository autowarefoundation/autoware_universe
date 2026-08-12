# Overall CI Infrastructure

## Machine Types

### Standard GitHub-hosted runners

- [Documentation](https://docs.github.com/en/actions/using-github-hosted-runners/using-github-hosted-runners/about-github-hosted-runners#standard-github-hosted-runners-for-public-repositories)

These runners are utilized by the majority of the workflows.
They are free to use for public repositories, with a concurrency limit of 20 jobs per organization.

**Listed specs:**

| vCPU | RAM   | Storage (SSD) |
| ---- | ----- | ------------- |
| 4    | 16 GB | 14 GB         |

**Note:** While the official documentation lists 14 GB of storage, the actual available storage is approximately 73 GB.

### Self-hosted runners

Registered with the labels `[self-hosted, Linux, X64]` and used by the heavier CUDA/merge jobs.

| Machine | CPU                                       | RAM   | Storage    |
| ------- | ----------------------------------------- | ----- | ---------- |
| SYS-3   | Intel Xeon-E 2288G (8 cores / 16 threads) | 32 GB | 855 GB SSD |

## Key workflows and their runners

| Workflow                                   | Trigger        | Runner                                        |
| ------------------------------------------ | -------------- | --------------------------------------------- |
| build-and-test                             | push to `main` | self-hosted                                   |
| build-and-test-daily                       | daily / manual | self-hosted (amd64), ubuntu-22.04-arm (arm64) |
| build-and-test-packages-above-differential | PR update      | self-hosted                                   |
| build-test-tidy-pr                         | PR update      | github-std (non-cuda), self-hosted (cuda)     |

## Additional notes

- We use [`taskset`](https://manpages.ubuntu.com/manpages/jammy/man1/taskset.1.html) from GNU Coreutils to limit the number of cores utilized by build processes. This is done to prevent overloading the self-hosted runners.
  - The number of cores is limited to `vCPU count - 1`.
