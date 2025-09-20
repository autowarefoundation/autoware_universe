# Autoware Universe

## Welcome to Autoware Universe

Autoware Universe serves as a foundational pillar within the Autoware ecosystem, playing a critical role in enhancing the core functionalities of autonomous driving technologies.
This repository is a pivotal element of the Autoware Core/Universe concept, managing a wide array of packages that significantly extend the capabilities of autonomous vehicles.

![autoware_universe_front](docs/assets/images/autoware_universe_front.png)

## Getting Started

To dive into the vast world of Autoware and understand how Autoware Universe fits into the bigger picture, we recommend starting with the [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/). This resource provides a thorough overview of the Autoware ecosystem, guiding you through its components, functionalities, and how to get started with development.

### Explore Autoware Universe documentation

For those looking to explore the specifics of Autoware Universe components, the [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/), deployed with MKDocs, offers detailed insights.

## Code Coverage Metrics

Below table shows the coverage rate of entire Autoware Universe and sub-components respectively.

### Entire Project Coverage

[![codecov](https://codecov.io/github/autowarefoundation/autoware_universe/graph/badge.svg?token=KQP68YQ65D)](https://codecov.io/github/autowarefoundation/autoware_universe)

### Component-wise Coverage

You can check more details by clicking the badge and navigating the codecov website.

| Component    | Coverage                                                                                                                                                                                                                                                                                                        |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Common       | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Common%20Packages&query=$.[0].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Common%20Packages)             |
| Control      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Control%20Packages&query=$.[1].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Control%20Packages)           |
| Evaluator    | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Evaluator%20Packages&query=$.[2].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Evaluator%20Packages)       |
| Launch       | TBD                                                                                                                                                                                                                                                                                                             |
| Localization | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Localization%20Packages&query=$.[4].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Localization%20Packages) |
| Map          | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Map%20Packages&query=$.[5].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Map%20Packages)                   |
| Perception   | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Perception%20Packages&query=$.[6].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Perception%20Packages)     |
| Planning     | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Planning%20Packages&query=$.[7].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Planning%20Packages)         |
| Sensing      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Sensing%20Packages&query=$.[8].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Sensing%20Packages)           |
| Simulator    | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Simulator%20Packages&query=$.[9].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Simulator%20Packages)       |
| System       | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=System%20Packages&query=$.[10].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=System%20Packages)            |
| Vehicle      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_universe/components&label=Vehicle%20Packages&query=$.[11].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware_universe?components%5B0%5D=Vehicle%20Packages)          |

<!-- NOTE: `query` fields to shields.io should be converted to slug form   -->
<!--
TODO(soblin):
- dynamic `label` name(maybe not supported by shields.io)
- use https://github.com/marketplace/actions/dynamic-badges
-->
