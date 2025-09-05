# autoware_traffic_light_multi_camera_fusion

## Overview
This node fuses traffic light recognition results from multiple cameras to produce a single, robust, and reliable traffic light state for the entire traffic light group. By integrating information from different viewpoints, it improves the system's robustness against occlusions and individual camera recognition errors.

Algorithm Description
The fusion process is performed in two main stages:

### 1. Per-Camera Fusion
For each individual traffic light instance (identified by traffic_light_id), this stage selects the most reliable detection result from all available camera views over a short time window. The selection is based on a priority queue:

1. Results with a known state (RED, GREEN, etc.) are prioritized over UNKNOWN.

2. Results from non-truncated ROIs (i.e., fully visible) are prioritized.

3. Results with the highest detection confidence are prioritized.

This produces a single best-guess recognition for every visible traffic light bulb.

### 2. Group Fusion using Bayesian Updating
This stage fuses the results of individual light bulbs into a single, coherent state for the entire regulatory element (the traffic light group). Instead of simply summing confidence scores, this node employs **Bayesian updating** for a more principled and robust fusion.

- **Belief Representation:** For each color (RED, GREEN, etc.) within a group, a "belief" is maintained. This belief is represented in log-odds (logit) space for numerical stability and ease of updating. An initial state represents a neutral prior belief.

- **Evidence Update:** Each high-priority detection from the Per-Camera Fusion stage is treated as a piece of evidence. The confidence score of the detection is converted into a log-odds value representing the strength of this evidence.

- **Belief Accumulation:** This evidence is then added to the corresponding color's accumulated log-odds. This addition in log-odds space is equivalent to a multiplicative Bayesian update in probability space.

- **Final Decision:** After accumulating evidence from all relevant traffic light bulbs, the color with the highest final log-odds is chosen as the definitive state for the entire group. This represents the color with the highest posterior probability.

This method prevents the overconfidence that can arise from summing many low-confidence detections and provides a more theoretically sound way to integrate multiple pieces of evidence.

## Theoretical Background of Bayesian Updating

This document explains the theory behind **Bayesian updating**, a method used for integrating information from multiple sources. Specifically, it details the mathematical rationale for using **Log-Odds** to perform these updates.

---

## 1. The Foundation: Bayes' Theorem

The foundation for this method is **Bayes' Theorem**. It describes how to update the probability of a hypothesis (H) being true after observing a piece of evidence (E).

$$
P(H|E) = \frac{P(E|H) \cdot P(H)}{P(E)}
$$

-   **`P(H|E)`: Posterior Probability**
    -   The probability of hypothesis H being true **after** observing evidence E. This is the "updated belief" we want to find.
-   **`P(H)`: Prior Probability**
    -   The probability of hypothesis H being believed to be true **before** observing evidence E.
-   **`P(E|H)`: Likelihood**
    -   The probability of observing evidence E, given that hypothesis H is true.
-   **`P(E)`: Evidence Probability**
    -   The probability of observing evidence E, regardless of the hypothesis. This is also known as a normalization constant.

When receiving multiple pieces of evidence sequentially, repeatedly calculating the denominator `P(E)` is complex. The "Odds" formulation solves this problem.

---

## 2. The Rationale for Summing Log-Odds: A Mathematical Derivation

This section proves how Bayesian updating can be simplified to the summation of Log-Odds.

### Step 1: Converting Probability to "Odds"

First, we convert a probability `p` into **Odds**. Odds represent the ratio of the probability of an event occurring to the probability of it not occurring.

$$
\text{Odds}(H) = \frac{P(H)}{1 - P(H)} = \frac{P(H)}{P(\neg H)}
$$

### Step 2: Expressing Bayes' Theorem in Odds Form

We consider Bayes' Theorem for both the hypothesis being true (`H`) and false (`¬H`), and then take their ratio.

$$
\frac{P(H|E)}{P(\neg H|E)} = \frac{\frac{P(E|H)P(H)}{P(E)}}{\frac{P(E|\neg H)P(\neg H)}{P(E)}}
$$

The `P(E)` term in the denominator cancels out. Simplifying the expression gives us a powerful relationship:

$$
\frac{P(H|E)}{P(\neg H|E)} = \frac{P(E|H)}{P(E|\neg H)} \cdot \frac{P(H)}{P(\neg H)}
$$

This can be expressed in words as:

> **Posterior Odds = Likelihood Ratio × Prior Odds**

Here, the **Likelihood Ratio**, `P(E|H) / P(E|¬H)`, is a crucial term that represents the **"strength of evidence E"**.

### Step 3: Taking the Logarithm to Arrive at a Sum

By taking the **logarithm** of both sides of the odds-form equation, we convert the multiplication into an **addition**.

$$
\log\left(\frac{P(H|E)}{P(\neg H|E)}\right) = \log\left(\frac{P(E|H)}{P(E|\neg H)}\right) + \log\left(\frac{P(H)}{P(\neg H)}\right)
$$

This "logarithm of odds" is known as **Log-Odds** or **Logit**. Rewriting this equation in words gives us the final form used in implementation:

> **New Log-Odds = Log-Odds of Evidence + Current Log-Odds**

This is the mathematical proof that demonstrates why Bayesian updating can be performed by summing Log-Odds.

---

## 3. Summary and Advantages

-   **Computational Simplicity**: Complex calculations involving the multiplication and division of probabilities become simple **additions** in log-odds space.
-   **Numerical Stability**: It prevents numerical underflow issues that can occur when multiplying many small probabilities together.
-   **Intuitive Interpretation**: The process can be intuitively understood as adding the "weight" of new evidence to a current belief.

## Input topics

For every camera, the following three topics are subscribed:

| Name                                                  | Type                                             | Description                           |
| ----------------------------------------------------- | ------------------------------------------------ | ------------------------------------- |
| `~/<camera_namespace>/camera_info`                    | sensor_msgs::msg::CameraInfo                     | camera info from map_based_detector   |
| `~/<camera_namespace>/detection/rois`                 | tier4_perception_msgs::msg::TrafficLightRoiArray | detection roi from fine_detector      |
| `~/<camera_namespace>/classification/traffic_signals` | tier4_perception_msgs::msg::TrafficLightArray    | classification result from classifier |

You don't need to configure these topics manually. Just provide the `camera_namespaces` parameter and the node will automatically extract the `<camera_namespace>` and create the subscribers.

## Output topics

| Name                       | Type                                                  | Description                        |
| -------------------------- | ----------------------------------------------------- | ---------------------------------- |
| `~/output/traffic_signals` | autoware_perception_msgs::msg::TrafficLightGroupArray | traffic light signal fusion result |

## Node parameters

{{ json_to_markdown("perception/autoware_traffic_light_multi_camera_fusion/schema/traffic_light_multi_camera_fusion.schema.json") }}
