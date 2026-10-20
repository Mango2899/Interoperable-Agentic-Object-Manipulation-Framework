---
title: "Approximating the Center of a Robot Interaction Region"
subtitle: "Voxel intersection, bounded central core, and BDI candidate ranking"
author: "WoMAT project documentation"
date: "18 August 2026"
lang: en-US
---

# Purpose

For a handover between two robots, a point must be reachable by both robots and should not lie close to the edge of either workspace. The implementation therefore does not choose a random point from the raw overlap. It constructs a discrete approximation of the shared workspace, bounds that set to a central core, calculates several quality features for each candidate, and lets the BDI agent select the candidate with the greatest score.

This document describes the implemented method in mathematical form. The geometric component calculates candidates and evidence; the Jason BDI agent performs the final weighted evaluation and adopts the selected handover intention.

# Notation

Let robots $A$ and $B$ have sampled reachable TCP workspaces

$$
W_A^{L}=\{\mathbf{p}_{A,1}^{L},\ldots,\mathbf{p}_{A,n_A}^{L}\},\qquad
W_B^{L}=\{\mathbf{p}_{B,1}^{L},\ldots,\mathbf{p}_{B,n_B}^{L}\}.
$$

The superscript $L$ denotes a robot-local coordinate frame. World-frame quantities use the superscript $W$. A point is written as

$$
\mathbf{p}=(x,y,z)^T.
$$

The implementation uses CoppeliaSim's Euler composition order. For robot $r$ with translation $\mathbf{t}_r$ and roll, pitch, and yaw $(\alpha_r,\beta_r,\gamma_r)$,

$$
R_r=R_x(\alpha_r)R_y(\beta_r)R_z(\gamma_r).
$$

Local samples are transformed into the shared world frame by

$$
\mathbf{p}_r^W=\mathbf{t}_r+R_r\mathbf{p}_r^L,
$$

and the inverse transformation is

$$
\mathbf{p}_r^L=R_r^T(\mathbf{p}_r^W-\mathbf{t}_r).
$$

This frame normalization is essential: intersection and centrality are evaluated in one common world frame, while final TCP commands are converted back into the local frame of each robot.

# Estimating the shared interaction region

## Native sample spacing

The point clouds can have different sampling densities. For every distinct sample $\mathbf{p}_i$ in a workspace, the nearest-neighbor distance is

$$
d_i^{\mathrm{NN}}=\min_{j\ne i}\lVert\mathbf{p}_i-\mathbf{p}_j\rVert_2.
$$

The characteristic spacing of robot $r$ is estimated with the configured quantile (currently $q_s=0.75$):

$$
s_r=Q_{q_s}\left(\{d_i^{\mathrm{NN}}\}\right).
$$

The conservative shared spacing is

$$
s=\max(s_A,s_B).
$$

Using the larger spacing prevents the more densely sampled robot from creating unjustified precision in the overlap estimate.

## Proximity estimate and adaptive voxel size

The proximity radius is

$$
r_p=k_p s,
$$

where the current proximity factor is $k_p=1.25$. KD-tree queries estimate how many samples in each workspace have a neighbor from the other workspace within $r_p$. With bidirectional estimation enabled, the smaller normalized overlap ratio is used so that both point clouds must support the estimate.

Let $\widehat{N}_{\mathrm{close}}$ be the resulting estimated number of close samples and $N_t$ the target voxel count (currently $N_t=1000$). The unbounded voxel-size estimate is

$$
\widehat{h}=s\sqrt[3]{\max\left(\frac{\widehat{N}_{\mathrm{close}}}{N_t},1\right)}.
$$

It is clamped to the native-resolution and configured limits:

$$
h=\min\left(h_{\max},\max(\widehat{h},s,h_{\min})\right).
$$

If the proximity estimator finds no close samples, the implementation falls back to a bounding-box volume estimate.

## Voxelization and intersection

Let $\mathbf{o}$ be the component-wise minimum of the two world-space bounding boxes. A world point is mapped to the integer voxel index

$$
\nu(\mathbf{p})=\left\lfloor\frac{\mathbf{p}-\mathbf{o}}{h}\right\rfloor,
$$

where division and the floor operation are component-wise. The occupied voxel sets are

$$
V_A=\{\nu(\mathbf{p})\mid\mathbf{p}\in W_A^W\},\qquad
V_B=\{\nu(\mathbf{p})\mid\mathbf{p}\in W_B^W\}.
$$

The discrete interaction region is their intersection:

$$
V_{AB}=V_A\cap V_B.
$$

For voxel index $\mathbf{k}=(k_x,k_y,k_z)^T$, its representative world-space center is

$$
\mathbf{c}(\mathbf{k})=\mathbf{o}+h\left(\mathbf{k}+\frac{1}{2}\mathbf{1}\right).
$$

Thus, the initial shared candidate set is

$$
C_{AB}=\{\mathbf{c}(\mathbf{k})\mid\mathbf{k}\in V_{AB}\}.
$$

# Bounding the interaction region to its central core

## Height constraint

Candidates are first restricted to a handover-height interval:

$$
C_H=\{\mathbf{c}\in C_{AB}\mid z_{\min}\le c_z\le z_{\max}\}.
$$

The agents currently request $z_{\min}=0.30\,\mathrm{m}$ and $z_{\max}=0.65\,\mathrm{m}$. These values are world-frame heights.

## Quantile bounds

For each axis $a\in\{x,y,z\}$, the lower and upper central bounds are calculated from the height-filtered set:

$$
b_a^-=Q_{q_l}(\{c_a\mid\mathbf{c}\in C_H\}),\qquad
b_a^+=Q_{q_u}(\{c_a\mid\mathbf{c}\in C_H\}).
$$

The current configuration uses $q_l=0.20$ and $q_u=0.80$. The bounded central core is

$$
C_{\mathrm{core}}=
\left\{\mathbf{c}\in C_H\mid
b_a^-\le c_a\le b_a^+\ \text{for every}\ a\in\{x,y,z\}
\right\}.
$$

This removes the outer 20 percent on both sides of every coordinate distribution. It is more robust than taking the center of an axis-aligned bounding box because isolated extreme voxels do not determine the central region.

If this filtering leaves fewer than the configured minimum of 10 candidates, the method falls back to $C_H$. This prevents an overly aggressive bound from eliminating a small but valid interaction region.

## Approximate center

The center used for ranking is the centroid of the bounded core:

$$
\boldsymbol{\mu}=\frac{1}{|C_{\mathrm{core}}|}
\sum_{\mathbf{c}\in C_{\mathrm{core}}}\mathbf{c}.
$$

This centroid is an approximation of the geometric center of the feasible overlap. It is not automatically selected as the handover point because the exact centroid may not coincide with an occupied, mutually reachable voxel. Instead, actual voxels near this center are ranked.

# Candidate quality features

For every candidate $\mathbf{c}_i\in C_{\mathrm{core}}$, four normalized features in $[0,1]$ are calculated.

## Centrality

Let

$$
d_i=\lVert\mathbf{c}_i-\boldsymbol{\mu}\rVert_2,
\qquad
d_{\max}=\max_j d_j.
$$

Then

$$
\mathrm{centrality}_i=\max\left(0,1-\frac{d_i}{\max(d_{\max},\varepsilon)}\right).
$$

A candidate at the centroid approaches 1; a candidate at the most distant part of the bounded core approaches 0.

## Local voxel density

The density neighborhood radius is

$$
r_d=2.5h.
$$

The raw density is the number of core voxels within that radius:

$$
n_i=\left|\left\{\mathbf{c}_j\in C_{\mathrm{core}}mid
\lVert\mathbf{c}_j-\mathbf{c}_i\rVert_2\le r_d\right\}\right|.
$$

It is normalized by the maximum observed density:

$$
\mathrm{density}_i=\frac{n_i}{\max_j n_j}.
$$

Density favors points surrounded by many alternative shared voxels and avoids narrow protrusions in the overlap.

## Robot-specific reachability support

The cube candidate is converted into a TCP target for each robot. If $\mathbf{o}_{J,r}^{L}$ is robot $r$'s jaw-center offset, then

$$
\mathbf{o}_{J,r}^{W}=R_r\mathbf{o}_{J,r}^{L},
$$

$$
\mathbf{p}_{\mathrm{TCP},r}^{W}=\mathbf{c}_i-\mathbf{o}_{J,r}^{W},
$$

$$
\mathbf{p}_{\mathrm{TCP},r}^{L}=R_r^T
(\mathbf{p}_{\mathrm{TCP},r}^{W}-\mathbf{t}_r).
$$

Let $\delta_{i,r}$ be the nearest-neighbor distance from this local TCP target to robot $r$'s sampled reachable workspace. With workspace tolerance $\tau_r$,

$$
\mathrm{support}_{i,r}=\max\left(0,1-\frac{\delta_{i,r}}{\tau_r}\right).
$$

Candidates are rejected before ranking if either local TCP target lies outside its sampled workspace bounds or if $\delta_{i,r}>\tau_r$. The current tolerance is $0.08\,\mathrm{m}$ for each generated robot.

# BDI ranking and selection

The geometry component publishes each surviving candidate as a belief containing its position and four evidence values. The Jason agent—not the geometry component—calculates

$$
S_i=
0.40\,\mathrm{centrality}_i+
0.25\,\mathrm{density}_i+
0.175\,\mathrm{support}_{i,A}+
0.175\,\mathrm{support}_{i,B}.
$$

The weights sum to 1. Centrality has the greatest influence, density rewards a thick and stable interaction region, and the two equal support terms ensure that neither robot's reachability evidence dominates the other.

The selected candidate is

$$
i^*=\operatorname*{arg\,max}_i S_i.
$$

The agent records `selectedHandoverCandidate`, commits that candidate through AgentThing, and then adopts the plans that move the giver and receiver to their corresponding local TCP targets.

# Algorithm summary

1. Transform both sampled workspaces from robot-local coordinates to the common world frame.
2. Estimate native point spacing using nearest-neighbor distance quantiles.
3. Select an adaptive voxel size and voxelize both workspaces.
4. Intersect the occupied voxel sets to approximate mutual reachability.
5. Filter by the allowed world-frame handover height.
6. Apply 20th-to-80th percentile bounds independently on $x$, $y$, and $z$.
7. Compute the centroid of this bounded central core.
8. For each real core voxel, calculate centrality, local density, and reachability support for both robots.
9. Reject candidates that are not sufficiently supported by either sampled workspace.
10. Publish candidate beliefs and let the BDI agent maximize the weighted score.
11. Convert the selected world cube point into robot-local TCP targets and begin collision-aware motion planning.

# Current generated example

The latest generated artifacts contain the following pairwise approximations:

| Pair | Effective spacing $s$ | Voxel size $h$ | Shared voxels before central filtering | Reachable samples |
|---|---:|---:|---:|---:|
| Robot1--Robot2 | 0.04510 m | 0.04510 m | 466 | 25,913 / 31,298 |
| Robot2--Robot3 | 0.03126 m | 0.03126 m | 114 | 31,298 / 24,833 |

For the current Robot2--Robot3 core, the BDI weighting selected candidate 6 during the latest test sequence. Its cube point was approximately

$$
\mathbf{c}_6^W=(1.5800,-0.0340,0.3736)^T\ \mathrm{m},
$$

and Robot3's corrected local TCP target was approximately

$$
\mathbf{p}_{\mathrm{TCP},3}^{L}=(-0.3736,0.0140,0.5250)^T\ \mathrm{m}.
$$

The latter transforms back to a world TCP height of approximately $0.374\,\mathrm{m}$, illustrating why frame conversion must be performed consistently with CoppeliaSim.

# Interpretation and limitations

The method approximates a safe central region from sampled kinematic reachability. It does not prove that every candidate has a collision-free path in the current dynamic scene. Walls, the other robot's current posture, carried objects, IK orientation choices, and swept-link collisions are evaluated later by the lower motion planner.

Consequently, the central-core score should be interpreted as a robust geometric prior. Collision-aware IK and path planning remain the final feasibility checks. A future extension can add a plan-only feasibility belief and let the BDI agent reject an infeasible candidate and evaluate the next-highest score without abandoning the handover session.

# Implementation references

- Interaction-space estimation and ranking evidence: `src/generator/interactionSpace.js`
- BDI weighted score and candidate selection: `src/agt/handoverPlans.asl`
- Shared-cell parameters: `generated/robots/Robot1_Robot2_Robot3.shared.json`
- Pair metadata: `generated/robots/interactions/<pair>/space.json`
