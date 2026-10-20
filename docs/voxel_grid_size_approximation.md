---
title: "Automatic Voxel-Grid Size Approximation for Intersecting Robot Workspaces"
subtitle: "Method, Formula Derivation, Implementation, and Scientific Context"
author: "WoMAT / Master’s Thesis Technical Note"
date: "17 August 2026"
bibliography: voxel_grid_size_approximation.bib
link-citations: true
reference-section-title: References
---

::: {custom-style="Title"}
Automatic Voxel-Grid Size Approximation for Intersecting Robot Workspaces
:::

::: {custom-style="Subtitle"}
Method, Formula Derivation, Implementation, and Scientific Context
:::

**WoMAT / Master’s Thesis Technical Note**  
17 August 2026

# Contents {.unnumbered}

- [Abstract](#abstract)
- [1. Problem definition](#problem-definition)
- [2. Input model and coordinate frames](#input-model-and-coordinate-frames)
- [3. Native point-cloud spacing](#step-1-native-point-cloud-spacing)
- [4. Bounding-box overlap gate](#step-2-bounding-box-overlap-gate)
- [5. Proximity approximation](#step-3-proximity-approximation-without-voxelisation)
- [6. Deriving voxel edge length](#step-4-deriving-voxel-edge-length)
- [7. One shared voxel grid](#step-5-one-shared-voxel-grid)
- [8. Complete algorithm](#complete-algorithm)
- [9. Current Robot1/Robot2 result](#current-robot1robot2-result)
- [10. Why this is a prior](#why-this-is-a-prior-not-an-optimum)
- [11. Parameter interpretation](#parameter-interpretation)
- [12. Computational considerations](#computational-considerations)
- [13. Recommended thesis evaluation](#recommended-thesis-evaluation)
- [14. Scientific context](#scientific-context-and-boundaries-of-support)
- [15. Implementation and reproduction](#implementation-mapping-and-reproduction)
- [16. Suggested future improvements](#suggested-future-improvements)
- References

# Abstract

This technical note documents the one-shot voxel-grid size approximation used
for the interaction space between two robot workspaces in WoMAT. Each workspace
is available as a Cartesian point cloud of reachable tool-centre-point (TCP)
positions. The objective is to choose a voxel edge length *before* voxelising,
then voxelise each workspace exactly once on one shared grid and intersect the
occupied voxel indices.

The method combines four ideas:

1. nearest-neighbour distances estimate the useful spatial resolution supported
   by each point cloud;
2. a KD-tree estimates how many raw samples lie near the other robot’s
   workspace without constructing a voxel grid;
3. the volume identity $V \approx N s^3$ converts a desired interaction-voxel
   count into a voxel edge length; and
4. an explicit common origin ensures that equal voxel indices denote the same
   physical cells.

The final cube-root estimator is a **project-specific engineering derivation**.
It is not presented as a formula copied from one scientific article. Its
components are consistent with established nearest-neighbour density estimation,
multidimensional search, point-cloud processing, and voxelized robot workspace
representations [@Loftsgaarden1965; @Bentley1975; @Rusu2011; @Zacharias2007].

# 1. Problem definition

Let the reachable TCP samples of two robots be

$$
P_1 = \{\mathbf{p}_{1,i}\}_{i=1}^{n_1}, \qquad
P_2 = \{\mathbf{p}_{2,j}\}_{j=1}^{n_2},
$$

where every point is expressed in metres and
$\mathbf{p}=(x,y,z)^\mathsf{T}\in\mathbb{R}^3$. The intended result is an
interaction space

$$
\mathcal{I}_s = \mathcal{V}_s(P_1) \cap \mathcal{V}_s(P_2),
$$

where $\mathcal{V}_s(P)$ is the set of occupied voxel indices generated from
point set $P$ using edge length $s$.

The design target is not “the mathematically optimal voxel size,” because that
quantity is undefined until an optimization criterion is chosen. Instead, the
configuration specifies a desired number of occupied interaction voxels,
$N_{\mathrm{target}}$. This is a computational and task-level resolution target:
it controls the approximate number of handover candidates and therefore the
cost and granularity of subsequent selection.

Voxelized workspace and capability maps are established representations for
robot-arm reachability [@Zacharias2007; @AndersonSprecher2012]. The special
problem addressed here is choosing the resolution automatically from already
sampled workspaces.

# 2. Input model and coordinate frames

## 2.1 Reachable samples

The WoMAT workspace CSV format contains

```text
X,Y,Z,Collision_state
```

Only rows with `Collision_state = 1` are used. Non-finite rows are rejected and
exact duplicate XYZ points are removed before spacing estimation. The method
assumes that these samples approximate a **three-dimensional reachable volume**.
If the CSV contains only a two-dimensional boundary surface, the cubic volume
model in Section 6 is inappropriate.

## 2.2 Transforming each workspace into the world frame

The workspace files store robot-local coordinates. For robot $r$, let its base
translation be

$$
\mathbf{t}_r=(t_{x,r},t_{y,r},t_{z,r})^\mathsf{T}
$$

and its yaw rotation be $\theta_r$. The world point is

$$
\mathbf{p}^{W} = \mathbf{t}_r + R_z(\theta_r)\mathbf{p}^{L},
$$

with

$$
R_z(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta & 0\\
\sin\theta &  \cos\theta & 0\\
0           & 0            & 1
\end{bmatrix}.
$$

All distance, bounding-box, proximity, origin, and voxel calculations are
performed after this transformation. Mixing local frames would produce a
geometrically meaningless intersection.

# 3. Step 1 — native point-cloud spacing

For every distinct point $\mathbf{p}_i$ in workspace $P_r$, define the distance
to the nearest *different* point:

$$
d_{r,i} = \min_{j\ne i}\left\|\mathbf{p}_{r,i}-\mathbf{p}_{r,j}\right\|_2.
$$

The project uses a robust percentile rather than the arithmetic mean:

$$
d_r = Q_q\left(\{d_{r,i}\}\right),
$$

where $Q_q$ denotes quantile $q$. The default is $q=0.75$. A median
($q=0.50$) is suitable for approximately uniform Cartesian sampling; the 75th
percentile is more conservative when Cartesian density varies because the
original samples were generated in joint space.

The effective supported spacing is

$$
d_{\mathrm{eff}}=\max(d_1,d_2).
$$

Taking the maximum prevents the denser point cloud from claiming a resolution
that the sparser cloud does not support. Nearest-neighbour distances are a
classical basis for multivariate density estimation [@Loftsgaarden1965]. Local
sampling density is also central to point-cloud approximation and
simplification [@Pauly2002]. Here, however, $d_{\mathrm{eff}}$ is used as an
engineering lower resolution bound, not as an unbiased statistical estimator.

For large clouds, at most $m$ points are selected with a deterministic seeded
shuffle. Defaults are $m=20{,}000$ and seed 42. Deterministic sampling makes the
result reproducible.

# 4. Step 2 — bounding-box overlap gate

For workspace $P_r$, define the axis-aligned bounding box

$$
B_r=[x_{r,\min},x_{r,\max}]\times
    [y_{r,\min},y_{r,\max}]\times
    [z_{r,\min},z_{r,\max}].
$$

The overlap length on axis $a\in\{x,y,z\}$ is

$$
L_a=\max\left(0,
\min(a_{1,\max},a_{2,\max})-
\max(a_{1,\min},a_{2,\min})
\right).
$$

Therefore,

$$
V_{\mathrm{bbox}}=L_xL_yL_z.
$$

If $V_{\mathrm{bbox}}=0$, the implementation returns
`no_geometric_overlap` without building the proximity estimator. If it is
positive, the value is only a coarse upper proxy. Curved, hollow, disconnected,
or non-convex workspaces can occupy far less volume than their boxes.

# 5. Step 3 — proximity approximation without voxelisation

A three-dimensional KD-tree is built for the target point cloud. KD-trees were
introduced for efficient multidimensional associative search [@Bentley1975]
and are used here for nearest-neighbour and fixed-radius queries.

For every sampled point in direction $P_1\rightarrow P_2$, define

$$
\delta_i^{1\rightarrow2}
=\min_{\mathbf{q}\in P_2}
\left\|\mathbf{p}_{1,i}-\mathbf{q}\right\|_2.
$$

The proximity radius is coupled to native resolution:

$$
\rho=\kappa d_{\mathrm{eff}},
$$

where $\kappa=1.25$ by default. A point is “close” when
$\delta_i^{1\rightarrow2}\le\rho$. If $c_{12}$ of $m_1$ sampled points are
close, the full-cloud count estimate is

$$
\widehat{C}_{12}=\frac{c_{12}}{m_1}n_1.
$$

The calculation is repeated in reverse to reduce density-asymmetry bias:

$$
\widehat{C}_{21}=\frac{c_{21}}{m_2}n_2.
$$

WoMAT combines the directional overlap fractions conservatively:

$$
\widehat{N}_{\mathrm{close}}
=\min\left(
\frac{\widehat{C}_{12}}{n_1},
\frac{\widehat{C}_{21}}{n_2}
\right)\min(n_1,n_2).
$$

This symmetrization is a project-specific choice. It suppresses a large result
when only the denser cloud reports a high close-point fraction.

# 6. Step 4 — deriving voxel edge length

## 6.1 Characteristic sample volume

If the point clouds approximately sample a 3-D volume, a characteristic sample
spacing $d_{\mathrm{eff}}$ corresponds dimensionally to a characteristic volume

$$
v_{\mathrm{sample}}\propto d_{\mathrm{eff}}^3.
$$

WoMAT chooses the proportionality constant as one and approximates interaction
volume by

$$
\widehat{V}_{\mathrm{int}}
=\widehat{N}_{\mathrm{close}}d_{\mathrm{eff}}^3.
$$

This is deliberately a prior approximation. It is not an exact union-volume
estimator, and no claim is made that every sample owns a disjoint cube of side
$d_{\mathrm{eff}}$.

## 6.2 Target voxel count

A cubic voxel of edge length $s$ has exact volume

$$
V_{\mathrm{voxel}}=s^3.
$$

Ignoring boundary and occupancy effects, a volume $V$ contains approximately

$$
N\approx\frac{V}{s^3}
$$

voxels. Solving for $s$ gives

$$
s\approx\sqrt[3]{\frac{V}{N}}.
$$

Substituting $\widehat{V}_{\mathrm{int}}$ and the configured
$N_{\mathrm{target}}$ yields

$$
s_{\mathrm{est}}
=d_{\mathrm{eff}}
\sqrt[3]{
\frac{\widehat{N}_{\mathrm{close}}}{N_{\mathrm{target}}}
}.
$$

The implementation does not refine below native sampling resolution. Equivalently,

$$
s_{\mathrm{est}}
=d_{\mathrm{eff}}
\sqrt[3]{
\max\left(\frac{\widehat{N}_{\mathrm{close}}}
{N_{\mathrm{target}}},1\right)
}.
$$

The cube root is not an empirical fitted exponent: it follows from using cubic
cells in three spatial dimensions. In general dimension $D$, the corresponding
scaling would be $N^{-1/D}$.

## 6.3 Bounding-box fallback

If the bounding boxes overlap but the sampled proximity count is zero, one
fallback estimate is used:

$$
s_{\mathrm{bbox}}
=\max\left(
d_{\mathrm{eff}},
\sqrt[3]{\frac{V_{\mathrm{bbox}}}{N_{\mathrm{target}}}}
\right).
$$

The algorithm then performs the normal single voxelisation. It does not run a
resolution sweep.

## 6.4 Engineering bounds

The final value is

$$
s_{\mathrm{final}}
=\min\left(
s_{\max},
\max(s_{\mathrm{est}},d_{\mathrm{eff}},s_{\min})
\right).
$$

$s_{\min}$ can represent robot accuracy, calibration uncertainty, or the
minimum meaningful handover tolerance. $s_{\max}$ limits geometric coarseness.
The configuration is rejected if $s_{\max}<\max(d_{\mathrm{eff}},s_{\min})$.

# 7. Step 5 — one shared voxel grid

The shared origin is the component-wise minimum over both world-space clouds:

$$
\mathbf{o}=
\begin{bmatrix}
\min_{\mathbf{p}\in P_1\cup P_2}p_x\\
\min_{\mathbf{p}\in P_1\cup P_2}p_y\\
\min_{\mathbf{p}\in P_1\cup P_2}p_z
\end{bmatrix}.
$$

Every point is mapped to an integer voxel index:

$$
\mathbf{v}(\mathbf{p})=
\left\lfloor
\frac{\mathbf{p}-\mathbf{o}}{s_{\mathrm{final}}}
\right\rfloor,
$$

where subtraction, division, and floor are component-wise. The occupied sets are

$$
\mathcal{V}_1=\{\mathbf{v}(\mathbf{p}):\mathbf{p}\in P_1\},\qquad
\mathcal{V}_2=\{\mathbf{v}(\mathbf{p}):\mathbf{p}\in P_2\}.
$$

The discrete interaction space is the set intersection

$$
\mathcal{I}=\mathcal{V}_1\cap\mathcal{V}_2.
$$

For an interaction index $\mathbf{k}=(k_x,k_y,k_z)$, its world-space centre is

$$
\mathbf{c}(\mathbf{k})
=\mathbf{o}+s_{\mathrm{final}}
\left(\mathbf{k}+\frac{1}{2}\mathbf{1}\right).
$$

The common origin is mandatory. Using
$\lfloor\mathbf{p}/s\rfloor$ independently may happen to align two grids only
when both silently share world zero; it does not express the invariant and is
fragile under coordinate changes.

# 8. Complete algorithm

```text
INPUT:
  reachable point clouds P1_local, P2_local
  base transforms T1, T2
  target count N_target
  percentile q, proximity factor kappa, sample limit m
  bounds s_min, s_max, deterministic seed

1. Filter valid reachable rows and remove exact duplicates.
2. Transform both point clouds into the world frame.
3. Compute AABBs; stop if their overlap volume is zero.
4. Estimate d1 and d2 from sampled nearest-neighbour distances.
5. Set d_eff = max(d1, d2) and rho = kappa * d_eff.
6. Estimate close-point counts in both directions with KD-trees.
7. Conservatively combine the two close fractions.
8. Compute s_est from d_eff * cbrt(N_close / N_target), with a
   lower bound of d_eff; use the bounding-box fallback only if necessary.
9. Clamp against engineering limits and validate the result.
10. Compute one shared component-wise origin.
11. Voxelise P1 once and P2 once using the same size and origin.
12. Intersect their occupied integer-key sets.
13. Store voxel centres and estimator diagnostics in Robot 1's private
    interaction-space controller; publish summary beliefs to Robot 1.
```

# 9. Current Robot1/Robot2 result

For the workspace files present on 17 August 2026, the deterministic default
configuration produced:

| Quantity | Value |
|---|---:|
| Reachable Robot 1 samples | 51,663 |
| Reachable Robot 2 samples | 62,924 |
| Robot 1 spacing, $d_1$ | 0.0356716 m |
| Robot 2 spacing, $d_2$ | 0.0138473 m |
| Effective spacing, $d_{\mathrm{eff}}$ | 0.0356716 m |
| Proximity radius, $\rho$ | 0.0445895 m |
| Estimated close points | 1,358.7369 |
| Target interaction voxels | 1,000 |
| Estimated/final voxel size | **0.0395095 m** |
| Actual occupied interaction voxels | **967** |
| Estimator mode | KD-tree proximity |

The shared origin was

$$
\mathbf{o}=(-1.2995830,-1.3036553,-1.1727232)^\mathsf{T}\;\mathrm{m}.
$$

Obtaining 967 instead of exactly 1,000 is expected. The formula estimates
continuous volume from discrete samples; boundary placement, duplicate
occupancy, nonuniform density, and workspace geometry determine the final
integer count.

# 10. Why this is a prior, not an optimum

The estimator answers: “Which voxel size is likely to produce roughly the
desired candidate count without repeatedly voxelising?” It does not solve a
universal optimization problem. Important distinctions are:

- **Task target:** $N_{\mathrm{target}}$ is chosen for downstream usefulness,
  not inferred from geometry alone.
- **Resolution support:** $d_{\mathrm{eff}}$ limits false precision, but robot
  calibration uncertainty may require a larger lower bound.
- **Occupancy approximation:** a sampled point causes its whole voxel to count
  as occupied; this can overestimate the continuous intersection near boundaries.
- **Position only:** the current grid represents TCP position in $\mathbb{R}^3$,
  not orientation in $SO(3)$ or complete pose in $SE(3)$. Capability maps often
  discretize orientations as well [@Zacharias2007].
- **Reachability, not path feasibility:** intersection means both workspaces
  contain samples in the same cell. Runtime collision checking and motion
  planning remain necessary.
- **Sampling bias:** uniform joint-space sampling is generally nonuniform in
  Cartesian space. A single global percentile cannot fully correct this.

# 11. Parameter interpretation

| Parameter | Default | Effect of increasing it |
|---|---:|---|
| $N_{\mathrm{target}}$ | 1,000 | Smaller voxels, more candidates and memory |
| $q$ | 0.75 | More conservative/larger native spacing |
| $\kappa$ | 1.25 | More points classified as near the other workspace |
| sample size $m$ | 20,000 | Lower sampling variance, higher pre-estimation time |
| $s_{\min}$ | 0 m | Enforces physical/accuracy lower resolution |
| $s_{\max}$ | 0.2 m | Caps excessive coarseness |
| bidirectional | true | Reduces asymmetric-density bias |

Sensitivity follows directly from the cube root. For example, multiplying the
estimated overlap count by eight multiplies the estimated voxel edge by two.
Likewise, multiplying $N_{\mathrm{target}}$ by eight divides the edge length by
two, before lower-bound clamping.

# 12. Computational considerations

With $n$ points and a balanced KD-tree, nearest and radius queries are efficient
on typical low-dimensional data, although worst-case query time remains linear.
The present JavaScript implementation recursively sorts point subsets, giving
approximately $O(n\log^2n)$ construction in this implementation rather than an
optimized linear-time median build. Estimation queries are performed only for
the deterministic sample of at most $m$ points.

After estimating $s$, voxelisation is linear in the number of raw points:

$$
T_{\mathrm{voxel}}=O(n_1+n_2).
$$

Hash-set intersection is linear in the smaller occupied set on average. Memory
is proportional to the number of points during estimation and to the number of
occupied cells during voxelisation. More sophisticated octree representations,
such as OctoMap, are valuable when probabilistic occupancy, multi-resolution
queries, or large environments are required [@Hornung2013]. WoMAT deliberately
uses a single-resolution hash grid because it needs a simple, reproducible set
intersection.

# 13. Recommended thesis evaluation

The method should be evaluated empirically rather than justified only by the
current successful example.

## 13.1 Baselines

Compare against:

1. the former fixed size, 0.05 m;
2. the bounding-box-only estimate;
3. the proposed bidirectional proximity estimate; and
4. an offline multi-resolution sweep used **only as an evaluation reference**.

## 13.2 Metrics

- absolute and relative error from $N_{\mathrm{target}}$;
- estimator time and one-shot voxelisation time;
- occupied-cell memory;
- Jaccard similarity of interaction cells against a fine-grid reference;
- stability across random workspace samples and estimator seeds;
- stability under small robot-base calibration perturbations;
- number of candidates passing both robots’ final workspace checks;
- end-to-end handover planning success and motion success.

For two voxel sets $A$ and $B$, Jaccard similarity is

$$
J(A,B)=\frac{|A\cap B|}{|A\cup B|}.
$$

When resolutions differ, compare physical cell volumes or resample both results
onto a common evaluation grid before computing $J$.

## 13.3 Sensitivity study

Recommended factors are

$$
q\in\{0.50,0.75,0.90\},\qquad
\kappa\in\{1.0,1.25,1.5\},
$$

and

$$
N_{\mathrm{target}}\in\{250,500,1000,2000,4000\}.
$$

Report not only the mean but also dispersion over repeated point-cloud
generation runs. This will expose whether the apparent precision of a single
voxel size is justified by the sampling process.

# 14. Scientific context and boundaries of support

| Component in WoMAT | Scientific support | What is specific to WoMAT |
|---|---|---|
| Voxelized robot reachability | Robot workspace/capability maps discretize reachable regions [@Zacharias2007; @AndersonSprecher2012] | Intersection is used to form a handover interaction space |
| Nearest-neighbour spacing | Nearest-neighbour radii support multivariate density estimation [@Loftsgaarden1965] | Maximum robust percentile becomes a resolution floor |
| Efficient proximity queries | KD-trees support multidimensional search [@Bentley1975] | Deterministic subsampling and bidirectional conservative fraction |
| Point-cloud processing | PCL establishes practical 3-D point-cloud processing infrastructure [@Rusu2011] | Dependency-free JavaScript implementation |
| Cube-root scaling | Exact cubic cell volume $s^3$ | $\widehat V=\widehat N_{close}d_{eff}^3$ and target-count inversion |
| Occupancy structures | OctoMap demonstrates efficient 3-D voxel/octree mapping [@Hornung2013] | Single-resolution hash sets are chosen for exact set intersection |

Thus, a defensible thesis statement is:

> The proposed voxel-size approximation is an engineering estimator derived
> from dimensional voxel-volume scaling and a nearest-neighbour proxy for
> overlap volume. Its constituent methods are established, while the combined
> estimator and its parameterization require empirical validation for the
> studied robot workspaces.

# 15. Implementation mapping and reproduction

The estimator and Robot 1-owned controller are implemented in:

```text
src/generator/interactionSpace.js
```

Agent belief creation and local point selection are wired in:

```text
src/generator/agentThingServer.js
src/agt/handoverPlans.asl
```

The shared lower runtime stores only physical session identity and cube state:

```text
src/generator/lowerThingServer.js
```

The configuration is stored in:

```text
generated/robots/Robot1_Robot2.shared.json
```

Reproduce the current calculation without CoppeliaSim:

```bash
npm run interaction:estimate
```

Run the automated tests:

```bash
npm test
```

Refresh the optional green CoppeliaSim visualization after estimation:

```bash
npm run robot:pipeline -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json \
  --update-shared-space-visual
```

# 16. Suggested future improvements

1. Estimate local density with $k$-nearest-neighbour radii rather than one global
   spacing statistic.
2. Model positional uncertainty by dilating occupied cells or using
   probabilistic occupancy.
3. Extend interaction beliefs from position to orientation-aware reachability.
4. Validate or learn $\kappa$ and $N_{\mathrm{target}}$ from handover outcomes.
5. Record content hashes of workspace CSV inputs in estimator metadata.
6. Quantify boundary aliasing by repeating only the evaluation with shifted grid
   origins; keep one canonical origin in production.
7. Compare conservative, optimistic, and probability-weighted definitions of
   shared occupancy.
