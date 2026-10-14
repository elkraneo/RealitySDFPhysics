# Tuning Collision Detection

Trade accuracy for speed with the paper's parameters and the solver settings.

## Overview

For every pair of bodies whose bounds overlap, the collision detection looks for the deepest
points of their intersection by minimizing a smooth maximum of the two distance fields (Eqs. 2–4
of the paper). It starts from the points it kept from the previous substep, the temporal cache
(Sec. 3.5.1), plus a few new random samples, biased towards high curvature regions where new
contacts tend to appear (Sec. 3.5.2). The defaults of ``SDFCollisionDetection`` are the paper's
and work for most scenes of objects about 1 unit across.

## Where the time goes

Collision detection usually dominates a step. Its cost grows with the number of overlapping pairs,
the substeps, and the samples per pair: ``SDFCollisionDetection/cachedSamples`` plus
``SDFCollisionDetection/randomSamples``. ``SDFSimulation/statistics`` reports both times, the
pairs, the contacts and the work of the optimizer for the last step.

## Parameters

- ``SDFCollisionDetection/cachedSamples`` (n_cache, 4): contact points kept per pair. More keep
  wide contacts, such as a box on a floor, steadier.
- ``SDFCollisionDetection/randomSamples`` (n_rand, 1): new samples per pair and substep, which find
  new contact regions. Raise it when bodies interlock in several places at once and some contacts
  are found late.
- ``SDFCollisionDetection/usesTemporalCache`` and ``SDFCollisionDetection/usesCurvatureSampling``:
  the paper's two accelerations. Turning them off shows what they buy, at a large cost.
- ``SDFCollisionDetection/skipsConvexSamples``: skips the random samples of a convex pair while
  the cache tracks its contact, since two convex shapes touch in one region only.
- ``SDFCollisionDetection/optimizer``: Nelder–Mead (the default) needs no gradients. The ellipsoid
  method is an alternative; gradient descent stalls more easily.
- ``SDFCollisionDetection/tolerance`` (τ, 0.01) and ``SDFCollisionDetection/maxIterations`` (50):
  when the optimizer stops. A larger tolerance is faster and coarser.
- ``SDFCollisionDetection/contactMargin`` (0.02) and
  ``SDFCollisionDetection/allowedPenetration`` (0.01): bodies rest a skin apart, between these
  two. Rather than scaling them for objects much larger or smaller than 1 unit, model the objects
  at about 1 unit in a scaled root (see <doc:GettingStarted>).

## Solver settings

``SDFSimulationComponent/substeps`` (4) runs the collision detection and the contact solver that
many times per step. More substeps make fast and stacked bodies more stable at a proportional
cost. ``SDFSimulationComponent/solverIterations`` sets the contact iterations of each substep:
`velocityIterations` (20) for contacts and friction, `positionIterations` (20) for pushing
penetrating bodies apart. Fewer iterations are faster and make tall stacks softer.

``SDFSimulationComponent/timeStep`` is the length of a step (1/60 s). Each frame takes as many
steps as the elapsed time covers, up to ``SDFSimulationComponent/maximumStepsPerFrame``; time
beyond that is dropped, so a slow frame slows the simulation down instead of spiraling.
