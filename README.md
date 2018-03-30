# Motion Control

## Table of Contents
- [About](#about)
  - [Introduction](#intro)
  - [Pathing](#pathing)
    - [Generator](#generator)
    - [Segment](#segment)
  - [~~Trajectory~~](#trajectory)

# About

## Introduction

This library aims to simplify and streamline the process of generating efficient curved pathing from two points, and the following thereof with consideration for physical considerations of the robot. These two components are referred to as the *path* — the abstract ideal path between point A and point B – and *trajectory*, the real world path the robot will take. 

Given the plethora of solutions for finding a path between two points, we have aimed to completely abstract out the concept of a path so that you could even feasibly use this library for solely the acceleration profiling aspect of the trajectory generation, running it on a simple linear path. 

To understand the mathematics behind much of this library, we highly suggest reading [*Planning Motion Trajectories for Mobile Robots Using Splines* by Christopher Sprunk (2008)](/sprunk-2008.pdf), which covers in understandable, well written terms the considerations and mathematics of both path and trajectory generation. 

## Pathing

Our abstracting of a path can be broken down into three pieces: the path generator, which produces path segments, which contain path points. To implement a new type of path, one simply need implement a generator, which has been generalized and simplified to a remarkable level.

We have already implemented cubic and quintic splines, as well as a further level of generalization for the generation of *n { n ≤ 5 }th* degree splines. Additionally, we've provided a straight-forward arc-based path generator which implements the standard path generator.  

### Generator

At its core, a path generator describes the *x* and *y* components of position, velocity, acceleration, and jerk at a given percentage along the path. Given this information, we iterate over the path from 0 -> 1, building an ordered list of "features" or path segments.

### Segment

A path segment/feature is defined by a change in curvature above a certain threshold. The segment contains individual path points separated by a constant amount. As it is feasible that the robot's traversal of the path doesn't line up perfectly with each path point, the path points are stored in a tree map (As of Java 8, it is implemented as a [Red-Black Tree](https://en.wikipedia.org/wiki/Red–black_tree)), allowing us to find the path point nearest to a given percentage along the path. This is vital to the ability to translate from the abstract path to a trajectory as is described later.

# Using

## Installing

- [motioncontrol-1.1.0.jar](https://crate.botprovoking.org/motioncontrol/motioncontrol-1.1.0.jar)
- [motioncontrol-1.1.0-sources.jar](https://crate.botprovoking.org/motioncontro/motioncontrol-1.1.0-sources.jar)

Include these in some way into your project classpath/dependencies. These are not currently hosted on a Maven server, although we may do so in the future. 

## Building

If you instead wish to build the library, simply go to the root of the project and run:
```
./gradlew build
```
The output for both the library and the sources can be found in `build/libs/[...]` Cheers!
