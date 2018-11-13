# Soft Body Simulation

## Summary
This is a quick exploration of soft body simulation. I'm looking into simple methods for deforming meshes in response to their collision with other objects, as a way to create procedural animation for characters made of jelly (slimes!).

This project has been tested with Unity 2018.2.15f1 Personal under Windows 8.1 64-bit.

## Usage
First open Scenes/OneSlimePrototype, and press play. To view the point mass locations and springs, click the Gizmos button above the Game window to make sure it is highlighted. Be aware that displaying gizmos in scenes with many soft bodies may have a noticeable effect on framerate.

## Code Sample
I have written this code as a personal project. My code can be found in the Assets/Scripts/ directory, as *.cs files (ignore *.meta files).

## My Next Steps
* Clean up the code
* Create a high performance ECS/Job system version, perhaps without using Unity's collision system
* Create a free form deformation shader that uses the point mass locations, or perhaps just applies shears
* Better enforce the fixed volume of the softbody, modelling an approximately uncompressible fluid like water
* Apply the soft body simulation to a variety of meshes

## My Previous Steps
* See how many springs I can remove until the simulation destabilizes
* Experiment with the public coefficient properties to find values that lead to numerically stable simulation, while allowing for a pleasing amount of deformation
* Assemble point masses in a bounding box shape, and create a spring lattice, adding springs and experimenting with coefficients until the simulation doesn't collapse over time
* Combine pressure simulation with the mass spring system, to simulate a volume of air
* Alter the pressure simulation to attempt to maintain constant volume, similar to a water balloon (since water doesn't noticeably compress under typical forces). 
* Write the code

## Author
Chris March
https://github.com/chrismarch

