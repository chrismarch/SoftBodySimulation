# Soft Body Simulation

This is a quick exploration of procedural methods for deforming meshes in response to collisions with other objects, specifically, character meshes made of jelly (slimes!). No animation assets are used in this project, the movement and deformation of the characters are all from the soft body simulation.

## Code
I have written this code as a personal project. My code can be found in the Assets/Scripts/ directory, currently as a single file, [SoftBodyPrototype.cs](Assets/Scripts/SoftBodyPrototype.cs). To read the code in a top-down fashion, I recommend this order: 1. public fields, used by the Unity Editor inspector 2. Awake 3. OnTrigger* 4. FixedUpdate 5. OnDrawGizmos

## Play in Unity Editor
Open this project with Unity Editor 2018.2 or later. Then start with the simplest scene, by opening Scenes/OneSlimePrototype, and pressing play. To view the point mass locations and springs, click the Gizmos button above the Game window to make sure it is highlighted. Be aware that displaying gizmos in scenes with many soft bodies may have a noticeable effect on framerate.
I've tested this project with Unity Editor 2018.2.15f1 on Windows 8.1.

## My Next Steps
* Create a high performance ECS/Job system version, perhaps without using Unity's collision system
* Create a free form deformation shader that uses the point mass locations, or perhaps just applies shears
* Better enforce the fixed volume of the softbody during compression, modelling a fluid like water, which typically stays close to constant volume

## My Previous Steps
* Debug the cause of NaN in the positions (this was due to too low a damping value, which led to very high velocities, and then point masses that were so far apart that the area of the parallelogram on the face of their hull, which I calculated as magnitude of cross product, was float.PositiveInfinity)
* Simulate lots of soft bodies, with some variety of meshes, in a new test scene. Evaluate frame rate, and check that dimensions of soft body can be configured via the transform's scale
* See how many springs I can remove until the simulation destabilizes
* Experiment with the public coefficient properties to find values that lead to numerically stable simulation, while allowing for a pleasing amount of deformation
* Assemble point masses in a bounding box shape, and create a spring lattice, adding springs and experimenting with coefficients until the simulation doesn't collapse over time
* Combine pressure simulation with the mass spring system, to simulate a volume of air
* Alter the pressure simulation to attempt to maintain constant volume, similar to a water balloon (since water doesn't noticeably compress under typical forces).
* Apply a nonuniform scale on a mesh associated with the soft body simulation
* Give the soft body a periodic jump behavior, to keep things lively
* Create a procedural animation for the build up to the jump by negating the instantaneous jump velocity (impulse) to drive the soft body into the ground, which causes it to compress and then rebound into the air
* Clean up the prototype (MonoBehavior) code a bit
* Write the code

## Author
Chris March
https://www.linkedin.com/in/cmarch/

