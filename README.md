# Soft Body Simulation

This is a quick exploration of soft body simulation. I'm looking into simple methods for deforming meshes in response to their collision with other objects, as a way to create procedural animation for characters made of jelly (slimes!).

<b>First open Scenes/ManySlimesPrototype, and press play. Click the Gizmos button to highlight it, and view the point mass locations and springs.</b>

My next steps are:
* Clean up and comment the code
* See how many springs I can remove until the simulation destabilizes
* Create a high performance ECS/Job system version, perhaps without using Unity's collision system
* Create a free form deformation shader that uses the point mass locations, or perhaps just applies shears
* Better enforce the fixed volume of the softbody, modelling an approximately uncompressible fluid like water
