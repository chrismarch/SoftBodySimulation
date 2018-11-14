// SoftBodyPrototype
// Author: 
// Chris March 
// https://github.com/chrismarch/SoftBodyPrototype

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Simulates soft body physics using point masses connected by springs, and pressure on the faces
// of the hull formed by the point masses, to simulate a contained fluid.
//
// This is a prototype, constructed with one MonoBehavior, and is intended to be refactored to
// use more performant and modular coding practices, such as the ECS and Job System.
public class SoftBodyPrototype : MonoBehaviour
{
    #region soft body simulation inspector coefficients
    public float SpringStiffness = 50.0f;
    public float SpringDamping = 1.0f;
    public float PressureMultiplier = 50.0f;
    public float BounceCoefficient = .05f;
    public float SlideCoefficient = .99f;
    #endregion

    #region jumping: inspector properties and private state
    // negative JumpSpeed creates a procedural "jump build up" animation and then the body rebounds
    public float JumpSpeed = -8.5f; 
    public float JumpWaitMin = 3.0f;
    public float JumpWaitMax = 4.0f;

    private float JumpWait;
    private float LandedTime;
    #endregion

    #region point mass private state
    private Vector3[] PointMassAccelerations;
    private Vector3[] PointMassVelocities;
    private Vector3[] PointMassPositions;
    #endregion

    #region private state, read only after Awake
    private int[,] FacePointMassIndexes;
    private int[,] DiagonalSpringPointMassIndexes;
    private float[] SpringRestLengths;
    private float HullRestVolume;
    private Vector3 TransformStartPosition;
    private Vector3 TransformStartScale;
    #endregion

    // components to cache for use during updates
    // TODO write an ECS/Job System version and see how the perf. is for naive custom collision testing
    private BoxCollider TriggerCollider;

    private void Awake()
    {
        TriggerCollider = GetComponent<BoxCollider>();

        // this soft body simulation uses a bounding box shaped (for simplicity) mass spring lattice 
        // to enclose the mesh to be deformed
        PointMassAccelerations = new Vector3[8];
        PointMassVelocities = new Vector3[8];
        PointMassPositions = new Vector3[8];
        InitializePointMassPositionsToBoundingBox();
        
        // Initialize the arrays that hold point mass indexes

        // The first index array has sets of 4 indexes for each of the 6 squares
        // that form the faces of the point mass hull (as a bounding box).
        FacePointMassIndexes =
                new int[6, 4]
                    { {3, 2, 1, 0}, {1, 5, 4, 0}, {2, 6, 5, 1}, {3, 7, 6, 2},
                    {0, 4, 7, 3}, {4, 5, 6, 7} };

        // The second index array enumerates the remaining springs, which are diagonal, since
        // they are not on the edges of the faces, and each spring is defined as a pair of indexes.
        DiagonalSpringPointMassIndexes =
            new int[16, 2]
                {
                    // crossing inside hull
                    {0, 6}, {1, 7}, {2, 4}, {3, 5}, 

                    // diagonals on hull
                    {3, 1}, {2, 0}, 
                    {0, 5}, {4, 1}, 
                    {1, 6}, {5, 2}, 
                    {2, 7}, {6, 3}, 
                    {3, 4}, {7, 0}, 
                    {4, 6}, {5, 7}, 
                };

        // calculate spring rest lengths
        int springIndex = 0;
        int numFaceEdgeSprings = FacePointMassIndexes.GetLength(0) * FacePointMassIndexes.GetLength(1);
        int numDiagonalSprings = DiagonalSpringPointMassIndexes.GetLength(0);
        SpringRestLengths = new float[numFaceEdgeSprings + numDiagonalSprings];

        // first the springs on the face edges of the hull
        int numFaces = FacePointMassIndexes.GetLength(0);
        int numPtsPerFace = FacePointMassIndexes.GetLength(1);
        for (int i = 0; i < numFaces; ++i)
        {
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                int pt0Index = FacePointMassIndexes[i, j];
                int pt1Index = FacePointMassIndexes[i, (j + 1) % numPtsPerFace];
                SpringRestLengths[springIndex] =
                    (PointMassPositions[pt0Index] - PointMassPositions[pt1Index]).magnitude;
                ++springIndex;
            }
        }

        // now calculate the rest lengths of the springs that aren't face edges on the hull
        for (int i = 0; i < numDiagonalSprings; ++i)
        {
            int pt0Index = DiagonalSpringPointMassIndexes[i, 0];
            int pt1Index = DiagonalSpringPointMassIndexes[i, 1];
            SpringRestLengths[springIndex] =
                (PointMassPositions[pt0Index] - PointMassPositions[pt1Index]).magnitude;
            ++springIndex;
        }

        HullRestVolume = transform.localScale.x * transform.localScale.y * transform.localScale.z;
        TransformStartPosition = transform.position;
        TransformStartScale = transform.localScale;
    }

    private void InitializePointMassPositionsToBoundingBox()
    {
        PointMassPositions[0] = transform.TransformPoint(new Vector3(.5f, .5f, .5f));
        PointMassPositions[1] = transform.TransformPoint(new Vector3(.5f, .5f, -.5f));
        PointMassPositions[2] = transform.TransformPoint(new Vector3(-.5f, .5f, -.5f));
        PointMassPositions[3] = transform.TransformPoint(new Vector3(-.5f, .5f, .5f));

        PointMassPositions[4] = transform.TransformPoint(new Vector3(.5f, -.5f, .5f));
        PointMassPositions[5] = transform.TransformPoint(new Vector3(.5f, -.5f, -.5f));
        PointMassPositions[6] = transform.TransformPoint(new Vector3(-.5f, -.5f, -.5f));
        PointMassPositions[7] = transform.TransformPoint(new Vector3(-.5f, -.5f, .5f));
    }

    private void OnTriggerEnter(Collider other)
    {
        // the TriggerCollider just touched another collider

        // track landing on a surface, for the jump behavior
        if (other.transform.position.y < transform.position.y)
        {
            LandedTime = Time.time;
            JumpWait = Random.Range(JumpWaitMin, JumpWaitMax);
        }
    }

    private void OnTriggerStay(Collider other)
    {
        // overlapping with another collider, so make sure the point masses don't interpenetrate,
        // and resolve their velocities for the collision
        Vector3 depenetrationDir;
        float depenetrationDist;
        if (Physics.ComputePenetration(TriggerCollider, transform.position, transform.rotation,
                                       other, other.transform.position, other.transform.rotation,
                                       out depenetrationDir, out depenetrationDist))
        {
            //Debug.LogFormat("Collision normal {0}", depenetrationDir);
            for (int i = 0; i < PointMassPositions.Length; ++i)
            {
                Vector3 p = PointMassPositions[i];
                if (other.bounds.Contains(p))
                {
                    // clamp interpenetrating point mass to surface of other collider
                    PointMassPositions[i] = 
                        other.ClosestPoint(p + depenetrationDir * (depenetrationDist + 1.0f));

                    // reflect component of velocity along other collider normal
                    // while maintaining the remainder of velocity, but reduce by
                    // energy loss coefficient
                    // (approximate average contact normals as depenetration direction)
                    float speedAlongNormalSigned = Vector3.Dot(PointMassVelocities[i], depenetrationDir);
                    float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
                    Vector3 velocityAlongNormal = speedAlongNormalSigned * depenetrationDir;
                    Vector3 slideVelocity = PointMassVelocities[i] - velocityAlongNormal;
                    velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing

                    // reduce velocityAlongNormal by bounce coefficient if reflecting
                    float bounceCoefficient = (speedAlongNormalSign >= 0.0f ? 1.0f : BounceCoefficient);
                    PointMassVelocities[i] = 
                        bounceCoefficient * velocityAlongNormal + 
                        slideVelocity * SlideCoefficient;                        
                }
            }
        }
    }

    // I've chosen fixed update to remove the variable of step size when investigating numeric
    // stability of the simulation.
    void FixedUpdate()
    {
        // accumulate forces for this update, start with gravity
        for (int i = 0; i < PointMassAccelerations.Length; ++i)
        {
            // simplify force to acceleration, as mass == 1
            PointMassAccelerations[i] = Physics.gravity;
        }

        // now accumulate spring forces:
        int springIndex = 0;
        // first the springs on the face edges of the hull
        int numFaces = FacePointMassIndexes.GetLength(0);
        int numPtsPerFace = FacePointMassIndexes.GetLength(1);
        for (int i = 0; i < numFaces; ++i)
        {
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                int pt0Index = FacePointMassIndexes[i, j];
                int pt1Index = FacePointMassIndexes[i, (j + 1) % numPtsPerFace];
                Vector3 springForce = 
                    CalcSpringForce(pt0Index, pt1Index, SpringRestLengths[springIndex], 
                                    SpringStiffness, SpringDamping);

                // mass == 1, so a = F/m = F
                PointMassAccelerations[pt0Index] += springForce;
                PointMassAccelerations[pt1Index] -= springForce;
                ++springIndex;
            }
        }

        // now accumulate the springs inside the hull
        int numDiagonalSprings = DiagonalSpringPointMassIndexes.GetLength(0);
        for (int i = 0; i < numDiagonalSprings; ++i)
        {
            int pt0Index = DiagonalSpringPointMassIndexes[i, 0];
            int pt1Index = DiagonalSpringPointMassIndexes[i, 1];
            Vector3 springForce =
                CalcSpringForce(pt0Index, pt1Index, SpringRestLengths[springIndex],
                                SpringStiffness, SpringDamping);
            PointMassAccelerations[pt0Index] += springForce;
            PointMassAccelerations[pt1Index] -= springForce;
            ++springIndex;
        }

        // estimate volume and pressure forces, similar to pseudocode: https://arxiv.org/ftp/physics/papers/0407/0407003.pdf
        // but replace the Ideal Gas Law approximation with a formula that is easier to tune for
        // an approximately fixed volume fluid (like water)
        float volume =
            transform.localScale.x * transform.localScale.y * transform.localScale.z;

        float volumeRatio = volume / HullRestVolume;
        float surfaceArea = 0.0f;
        //Debug.LogFormat("volume {0}, ratio {1}", volume, volumeRatio);
        Debug.Assert(numPtsPerFace == 4); // assume rectangles

        // TODO cache normals and area instead of recalculating
        for (int i = 0; i < FacePointMassIndexes.GetLength(0); ++i)
        {
            Vector3 a = PointMassPositions[FacePointMassIndexes[i, 0]];
            Vector3 b = PointMassPositions[FacePointMassIndexes[i, 1]];
            Vector3 c = PointMassPositions[FacePointMassIndexes[i, 2]];

            Vector3 faceNormal = CalcCross(a, b, c);
            float faceArea = faceNormal.magnitude; // magnitude of cross is area of parallelogram
            surfaceArea += faceArea;
        }

        for (int i = 0; i < FacePointMassIndexes.GetLength(0); ++i)
        {
            Vector3 a = PointMassPositions[FacePointMassIndexes[i, 0]];
            Vector3 b = PointMassPositions[FacePointMassIndexes[i, 1]];
            Vector3 c = PointMassPositions[FacePointMassIndexes[i, 2]];

            Vector3 faceNormal = CalcCross(a, b, c);
            float faceArea = faceNormal.magnitude; // magnitude of cross is area of parallelogram
            if (faceArea > 0.0f)
            {
                // normalize the cross product
                faceNormal /= faceArea;
            }

            // approximate force distribution through fluid by using more force on the 
            // faces of the hull that have a smaller area (assuming the rest area for each face
            // is equal)
            //
            // TODO refine the pressure algorithm so that the volume doesn't compress as much
            //
            float pressureForceMult = 1.0f - faceArea / surfaceArea;
            pressureForceMult *= pressureForceMult;            
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                PointMassAccelerations[FacePointMassIndexes[i, j]] +=
                    faceNormal * (PressureMultiplier * pressureForceMult * (1.0f - volumeRatio));
            }
        }

        // Jump every few seconds, to keep the simulation lively
        Vector3 jumpVelocity = Vector3.zero;
        if (LandedTime > 0.0f && Time.fixedTime - LandedTime >= JumpWait)
        {
            jumpVelocity = Vector3.up * JumpSpeed;
            LandedTime = 0.0f;
        }

        // solve for velocities and positions of point masses, and recalculate the bounding box
        Bounds ptBounds = new Bounds();
        for (int i = 0; i < PointMassPositions.Length; ++i)
        {
            PointMassVelocities[i] += PointMassAccelerations[i] * Time.fixedDeltaTime + jumpVelocity;
            PointMassPositions[i] += PointMassVelocities[i] * Time.fixedDeltaTime;
            if (i == 0)
            {
                ptBounds = new Bounds(PointMassPositions[i], Vector3.zero);
            }
            else
            {
                ptBounds.Encapsulate(PointMassPositions[i]);
            }
        }

        // The position and scale are set to fit the TriggerCollider (BoxCollider) to be
        // a conservative bounds for the point masses. Also, the child mesh will inherit this 
        // transform, and go squish, etc.
        if (float.IsNaN(ptBounds.center.x) || float.IsNaN(ptBounds.center.y) || float.IsNaN(ptBounds.center.z) ||
            float.IsNaN(ptBounds.size.x) || float.IsNaN(ptBounds.size.y) || float.IsNaN(ptBounds.size.z))
        {
            Debug.LogWarning("Simulation destabilized, NaN detected");
            /* TODO debug this case
            transform.position = TransformStartPosition;
            transform.localScale = TransformStartScale;
            InitializePointMassPositionsToBoundingBox();
            */
            gameObject.SetActive(false);
        }
        else
        {
            transform.position = ptBounds.center;
            transform.localScale = ptBounds.size;
        }
    }

    // Returns the cross product: (a - b) X (c - b)
    static Vector3 CalcCross(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 bToA = a - b;
        Vector3 btoC = c - b;
        return Vector3.Cross(bToA, btoC);
    }

    // Calculates the force from the spring to apply to point mass 0, 
    // and the inverse for point mass 1
    //
    // Adapted from http://joeyfladderak.com/lets-talk-physics-soft-body-dynamics/
    //
    Vector3 CalcSpringForce(int pt0Index, int pt1Index, float restLength, float stiffness, 
                            float damping)
    {
        Vector3 pt0ToPt1 = PointMassPositions[pt1Index] - PointMassPositions[pt0Index];
        float springLength = pt0ToPt1.magnitude;
        if (springLength == 0.0f)
        {
            // if fully compressed, avoid a zero force in the final multiplication
            pt0ToPt1 = Vector3.up;
        }
        else
        {
            // normalize
            pt0ToPt1 /= springLength;
        }
        float stretch = springLength - restLength;
        Vector3 relativePtVelocity = PointMassVelocities[pt1Index] - PointMassVelocities[pt0Index];

        // spring force magnitude =
        //      delta from rest length * stiffness coefficient + 
        //      relative pt velocity along spring * damping coefficient
        float springForceMagnitude =
            (stretch * stiffness) + Vector3.Dot(relativePtVelocity, pt0ToPt1) * damping;

        return springForceMagnitude * pt0ToPt1;
    }

    private void OnDrawGizmos()
    {
        if (PointMassPositions == null || PointMassPositions.Length < 4)
            return;

        // draw the top point masses
        Gizmos.color = Color.red;
        for (int i = 0; i < 4; ++i)
        {
            Gizmos.DrawSphere(PointMassPositions[i], .1f);
        }

        // draw the bottom point masses
        Gizmos.color = Color.blue;
        for (int i = 4; i < PointMassPositions.Length; ++i)
        {
            Gizmos.DrawSphere(PointMassPositions[i], .1f);
        }

        // draw the springs used to form the faces of the point mass hull and draw
        // their normals
        for (int i = 0; i < FacePointMassIndexes.GetLength(0); ++i)
        {
            Vector3 a = PointMassPositions[FacePointMassIndexes[i, 0]];
            Vector3 b = PointMassPositions[FacePointMassIndexes[i, 1]];
            Vector3 c = PointMassPositions[FacePointMassIndexes[i, 2]];
            Vector3 d = PointMassPositions[FacePointMassIndexes[i, 3]];

            // face edges
            Gizmos.color = Color.white;
            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, d);
            Gizmos.DrawLine(d, a);

            // face normals
            Gizmos.color = Color.blue;        
            Vector3 faceNormal = CalcCross(a, b, c).normalized;
            Vector3 faceMidpoint = Vector3.Lerp(a, c, .5f);
            Gizmos.DrawLine(faceMidpoint, faceMidpoint + faceNormal);
        }

        // draw the springs that cross inside the hull and hull faces
        Gizmos.color = Color.yellow;
        for (int i = 0; i < DiagonalSpringPointMassIndexes.GetLength(0); ++i)
        {
            int pt0Index = DiagonalSpringPointMassIndexes[i, 0];
            int pt1Index = DiagonalSpringPointMassIndexes[i, 1];
            Gizmos.DrawLine(PointMassPositions[pt0Index], PointMassPositions[pt1Index]);
        }
    }
}
