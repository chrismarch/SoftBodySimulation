using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Simulates soft body physics using point masses connected by springs, and pressure on the polygons
// on the hull formed by the springs to simulate a contained fluid.
public class SoftBodyPrototype : MonoBehaviour
{
    #region soft body simulation inspector coefficients
    public float SpringStiffness = 30.0f;
    public float SpringDamping = 1.0f;
    public float SpringRestLength = 1.0f;
    public float InternalSpringRestLength = 1.732051f;
    public float PressureMultiplier = 4.0f;
    public float HullRestVolume = 1.0f;
    public float HullHalfWidth = 0.5f;
    public float BounceCoefficient = .05f;
    public float SlideCoefficient = .99f;
    #endregion

    #region jumping: inspector properties and private state
    public float JumpSpeed = 5.0f;
    public float JumpWaitMin = 2.0f;
    public float JumpWaitMax = 5.0f;

    private float JumpWait;
    private float LandedTime;
    #endregion

    // cache components for use during updates
    // TODO write an ECS/Job System version and see how the perf. is for naive custom collision testing
    private BoxCollider TriggerCollider;

    #region point mass private state
    private Vector3[] Accelerations;
    private Vector3[] Velocities;
    private Vector3[] Positions;
    private int[,] FacePointIndexes;
    private int[,] InternalSpringPointIndexes;
    #endregion

    private void Awake()
    {
        TriggerCollider = GetComponent<BoxCollider>();

        // this soft body simulation uses the simplest mass spring latice to enclose a mesh,
        // a bounding box
        Accelerations = new Vector3[8];
        Velocities = new Vector3[8];
        Positions = new Vector3[8];

        Positions[0] = transform.TransformPoint(new Vector3(HullHalfWidth, HullHalfWidth, HullHalfWidth));
        Positions[1] = transform.TransformPoint(new Vector3(HullHalfWidth, HullHalfWidth, -HullHalfWidth));
        Positions[2] = transform.TransformPoint(new Vector3(-HullHalfWidth, HullHalfWidth, -HullHalfWidth));
        Positions[3] = transform.TransformPoint(new Vector3(-HullHalfWidth, HullHalfWidth, HullHalfWidth));

        Positions[4] = transform.TransformPoint(new Vector3(HullHalfWidth, -HullHalfWidth, HullHalfWidth));
        Positions[5] = transform.TransformPoint(new Vector3(HullHalfWidth, -HullHalfWidth, -HullHalfWidth));
        Positions[6] = transform.TransformPoint(new Vector3(-HullHalfWidth, -HullHalfWidth, -HullHalfWidth));
        Positions[7] = transform.TransformPoint(new Vector3(-HullHalfWidth, -HullHalfWidth, HullHalfWidth));

        FacePointIndexes =
                new int[6, 4]
                    { {3, 2, 1, 0}, {0, 4, 5, 1}, {1, 5, 6, 2}, {2, 6, 7, 3},
                    {3, 7, 4, 0}, {4, 5, 6, 7} };

        InternalSpringPointIndexes =
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
    }

    private void OnTriggerEnter(Collider other)
    {
        // the TriggerCollider just touched another collider

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
            for (int i = 0; i < Positions.Length; ++i)
            {
                Vector3 p = Positions[i];
                if (other.bounds.Contains(p))
                {
                    // clamp interpenetrating point mass to surface of other collider
                    Positions[i] = 
                        other.ClosestPoint(p + depenetrationDir * (depenetrationDist + 1.0f));

                    // reflect component of velocity along other collider normal
                    // while maintaining the remainder of velocity, but reduce by
                    // energy loss coefficient
                    // (approximate average contact normals as depenetration direction)
                    float speedAlongNormalSigned = Vector3.Dot(Velocities[i], depenetrationDir);
                    float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
                    Vector3 velocityAlongNormal = speedAlongNormalSigned * depenetrationDir;
                    Vector3 slideVelocity = Velocities[i] - velocityAlongNormal;
                    velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing
                    Velocities[i] = 
                        (speedAlongNormalSign >= 0.0f ? 1.0f : BounceCoefficient) * 
                        velocityAlongNormal + slideVelocity * SlideCoefficient;                        
                }
            }
        }
    }

    // I've chosen fixed update to remove the variable of step size when investigating numeric
    // stability of the simulation.
    void FixedUpdate()
    {
        // accumulate forces for this update, start with gravity
        for (int i = 0; i < Accelerations.Length; ++i)
        {
            // mass == 1
            Accelerations[i] = Physics.gravity;
        }
            
        // now accumulate spring forces
        // first the springs on the point mass hull
        int numPtsPerFace = FacePointIndexes.GetLength(1);
        for (int i = 0; i < FacePointIndexes.GetLength(0); ++i)
        {
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                int pt0Index = FacePointIndexes[i, j];
                int pt1Index = FacePointIndexes[i, (j + 1) % numPtsPerFace];
                Vector3 springForce = 
                    CalcSpringForce(pt0Index, pt1Index, SpringRestLength, SpringStiffness, SpringDamping);

                // mass == 1, so a = F/m = F
                Accelerations[pt0Index] += springForce;
                Accelerations[pt1Index] -= springForce;
            }
        }

        // now accumulate the springs inside the hull
        // TODO simplify this code by having all the spring point indexes in the same array
        for (int i = 0; i < InternalSpringPointIndexes.GetLength(0); ++i)
        {
            int pt0Index = InternalSpringPointIndexes[i, 0];
            int pt1Index = InternalSpringPointIndexes[i, 1];
            // TODO clean up the rest length code by precalculating an array
            Vector3 springForce =
                CalcSpringForce(pt0Index, pt1Index, i < 4 ? InternalSpringRestLength : 1.4142f,
                    SpringStiffness, SpringDamping);
            Accelerations[pt0Index] += springForce;
            Accelerations[pt1Index] -= springForce;
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
        for (int i = 0; i < FacePointIndexes.GetLength(0); ++i)
        {
            Vector3 a = Positions[FacePointIndexes[i, 0]];
            Vector3 b = Positions[FacePointIndexes[i, 1]];
            Vector3 c = Positions[FacePointIndexes[i, 2]];

            Vector3 edge0 = a - b;
            Vector3 edge1 = c - b;
            surfaceArea += edge0.magnitude * edge1.magnitude;
        }

        for (int i = 0; i < FacePointIndexes.GetLength(0); ++i)
        {
            Vector3 a = Positions[FacePointIndexes[i, 0]];
            Vector3 b = Positions[FacePointIndexes[i, 1]];
            Vector3 c = Positions[FacePointIndexes[i, 2]];

            Vector3 edge0 = a - b;
            Vector3 edge1 = c - b;
            float area = edge0.magnitude * edge1.magnitude;

            // approximate force distribution through fluid by using more force on the 
            // faces of the hull that have a smaller area (assuming the rest area for each face
            // is equal)
            //
            // TODO refine the pressure algorithm so that the volume doesn't compress as much
            //
            float pressureForceMult = 1.0f - area / surfaceArea;
            pressureForceMult *= pressureForceMult;
            Vector3 normal = Vector3.Cross(edge0, edge1).normalized;
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                Accelerations[FacePointIndexes[i, j]] +=
                    normal * (PressureMultiplier * pressureForceMult * (1.0f - volumeRatio));
            }
        }

        // Jump every few seconds, I'm a perky slime!
        Vector3 jumpVelocity = Vector3.zero;
        if (LandedTime > 0.0f && Time.fixedTime - LandedTime >= JumpWait)
        {
            jumpVelocity = Vector3.up * JumpSpeed;
            LandedTime = 0.0f;
        }

        // solve for velocities and positions of point masses, and recalc bounds
        Bounds ptBounds = new Bounds();
        for (int i = 0; i < Positions.Length; ++i)
        {
            Velocities[i] += Accelerations[i] * Time.fixedDeltaTime + jumpVelocity;
            Positions[i] += Velocities[i] * Time.fixedDeltaTime;
            if (i == 0)
            {
                ptBounds = new Bounds(Positions[i], Vector3.zero);
            }
            else
            {
                ptBounds.Encapsulate(Positions[i]);
            }
        }

        // The position and scale are set to fit the TriggerCollider (BoxCollider) to be
        // a conservative bounds for the point masses. Also, the child mesh will inherit this 
        // transform, and go squish, etc.
        transform.position = ptBounds.center;
        transform.localScale = ptBounds.size;
    }

    // Calculates the force from the spring to apply to point mass 0, 
    // and the inverse for point mass 1
    //
    // Adapted from http://joeyfladderak.com/lets-talk-physics-soft-body-dynamics/
    //
    Vector3 CalcSpringForce(int pt0Index, int pt1Index, float restLength, float stiffness, 
                            float damping)
    {
        Vector3 pt0ToPt1 = Positions[pt1Index] - Positions[pt0Index];
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
        Vector3 relativePtVelocity = Velocities[pt1Index] - Velocities[pt0Index];

        // spring force magnitude =
        //      delta from rest length * stiffness coefficient + 
        //      relative pt velocity along spring * damping coefficient
        float springForceMagnitude =
            (stretch * stiffness) + Vector3.Dot(relativePtVelocity, pt0ToPt1) * damping;

        return springForceMagnitude * pt0ToPt1;
    }

    private void OnDrawGizmos()
    {
        if (Positions == null || Positions.Length < 4)
            return;

        // draw the top point masses
        Gizmos.color = Color.red;
        for (int i = 0; i < 4; ++i)
        {
            Gizmos.DrawSphere(Positions[i], .1f);
        }

        // draw the bottom point masses
        Gizmos.color = Color.blue;
        for (int i = 4; i < Positions.Length; ++i)
        {
            Gizmos.DrawSphere(Positions[i], .1f);
        }

        // draw the springs used to form the faces of the point mass hull and draw
        // their normals
        for (int i = 0; i < FacePointIndexes.GetLength(0); ++i)
        {
            Vector3 a = Positions[FacePointIndexes[i, 0]];
            Vector3 b = Positions[FacePointIndexes[i, 1]];
            Vector3 c = Positions[FacePointIndexes[i, 2]];
            Vector3 d = Positions[FacePointIndexes[i, 3]];

            Gizmos.color = Color.white;
            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, d);
            Gizmos.DrawLine(d, a);

            Gizmos.color = Color.blue;
            a -= b;
            c -= b;
            b = Vector3.Cross(a, c).normalized;
            Gizmos.DrawLine(transform.position + b * HullHalfWidth,
                            transform.position + b * (HullHalfWidth + 1.0f));
        }

        // draw the springs that cross inside the hull and hull faces
        Gizmos.color = Color.yellow;
        for (int i = 0; i < InternalSpringPointIndexes.GetLength(0); ++i)
        {
            int pt0Index = InternalSpringPointIndexes[i, 0];
            int pt1Index = InternalSpringPointIndexes[i, 1];
            Gizmos.DrawLine(Positions[pt0Index], Positions[pt1Index]);
        }
    }
}
