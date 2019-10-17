using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic point constraint between two rigid bodies.
/// </summary>
public class PointConstraint : MonoBehaviour, IConstraint
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public PointConstraint()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Stiffness;

    public RigidBody bodyA;
    public RigidBody bodyB;

    #endregion

    #region OtherVariables

    int index;
    private PhysicsManager Manager;

    protected Vector3 pointA;
    protected Vector3 pointB;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Compute the average position
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        Vector3 pos = 0.5f * (posA + posB);

        // Apply the position
        Transform xform = GetComponent<Transform>();
        xform.position = pos;
    }

    #endregion

    #region IConstraint

    public void Initialize(int ind, PhysicsManager m)
    {
        index = ind;
        Manager = m;

        // Initialize local positions. We assume that the object is connected to a Sphere mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the constraint");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the constraint");
        }

        // Initialize kinematics
        Vector3 pos = xform.position;

        // Local positions on objects
        pointA = (bodyA != null) ? bodyA.PointGlobalToLocal(pos) : pos;
        pointB = (bodyB != null) ? bodyB.PointGlobalToLocal(pos) : pos;

    }

    public int GetNumConstraints()
    {
        return 3;
    }

    public void GetConstraints(VectorXD c)
    {
        // TO BE COMPLETED
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        // TO BE COMPLETED
    }

    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED
        Vector3 posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
        Vector3 posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;

        /////////////////////////////////////////////////////
        ///Calculo de C
        float cx;
        float cy;
        float cz;

        cx = (posA[0] - posB[0]);
        cy = (posA[1] - posB[1]);
        cz = (posA[2] - posB[2]);

        VectorXD c = Utils.ToVectorXD(new Vector3(cx, cy, cz)); //|xa - xb|
        /////////////////////////////////////////////////////

        /////////////////////////////////////////////////////
        ///Calculo de GradC y F
        MatrixXD identity = DenseMatrixXD.CreateIdentity(3);
        MatrixXD dcdax = MatrixXD.Build.Dense(3, 3);
        MatrixXD dcdaTheta = MatrixXD.Build.Dense(3, 3);
        MatrixXD dcdbx = MatrixXD.Build.Dense(3, 3);
        MatrixXD dcdbTheta = MatrixXD.Build.Dense(3, 3);

        if (bodyA != null)
        {
            dcdax = identity;
            dcdaTheta = -Utils.Skew(posA - bodyA.m_pos);

            VectorXD fax = -Stiffness * dcdax.Transpose() * c;
            VectorXD faTheta = -Stiffness * dcdaTheta.Transpose() * c;

            force.SetSubVector(bodyA.index, 3, force.SubVector(bodyA.index, 3) + fax);
            force.SetSubVector(bodyA.index + 3, 3, force.SubVector(bodyA.index + 3, 3) + faTheta);
        }
        if (bodyB != null)
        {
            dcdbx = -identity;
            dcdbTheta = Utils.Skew(posB - bodyB.m_pos);

            VectorXD fbx = -Stiffness * dcdbx.Transpose() * c;
            VectorXD fbTheta = -Stiffness * dcdbTheta.Transpose() * c;

            force.SetSubVector(bodyB.index, 3, force.SubVector(bodyB.index, 3) + fbx);
            force.SetSubVector(bodyB.index + 3, 3, force.SubVector(bodyB.index + 3, 3) + fbTheta);
        }
		/////////////////////////////////////////////////////
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
    }

    #endregion

}
