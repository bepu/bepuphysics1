using System;
using BEPUphysics.Entities;

using BEPUutilities;

namespace BEPUphysics.Constraints.TwoEntity.JointLimits
{
    /// <summary>
    /// Constrains the relative orientation of two entities to within an ellipse.
    /// </summary>
    public class EllipseSwingLimit : JointLimit, I1DImpulseConstraintWithError, I1DJacobianConstraint
    {
        private readonly JointBasis3D basis = new JointBasis3D();
        private float accumulatedImpulse;
        private float biasVelocity;
        private Vector3 jacobianA, jacobianB;
        private float error;

        private Vector3 localTwistAxisB;
        private float maximumAngleX;
        private float maximumAngleY;
        private Vector3 worldTwistAxisB;
        private float velocityToImpulse;

        /// <summary>
        /// Constructs a new swing limit.
        /// To finish the initialization, specify the connections (ConnectionA and ConnectionB) 
        /// as well as the TwistAxis (or its entity-local version),
        /// the MaximumAngleX and MaximumAngleY,
        /// and the Basis.
        /// This constructor sets the constraint's IsActive property to false by default.
        /// </summary>
        public EllipseSwingLimit()
        {
            IsActive = false;
        }

        /// <summary>
        /// Constructs a new swing limit.
        /// </summary>
        /// <param name="connectionA">First entity connected by the constraint.</param>
        /// <param name="connectionB">Second entity connected by the constraint.</param>
        /// <param name="twistAxis">Axis in world space to use as the initial unrestricted twist direction.
        /// This direction will be transformed to entity A's local space to form the basis's primary axis
        /// and to entity B's local space to form its twist axis.
        /// The basis's x and y axis are automatically created from the twist axis.</param>
        /// <param name="maximumAngleX">Maximum angle of rotation around the basis X axis.</param>
        /// <param name="maximumAngleY">Maximum angle of rotation around the basis Y axis.</param>
        public EllipseSwingLimit(Entity connectionA, Entity connectionB, Vector3 twistAxis, float maximumAngleX, float maximumAngleY)
        {
            ConnectionA = connectionA;
            ConnectionB = connectionB;
            SetupJointTransforms(twistAxis);
            MaximumAngleX = maximumAngleX;
            MaximumAngleY = maximumAngleY;
        }

        /// <summary>
        /// Constructs a new swing limit.
        /// Using this constructor will leave the limit uninitialized.  Before using the limit in a simulation, be sure to set the basis axes using
        /// limit.basis.setLocalAxes or limit.basis.setWorldAxes and b's twist axis using the localTwistAxisB or twistAxisB properties.
        /// </summary>
        /// <param name="connectionA">First entity connected by the constraint.</param>
        /// <param name="connectionB">Second entity connected by the constraint.</param>
        public EllipseSwingLimit(Entity connectionA, Entity connectionB)
        {
            ConnectionA = connectionA;
            ConnectionB = connectionB;
        }

        /// <summary>
        /// Gets the basis attached to entity A.
        /// The primary axis is the "twist" axis attached to entity A.
        /// The xAxis is the axis around which the angle will be limited by maximumAngleX.
        /// Similarly, the yAxis is the axis around which the angle will be limited by maximumAngleY.
        /// </summary>
        public JointBasis3D Basis
        {
            get { return basis; }
        }

        /// <summary>
        /// Gets or sets the twist axis attached to entity B in its local space.
        /// The transformed twist axis will be used to determine the angles around entity A's basis axes.
        /// </summary>
        public Vector3 LocalTwistAxisB
        {
            get { return localTwistAxisB; }
            set
            {
                localTwistAxisB = value;
                Matrix3x3.Transform(ref localTwistAxisB, ref connectionB.orientationMatrix, out worldTwistAxisB);
            }
        }

        /// <summary>
        /// Gets or sets the maximum angle of rotation around the x axis.
        /// This can be thought of as the major radius of the swing limit's ellipse.
        /// </summary>
        public float MaximumAngleX
        {
            get { return maximumAngleX; }
            set { maximumAngleX = MathHelper.Clamp(value, Toolbox.BigEpsilon, MathHelper.Pi); }
        }

        /// <summary>
        /// Gets or sets the maximum angle of rotation around the y axis.
        /// This can be thought of as the minor radius of the swing limit's ellipse.
        /// </summary>
        public float MaximumAngleY
        {
            get { return maximumAngleY; }
            set { maximumAngleY = MathHelper.Clamp(value, Toolbox.BigEpsilon, MathHelper.Pi); }
        }

        /// <summary>
        /// Gets or sets the twist axis attached to entity B in world space.
        /// The transformed twist axis will be used to determine the angles around entity A's basis axes.
        /// </summary>
        public Vector3 TwistAxisB
        {
            get { return worldTwistAxisB; }
            set
            {
                worldTwistAxisB = value;
                Matrix3x3.TransformTranspose(ref worldTwistAxisB, ref connectionB.orientationMatrix, out localTwistAxisB);
            }
        }

        #region I1DImpulseConstraintWithError Members

        /// <summary>
        /// Gets the current relative velocity between the connected entities with respect to the constraint.
        /// </summary>
        public float RelativeVelocity
        {
            get
            {
                if (isLimitActive)
                {
                    float velocityA, velocityB;
                    Vector3.Dot(ref connectionA.angularVelocity, ref jacobianA, out velocityA);
                    Vector3.Dot(ref connectionB.angularVelocity, ref jacobianB, out velocityB);
                    return velocityA + velocityB;
                }
                return 0;
            }
        }


        /// <summary>
        /// Gets the total impulse applied by this constraint.
        /// </summary>
        public float TotalImpulse
        {
            get { return accumulatedImpulse; }
        }

        /// <summary>
        /// Gets the current constraint error.
        /// </summary>
        public float Error
        {
            get { return error; }
        }

        #endregion

        #region I1DJacobianConstraint Members

        /// <summary>
        /// Gets the linear jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobian">Linear jacobian entry for the first connected entity.</param>
        public void GetLinearJacobianA(out Vector3 jacobian)
        {
            jacobian = Toolbox.ZeroVector;
        }

        /// <summary>
        /// Gets the linear jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobian">Linear jacobian entry for the second connected entity.</param>
        public void GetLinearJacobianB(out Vector3 jacobian)
        {
            jacobian = Toolbox.ZeroVector;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobian">Angular jacobian entry for the first connected entity.</param>
        public void GetAngularJacobianA(out Vector3 jacobian)
        {
            jacobian = jacobianA;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobian">Angular jacobian entry for the second connected entity.</param>
        public void GetAngularJacobianB(out Vector3 jacobian)
        {
            jacobian = jacobianB;
        }

        /// <summary>
        /// Gets the mass matrix of the constraint.
        /// </summary>
        /// <param name="outputMassMatrix">Constraint's mass matrix.</param>
        public void GetMassMatrix(out float outputMassMatrix)
        {
            outputMassMatrix = velocityToImpulse;
        }

        #endregion

        /// <summary>
        /// Sets up the joint transforms by automatically creating perpendicular vectors to complete the bases.
        /// </summary>
        /// <param name="twistAxis">Axis around which rotation is allowed.</param>
        public void SetupJointTransforms(Vector3 twistAxis)
        {
            //Compute a vector which is perpendicular to the axis.  It'll be added in local space to both connections.
            Vector3 xAxis;
            Vector3.Cross(ref twistAxis, ref Toolbox.UpVector, out xAxis);
            float length = xAxis.LengthSquared();
            if (length < Toolbox.Epsilon)
            {
                Vector3.Cross(ref twistAxis, ref Toolbox.RightVector, out xAxis);
            }

            Vector3 yAxis;
            Vector3.Cross(ref twistAxis, ref xAxis, out yAxis);

            //Put the axes into the joint transform of A.
            basis.rotationMatrix = connectionA.orientationMatrix;
            basis.SetWorldAxes(twistAxis, xAxis, yAxis);


            //Put the axes into the 'joint transform' of B too.
            TwistAxisB = twistAxis;
        }


        ///<summary>
        /// Performs the frame's configuration step.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(float dt)
        {
            //Transform the axes into world space.
            basis.rotationMatrix = connectionA.orientationMatrix;
            basis.ComputeWorldSpaceAxes();
            Matrix3x3.Transform(ref localTwistAxisB, ref connectionB.orientationMatrix, out worldTwistAxisB);

            //Compute the individual swing angles.
            Quaternion relativeRotation;
            Quaternion.GetQuaternionBetweenNormalizedVectors(ref worldTwistAxisB, ref basis.primaryAxis, out relativeRotation);
            Vector3 axis;
            float angle;
            Quaternion.GetAxisAngleFromQuaternion(ref relativeRotation, out axis, out angle);

#if !WINDOWS
            Vector3 axisAngle = new Vector3();
#else
            Vector3 axisAngle;
#endif
            //This combined axis-angle representation is similar to angular velocity in describing a rotation.
            //Just like you can dot an axis with angular velocity to get a velocity around that axis,
            //dotting an axis with the axis-angle representation gets the angle of rotation around that axis.
            //(As far as the constraint is concerned, anyway.)
            axisAngle.X = axis.X * angle;
            axisAngle.Y = axis.Y * angle;
            axisAngle.Z = axis.Z * angle;

            float angleX;
            Vector3.Dot(ref axisAngle, ref basis.xAxis, out angleX);
            float angleY;
            Vector3.Dot(ref axisAngle, ref basis.yAxis, out angleY);


            //The position constraint states that the angles must be within an ellipse. The following is just a reorganization of the x^2 / a^2 + y^2 / b^2 <= 1 definition of an ellipse's area.
            float maxAngleXSquared = maximumAngleX * maximumAngleX;
            float maxAngleYSquared = maximumAngleY * maximumAngleY;
            error = angleX * angleX * maxAngleYSquared + angleY * angleY * maxAngleXSquared - maxAngleXSquared * maxAngleYSquared;

            if (error < 0)
            {
                isActiveInSolver = false;
                error = 0;
                accumulatedImpulse = 0;
                isLimitActive = false;
                return;
            }
            isLimitActive = true;


            //Derive the position constraint with respect to time to get the velocity constraint.
            //d/dt(x^2 / a^2 + y^2 / b^2) <= d/dt(1)
            //(2x / a^2) * d/dt(x) + (2y / b^2) * d/dt(y) <= 0
            //d/dt(x) is dot(angularVelocity, xAxis).
            //d/dt(y) is dot(angularVelocity, yAxis).
            //By the scalar multiplication properties of dot products, this can be written as:
            //dot((2x / a^2) * xAxis, angularVelocity) + dot((2y / b^2) * yAxis, angularVelocity) <= 0
            //And by the distribute property, rewrite it as:
            //dot((2x / a^2) * xAxis + (2y / b^2) * yAxis, angularVelocity) <= 0
            //So, by inspection, the jacobian is:
            //(2x / a^2) * xAxis + (2y / b^2) * yAxis

            //[some handwaving in the above: 'angularVelocity' is actually the angular velocities of the involved entities combined.
            //Splitting it out fully would reveal two dot products with equivalent but negated jacobians.]

            //The jacobian is implemented by first considering the local values (2x / a^2) and (2y / b^2).
#if !WINDOWS
            Vector2 tangent = new Vector2();
#else
            Vector2 tangent;
#endif
            tangent.X = 2 * angleX / maxAngleXSquared;
            tangent.Y = 2 * angleY / maxAngleYSquared;

            //The tangent is then taken into world space using the basis.

            //Create a rotation which swings our basis 'out' to b's world orientation.
            Quaternion.Conjugate(ref relativeRotation, out relativeRotation);
            Vector3 sphereTangentX, sphereTangentY;
            Quaternion.Transform(ref basis.xAxis, ref relativeRotation, out sphereTangentX);
            Quaternion.Transform(ref basis.yAxis, ref relativeRotation, out sphereTangentY);

            Vector3.Multiply(ref sphereTangentX, tangent.X, out jacobianA); //not actually jA, just storing it there.
            Vector3.Multiply(ref sphereTangentY, tangent.Y, out jacobianB); //not actually jB, just storing it there.
            Vector3.Add(ref jacobianA, ref jacobianB, out jacobianA);

            jacobianB.X = -jacobianA.X;
            jacobianB.Y = -jacobianA.Y;
            jacobianB.Z = -jacobianA.Z;


            float errorReduction;
            float inverseDt = 1 / dt;
            springSettings.ComputeErrorReductionAndSoftness(dt, inverseDt, out errorReduction, out softness);

            //Compute the error correcting velocity
            error = error - margin;
            biasVelocity = MathHelper.Min(Math.Max(error, 0) * errorReduction, maxCorrectiveVelocity);


            if (bounciness > 0)
            {
                float relativeVelocity;
                float dot;
                //Find the velocity contribution from each connection
                Vector3.Dot(ref connectionA.angularVelocity, ref jacobianA, out relativeVelocity);
                Vector3.Dot(ref connectionB.angularVelocity, ref jacobianB, out dot);
                relativeVelocity += dot;
                biasVelocity = MathHelper.Max(biasVelocity, ComputeBounceVelocity(relativeVelocity));

            }



            //****** EFFECTIVE MASS MATRIX ******//
            //Connection A's contribution to the mass matrix
            float entryA;
            Vector3 transformedAxis;
            if (connectionA.isDynamic)
            {
                Matrix3x3.Transform(ref jacobianA, ref connectionA.inertiaTensorInverse, out transformedAxis);
                Vector3.Dot(ref transformedAxis, ref jacobianA, out entryA);
            }
            else
                entryA = 0;

            //Connection B's contribution to the mass matrix
            float entryB;
            if (connectionB.isDynamic)
            {
                Matrix3x3.Transform(ref jacobianB, ref connectionB.inertiaTensorInverse, out transformedAxis);
                Vector3.Dot(ref transformedAxis, ref jacobianB, out entryB);
            }
            else
                entryB = 0;

            //Compute the inverse mass matrix
            velocityToImpulse = 1 / (softness + entryA + entryB);


        }


        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public override void ExclusiveUpdate()
        {
            //****** WARM STARTING ******//
            //Apply accumulated impulse
            Vector3 impulse;
            if (connectionA.isDynamic)
            {
                Vector3.Multiply(ref jacobianA, accumulatedImpulse, out impulse);
                connectionA.ApplyAngularImpulse(ref impulse);
            }
            if (connectionB.isDynamic)
            {
                Vector3.Multiply(ref jacobianB, accumulatedImpulse, out impulse);
                connectionB.ApplyAngularImpulse(ref impulse);
            }
        }

        /// <summary>
        /// Computes one iteration of the constraint to meet the solver updateable's goal.
        /// </summary>
        /// <returns>The rough applied impulse magnitude.</returns>
        public override float SolveIteration()
        {
            float velocityA, velocityB;
            //Find the velocity contribution from each connection
            Vector3.Dot(ref connectionA.angularVelocity, ref jacobianA, out velocityA);
            Vector3.Dot(ref connectionB.angularVelocity, ref jacobianB, out velocityB);
            //Add in the constraint space bias velocity
            float lambda = (-velocityA - velocityB) - biasVelocity - softness * accumulatedImpulse;

            //Transform to an impulse
            lambda *= velocityToImpulse;

            //Clamp accumulated impulse (can't go negative)
            float previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = MathHelper.Min(accumulatedImpulse + lambda, 0);
            lambda = accumulatedImpulse - previousAccumulatedImpulse;

            //Apply the impulse
            Vector3 impulse;
            if (connectionA.isDynamic)
            {
                Vector3.Multiply(ref jacobianA, lambda, out impulse);
                connectionA.ApplyAngularImpulse(ref impulse);
            }
            if (connectionB.isDynamic)
            {
                Vector3.Multiply(ref jacobianB, lambda, out impulse);
                connectionB.ApplyAngularImpulse(ref impulse);
            }

            return (Math.Abs(lambda));
        }

    }
}