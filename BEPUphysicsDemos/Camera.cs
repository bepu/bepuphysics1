using System;
using BEPUutilities;
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Simple camera class for moving around the demos area.
    /// </summary>
    public class Camera
    {
        /// <summary>
        /// Gets or sets the position of the camera.
        /// </summary>
        public Vector3 Position { get; set; }


        /// <summary>
        /// Gets or sets the projection matrix of the camera.
        /// </summary>
        public Matrix ProjectionMatrix { get; set; }


        /// <summary>
        /// Gets the view matrix of the camera.
        /// </summary>
        public Matrix ViewMatrix
        {
            get { return Matrix.CreateViewRH(Position, viewDirection, lockedUp); }
        }

        /// <summary>
        /// Gets the world transformation of the camera.
        /// </summary>
        public Matrix WorldMatrix
        {
            get { return Matrix.CreateWorldRH(Position, viewDirection, lockedUp); }
        }

        private Vector3 viewDirection = Vector3.Forward;
        /// <summary>
        /// Gets or sets the view direction of the camera.
        /// </summary>
        public Vector3 ViewDirection
        {
            get { return viewDirection; }
            set
            {
                float lengthSquared = value.LengthSquared();
                if (lengthSquared > Toolbox.Epsilon)
                {
                    Vector3.Divide(ref value, (float) Math.Sqrt(lengthSquared), out value);
                    //Validate the input. A temporary violation of the maximum pitch is permitted as it will be fixed as the user looks around.
                    //However, we cannot allow a view direction parallel to the locked up direction.
                    float dot;
                    Vector3.Dot(ref value, ref lockedUp, out dot);
                    if (Math.Abs(dot) > 1 - Toolbox.BigEpsilon)
                    {
                        //The view direction must not be aligned with the locked up direction.
                        //Silently fail without changing the view direction.
                        return;
                    }
                    viewDirection = value;
                }
            }
        }

        private float maximumPitch = MathHelper.PiOver2 * 0.99f;
        /// <summary>
        /// Gets or sets how far the camera can look up or down in radians.
        /// </summary>
        public float MaximumPitch
        {
            get { return maximumPitch; }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Maximum pitch corresponds to pitch magnitude; must be positive.");
                if (value >= MathHelper.PiOver2)
                    throw new ArgumentException("Maximum pitch must be less than Pi/2.");
                maximumPitch = value;
            }
        }

        private Vector3 lockedUp = Vector3.Up;
        /// <summary>
        /// Gets or sets the current locked up vector of the camera.
        /// </summary>
        public Vector3 LockedUp
        {
            get { return lockedUp; }
            set
            {
                var oldUp = lockedUp;
                float lengthSquared = value.LengthSquared();
                if (lengthSquared > Toolbox.Epsilon)
                {
                    Vector3.Divide(ref value, (float)Math.Sqrt(lengthSquared), out lockedUp);
                    //Move the view direction with the transform. This helps guarantee that the view direction won't end up aligned with the up vector.
                    Quaternion rotation;
                    Quaternion.GetQuaternionBetweenNormalizedVectors(ref oldUp, ref lockedUp, out rotation);
                    Quaternion.Transform(ref viewDirection, ref rotation, out viewDirection);
                }
                //If the new up vector was a near-zero vector, silently fail without changing the up vector.
            }
        }


        /// <summary>
        /// Creates a camera.
        /// </summary>
        /// <param name="position">Initial position of the camera.</param>
        /// <param name="pitch">Initial pitch angle of the camera.</param>
        /// <param name="yaw">Initial yaw value of the camera.</param>
        /// <param name="projectionMatrix">Projection matrix used.</param>
        public Camera(Vector3 position, float pitch, float yaw, Matrix projectionMatrix)
        {
            Position = position;
            Yaw(yaw);
            Pitch(pitch);
            ProjectionMatrix = projectionMatrix;
        }



        /// <summary>
        /// Moves the camera forward.
        /// </summary>
        /// <param name="distance">Distance to move.</param>
        public void MoveForward(float distance)
        {
            Position += WorldMatrix.Forward * distance;
        }

        /// <summary>
        /// Moves the camera to the right.
        /// </summary>
        /// <param name="distance">Distance to move.</param>
        public void MoveRight(float distance)
        {
            Position += WorldMatrix.Right * distance;
        }

        /// <summary>
        /// Moves the camera up.
        /// </summary>
        /// <param name="distance">Distance to move.</param>
        public void MoveUp(float distance)
        {
            Position += new Vector3(0, distance, 0);
        }


        /// <summary>
        /// Rotates the camera around its locked up vector.
        /// </summary>
        /// <param name="radians">Amount to rotate.</param>
        public void Yaw(float radians)
        {
            //Rotate around the up vector.
            Matrix3x3 rotation;
            Matrix3x3.CreateFromAxisAngle(ref lockedUp, radians, out rotation);
            Matrix3x3.Transform(ref viewDirection, ref rotation, out viewDirection);

            //Avoid drift by renormalizing.
            viewDirection.Normalize();
        }

        /// <summary>
        /// Rotates the view direction up or down relative to the locked up vector.
        /// </summary>
        /// <param name="radians">Amount to rotate.</param>
        public void Pitch(float radians)
        {
            //Do not allow the new view direction to violate the maximum pitch.
            float dot;
            Vector3.Dot(ref viewDirection, ref lockedUp, out dot);

            //While this could be rephrased in terms of dot products alone, converting to actual angles can be more intuitive.
            //Consider +Pi/2 to be up, and -Pi/2 to be down.
            float currentPitch = (float)Math.Acos(MathHelper.Clamp(-dot, -1, 1)) - MathHelper.PiOver2;
            //Compute our new pitch by clamping the current + change.
            float newPitch = MathHelper.Clamp(currentPitch + radians, -maximumPitch, maximumPitch);
            float allowedChange = newPitch - currentPitch;

            //Compute and apply the rotation.
            Vector3 pitchAxis;
            Vector3.Cross(ref viewDirection, ref lockedUp, out pitchAxis);
            //This is guaranteed safe by all interaction points stopping viewDirection from being aligned with lockedUp.
            pitchAxis.Normalize();
            Matrix3x3 rotation;
            Matrix3x3.CreateFromAxisAngle(ref pitchAxis, allowedChange, out rotation);
            Matrix3x3.Transform(ref viewDirection, ref rotation, out viewDirection);

            //Avoid drift by renormalizing.
            viewDirection.Normalize();
        }

    }
}