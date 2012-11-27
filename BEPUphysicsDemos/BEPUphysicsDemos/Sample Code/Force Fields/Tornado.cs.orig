
using BEPUphysics;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems.ForceFields;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Force field simulating a tornado with circular a 'wind' force, an inward pointing force, and an upward pointing force.
    /// </summary>
    public class Tornado : ForceField
    {
        /// <summary>
        /// Axis of rotation of the tornado.
        /// </summary>
        public Vector3 Axis;

        /// <summary>
        /// Radius of the tornado at the bottom.
        /// </summary>
        public float BottomRadius;

        /// <summary>
        /// Height of the tornado; objects above or below the tornado will not be affected by its winds.
        /// </summary>
        public float Height;

        /// <summary>
        /// Circular force applied within the tornado.  Force magnitude decreases as distance from axis increases past the radius.
        /// </summary>
        public float HorizontalForce;

        /// <summary>
        /// Maximum horizontal wind speed of the tornado; objects will not be accelerated by the wind past this speed in the direction of the wind.
        /// </summary>
        public float HorizontalWindSpeed;

        /// <summary>
        /// Magnitude of the inward-sucking force within the tornado.  Magnitude decreases as distance from the axis increases past the radius.
        /// </summary>
        public float InwardForce;

        /// <summary>
        /// Maximum inward sucking wind speed; objects will not be accelerated by the wind past this speed inward.
        /// </summary>
        public float InwardSuctionSpeed;

        /// <summary>
        /// Spin direction of the tornado.  Looking down from the top of the tornado (the furthest forward along the tornado axis).
        /// </summary>
        public bool SpinClockwise = true;

        /// <summary>
        /// Radius of the tornado at the top.
        /// </summary>
        public float TopRadius;

        /// <summary>
        /// Magnitude of upward-pushing force within the tornado.  Magnitude decreases as distance from the axis increases past the radius.
        /// </summary>
        public float UpwardForce;

        /// <summary>
        /// Maximum upward pushing wind speed; objects will not be accelerated by the wind past this speed upward.
        /// </summary>
        public float UpwardSuctionSpeed;

        /// <summary>
        /// Creates a simple, constant force field.
        /// </summary>
        /// <param name="shape">Shape representing the tornado-affected volume.</param>
        /// <param name="position">Position of the tornado.</param>
        /// <param name="axis">Axis of rotation of the tornado.</param>
        /// <param name="height">Height of the tornado; objects above or below the tornado will not be affected by its winds.</param>
        /// <param name="spinClockwise">Whether or not the tornado's rotation is clockwise.</param>
        /// <param name="horizontalWindSpeed">Maximum tangential wind speed; objects will not be accelerated by the wind past this speed sideways.</param>
        /// <param name="upwardSuctionSpeed">Maximum upward pushing wind speed; objects will not be accelerated by the wind past this speed upward.</param>
        /// <param name="inwardSuctionSpeed">Maximum inward sucking wind speed; objects will not be accelerated by the wind past this speed inward.</param>
        /// <param name="horizontalForce">Circular force applied within the tornado.  Force magnitude decreases as distance from axis increases past the radius.</param>
        /// <param name="upwardForce">Magnitude of upward-pushing force within the tornado.  Magnitude decreases as distance from the axis increases past the radius.</param>
        /// <param name="inwardForce">Magnitude of the inward-sucking force within the tornado.  Magnitude decreases as distance from the axis increases past the radius.</param>
        /// <param name="topRadius">Radius of the tornado at the top.</param>
        /// <param name="bottomRadius">Radius of the tornado at the bottom.</param>
        public Tornado(ForceFieldShape shape, Vector3 position, Vector3 axis,
                       float height, bool spinClockwise, float horizontalWindSpeed,
                       float upwardSuctionSpeed, float inwardSuctionSpeed,
                       float horizontalForce, float upwardForce, float inwardForce,
                       float topRadius, float bottomRadius, IQueryAccelerator accelerator)
            : base(shape, accelerator)
        {
            Axis = Vector3.Normalize(axis);
            Position = position;
            Height = height;
            SpinClockwise = spinClockwise;
            HorizontalWindSpeed = horizontalWindSpeed;
            UpwardSuctionSpeed = upwardSuctionSpeed;
            InwardSuctionSpeed = inwardSuctionSpeed;
            HorizontalForce = horizontalForce;
            UpwardForce = upwardForce;
            InwardForce = inwardForce;
            BottomRadius = bottomRadius;
            TopRadius = topRadius;
        }

        /// <summary>
        /// Gets or sets the position of the tornado.  This is only the origin of the force; move the shape along with it if the position moves away from the shape.
        /// </summary>
        public Vector3 Position { get; set; }

        /// <summary>
        /// Calculates the impulse to apply to the center of mass of physically simulated bodies within the volume.
        /// </summary>
        /// <param name="e">Target of the impulse.</param>
        /// <param name="dt">Time since the last frame in simulation seconds.</param>
        /// <param name="impulse">Force to apply at the given position.</param>
        protected override void CalculateImpulse(Entity e, float dt, out Vector3 impulse)
        {
            Vector3 position = Position; //Referenced a lot, and passed using ref parameter.
            Vector3 entityPosition = e.Position;

            float entityHeight = Vector3.Dot(Axis, entityPosition - position + Axis * (Height / 2));
            if (entityHeight < 0 || entityHeight > Height)
                impulse = Toolbox.ZeroVector;
            else
            {
                float tornadoRadius = BottomRadius * (1 - entityHeight / Height) + TopRadius * (entityHeight / Height);
                Vector3 closestPoint;
                Vector3 endpointA = position + Axis * Height / 2;
                Vector3 endpointB = position - Axis * Height / 2;
                Toolbox.GetClosestPointOnSegmentToPoint(ref endpointA, ref endpointB, ref entityPosition, out closestPoint);
                float entityDistanceFromTornado;
                Vector3.Distance(ref entityPosition, ref closestPoint, out entityDistanceFromTornado);
                //Compute the axis to the 
                Vector3 posClosest;
                float forceMultiplier;
                if (entityDistanceFromTornado > tornadoRadius)
                {
                    //outside tornado
                    forceMultiplier = tornadoRadius / entityDistanceFromTornado;
                    posClosest = (closestPoint - entityPosition) / entityDistanceFromTornado;
                }
                else if (entityDistanceFromTornado > Toolbox.Epsilon)
                {
                    //inside tornado
                    forceMultiplier = .5f + .5f * entityDistanceFromTornado / tornadoRadius;
                    posClosest = (closestPoint - entityPosition) / entityDistanceFromTornado;
                }
                else
                {
                    forceMultiplier = .5f;
                    posClosest = Toolbox.ZeroVector;
                }

                Vector3 tangentialForceVector;
                //Don't need to normalize the direction.  
                //Axis and posClosest are perpendicular and each normal, so the result is normal.
                Vector3 tangentDirection;

                if (SpinClockwise)
                    Vector3.Cross(ref Axis, ref posClosest, out tangentDirection);
                else
                    Vector3.Cross(ref posClosest, ref Axis, out tangentDirection);

                //Current velocity along the tangent direction.
                float dot = Vector3.Dot(e.LinearVelocity, tangentDirection);
                //Compute the velocity difference between the current and the maximum
                dot = HorizontalWindSpeed - dot;
                //Compute the force needed to reach the maximum, but clamp it to the amount of force that the tornado can apply
                dot = MathHelper.Clamp(dot * e.Mass, 0, HorizontalForce * dt);
                Vector3.Multiply(ref tangentDirection, dot, out tangentialForceVector);

                //Do a similar process for the other tornado force axes.
                Vector3 upwardForceVector;
                dot = Vector3.Dot(e.LinearVelocity, Axis);
                dot = UpwardSuctionSpeed - dot;
                dot = MathHelper.Clamp(dot * e.Mass, 0, UpwardForce * dt);
                Vector3.Multiply(ref Axis, dot, out upwardForceVector);

                Vector3 inwardForceVector;
                dot = Vector3.Dot(e.LinearVelocity, posClosest);
                dot = InwardSuctionSpeed - dot;
                dot = MathHelper.Clamp(dot * e.Mass, 0, InwardForce * dt);
                Vector3.Multiply(ref posClosest, dot, out inwardForceVector);

                //if (posClosest.X > 0)
                //    Debug.WriteLine("Break.");
                impulse = forceMultiplier * (tangentialForceVector + upwardForceVector + inwardForceVector);
            }
        }
    }
}