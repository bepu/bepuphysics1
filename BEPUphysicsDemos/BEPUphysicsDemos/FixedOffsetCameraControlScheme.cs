using BEPUphysics.Entities;
using BEPUutilities;

namespace BEPUphysicsDemos
{
    public class FixedOffsetCameraControlScheme : CameraControlScheme
    {


        /// <summary>
        /// Gets or sets the offset from the position of the entity to the camera.
        /// </summary>
        public Vector3 CameraOffset { get; set; }


        /// <summary>
        /// Gets the character associated with the control scheme.
        /// </summary>
        public Entity Entity { get; private set; }

        /// <summary>
        /// Gets or sets whether or not to smooth the motion of the camera when the character moves discontinuously.
        /// </summary>
        public bool UseCameraSmoothing { get; set; }


        public FixedOffsetCameraControlScheme(Entity entity, Camera camera, DemosGame game)
            : base(camera, game)
        {
            Entity = entity;
            UseCameraSmoothing = true;
            CameraOffset = new Vector3(0, 0.7f, 0);
        }

        public override void Update(float dt)
        {
            base.Update(dt);

            ////Rotate the camera of the character based on the support velocity, if a support with velocity exists.
            ////This can be very disorienting in some cases; that's why it is off by default!
            //if (Character.SupportFinder.HasSupport)
            //{
            //    SupportData? data;
            //    if (Character.SupportFinder.HasTraction)
            //        data = Character.SupportFinder.TractionData;
            //    else
            //        data = Character.SupportFinder.SupportData;
            //    var support = data.Value.SupportObject as EntityCollidable;
            //    if (support != null && !support.Entity.IsDynamic) //Having the view turned by dynamic entities is extremely confusing for the most part.
            //    {
            //        float dot = Vector3.Dot(support.Entity.AngularVelocity, Character.Body.OrientationMatrix.Up);
            //        Camera.Yaw(dot * dt);
            //    }
            //}


            Camera.Position = Entity.Position + Matrix3x3.Transform(CameraOffset, Entity.OrientationMatrix);

        }
    }
}
