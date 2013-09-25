using System;
using BEPUphysics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Entities;
using BEPUphysics.Vehicle;
using BEPUutilities;

namespace BEPUphysicsDemos
{
    public class ChaseCameraControlScheme : CameraControlScheme
    {
        /// <summary>
        /// Entity to follow around and point at.
        /// </summary>
        public Entity ChasedEntity { get; set; }

        /// <summary>
        /// Gets or sets the offset vector from the center of the target chase entity to look at.
        /// </summary>
        public Vector3 OffsetFromChaseTarget { get; set; }

        /// <summary>
        /// Gets or sets whether or not to transform the offset vector with the rotation of the entity.
        /// </summary>
        private bool TransformOffset { get; set; }

        /// <summary>
        /// Gets or sets the distance away from the target entity to try to maintain.  The distance will be shorter at times if the ray hits an object.
        /// </summary>
        public float DistanceToTarget { get; set; }
        
        /// <summary>
        /// Gets or sets the margin of the camera. The camera will be placed no closer to any obstacle than this margin along the ray cast.
        /// </summary>
        public float ChaseCameraMargin { get; set; }

        //The raycast filter limits the results retrieved from the Space.RayCast while in chase camera mode.
        Func<BroadPhaseEntry, bool> rayCastFilter;
        bool RayCastFilter(BroadPhaseEntry entry)
        {
            return entry != ChasedEntity.CollisionInformation && (entry.CollisionRules.Personal <= CollisionRule.Normal);
        }

        /// <summary>
        /// Sets up all the information required by the chase camera.
        /// </summary>
        /// <param name="chasedEntity">Target to follow.</param>
        /// <param name="offsetFromChaseTarget">Offset from the center of the entity target to point at.</param>
        /// <param name="transformOffset">Whether or not to transform the offset with the target entity's rotation.</param>
        /// <param name="distanceToTarget">Distance from the target position to try to maintain.</param>
        /// <param name="camera">Camera controlled by the scheme.</param>
        /// <param name="game">Running game.</param>
        public ChaseCameraControlScheme(Entity chasedEntity, Vector3 offsetFromChaseTarget, bool transformOffset, float distanceToTarget, Camera camera, DemosGame game)
            : base(camera, game)
        {
            ChasedEntity = chasedEntity;
            OffsetFromChaseTarget = offsetFromChaseTarget;
            TransformOffset = transformOffset;
            DistanceToTarget = distanceToTarget;
            ChaseCameraMargin = 1;

            rayCastFilter = RayCastFilter;
        }

        public override void Update(float dt)
        {
            base.Update(dt);

            Vector3 offset = TransformOffset ? Matrix3x3.Transform(OffsetFromChaseTarget, ChasedEntity.BufferedStates.InterpolatedStates.OrientationMatrix) : OffsetFromChaseTarget;
            Vector3 lookAt = ChasedEntity.BufferedStates.InterpolatedStates.Position + offset;
            Vector3 backwards = -Camera.ViewDirection;

            //Find the earliest ray hit that isn't the chase target to position the camera appropriately.
            RayCastResult result;
            float cameraDistance = ChasedEntity.Space.RayCast(new Ray(lookAt, backwards), DistanceToTarget, rayCastFilter, out result) ? result.HitData.T : DistanceToTarget;

            Camera.Position = lookAt + (Math.Max(cameraDistance - ChaseCameraMargin, 0)) * backwards; //Put the camera just before any hit spot.


        }
    }
}
