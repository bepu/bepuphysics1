using System;
using System.Collections.Generic;

namespace BEPUphysics.Materials
{
    ///<summary>
    /// Manages the relationship between materials.
    ///</summary>
    public static class MaterialManager
    {
        /// <summary>
        /// Determines how to factor together friction values from two objects in a collision.
        /// Defaults to using the maximum bounciness of the pair.
        /// </summary>
        public static PropertyBlendMethod BouncinessBlendMethod = PropertyBlendMethod.Max;

        /// <summary>
        /// The property blender used by default in collision bounciness.
        /// Uses the space's bounciness PropertyBlendMethod to blend.
        /// </summary>
        public static PropertyBlender BouncinessBlender;

        /// <summary>
        /// Determines how to factor together bounciness values from two objects in a collision.
        /// Defaults to using the average friction of the pair.
        /// </summary>
        public static PropertyBlendMethod FrictionBlendMethod = PropertyBlendMethod.Average;

        /// <summary>
        /// The property blender used by default in collision friction.
        /// Uses the space's friction PropertyBlendMethod to blend.
        /// </summary>
        public static PropertyBlender FrictionBlender;

        ///<summary>
        /// Default coefficient of kinetic friction.
        ///</summary>
        public static float DefaultKineticFriction = .6f;
        ///<summary>
        /// Default coefficient of static friction.
        ///</summary>
        public static float DefaultStaticFriction = .8f;
        ///<summary>
        /// Default coefficient of restitution.
        ///</summary>
        public static float DefaultBounciness;

        static MaterialManager()
        {
            FrictionBlender = DefaultFrictionBlender;
            BouncinessBlender = DefaultBouncinessBlender;
            MaterialInteractions = new Dictionary<MaterialPair, InteractionProperties>();
        }

        ///<summary>
        /// Computes the interaction properties between two materials.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        ///<param name="properties">Interaction properties between two materials.</param>
        public static void GetInteractionProperties(Material materialA, Material materialB, out InteractionProperties properties)
        {
            if (MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                return;
            }
            properties = new InteractionProperties();
            properties.StaticFriction = FrictionBlender(materialA.staticFriction, materialB.staticFriction, null);
            properties.KineticFriction = FrictionBlender(materialA.kineticFriction, materialB.kineticFriction, null);
            properties.Bounciness = BouncinessBlender(materialA.bounciness, materialB.bounciness, null);

        }

        ///<summary>
        /// Blends the static friction of the two materials together.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        ///<param name="blendedCoefficient">Blended friction coefficient.</param>
        public static void GetStaticFriction(Material materialA, Material materialB, out float blendedCoefficient)
        {
            InteractionProperties properties;
            if (materialA != null && materialB != null &&
                MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                blendedCoefficient = properties.StaticFriction;
                return;
            }

            blendedCoefficient = FrictionBlender(materialA.staticFriction, materialB.staticFriction, null);

        }


        ///<summary>
        /// Blends the kinetic friction of the two materials together.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        ///<param name="blendedCoefficient">Blended friction coefficient.</param>
        public static void GetKineticFriction(Material materialA, Material materialB, out float blendedCoefficient)
        {
            InteractionProperties properties;
            if (materialA != null && materialB != null &&
                MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                blendedCoefficient = properties.KineticFriction;
                return;
            }

            blendedCoefficient = FrictionBlender(materialA.KineticFriction, materialB.KineticFriction, null);

        }


        ///<summary>
        /// Blends the bounciness of the two materials together.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        ///<param name="blendedCoefficient">Blended bounciness.</param>
        public static void GetBounciness(Material materialA, Material materialB, out float blendedCoefficient)
        {
            InteractionProperties properties;
            if (materialA != null && materialB != null &&
                MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                blendedCoefficient = properties.Bounciness;
                return;
            }

            blendedCoefficient = BouncinessBlender(materialA.Bounciness, materialB.Bounciness, null);

        }

        ///<summary>
        /// Blends the static friction of the two materials together.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        /// <param name="blender">Blender to use to blend the material properties.</param>
        ///<param name="blendedCoefficient">Blended friction coefficient.</param>
        public static void GetStaticFriction(Material materialA, Material materialB, PropertyBlender blender, out float blendedCoefficient)
        {
            InteractionProperties properties;
            if (materialA != null && materialB != null &&
                MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                blendedCoefficient = properties.StaticFriction;
                return;
            }

            blendedCoefficient = blender(materialA.staticFriction, materialB.staticFriction, null);

        }

        ///<summary>
        /// Blends the kinetic friction of the two materials together.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        /// <param name="blender">Blender to use to blend the material properties.</param>
        ///<param name="blendedCoefficient">Blended friction coefficient.</param>
        public static void GetKineticFriction(Material materialA, Material materialB, PropertyBlender blender, out float blendedCoefficient)
        {
            InteractionProperties properties;
            if (materialA != null && materialB != null &&
                MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                blendedCoefficient = properties.KineticFriction;
                return;
            }

            blendedCoefficient = blender(materialA.KineticFriction, materialB.KineticFriction, null);

        }

        ///<summary>
        /// Blends the bounciness of the two materials together.
        ///</summary>
        ///<param name="materialA">First material of the pair.</param>
        ///<param name="materialB">Second material of the pair.</param>
        /// <param name="blender">Blender to use to blend the material properties.</param>
        ///<param name="blendedCoefficient">Blended bounciness coefficient.</param>
        public static void GetBounciness(Material materialA, Material materialB, PropertyBlender blender, out float blendedCoefficient)
        {
            InteractionProperties properties;
            if (materialA != null && materialB != null &&
                MaterialInteractions.TryGetValue(new MaterialPair(materialA, materialB), out properties))
            {
                blendedCoefficient = properties.Bounciness;
                return;
            }

            blendedCoefficient = blender(materialA.Bounciness, materialB.Bounciness, null);

        }

        ///<summary>
        /// Gets or sets the material interactions dictionary.
        /// This dictionary contains all the special relationships between specific materials.
        /// These interaction properties will override properties obtained by normal blending.
        ///</summary>
        public static Dictionary<MaterialPair, InteractionProperties> MaterialInteractions { get; set; }

        /// <summary>
        /// The property blender used by default in collision bounciness.
        /// Uses the space's bounciness PropertyBlendMethod to blend.
        /// </summary>
        /// <param name="aValue">Value associated with the first object to blend.</param>
        /// <param name="bValue">Value associated with the second object to blend.</param>
        /// <param name="extraInfo">Extra data to use in the calculation.</param>
        /// <returns>Blended property value.</returns>
        public static float DefaultBouncinessBlender(float aValue, float bValue, object extraInfo)
        {

            switch (BouncinessBlendMethod)
            {
                case PropertyBlendMethod.Average:
                    return (aValue + bValue) / 2;
                case PropertyBlendMethod.Max:
                    return Math.Max(aValue, bValue);
                case PropertyBlendMethod.Min:
                    return Math.Min(aValue, bValue);
                case PropertyBlendMethod.BiasHigh:
                    return Math.Max(aValue, bValue) * .75f + Math.Min(aValue, bValue) * .25f;
                case PropertyBlendMethod.BiasLow:
                    return Math.Max(aValue, bValue) * .25f + Math.Min(aValue, bValue) * .75f;
            }
            return (aValue + bValue) / 2;
        }


        /// <summary>
        /// The property blender used by default in collision friction.
        /// Uses the space's friction PropertyBlendMethod to blend.
        /// </summary>
        /// <param name="aValue">Value associated with the first object to blend.</param>
        /// <param name="bValue">Value associated with the second object to blend.</param>
        /// <param name="extraInfo">Extra data to use in the calculation.</param>
        /// <returns>Blended property value.</returns>
        public static float DefaultFrictionBlender(float aValue, float bValue, object extraInfo)
        {
            switch (FrictionBlendMethod)
            {
                case PropertyBlendMethod.Average:
                    return (aValue + bValue) / 2;
                case PropertyBlendMethod.Max:
                    return Math.Max(aValue, bValue);
                case PropertyBlendMethod.Min:
                    return Math.Min(aValue, bValue);
                case PropertyBlendMethod.BiasHigh:
                    return Math.Max(aValue, bValue) * .75f + Math.Min(aValue, bValue) * .25f;
                case PropertyBlendMethod.BiasLow:
                    return Math.Max(aValue, bValue) * .25f + Math.Min(aValue, bValue) * .75f;
            }
            return (aValue + bValue) / 2;
        }


    }
}
