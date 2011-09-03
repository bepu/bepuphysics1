using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Contains helper methods for extracting vertices and indices from XNA models.
    /// </summary>
    public static class ModelDataExtractor
    {
        //TODO: This section could use some improvements.  It's not very robust, especially on WP7.  Remember the VertexBuffer.GetData bug on WP7.
#if WINDOWS_PHONE
        /// <summary>
        /// Gets an array of vertices and indices from the provided model.
        /// </summary>
        /// <param name="collisionModel">Model to use for the collision shape.</param>
        /// <param name="vertices">Compiled set of vertices from the model.</param>
        /// <param name="indices">Compiled set of indices from the model.</param>
        public static void GetVerticesAndIndicesFromModel(Model collisionModel, out Vector3[] vertices, out int[] indices)
        {
            var verticesList = new List<Vector3>();
            var indicesList = new List<int>();
            var transforms = new Matrix[collisionModel.Bones.Count];
            collisionModel.CopyAbsoluteBoneTransformsTo(transforms);

            Matrix transform;
            foreach (ModelMesh mesh in collisionModel.Meshes)
            {
                if (mesh.ParentBone != null)
                    transform = transforms[mesh.ParentBone.Index];
                else
                    transform = Matrix.Identity;
                AddMesh(mesh, transform, verticesList, indicesList);
            }

            vertices = verticesList.ToArray();
            indices = indicesList.ToArray();


        }

        /// <summary>
        /// Adds a mesh's vertices and indices to the given lists.
        /// </summary>
        /// <param name="collisionModelMesh">Model to use for the collision shape.</param>
        /// <param name="transform">Transform to apply to the mesh.</param>
        /// <param name="vertices">List to receive vertices from the mesh.</param>
        /// <param name="indices">List to receive indices from the mesh.</param>
        public static void AddMesh(ModelMesh collisionModelMesh, Matrix transform, List<Vector3> vertices, List<int> indices)
        {
            foreach (ModelMeshPart meshPart in collisionModelMesh.MeshParts)
            {
                int startIndex = vertices.Count;
                VertexElement[] elements = meshPart.VertexBuffer.VertexDeclaration.GetVertexElements();
                Vector3[] meshPartVertices = new Vector3[meshPart.NumVertices];

                //Check for the built in vertex types.
                //This could be generalized to support any kind of vertex, but the GetData(... vertexStride) bug makes it a little too annoying.
                if (elements.Length == 3 &&
                    elements[0].VertexElementUsage == VertexElementUsage.Position &&
                    elements[0].VertexElementFormat == VertexElementFormat.Vector3 &&
                    elements[1].VertexElementUsage == VertexElementUsage.Normal &&
                    elements[1].VertexElementFormat == VertexElementFormat.Vector3 &&
                    elements[2].VertexElementUsage == VertexElementUsage.TextureCoordinate &&
                    elements[2].VertexElementFormat == VertexElementFormat.Vector2)
                {
                    var verts = new VertexPositionNormalTexture[meshPart.VertexBuffer.VertexCount];
                    meshPart.VertexBuffer.GetData(verts);
                    for (int i = meshPart.VertexOffset; i < meshPart.VertexOffset + meshPart.NumVertices; i++)
                    {
                        Vector3.Transform(ref verts[i].Position, ref transform, out meshPartVertices[i - meshPart.VertexOffset]);
                    }
                }
                else if (elements.Length == 2 &&
                    elements[0].VertexElementUsage == VertexElementUsage.Position &&
                    elements[0].VertexElementFormat == VertexElementFormat.Vector3 &&
                    elements[1].VertexElementUsage == VertexElementUsage.Color &&
                    elements[1].VertexElementFormat == VertexElementFormat.Color)
                {
                    var verts = new VertexPositionColor[meshPart.VertexBuffer.VertexCount];
                    meshPart.VertexBuffer.GetData(verts);
                    for (int i = meshPart.VertexOffset; i < meshPart.VertexOffset + meshPart.NumVertices; i++)
                    {
                        Vector3.Transform(ref verts[i].Position, ref transform, out meshPartVertices[i - meshPart.VertexOffset]);
                    }
                }
                else if (elements.Length == 3 &&
                   elements[0].VertexElementUsage == VertexElementUsage.Position &&
                   elements[0].VertexElementFormat == VertexElementFormat.Vector3 &&
                   elements[1].VertexElementUsage == VertexElementUsage.Color &&
                   elements[1].VertexElementFormat == VertexElementFormat.Color &&
                   elements[2].VertexElementUsage == VertexElementUsage.TextureCoordinate &&
                   elements[2].VertexElementFormat == VertexElementFormat.Vector2)
                {
                    var verts = new VertexPositionColorTexture[meshPart.VertexBuffer.VertexCount];
                    meshPart.VertexBuffer.GetData(verts);
                    for (int i = meshPart.VertexOffset; i < meshPart.VertexOffset + meshPart.NumVertices; i++)
                    {
                        Vector3.Transform(ref verts[i].Position, ref transform, out meshPartVertices[i - meshPart.VertexOffset]);
                    }
                }
                else if (elements.Length == 2 &&
                  elements[0].VertexElementUsage == VertexElementUsage.Position &&
                  elements[0].VertexElementFormat == VertexElementFormat.Vector3 &&
                  elements[1].VertexElementUsage == VertexElementUsage.TextureCoordinate &&
                  elements[1].VertexElementFormat == VertexElementFormat.Vector2)
                {
                    var verts = new VertexPositionTexture[meshPart.VertexBuffer.VertexCount];
                    meshPart.VertexBuffer.GetData(verts);
                    for (int i = meshPart.VertexOffset; i < meshPart.VertexOffset + meshPart.NumVertices; i++)
                    {
                        Vector3.Transform(ref verts[i].Position, ref transform, out meshPartVertices[i - meshPart.VertexOffset]);
                    }
                }
                else
                    throw new FormatException("Unsupported vertex type in mesh.");


                //Transform it so its vertices are located in the model's space as opposed to mesh part space.
                vertices.AddRange(meshPartVertices);

                if (meshPart.IndexBuffer.IndexElementSize == IndexElementSize.ThirtyTwoBits)
                {
                    var meshIndices = new int[meshPart.PrimitiveCount * 3];
                    meshPart.IndexBuffer.GetData(meshPart.StartIndex * 4, meshIndices, 0, meshPart.PrimitiveCount * 3);
                    for (int k = 0; k < meshIndices.Length; k++)
                    {
                        indices.Add(startIndex + meshIndices[k]);
                    }
                }
                else
                {
                    var meshIndices = new ushort[meshPart.PrimitiveCount * 3];
                    meshPart.IndexBuffer.GetData(meshPart.StartIndex * 2, meshIndices, 0, meshPart.PrimitiveCount * 3);
                    for (int k = 0; k < meshIndices.Length; k++)
                    {
                        indices.Add(startIndex + meshIndices[k]);
                    }


                }
            }
        }
#else
        /// <summary>
        /// Gets an array of vertices and indices from the provided model.
        /// </summary>
        /// <param name="collisionModel">Model to use for the collision shape.</param>
        /// <param name="vertices">Compiled set of vertices from the model.</param>
        /// <param name="indices">Compiled set of indices from the model.</param>
        public static void GetVerticesAndIndicesFromModel(Model collisionModel, out BEPUphysics.MathExtensions.Vector3[] vertices, out int[] indices)
        {
            Vector3[] tempVertices;
            GetVerticesAndIndicesFromModel(collisionModel, out tempVertices, out indices);
            vertices = MathConverter.Convert(tempVertices);
        }

        /// <summary>
        /// Gets an array of vertices and indices from the provided model.
        /// </summary>
        /// <param name="collisionModel">Model to use for the collision shape.</param>
        /// <param name="vertices">Compiled set of vertices from the model.</param>
        /// <param name="indices">Compiled set of indices from the model.</param>
        public static void GetVerticesAndIndicesFromModel(Model collisionModel, out Vector3[] vertices, out int[] indices)
        {
            var verticesList = new List<Vector3>();
            var indicesList = new List<int>();
            var transforms = new Matrix[collisionModel.Bones.Count];
            collisionModel.CopyAbsoluteBoneTransformsTo(transforms);

            Matrix transform;
            foreach (ModelMesh mesh in collisionModel.Meshes)
            {
                if (mesh.ParentBone != null)
                    transform = transforms[mesh.ParentBone.Index];
                else
                    transform = Matrix.Identity;
                AddMesh(mesh, transform, verticesList, indicesList);
            }

            vertices = verticesList.ToArray();
            indices = indicesList.ToArray();


        }

        /// <summary>
        /// Adds a mesh's vertices and indices to the given lists.
        /// </summary>
        /// <param name="collisionModelMesh">Model to use for the collision shape.</param>
        /// <param name="transform">Transform to apply to the mesh.</param>
        /// <param name="vertices">List to receive vertices from the mesh.</param>
        /// <param name="indices">List to receive indices from the mesh.</param>
        public static void AddMesh(ModelMesh collisionModelMesh, Matrix transform, List<Vector3> vertices, IList<int> indices)
        {
            foreach (ModelMeshPart meshPart in collisionModelMesh.MeshParts)
            {
                int startIndex = vertices.Count;
                var meshPartVertices = new Vector3[meshPart.NumVertices];
                //Grab position data from the mesh part.
                int stride = meshPart.VertexBuffer.VertexDeclaration.VertexStride;
                meshPart.VertexBuffer.GetData(
                        meshPart.VertexOffset * stride,
                        meshPartVertices,
                        0,
                        meshPart.NumVertices,
                        stride);

                //Transform it so its vertices are located in the model's space as opposed to mesh part space.
                Vector3.Transform(meshPartVertices, ref transform, meshPartVertices);
                vertices.AddRange(meshPartVertices);

                if (meshPart.IndexBuffer.IndexElementSize == IndexElementSize.ThirtyTwoBits)
                {
                    var meshIndices = new int[meshPart.PrimitiveCount * 3];
                    meshPart.IndexBuffer.GetData(meshPart.StartIndex * 4, meshIndices, 0, meshPart.PrimitiveCount * 3);
                    for (int k = 0; k < meshIndices.Length; k++)
                    {
                        indices.Add(startIndex + meshIndices[k]);
                    }
                }
                else
                {
                    var meshIndices = new ushort[meshPart.PrimitiveCount * 3];
                    meshPart.IndexBuffer.GetData(meshPart.StartIndex * 2, meshIndices, 0, meshPart.PrimitiveCount * 3);
                    for (int k = 0; k < meshIndices.Length; k++)
                    {
                        indices.Add(startIndex + meshIndices[k]);
                    }


                }
            }




        }
#endif

    }
}
