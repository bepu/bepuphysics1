using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Assimp;
using BEPUutilities.DataStructures;

namespace MeshProcessor
{
    public partial class ProcessorForm : Form
    {
        private AssimpImporter importer;
        public ProcessorForm()
        {
            InitializeComponent();
            importer = new AssimpImporter();
        }

        private void AddMesh(string path)
        {
            if (!meshesToLoadListBox.Items.Contains(path))
                meshesToLoadListBox.Items.Add(path);
        }


        private void meshesToLoadListBox_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
                e.Effect = DragDropEffects.Copy;
        }

        private void meshesToLoadListBox_DragDrop(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
            {
                foreach (var path in (string[])e.Data.GetData(DataFormats.FileDrop))
                {
                    AddMesh(path);
                }
            }
        }

        private void addButton_Click(object sender, EventArgs e)
        {
            chooseSourceFilesDialog.ShowDialog();
        }

        private void chooseSourceFilesDialog_FileOk(object sender, CancelEventArgs e)
        {
            foreach (var path in chooseSourceFilesDialog.FileNames)
            {
                AddMesh(path);
            }
        }


        private void SaveButton_Click(object sender, EventArgs e)
        {
            if (saveMeshesDialog.ShowDialog() == DialogResult.OK)
                SaveMeshes(saveMeshesDialog.SelectedPath);
        }

        private RawValueList<Vector3D> positions = new RawValueList<Vector3D>();
        private RawValueList<Vector3D> normals = new RawValueList<Vector3D>();
        private RawList<int> indices = new RawList<int>();

        private unsafe void SaveMeshes(string destinationFolder)
        {
            foreach (var sourcePathObject in meshesToLoadListBox.Items)
            {
                var sourcePath = (string)sourcePathObject;
                using (var stream = File.OpenWrite(destinationFolder + Path.GetFileNameWithoutExtension(sourcePath) + ".bpm"))
                {
                    var scene = importer.ImportFile(sourcePath,
                                                    PostProcessSteps.FindDegenerates |
                                                    PostProcessSteps.FindInvalidData |
                                                    PostProcessSteps.ImproveCacheLocality |
                                                    PostProcessSteps.JoinIdenticalVertices |
                                                    PostProcessSteps.OptimizeMeshes |
                                                    PostProcessSteps.OptimizeGraph |
                                                    PostProcessSteps.PreTransformVertices |
                                                    PostProcessSteps.RemoveComponent |
                                                    PostProcessSteps.RemoveRedundantMaterials |
                                                    PostProcessSteps.Triangulate);

                    //Collect all vertex data.
                    positions.Clear();
                    normals.Clear();
                    indices.Clear();
                    foreach (var mesh in scene.Meshes)
                    {
                        //We require positions and normals.
                        if (!mesh.HasVertices || !mesh.HasNormals)
                            continue;

                        var meshIndices = mesh.GetIntIndices();
                        for (int i = 0; i < meshIndices.Length; ++i)
                        {
                            indices.Add(meshIndices[i] + positions.Count);
                        }

                        for (int i = 0; i < mesh.Vertices.Length; ++i)
                        {
                            positions.Add(ref mesh.Vertices[i]);
                        }
                        for (int i = 0; i < mesh.Normals.Length; ++i)
                        {
                            normals.Add(ref mesh.Normals[i]);
                        }
                    }

                    //Write the bytes into a byte array.
                    int byteCount = 4 + 4 + positions.Count * 12 + normals.Count * 12 + indices.Count * 4;
                    byte[] bytes = new byte[byteCount];
                    fixed (byte* buffer = &bytes[0])
                    {
                        //Write the element counts first.
                        var intPointer = (int*)buffer;
                        intPointer[0] = positions.Count;
                        intPointer[1] = indices.Count;
                        
                        //Follow up with all the positions and normals in sequence.
                        var vectorPointers = (Vector3D*)(buffer + 8);
                        for (int i = 0; i < positions.Count; ++i)
                        {
                            vectorPointers[i] = positions.Elements[i];
                        }
                        for (int i = 0; i < positions.Count; ++i)
                        {
                            vectorPointers[i + positions.Count] = normals.Elements[i];
                        }
                    }
                    stream.Write(bytes, 0, byteCount);
                }
            }
        }

    }
}
