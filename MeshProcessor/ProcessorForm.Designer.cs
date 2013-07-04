namespace MeshProcessor
{
    partial class ProcessorForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.chooseSourceFilesDialog = new System.Windows.Forms.OpenFileDialog();
            this.AddButton = new System.Windows.Forms.Button();
            this.SaveButton = new System.Windows.Forms.Button();
            this.meshesToLoadListBox = new System.Windows.Forms.ListBox();
            this.helperLabel = new System.Windows.Forms.Label();
            this.saveMeshesDialog = new System.Windows.Forms.FolderBrowserDialog();
            this.SuspendLayout();
            // 
            // chooseSourceFilesDialog
            // 
            this.chooseSourceFilesDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.chooseSourceFilesDialog_FileOk);
            // 
            // AddButton
            // 
            this.AddButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.AddButton.Location = new System.Drawing.Point(12, 244);
            this.AddButton.Name = "AddButton";
            this.AddButton.Size = new System.Drawing.Size(75, 23);
            this.AddButton.TabIndex = 0;
            this.AddButton.Text = "Add";
            this.AddButton.UseVisualStyleBackColor = true;
            this.AddButton.Click += new System.EventHandler(this.addButton_Click);
            // 
            // SaveButton
            // 
            this.SaveButton.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.SaveButton.Location = new System.Drawing.Point(495, 244);
            this.SaveButton.Name = "SaveButton";
            this.SaveButton.Size = new System.Drawing.Size(75, 23);
            this.SaveButton.TabIndex = 1;
            this.SaveButton.Text = "Save";
            this.SaveButton.UseVisualStyleBackColor = true;
            this.SaveButton.Click += new System.EventHandler(this.SaveButton_Click);
            // 
            // meshesToLoadListBox
            // 
            this.meshesToLoadListBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.meshesToLoadListBox.FormattingEnabled = true;
            this.meshesToLoadListBox.Location = new System.Drawing.Point(12, 39);
            this.meshesToLoadListBox.Name = "meshesToLoadListBox";
            this.meshesToLoadListBox.Size = new System.Drawing.Size(558, 199);
            this.meshesToLoadListBox.TabIndex = 2;
            // 
            // helperLabel
            // 
            this.helperLabel.AutoSize = true;
            this.helperLabel.Location = new System.Drawing.Point(13, 13);
            this.helperLabel.Name = "helperLabel";
            this.helperLabel.Size = new System.Drawing.Size(558, 13);
            this.helperLabel.TabIndex = 3;
            this.helperLabel.Text = "Click \"Add\" or drag .obj mesh files over the list box, then click \"Save\" to proce" +
    "ss the listed files to the target directory.";
            // 
            // ProcessorForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(582, 279);
            this.Controls.Add(this.helperLabel);
            this.Controls.Add(this.meshesToLoadListBox);
            this.Controls.Add(this.SaveButton);
            this.Controls.Add(this.AddButton);
            this.Name = "ProcessorForm";
            this.Text = "Mesh Processor";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.OpenFileDialog chooseSourceFilesDialog;
        private System.Windows.Forms.Button AddButton;
        private System.Windows.Forms.Button SaveButton;
        private System.Windows.Forms.ListBox meshesToLoadListBox;
        private System.Windows.Forms.Label helperLabel;
        private System.Windows.Forms.FolderBrowserDialog saveMeshesDialog;
    }
}

